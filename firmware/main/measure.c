/* gonzales measurement engine
 * One pulse: settle dark -> t0 + LED on -> poll ADC, timestamp crossings of
 * dark+{th10,th25,th50,th90}*span -> LED off -> cool-down. All timestamps are
 * esp_timer 64-bit microseconds; M-line latencies are relative to LED-on t0.
 */
#include "measure.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_random.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_continuous.h"
#include "soc/gpio_struct.h"
#include "cfg.h"
#include "console.h"

#define ADC_CHANNEL   ADC_CHANNEL_6     /* GPIO34, input-only */
#define CAL_MIN_SPAN  100               /* counts; below = no usable signal */
#define BURST_N       15
#define MAX_RUN_N     10000
#define FRAME_BYTES   64                /* DMA frame: 32 samples -> cross
                                           output granularity = 64 bytes /
                                           (2 * adcrate) = 64 us @ 500 kHz */

static adc_oneshot_unit_handle_t s_adc;        /* oneshot backend  */
static adc_continuous_handle_t s_adc_c;        /* continuous backend */
static int s_last_val;                         /* latest continuous sample */
static uint8_t s_buf[FRAME_BYTES];             /* DMA frame read buffer    */
static uint32_t s_seq;

/* ---- low level -------------------------------------------------------- */

static inline bool mirror_active(void)
{
    return g_cfg.mirror_gpio >= 0 && g_cfg.mirror_gpio != g_cfg.led_gpio;
}

static inline void led_on(void)
{
    gpio_set_level(g_cfg.led_gpio, 1);
    if (mirror_active()) gpio_set_level(g_cfg.mirror_gpio, 1);
}

static inline void led_off(void)
{
    gpio_set_level(g_cfg.led_gpio, 0);
    if (mirror_active()) gpio_set_level(g_cfg.mirror_gpio, 0);
}

static inline int decode_sample(const uint8_t *p)
{
    return ((uint16_t)p[0] | ((uint16_t)p[1] << 8)) & 0x0FFF;
}

/* Digitized sensor output: high while the signal holds above the th25
 * level, low again below it minus hysteresis (Schmitt, no chatter).
 * In continuous mode it is driven from the DMA frame-done ISR on every
 * frame (always live, granularity = one frame); in oneshot mode it only
 * updates while a command reads the ADC. */
static int s_cross_state;

static void cross_levels(uint32_t *thr, uint32_t *hyst)
{
    if (g_cfg.span >= CAL_MIN_SPAN) {
        *thr  = g_cfg.dark +
                (uint32_t)((uint64_t)g_cfg.th25 * g_cfg.span / 10000);
        *hyst = g_cfg.span / 20;
        if (*hyst < 10) *hyst = 10;
    } else {
        *thr = g_cfg.dark + 50;    /* pre-cal fallback: absolute rise level */
        *hyst = 10;
    }
}

static void cross_update(int v)
{
    if (g_cfg.cross_gpio < 0 || v < 0) return;
    uint32_t thr, hyst;
    cross_levels(&thr, &hyst);
    bool want = s_cross_state
        ? (v >= (int)(thr - hyst))    /* high: hold until below thr - hyst */
        : (v >= (int)thr);
    if (want != (bool)s_cross_state) {
        s_cross_state = want;
        gpio_set_level(g_cfg.cross_gpio, want);
    }
}

/* ISR path: direct register writes only (no flash-resident calls, safe
 * while NVS flash operations disable the cache). */
static inline void IRAM_ATTR cross_write_fast(int pin, int lvl)
{
    if (pin < 32) {
        if (lvl) GPIO.out_w1ts = 1U << pin;
        else     GPIO.out_w1tc = 1U << pin;
    } else {
        if (lvl) GPIO.out1_w1ts.val = 1U << (pin - 32);
        else     GPIO.out1_w1tc.val = 1U << (pin - 32);
    }
}

static bool IRAM_ATTR on_frame_done(adc_continuous_handle_t handle,
                                    const adc_continuous_evt_data_t *ed,
                                    void *user)
{
    if (g_cfg.cross_gpio < 0) return false;
    uint32_t thr, hyst;
    cross_levels(&thr, &hyst);
    int ns = ed->size / SOC_ADC_DIGI_RESULT_BYTES;
    for (int i = 0; i < ns; i++) {
        int v = decode_sample(&ed->conv_frame_buffer[i * SOC_ADC_DIGI_RESULT_BYTES]);
        bool want = s_cross_state
            ? (v >= (int)(thr - hyst))
            : (v >= (int)thr);
        if (want != (bool)s_cross_state) {
            s_cross_state = want;
            cross_write_fast(g_cfg.cross_gpio, want);
        }
    }
    return false;
}

static int read_once(void)
{
    int v = -1;
    if (adc_oneshot_read(s_adc, ADC_CHANNEL, &v) != ESP_OK) return -1;
    cross_update(v);
    return v;
}

static void drain_cont(void)
{
    uint32_t got = 0;
    while (adc_continuous_read(s_adc_c, s_buf, sizeof s_buf, &got, 0) == ESP_OK
           && got >= SOC_ADC_DIGI_RESULT_BYTES) {
        int ns = got / SOC_ADC_DIGI_RESULT_BYTES;
        s_last_val = decode_sample(
            &s_buf[(ns - 1) * SOC_ADC_DIGI_RESULT_BYTES]);
    }
}

static int read_latest(void)
{
    if (!s_adc_c) return read_once();
    drain_cont();
    return s_last_val;
}

static int read_raw(void)
{
    if (s_adc_c) return read_latest();
    if (!g_cfg.med) return read_once();
    int a = read_once(), b = read_once(), c = read_once();
    if (a < 0 || b < 0 || c < 0) return -1;
    int lo = a < b ? a : b, hi = a > b ? a : b;
    return c < lo ? lo : (c > hi ? hi : c);    /* clamp = median of 3 */
}

static int cmp_u32(const void *a, const void *b)
{
    return (int)(*(const uint32_t *)a - *(const uint32_t *)b);
}

static uint32_t burst_median(int n, int gap_ms)
{
    uint32_t v[BURST_N];
    if (n > BURST_N) n = BURST_N;
    for (int i = 0; i < n; i++) {
        int r = read_raw();          /* mode-aware: oneshot or DMA latest */
        v[i] = (r < 0) ? 0 : (uint32_t)r;
        vTaskDelay(pdMS_TO_TICKS(gap_ms));
    }
    qsort(v, n, sizeof v[0], cmp_u32);
    return v[n / 2];
}

static bool cont_reconfig(void)
{
    if (adc_continuous_stop(s_adc_c) != ESP_OK) return false;
    adc_digi_pattern_config_t pat = {
        .atten = ADC_ATTEN_DB_12, .channel = ADC_CHANNEL,
        .unit = ADC_UNIT_1, .bit_width = ADC_BITWIDTH_12,
    };
    adc_continuous_config_t cc = {
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE1,
        .sample_freq_hz = g_cfg.adcrate,
        .pattern_num = 1,
        .adc_pattern = &pat,
    };
    if (adc_continuous_config(s_adc_c, &cc) != ESP_OK) return false;
    return adc_continuous_start(s_adc_c) == ESP_OK;
}

void measure_adc_apply(bool announce)
{
    /* same-mode re-apply: light reconfig, never deinit a live DMA driver
     * (teardown-while-running trips xTaskPriorityDisinherit) */
    if (g_cfg.adc_mode && s_adc_c) {
        if (cont_reconfig()) {
            if (announce)
                printf("# adc: continuous @ %" PRIu32 " Hz\n", g_cfg.adcrate);
            return;
        }
        /* fall through to full rebuild on failure */
    }
    if (!g_cfg.adc_mode && !s_adc_c && s_adc) return;

    if (s_adc_c) {
        adc_continuous_stop(s_adc_c);
        adc_continuous_deinit(s_adc_c);
        s_adc_c = NULL;
    }
    if (s_adc) {
        adc_oneshot_del_unit(s_adc);
        s_adc = NULL;
    }

    if (g_cfg.adc_mode) {
        adc_continuous_handle_cfg_t hc = {
            .max_store_buf_size = 8192,
            .conv_frame_size = FRAME_BYTES,
        };
        if (adc_continuous_new_handle(&hc, &s_adc_c) == ESP_OK) {
            adc_continuous_evt_cbs_t cbs = {
                .on_conv_done = on_frame_done,
            };
            adc_continuous_register_event_callbacks(s_adc_c, &cbs, NULL);
            if (cont_reconfig()) {
                if (announce)
                    printf("# adc: continuous @ %" PRIu32 " Hz\n",
                           g_cfg.adcrate);
                return;
            }
            adc_continuous_deinit(s_adc_c);
            s_adc_c = NULL;
        }
        printf("# adc: continuous init failed, falling back to oneshot\n");
    }

    adc_oneshot_unit_init_cfg_t ucfg = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    if (adc_oneshot_new_unit(&ucfg, &s_adc) != ESP_OK) return;
    adc_oneshot_chan_cfg_t ccfg = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten = ADC_ATTEN_DB_12,
    };
    adc_oneshot_config_channel(s_adc, ADC_CHANNEL, &ccfg);
}

void measure_init(void)
{
    measure_adc_apply(false);
    measure_led_apply();
}

void measure_led_apply(void)
{
    gpio_reset_pin(g_cfg.led_gpio);
    gpio_set_direction(g_cfg.led_gpio, GPIO_MODE_OUTPUT);
    if (mirror_active()) {
        gpio_reset_pin(g_cfg.mirror_gpio);
        gpio_set_direction(g_cfg.mirror_gpio, GPIO_MODE_OUTPUT);
    }
    if (g_cfg.cross_gpio >= 0 && g_cfg.cross_gpio != g_cfg.led_gpio &&
        g_cfg.cross_gpio != g_cfg.mirror_gpio) {
        gpio_reset_pin(g_cfg.cross_gpio);
        gpio_set_direction(g_cfg.cross_gpio, GPIO_MODE_OUTPUT);
        gpio_set_level(g_cfg.cross_gpio, 0);
    }
    s_cross_state = 0;
    led_off();
}

/* ---- helpers ----------------------------------------------------------- */

static bool delay_abortable(uint32_t ms)
{
    while (ms) {
        uint32_t c = ms > 20 ? 20 : ms;
        vTaskDelay(pdMS_TO_TICKS(c));
        if (console_poll_stop()) return true;
        ms -= c;
    }
    return console_poll_stop();
}

/* Wait until ADC falls back to dark level (LDR light->dark edge is slow). */
static bool wait_dark(void)
{
    int thr = (int)g_cfg.dark + (int)g_cfg.margin;
    int64_t t = esp_timer_get_time();
    while (esp_timer_get_time() - t < (int64_t)g_cfg.settle_ms * 1000LL) {
        if (read_raw() <= thr) return true;
        if (console_poll_stop()) return false;
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    return read_raw() <= thr;
}

static int64_t jitter_us(void)
{
    uint32_t j = g_cfg.jitter_ms;
    if (!j) return 0;
    return ((int64_t)(esp_random() % (2 * j + 1)) - (int64_t)j) * 1000LL;
}

/* ---- calibration ------------------------------------------------------- */

/* One dark/rise/bright measurement. Returns false on abort or when the
 * light never rose within timeout (caller disambiguates via the sticky
 * stop flag). */
static bool cal_measure(uint32_t *dark, uint32_t *bright)
{
    led_off();
    vTaskDelay(pdMS_TO_TICKS(g_cfg.settle_ms));
    *dark = burst_median(BURST_N, 2);

    /* Blob must travel camera->display before the bright level exists:
     * wait for a real rise (up to timeout), then stabilize calbright.
     * Flat 50-count rise test: stale cal data must not block recal. */
    int rise_thr = (int)*dark + 50;
    led_on();
    int64_t deadline = esp_timer_get_time() +
                       (int64_t)g_cfg.timeout_ms * 1000LL;
    bool rose = false;
    while (esp_timer_get_time() < deadline) {
        if (read_raw() >= rise_thr) { rose = true; break; }
        if (console_poll_stop()) { led_off(); return false; }
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    if (rose && delay_abortable(g_cfg.calbright_ms)) {
        led_off();
        return false;
    }
    *bright = burst_median(BURST_N, 2);
    led_off();
    return rose;
}

void measure_cal(bool automatic)
{
    if (!automatic) printf("# cal: learning dark/bright (chain-aware, up to "
                           "%" PRIu32 "+%" PRIu32 " ms)\n",
                           g_cfg.timeout_ms, g_cfg.settle_ms);
    uint32_t dark, bright;
    if (!cal_measure(&dark, &bright)) {
        if (console_poll_stop()) {
            printf("E,%" PRIu32 ",cal,aborted\n", ++s_seq);
        } else {
            printf("E,%" PRIu32 ",cal,no_rise - light never arrived within "
                   "timeout=%" PRIu32 " ms; check the chain/alignment\n",
                   ++s_seq, g_cfg.timeout_ms);
        }
        return;
    }

    uint32_t span = (bright > dark) ? bright - dark : 0;
    if (span < CAL_MIN_SPAN) {
        printf("E,%" PRIu32 ",cal,span=%" PRIu32 " too small (need >=%d); "
               "keeping previous cal - check LDR alignment/screen brightness\n",
               ++s_seq, span, CAL_MIN_SPAN);
        return;
    }
    g_cfg.dark = dark;
    g_cfg.span = span;
    cfg_save(&g_cfg);
    printf("C,%" PRIu32 ",%" PRIu32 ",%" PRIu32 "\n", dark, bright, span);
}

/* ---- one pulse --------------------------------------------------------- */

typedef struct {
    bool emitted;      /* M or E line printed        */
    bool usable;       /* t50 captured -> stats ok   */
    bool aborted;
    bool dark_settled; /* dark level reached before the pulse */
    int64_t t0;        /* absolute us of LED on      */
    int64_t lat50;     /* us, 0 if not crossed       */
} pulse_out_t;

static void do_pulse(pulse_out_t *out)
{
    memset(out, 0, sizeof *out);
    if (g_cfg.span < CAL_MIN_SPAN) {
        printf("# auto-cal\n");
        measure_cal(true);
        if (g_cfg.span < CAL_MIN_SPAN) {
            printf("E,%" PRIu32 ",run,no_cal\n", ++s_seq);
            out->emitted = true;
            return;
        }
    }

    bool dark_ok = wait_dark();
    if (console_poll_stop()) {
        printf("E,%" PRIu32 ",abort\n", ++s_seq);
        out->aborted = out->emitted = true;
        return;
    }

    uint32_t seq = ++s_seq;
    uint32_t thr[4] = {
        g_cfg.dark + (uint32_t)((uint64_t)g_cfg.th10 * g_cfg.span / 10000),
        g_cfg.dark + (uint32_t)((uint64_t)g_cfg.th25 * g_cfg.span / 10000),
        g_cfg.dark + (uint32_t)((uint64_t)g_cfg.th50 * g_cfg.span / 10000),
        g_cfg.dark + (uint32_t)((uint64_t)g_cfg.th90 * g_cfg.span / 10000),
    };

    int64_t tx[4] = {0, 0, 0, 0};
    uint32_t polls = 0;
    int64_t t0 = esp_timer_get_time();
    int64_t deadline = t0 + (int64_t)g_cfg.timeout_ms * 1000LL;
    int64_t on_deadline = g_cfg.on_ms ?
        t0 + (int64_t)g_cfg.on_ms * 1000LL : INT64_MAX;

    led_on();
    bool led_is_on = true;
    if (s_adc_c) {
        /* continuous DMA: per-sample timestamps reconstructed from the
         * hardware pace: t(i) = t_frame_return - (n-1-i)/adcrate.
         * Drain the backlog first so t_ret cannot lag the samples, and
         * ignore samples predating t0 (pre-LED tail of a frame). */
        uint32_t frames = 0;
        drain_cont();
        while (esp_timer_get_time() < deadline) {
            uint32_t got = 0;
            if (adc_continuous_read(s_adc_c, s_buf, sizeof s_buf, &got, 0)
                    == ESP_OK && got) {
                int64_t t_ret = esp_timer_get_time();
                int ns = got / SOC_ADC_DIGI_RESULT_BYTES;
                for (int i = 0; i < ns; i++) {
                    int v = decode_sample(&s_buf[i * SOC_ADC_DIGI_RESULT_BYTES]);
                    int64_t t_s = t_ret - (int64_t)(ns - 1 - i) * 1000000LL
                                  / (int64_t)g_cfg.adcrate;
                    if (t_s < t0) continue;
                    if (!tx[0] && v >= (int)thr[0]) tx[0] = t_s;
                    if (!tx[1] && v >= (int)thr[1]) tx[1] = t_s;
                    if (!tx[2] && v >= (int)thr[2]) tx[2] = t_s;
                    if (!tx[3] && v >= (int)thr[3]) tx[3] = t_s;
                    if (tx[3]) break;
                }
                polls += ns;
            }
            if (led_is_on && esp_timer_get_time() >= on_deadline) {
                led_off();
                led_is_on = false;
            }
            if (tx[3]) break;
            if ((++frames & 0x7) == 0 && console_poll_stop()) break;
        }
    } else {
        while (esp_timer_get_time() < deadline) {
            int64_t now = esp_timer_get_time();
            int v = read_raw();
            if (v >= 0) {
                if (!tx[0] && v >= (int)thr[0]) tx[0] = now;
                if (!tx[1] && v >= (int)thr[1]) tx[1] = now;
                if (!tx[2] && v >= (int)thr[2]) tx[2] = now;
                if (!tx[3] && v >= (int)thr[3]) tx[3] = now;
            }
            if (led_is_on && now >= on_deadline) {
                led_off();          /* fixed flash over; keep polling crossings */
                led_is_on = false;
            }
            if (tx[3]) break;
            if ((++polls & 0xFF) == 0 && console_poll_stop()) break;
        }
    }
    if (led_is_on) led_off();

    if (console_poll_stop()) {
        printf("E,%" PRIu32 ",abort\n", seq);
        out->aborted = out->emitted = true;
        return;
    }

    if (g_cfg.dbg) {
        printf("# dbg polls=%" PRIu32 " (%.1f kHz)\n", polls,
               polls * 1000.0 / (esp_timer_get_time() - t0));
    }

    if (!tx[0]) {
        printf("E,%" PRIu32 ",to,no_crossing (dark=%" PRIu32 " span=%" PRIu32 " "
               "thr10=%" PRIu32 " - is the screen showing the LED?)\n",
               seq, g_cfg.dark, g_cfg.span, thr[0]);
        out->emitted = true;
        return;
    }

    char fl[40];
    fl[0] = 0;
    if (!dark_ok) strcat(fl, "d?");
    if (tx[3]) {
        strcat(fl, fl[0] ? "|ok" : "ok");
    } else {
        if (!tx[1]) strcat(fl, fl[0] ? "|p25" : "p25");
        if (!tx[2]) strcat(fl, fl[0] ? "|p50" : "p50");
        if (!tx[3]) strcat(fl, fl[0] ? "|p90" : "p90");
    }

    printf("M,%" PRIu32 ",%" PRId64 ",%" PRId64 ",%" PRId64 ",%" PRId64
           ",%" PRId64 ",%" PRIu32 ",%" PRIu32 ",%s\n",
           seq,
           tx[0] ? tx[0] - t0 : 0,
           tx[1] ? tx[1] - t0 : 0,
           tx[2] ? tx[2] - t0 : 0,
           tx[3] ? tx[3] - t0 : 0,
           t0, g_cfg.dark, g_cfg.span, fl);

    out->emitted = true;
    out->usable = tx[2] != 0;
    out->dark_settled = dark_ok;
    out->t0 = t0;
    out->lat50 = tx[2] ? tx[2] - t0 : 0;
}

/* ---- auto-tune ---------------------------------------------------------- */

static void sort_i64(int64_t *a, int n)
{
    for (int i = 1; i < n; i++) {
        int64_t k = a[i];
        int j = i;
        while (j > 0 && a[j - 1] > k) { a[j] = a[j - 1]; j--; }
        a[j] = k;
    }
}

static void sort_u32(uint32_t *a, int n)
{
    for (int i = 1; i < n; i++) {
        uint32_t k = a[i];
        int j = i;
        while (j > 0 && a[j - 1] > k) { a[j] = a[j - 1]; j--; }
        a[j] = k;
    }
}

/* Fire n pulses at current params. Discovery (strict=false): majority of
 * pulses must yield usable lat50. Tier validation (strict=true): >= n-1
 * usable AND dark settled on all but one pulse.
 * Returns the median latency in ms via med_ms. */
static bool probe_batch(uint32_t n, bool strict, uint32_t *med_ms)
{
    int64_t lat[16];
    uint32_t ok = 0, bad_dark = 0;
    for (uint32_t i = 0; i < n; i++) {
        if (i) {
            if (delay_abortable(g_cfg.interval_ms)) return false;
            if (console_poll_stop()) return false;
        }
        pulse_out_t r;
        do_pulse(&r);
        if (r.aborted) return false;
        if (!r.dark_settled) bad_dark++;
        if (r.usable && ok < 16) lat[ok++] = r.lat50;
    }
    bool enough = strict
        ? (ok >= n - 1 && bad_dark <= 1)
        : (ok >= (n + 1) / 2 && ok > 0);
    if (!enough) return false;
    sort_i64(lat, ok);
    *med_ms = (uint32_t)(lat[ok / 2] / 1000);
    return true;
}

typedef struct {
    const char *name;
    uint32_t settle_x10, timeout_x10, interval_x10;
} tune_tier_t;

void measure_tune(uint32_t latency_ms)
{
    /* tune = multi-pass calibration + measured-latency parameter search.
     * latency_ms == 0: discover the chain latency by probing, then run
     * the tier search. Non-zero: trust the given latency, conservative
     * formulas, one validation probe.
     * Tiers run conservative -> fast; every tier must pass the same
     * validation bar (>=7/8 pulses usable and dark-settled, median stable),
     * so on ties the fastest tier wins. */
    static const tune_tier_t tiers[] = {
        { "conservative", 25, 50, 30 },
        { "balanced",     16, 30, 22 },
        { "fast",         13, 20, 17 },
    };

    struct { uint32_t interval, settle, timeout, jitter; } backup = {
        g_cfg.interval_ms, g_cfg.settle_ms, g_cfg.timeout_ms, g_cfg.jitter_ms,
    };
    bool restore_on_fail = true;

    /* safe probing params: generous for any chain up to ~1.5 s */
    g_cfg.interval_ms = 1200;
    g_cfg.settle_ms = 800;
    g_cfg.timeout_ms = 2000;
    g_cfg.jitter_ms = 100;

    printf("# tune: calibrating (3 passes)\n");
    uint32_t spans[3], darks[3], brights[3];
    for (int i = 0; i < 3; i++) {
        if (!cal_measure(&darks[i], &brights[i])) {
            printf("E,%" PRIu32 ",tune,cal_aborted_or_no_rise\n", ++s_seq);
            goto out;
        }
        spans[i] = brights[i] > darks[i] ? brights[i] - darks[i] : 0;
        printf("# tune: cal pass %d: dark=%" PRIu32 " span=%" PRIu32 "\n",
               i + 1, darks[i], spans[i]);
        if (console_poll_stop()) goto out;
    }
    if (spans[0] < CAL_MIN_SPAN || spans[1] < CAL_MIN_SPAN ||
        spans[2] < CAL_MIN_SPAN) {
        printf("E,%" PRIu32 ",tune,span_too_small - check signal\n", ++s_seq);
        goto out;
    }
    /* median of 3 */
    sort_u32(spans, 3);
    sort_u32(darks, 3);
    sort_u32(brights, 3);
    if (spans[2] > 2 * spans[0])
        printf("# tune: warning: span unstable across passes\n");
    g_cfg.dark = darks[1];
    g_cfg.span = spans[1];
    printf("C,%" PRIu32 ",%" PRIu32 ",%" PRIu32 "\n",
           g_cfg.dark, brights[1], g_cfg.span);

    /* phase 2: discover latency, escalating the window if pulses time out */
    uint32_t L = latency_ms;
    bool locked = latency_ms != 0;
    if (!locked) {
        for (int attempt = 0; attempt < 3 && !locked; attempt++) {
            printf("# tune: probing latency (interval=%" PRIu32 " settle=%" PRIu32
                   " timeout=%" PRIu32 ")\n",
                   g_cfg.interval_ms, g_cfg.settle_ms, g_cfg.timeout_ms);
            locked = probe_batch(5, false, &L);
            if (locked) break;
            if (console_poll_stop()) goto out;
            g_cfg.timeout_ms = g_cfg.timeout_ms * 2 > 10000 ?
                10000 : g_cfg.timeout_ms * 2;
            g_cfg.settle_ms = g_cfg.settle_ms * 2 > 6000 ?
                6000 : g_cfg.settle_ms * 2;
            g_cfg.interval_ms = g_cfg.interval_ms * 3 / 2;
        }
        if (!locked) {
            printf("E,%" PRIu32 ",tune,no_lock - chain never responded\n",
                   ++s_seq);
            goto out;
        }
    }
    printf("# tune: chain latency ~%" PRIu32 " ms\n", L);

    /* phase 3: tier search, fastest passing tier wins */
    int tier_max = latency_ms ? 1 : 3;
    for (int t = 0; t < tier_max; t++) {
        const tune_tier_t *tier = &tiers[t];
        uint32_t settle = tier->settle_x10 * L / 10;
        uint32_t timeout = tier->timeout_x10 * L / 10 + 100;
        uint32_t interval = tier->interval_x10 * L / 10;
        uint32_t floor_s = L * 11 / 10 + 20;
        if (settle < floor_s) settle = floor_s;
        if (settle < 150) settle = 150;
        if (timeout < 500) timeout = 500;
        if (interval < 220) interval = 220;
        uint32_t jitter = L / 2;
        if (jitter < 50) jitter = 50;
        if (jitter > 150) jitter = 150;

        g_cfg.settle_ms = settle;
        g_cfg.timeout_ms = timeout;
        g_cfg.interval_ms = interval;
        g_cfg.jitter_ms = jitter;

        printf("# tune: tier %-13s interval=%" PRIu32 " settle=%" PRIu32
               " timeout=%" PRIu32 " jitter=%" PRIu32 ": probing 8\n",
               tier->name, interval, settle, timeout, jitter);
        uint32_t m = 0;
        bool pass = probe_batch(8, true, &m);
        uint32_t shift = m > L ? m - L : L - m;
        if (pass && shift <= (L / 4 > 30 ? L / 4 : 30)) {
            printf("# tune: tier %s passed (median %" PRIu32 " ms)\n",
                   tier->name, m);
            L = m;
            restore_on_fail = false;
        } else {
            printf("# tune: tier %s rejected%s\n", tier->name,
                   pass ? " (median shifted)" : "");
            break;
        }
        if (console_poll_stop()) goto out;
    }

    if (restore_on_fail) {
        /* even the conservative tier failed against the measured L:
         * fall back to the conservative formulas */
        g_cfg.settle_ms = 25 * L / 10;
        g_cfg.timeout_ms = 50 * L / 10 + 100;
        g_cfg.interval_ms = 30 * L / 10;
        if (g_cfg.settle_ms < 150) g_cfg.settle_ms = 150;
        if (g_cfg.timeout_ms < 500) g_cfg.timeout_ms = 500;
        if (g_cfg.interval_ms < 220) g_cfg.interval_ms = 220;
        printf("# tune: no tier validated; conservative fallback applied\n");
    }
    cfg_save(&g_cfg);
    printf("# tune done: L=%" PRIu32 " ms interval=%" PRIu32 " settle=%" PRIu32
           " timeout=%" PRIu32 " jitter=%" PRIu32 " dark=%" PRIu32
           " span=%" PRIu32 "\n",
           L, g_cfg.interval_ms, g_cfg.settle_ms, g_cfg.timeout_ms,
           g_cfg.jitter_ms, g_cfg.dark, g_cfg.span);
    return;

out:
    g_cfg.interval_ms = backup.interval;
    g_cfg.settle_ms = backup.settle;
    g_cfg.timeout_ms = backup.timeout;
    g_cfg.jitter_ms = backup.jitter;
    cfg_save(&g_cfg);
    printf("# tune aborted, previous timing parameters restored\n");
}

/* ---- commands ---------------------------------------------------------- */

void measure_oneshot(void)
{
    pulse_out_t r;
    do_pulse(&r);
}

void measure_run(uint32_t n, bool has_iv, uint32_t iv)
{
    if (n > MAX_RUN_N) n = MAX_RUN_N;
    uint32_t interval = has_iv ? iv : g_cfg.interval_ms;

    int64_t *lat = calloc(n, sizeof(int64_t));
    if (!lat) {
        printf("E,0,run,nomem\n");
        return;
    }

    uint32_t ok = 0, err = 0;
    bool aborted = false;
    int64_t prev_t0 = 0;

    for (uint32_t i = 0; i < n; i++) {
        if (i > 0) {
            int64_t target = prev_t0 + (int64_t)interval * 1000LL + jitter_us();
            int64_t rem = target - esp_timer_get_time();
            if (rem <= 0) {
                /* interval floor no longer binds (chain round-trip overran
                 * it): still wait a random 0..2*jitter so pulse timing can
                 * never phase-lock to the chain's own dark-settle cadence */
                rem = (int64_t)(esp_random() % (2 * g_cfg.jitter_ms + 1)) * 1000LL;
            }
            if (delay_abortable((uint32_t)(rem / 1000 + 1))) {
                aborted = true;
                break;
            }
            if (console_poll_stop()) { aborted = true; break; }
        }

        pulse_out_t r;
        do_pulse(&r);
        if (r.aborted) { aborted = true; break; }
        if (r.emitted) {
            if (r.usable) lat[ok++] = r.lat50; else err++;
        } else {
            err++;    /* defensive: do_pulse always emits */
        }
        prev_t0 = r.t0;
    }

    printf("# run done n=%" PRIu32 " ok=%" PRIu32 " err=%" PRIu32 "%s",
           n, ok, err, aborted ? " stopped=1" : "");
    if (ok) {
        /* lat[] is filled densely 0..ok-1 */
        for (uint32_t i = 1; i < ok; i++) {           /* insertion sort */
            int64_t k = lat[i];
            uint32_t j = i;
            while (j > 0 && lat[j - 1] > k) { lat[j] = lat[j - 1]; j--; }
            lat[j] = k;
        }
        printf(" lat50_ms min=%.3f med=%.3f max=%.3f",
               lat[0] / 1000.0, lat[ok / 2] / 1000.0, lat[ok - 1] / 1000.0);
    }
    printf("\n");
    free(lat);
}

void measure_mon(uint32_t period_ms, uint32_t count)
{
    console_clear_stop();
    uint32_t i = 0;
    do {
        int v = read_raw();
        if (v >= 0) printf("R,%d\n", v);
        i++;
        if (delay_abortable(period_ms)) {
            printf("# mon stopped n=%" PRIu32 "\n", i);
            return;
        }
    } while (!count || i < count);
    printf("# mon done n=%" PRIu32 "\n", i);
}
