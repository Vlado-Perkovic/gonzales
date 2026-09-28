#include "cfg.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <inttypes.h>
#include "nvs_flash.h"
#include "nvs.h"

gonz_cfg_t g_cfg;

#define NVS_NS "gonz"

static const gonz_cfg_t c_defaults = {
    /* Tuned on a ~80 ms phone chain for fast, artifact-free recording
     * (4-5 samples/s, min-latency floor 2x above the residual-artifact
     * zone). Re-tune per chain with the README parameter guide. */
    .interval_ms  = 220,
    .settle_ms    = 200,
    .timeout_ms   = 500,
    .on_ms        = 0,
    .jitter_ms    = 50,
    .calbright_ms = 300,
    .margin       = 10,
    .mon_period_ms= 100,
    .med          = 1,
    .dbg          = 0,
    .adc_mode     = 0,
    .adcrate      = 500000,
    .th10 = 1000, .th25 = 2500, .th50 = 5000, .th90 = 7000,
    .led_gpio     = 26,
    .mirror_gpio  = 27,
    .dark         = 0,
    .span         = 0,
};

void cfg_defaults(gonz_cfg_t *c)
{
    memcpy(c, &c_defaults, sizeof(*c));
}

static esp_err_t rd_u32(nvs_handle_t h, const char *k, uint32_t *v)
{
    uint32_t t;
    esp_err_t e = nvs_get_u32(h, k, &t);
    if (e == ESP_OK) *v = t;
    return e;
}

/* narrow-field read: casting &uint8_field to u32* would spill 3 bytes of
 * zeros into the next struct member (this bug corrupted th10/adcrate) */
static void rd_u8(nvs_handle_t h, const char *k, uint8_t *v)
{
    uint32_t t;
    if (nvs_get_u32(h, k, &t) == ESP_OK) *v = (uint8_t)t;
}

esp_err_t cfg_load(gonz_cfg_t *c)
{
    cfg_defaults(c);
    nvs_handle_t h;
    esp_err_t e = nvs_open(NVS_NS, NVS_READONLY, &h);
    if (e != ESP_OK) return e;                    /* fresh NVS: defaults */

    rd_u32(h, "interval",  &c->interval_ms);
    rd_u32(h, "settle",    &c->settle_ms);
    rd_u32(h, "timeout",   &c->timeout_ms);
    rd_u32(h, "on",        &c->on_ms);
    rd_u32(h, "jitter",    &c->jitter_ms);
    rd_u32(h, "calbright", &c->calbright_ms);
    rd_u32(h, "margin",    &c->margin);
    rd_u32(h, "monperiod", &c->mon_period_ms);
    rd_u8(h, "med",       &c->med);
    rd_u8(h, "dbg",       &c->dbg);
    rd_u8(h, "adcmode",   &c->adc_mode);
    rd_u32(h, "adcrate",   &c->adcrate);
    rd_u32(h, "th10",      &c->th10);
    rd_u32(h, "th25",      &c->th25);
    rd_u32(h, "th50",      &c->th50);
    rd_u32(h, "th90",      &c->th90);
    rd_u32(h, "led",       (uint32_t *)&c->led_gpio);
    rd_u32(h, "mirror",    (uint32_t *)&c->mirror_gpio);
    rd_u32(h, "dark",      &c->dark);
    rd_u32(h, "span",      &c->span);
    nvs_close(h);
    return ESP_OK;
}

esp_err_t cfg_save(const gonz_cfg_t *c)
{
    nvs_handle_t h;
    esp_err_t e = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (e != ESP_OK) return e;
    nvs_set_u32(h, "interval",  c->interval_ms);
    nvs_set_u32(h, "settle",    c->settle_ms);
    nvs_set_u32(h, "timeout",   c->timeout_ms);
    nvs_set_u32(h, "on",        c->on_ms);
    nvs_set_u32(h, "jitter",    c->jitter_ms);
    nvs_set_u32(h, "calbright", c->calbright_ms);
    nvs_set_u32(h, "margin",    c->margin);
    nvs_set_u32(h, "monperiod", c->mon_period_ms);
    nvs_set_u32(h, "med",       c->med);
    nvs_set_u32(h, "dbg",       c->dbg);
    nvs_set_u32(h, "adcmode",   c->adc_mode);
    nvs_set_u32(h, "adcrate",   c->adcrate);
    nvs_set_u32(h, "th10",      c->th10);
    nvs_set_u32(h, "th25",      c->th25);
    nvs_set_u32(h, "th50",      c->th50);
    nvs_set_u32(h, "th90",      c->th90);
    nvs_set_u32(h, "led",       (uint32_t)c->led_gpio);
    nvs_set_u32(h, "mirror",    (uint32_t)c->mirror_gpio);
    nvs_set_u32(h, "dark",      c->dark);
    nvs_set_u32(h, "span",      c->span);
    e = nvs_commit(h);
    nvs_close(h);
    return e;
}

typedef struct { const char *key; uint32_t *v; uint32_t lo, hi; } kv_t;

static bool parse_set(gonz_cfg_t *c, const char *key, const char *val,
                      char *err, size_t errlen)
{
    static const char *ledwarn = "led gpio %ld outside 0..33";

    if (strcasecmp(key, "adc") == 0) {
        if (strcasecmp(val, "oneshot") == 0 || strcmp(val, "0") == 0) {
            c->adc_mode = 0;
        } else if (strcasecmp(val, "continuous") == 0 || strcmp(val, "1") == 0) {
            c->adc_mode = 1;
        } else {
            snprintf(err, errlen, "adc: oneshot|continuous");
            return false;
        }
        return true;
    }

    char *end = NULL;
    long v = strtol(val, &end, 0);
    if (end == val || *end != '\0') {
        snprintf(err, errlen, "'%s' is not an integer", val);
        return false;
    }
    kv_t kv[] = {
        {"interval",  &c->interval_ms,  50, 600000},
        {"settle",    &c->settle_ms,     0, 60000},
        {"timeout",   &c->timeout_ms,   50, 10000},
        {"on",        &c->on_ms,         0, 60000},
        {"jitter",    &c->jitter_ms,     0,  5000},
        {"calbright", &c->calbright_ms, 50, 10000},
        {"margin",    &c->margin,        0,  2000},
        {"monperiod", &c->mon_period_ms,10, 60000},
        {"med",       (uint32_t *)&c->med, 0, 1},
        {"dbg",       (uint32_t *)&c->dbg, 0, 1},
        {"adcrate",   &c->adcrate,     10000, 2000000},
        {"th10",      &c->th10,          1,  9999},
        {"th25",      &c->th25,          1,  9999},
        {"th50",      &c->th50,          1,  9999},
        {"th90",      &c->th90,          1,  9999},
    };
    for (size_t i = 0; i < sizeof(kv) / sizeof(kv[0]); i++) {
        if (strcasecmp(key, kv[i].key) == 0) {
            if ((uint32_t)v < kv[i].lo || (uint32_t)v > kv[i].hi) {
                snprintf(err, errlen, "%s out of range %" PRIu32 "..%" PRIu32,
                         kv[i].key, kv[i].lo, kv[i].hi);
                return false;
            }
            *kv[i].v = (uint32_t)v;
            return true;
        }
    }
    if (strcasecmp(key, "led") == 0) {
        if (v < 0 || v > 33) {
            snprintf(err, errlen, ledwarn, v);
            return false;
        }
        c->led_gpio = (int32_t)v;
        return true;
    }
    if (strcasecmp(key, "mirror") == 0) {
        if (v < -1 || v > 33) {
            snprintf(err, errlen, "mirror gpio %ld outside -1..33", v);
            return false;
        }
        c->mirror_gpio = (int32_t)v;
        return true;
    }
    snprintf(err, errlen, "unknown key '%s' (see get)", key);
    return false;
}

bool cfg_set_key(gonz_cfg_t *c, const char *key, const char *val,
                 char *err, size_t errlen)
{
    if (!parse_set(c, key, val, err, errlen)) return false;
    cfg_save(c);
    return true;
}

void cfg_print(const gonz_cfg_t *c)
{
    printf("# interval=%" PRIu32 " settle=%" PRIu32 " timeout=%" PRIu32
           " on=%" PRIu32 " jitter=%" PRIu32 " calbright=%" PRIu32
           " margin=%" PRIu32 " monperiod=%" PRIu32 "\n",
           c->interval_ms, c->settle_ms, c->timeout_ms, c->on_ms,
           c->jitter_ms, c->calbright_ms, c->margin, c->mon_period_ms);
    printf("# med=%" PRIu8 " dbg=%" PRIu8 " adc=%s adcrate=%" PRIu32
           " th10=%" PRIu32 " th25=%" PRIu32
           " th50=%" PRIu32 " th90=%" PRIu32 " led=%" PRId32 " mirror=%" PRId32 "\n",
           c->med, c->dbg, c->adc_mode ? "continuous" : "oneshot", c->adcrate,
           c->th10, c->th25, c->th50, c->th90, c->led_gpio,
           c->mirror_gpio);
    printf("# dark=%" PRIu32 " span=%" PRIu32 "\n", c->dark, c->span);
}
