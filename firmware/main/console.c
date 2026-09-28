#include "console.h"
#include <stdio.h>
#include <string.h>
#include <strings.h>
#include <stdlib.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/uart_vfs.h"
#include "esp_vfs_dev.h"
#include "esp_idf_version.h"
#include "cfg.h"
#include "measure.h"

#define GONZ_FW_VERSION "0.3.1"

#define RX_BUF_SZ 2048
#define TX_BUF_SZ 2048
#define GONZ_LINE_MAX 128

static volatile bool s_stop;
static char s_scan[32];
static size_t s_scan_len;

bool console_poll_stop(void)
{
    uint8_t tmp[32];
    int n;
    while ((n = uart_read_bytes(UART_NUM_0, tmp, sizeof tmp, 0)) > 0) {
        for (int i = 0; i < n; i++) {
            char c = (char)tmp[i];
            if (c == '\n' || c == '\r') {
                if (s_scan_len) {
                    s_scan[s_scan_len] = 0;
                    if (strcasecmp(s_scan, "stop") == 0) s_stop = true;
                    s_scan_len = 0;
                }
            } else if (s_scan_len < sizeof s_scan - 1) {
                s_scan[s_scan_len++] = c;
            } else {
                s_scan_len = 0;    /* garbled oversize token: drop */
            }
        }
    }
    return s_stop;
}

void console_clear_stop(void)
{
    s_stop = false;
    s_scan_len = 0;
}

static void cmd_help(void)
{
    printf("# gonzales v%s - glass-to-glass latency shell\n", GONZ_FW_VERSION);
    printf("# commands:\n");
    printf("#   help                     this text\n");
    printf("#   version                  firmware/idf version (V line)\n");
    printf("#   cal                      learn dark/bright levels (C line)\n");
    printf("#   oneshot                  one measurement (M line)\n");
    printf("#   run [n] [interval_ms]    n measurements (default 10), M each + summary\n");
    printf("#   mon [period_ms] [count]  stream R lines; count 0 = until stop\n");
    printf("#   stop                     abort run/mon\n");
    printf("#   set <key> <value>        set param (keys: see get; adc oneshot|continuous)\n");
    printf("#   get                      list params + calibration\n");
    printf("#   reset                    restore default params\n");
    printf("# lines: '# ' info | M,... meas | C,... cal | R,... raw | E,... error\n");
    printf("# during run/mon only 'stop' is honored; other input is discarded\n");
}

static void cmd_version(void)
{
    printf("V,gonzales,%s,%s,%s %s\n", GONZ_FW_VERSION,
           esp_get_idf_version(), __DATE__, __TIME__);
}

static void dispatch(char *line)
{
    char *argv[8];
    int argc = 0;
    for (char *t = strtok(line, " \t"); t && argc < 8; t = strtok(NULL, " \t"))
        argv[argc++] = t;
    if (argc == 0) return;

    console_clear_stop();

    if (strcasecmp(argv[0], "help") == 0) {
        cmd_help();
    } else if (strcasecmp(argv[0], "version") == 0 || strcasecmp(argv[0], "ver") == 0) {
        cmd_version();
    } else if (strcasecmp(argv[0], "reset") == 0) {
        cfg_defaults(&g_cfg);
        cfg_save(&g_cfg);
        measure_led_apply();
        printf("# defaults restored\n");
    } else if (strcasecmp(argv[0], "get") == 0 || strcasecmp(argv[0], "params") == 0) {
        cfg_print(&g_cfg);
    } else if (strcasecmp(argv[0], "set") == 0) {
        if (argc != 3) {
            printf("# usage: set <key> <value>\n");
            return;
        }
        char err[80];
        if (cfg_set_key(&g_cfg, argv[1], argv[2], err, sizeof err)) {
            if (strcasecmp(argv[1], "led") == 0 || strcasecmp(argv[1], "mirror") == 0)
                measure_led_apply();
            if (strcasecmp(argv[1], "adc") == 0 || strcasecmp(argv[1], "adcrate") == 0)
                measure_adc_apply(true);
            printf("# set %s=%s\n", argv[1], argv[2]);
        } else {
            printf("E,0,set,%s\n", err);
        }
    } else if (strcasecmp(argv[0], "cal") == 0) {
        measure_cal(false);
    } else if (strcasecmp(argv[0], "oneshot") == 0 || strcasecmp(argv[0], "one") == 0) {
        measure_oneshot();
    } else if (strcasecmp(argv[0], "run") == 0) {
        long n = 10;                       /* bare "run" = 10 pulses */
        if (argc > 1) {
            char *end = NULL;
            n = strtol(argv[1], &end, 0);
            if (end == argv[1] || *end != '\0' || n < 1 || n > 10000) {
                printf("# usage: run [n 1..10000] [interval_ms]\n");
                return;
            }
        }
        long iv = 0;
        bool has_iv = false;
        if (argc > 2) {
            char *end = NULL;
            iv = strtol(argv[2], &end, 0);
            if (end == argv[2] || *end != '\0' || iv < 50 || iv > 600000) {
                printf("# interval_ms out of range 50..600000\n");
                return;
            }
            has_iv = true;
        }
        measure_run((uint32_t)n, has_iv, (uint32_t)iv);
    } else if (strcasecmp(argv[0], "mon") == 0 || strcasecmp(argv[0], "monitor") == 0) {
        long p = 100, cnt = 0;
        char *end = NULL;
        if (argc > 1) {
            p = strtol(argv[1], &end, 0);
            if (end == argv[1] || *end != '\0' || p < 10 || p > 60000) {
                printf("# period_ms out of range 10..60000\n");
                return;
            }
        }
        if (argc > 2) {
            cnt = strtol(argv[2], &end, 0);
            if (end == argv[2] || *end != '\0' || cnt < 0 || cnt > 1000000) {
                printf("# count out of range 0..1000000\n");
                return;
            }
        }
        measure_mon((uint32_t)p, (uint32_t)cnt);
    } else if (strcasecmp(argv[0], "stop") == 0) {
        printf("# nothing running\n");
    } else {
        printf("# unknown: %s (try help)\n", argv[0]);
    }
}

static void console_task(void *arg)
{
    /* UART0: driver + VFS so fgets/printf share the same ring buffers. */
    uart_config_t uc = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_param_config(UART_NUM_0, &uc);
    uart_set_pin(UART_NUM_0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, RX_BUF_SZ, TX_BUF_SZ,
                                        0, NULL, 0));
    uart_vfs_dev_use_driver(UART_NUM_0);
    /* CR mode (not CRLF): lone \r, lone \n and \r\n must all terminate lines. */
    uart_vfs_dev_port_set_rx_line_endings(UART_NUM_0, ESP_LINE_ENDINGS_CR);
    uart_vfs_dev_port_set_tx_line_endings(UART_NUM_0, ESP_LINE_ENDINGS_CRLF);

    printf("# gonzales v%s ready - type help\n", GONZ_FW_VERSION);
    fflush(stdout);

    char line[GONZ_LINE_MAX];
    for (;;) {
        if (!fgets(line, sizeof line, stdin)) continue;
        size_t l = strlen(line);
        while (l && (line[l - 1] == '\n' || line[l - 1] == '\r')) line[--l] = 0;
        if (l == 0) continue;
        printf("# > %s\n", line);
        dispatch(line);
        fflush(stdout);
    }
}

void console_start(void)
{
    xTaskCreatePinnedToCore(console_task, "console", 6144, NULL, 10, NULL, 1);
}
