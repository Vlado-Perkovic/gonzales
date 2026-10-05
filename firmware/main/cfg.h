#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "esp_err.h"

/* Runtime configuration, persisted in NVS. Owned as global g_cfg (cfg.c). */
typedef struct {
    uint32_t interval_ms;    /* min spacing between pulse starts            */
    uint32_t settle_ms;      /* max wait for dark level before each pulse   */
    uint32_t timeout_ms;     /* max capture window after LED on             */
    uint32_t on_ms;          /* fixed LED on-time; 0 = until th90/timeout   */
    uint32_t jitter_ms;      /* +- random jitter on pulse spacing           */
    uint32_t calbright_ms;   /* LED-on settle time while learning bright    */
    uint32_t margin;         /* dark-settle margin, ADC counts              */
    uint32_t mon_period_ms;  /* mon stream period                           */
    uint8_t  med;            /* 1 = median-of-3 ADC reads                   */
    uint8_t  dbg;            /* 1 = debug prints (poll rate etc.)           */
    uint8_t  adc_mode;       /* 0 = oneshot poll, 1 = continuous DMA        */
    uint32_t adcrate;        /* continuous sample rate, Hz                  */
    uint32_t th10;           /* threshold fractions, x10000                 */
    uint32_t th25;
    uint32_t th50;
    uint32_t th90;
    int32_t  led_gpio;       /* stimulus LED gpio                           */
    int32_t  mirror_gpio;    /* mirrors LED level for loopback tests, -1=off*/
    int32_t  cross_gpio;     /* digitized sensor output (th25 Schmitt), -1=off*/
    uint32_t dark;           /* calibration: dark ADC level                 */
    uint32_t span;           /* calibration: bright-dark span               */
} gonz_cfg_t;

extern gonz_cfg_t g_cfg;

void cfg_defaults(gonz_cfg_t *c);
esp_err_t cfg_load(gonz_cfg_t *c);
esp_err_t cfg_save(const gonz_cfg_t *c);

/* Parse and clamp one "key value"; on failure returns false and fills err. */
bool cfg_set_key(gonz_cfg_t *c, const char *key, const char *val,
                 char *err, size_t errlen);
void cfg_print(const gonz_cfg_t *c);
