#pragma once
#include <stdbool.h>
#include <stdint.h>

/* Measurement engine: ADC (LDR divider on GPIO34 / ADC1_CH6) + stimulus LED. */
void measure_init(void);
void measure_led_apply(void);                 /* (re)init LED gpio after set led */

void measure_cal(bool automatic);             /* learn dark/bright -> C or E line */
void measure_oneshot(void);                   /* one pulse -> M or E line */
void measure_run(uint32_t n, bool has_iv, uint32_t iv); /* n pulses + summary */
void measure_mon(uint32_t period_ms, uint32_t count);   /* R stream */
