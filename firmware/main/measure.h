#pragma once
#include <stdbool.h>
#include <stdint.h>

/* Measurement engine: sensor on GPIO34 (ADC1_CH6), stimulus LED.
 * Two ADC backends, runtime-selectable: software oneshot poll (~9 kHz) or
 * continuous DMA (hardware-paced, adcrate Hz, timestamps reconstructed
 * from sample index). */
void measure_init(void);
void measure_adc_apply(bool announce);       /* (re)configure ADC backend     */
void measure_led_apply(void);                /* (re)init LED gpio             */

void measure_cal(bool automatic);             /* learn dark/bright -> C or E line */
void measure_tune(uint32_t latency_ms);       /* cal passes + parameter search;
                                                 0 = discover latency */
void measure_oneshot(void);                   /* one pulse -> M or E line */
void measure_run(uint32_t n, bool has_iv, uint32_t iv); /* n pulses + summary */
void measure_mon(uint32_t period_ms, uint32_t count);   /* R stream */
