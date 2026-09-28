/* gonzales — camera-to-display glass-to-glass latency meter */
#include "cfg.h"
#include "console.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "measure.h"
#include "nvs_flash.h"
#include <stdio.h>

void app_main(void) {
  esp_err_t e = nvs_flash_init();
  if (e == ESP_ERR_NVS_NO_FREE_PAGES || e == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    e = nvs_flash_init();
  }
  ESP_ERROR_CHECK(e);

  cfg_defaults(&g_cfg);
  cfg_load(&g_cfg); /* missing keys keep defaults */

  measure_init();
  // gpio_set_level(g_cfg.led_gpio, 1);
  console_start(); /* does not return; shell runs on core 1 */
}
