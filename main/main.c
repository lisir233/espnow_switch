#include "esp32_c2_wireless_switch_board.h"
#include "esp_sleep.h"
#include "espnow_lite.h"
#include "nvs_flash.h"
#include <stdio.h>

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    board_init();
    board_sleep_set(10, true);
    espnow_wifi_init();
    espnow_init();
}
