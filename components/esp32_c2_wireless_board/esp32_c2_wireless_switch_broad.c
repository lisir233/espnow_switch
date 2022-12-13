// Copyright 2020-2021 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include "driver/gpio.h"
#include "esp32_c2_wireless_switch_board.h"
#include "esp_log.h"
#include "esp_sleep.h"

static const char* TAG = "Board";
static esp_err_t board_gpio_init(void)
{
    esp_err_t ret = ESP_OK;
    // zero-initialize the config structure.
    gpio_config_t io_conf = {};
    // disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    // set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    // bit mask of the pins that you want to set
    io_conf.pin_bit_mask = BOARD_IO_PIN_SEL_OUTPUT;
    // disable pull-down mode
    io_conf.pull_down_en = 0;
    // disable pull-up mode
    io_conf.pull_up_en = 0;
    // configure GPIO with the given settings
    gpio_config(&io_conf);
    gpio_set_level(BOARD_IO_PWRLOCK, 1);
    gpio_set_level(BOARD_IO_LED, 1);

    return ret;
}

esp_err_t board_init(void)
{
    esp_err_t ret;
    ret = board_gpio_init();

    return ret;
}

void board_sleep_set(uint32_t ms, uint8_t power_is_en)
{
    esp_sleep_enable_timer_wakeup(ms * 1000);
    if (power_is_en) {
        gpio_set_level(BOARD_IO_PWRLOCK, 1);
        gpio_hold_en(BOARD_IO_PWRLOCK);
    } else {
        gpio_hold_dis(BOARD_IO_PWRLOCK);
    }
    esp_light_sleep_start();
    gpio_hold_dis(BOARD_IO_PWRLOCK);
}
void board_bulb_toggle(void)
{
    static int level = 0;
    if (level) {
        level = 0;
    } else {
        level = 1;
    }
    gpio_set_level(BOARD_IO_BULB, level);
}
