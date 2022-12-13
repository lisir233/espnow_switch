// Copyright 2020-2022 Espressif Systems (Shanghai) PTE LTD
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

#ifndef ESP32_C2_WIRELESS_SWITCH_BOARD_H
#define ESP32_C2_WIRELESS_SWITCH_BOARD_H

#include "esp_err.h"

#define BOARD_IO_PWRLOCK (0)
#define BOARD_IO_LED (1)
#define BOARD_IO_BULB (8)
#define BOARD_IO_PIN_SEL_OUTPUT ((1ULL << BOARD_IO_PWRLOCK) \
    | (1ULL << BOARD_IO_LED) | (1ULL << BOARD_IO_BULB))

/**
 * @brief Board level init.
 *        Init Peripherals
 *
 * @return esp_err_t
 */
esp_err_t board_init(void);

/**
 * @brief Configure the sleep time and whether the sleep power is turned on.
 *
 * @return None
 */
void board_sleep_set(uint32_t ms, uint8_t power_is_en);


void board_bulb_toggle(void);
#endif
