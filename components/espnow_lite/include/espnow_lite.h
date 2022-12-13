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

#ifndef ESPNOW_LITE_H
#define ESPNOW_LITE_H

#include "esp_err.h"

#include "esp_now.h"
#include <assert.h>
#include <string.h>
#define WIFI_MAX_CHANNEL 14

#define ESPNOW_PAYLOADMAX 250
#define ESPNOW_PAYLOADLEN 4

#define ESPNOW_HEADLEN 3
#define ESPNOW_SENDLEN (ESPNOW_PAYLOADLEN + ESPNOW_HEADLEN)

#define ESPNOW_BROADCAST 1
#define ESPNOW_UNICAST 0

#define ESP_NOW_PEER_MAX 6 //最大允许添加的PEER数量

/* User defined field of ESPNOW data */
typedef struct {
    uint8_t casttype; // Broadcast or unicast ESPNOW data.
    uint16_t crc; // CRC16 value of ESPNOW data.
    uint8_t payload[0]; // Real payload of ESPNOW data.

} __attribute__((packed)) espnow_data_t;

/* Parameters of sending ESPNOW data. */
typedef struct {
    uint8_t casttype; // ESPNOW cast data type.
    int len; // Length of ESPNOW data to be sent, unit: byte.
    uint8_t* buffer; // Buffer pointing to ESPNOW data.
    uint8_t dest_mac[ESP_NOW_ETH_ALEN]; // MAC address of destination device.
} espnow_send_param_t;

/* Parameters of saved peer_mac */
typedef struct {
    uint8_t peer_num;
    uint8_t peer_mac[ESP_NOW_PEER_MAX][ESP_NOW_ETH_ALEN];
} espnow_peer_info_t;
typedef enum {
    SEND_SATRT = 0,
    WAIT_BIND_DEVICE,
    BIND_START,
    BIND_DEVICE,
    WAIT_INFO,
    SEND_INFO,
    SEND_INFO_SCAN,
    BIND_GAP_SLEEP,
    TASK_STATE_DONE,
} espnow_task_state_t;

typedef enum {
    OVERALL,
    KEY1,
    KEY2,
    KEY3,
} device_type_t;

typedef enum {
    SWITCH_ACK,
    SINGLE_CLICK,
    DOUBLE_CLICK,
    LONG_PRESS,
    SWITCH_BIND,
    SWITCH_UNBIND,
} switch_evt_t;

typedef struct {
    uint16_t magic;
    uint8_t device_type;
    uint8_t evt;
} espnow_payload_t;

typedef struct {
    espnow_payload_t payload;
    uint8_t mac_addr[ESP_NOW_ETH_ALEN];
} espnow_event_t;
esp_err_t espnow_wifi_init(void);
esp_err_t espnow_init(void);
#endif