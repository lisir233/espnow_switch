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

#include "espnow_lite.h"
#include "esp32_c2_wireless_switch_board.h"
#include "esp_crc.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_now.h"
#include "esp_random.h"
#include "esp_sleep.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "nvs.h"
#include "nvs_flash.h"

#define STORAGE_NAMESPACE "storage"

#define RESEND_MAX_COUNT 1 //最大重发次数
#define RESEND_MAX_SCAN 2 //多信道发送次数
#define RESEND_MAX_DELAY 5
#define BIND_GAP_TIME 2000
#define SEND_REST_TIME 11
#define SEND_GAP_TIME 30
#define SEND_WIATCALL_DELAY 8

#define ACKNUMBER 5
#define ESPNOW_QUEUE_SIZE 6

#define ISBROADCAST(addr) (((addr)[0] & (addr)[1] & (addr)[2] & (addr)[3] & (addr)[4] & (addr)[5]) == 0xFF)

static espnow_peer_info_t espnow_peer_info;
static uint8_t broadcast_mac[ESP_NOW_ETH_ALEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

static const char* TAG = "espnow_lite";

static QueueHandle_t s_espnow_queue;
static volatile espnow_task_state_t task_state = TASK_STATE_DONE;
static espnow_send_param_t* send_param;
static esp_err_t espnow_lite_read_target_channel_from_nvs(uint8_t* target_channel)
{
    nvs_handle_t my_handle;
    esp_err_t err;
    // Open
    err = nvs_open(STORAGE_NAMESPACE, NVS_READONLY, &my_handle);
    if (err != ESP_OK)
        return ESP_FAIL;
    err = nvs_get_u8(my_handle, "target_channel", target_channel);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND)
        return ESP_FAIL;
    // Close
    nvs_close(my_handle);
    return ESP_OK;
}
static esp_err_t espnow_lite_save_target_channel_to_nvs(uint8_t target_channel)
{
    nvs_handle_t my_handle;
    esp_err_t err;
    // Open
    err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &my_handle);
    if (err != ESP_OK)
        return ESP_FAIL;
    err = nvs_set_u8(my_handle, "target_channel", target_channel);
    if (err != ESP_OK)
        return ESP_FAIL;
    // Close

    nvs_close(my_handle);
    return ESP_OK;
}
static esp_err_t espnow_lite_read_target_mac_from_nvs(espnow_peer_info_t* espnow_peer_info)
{
    nvs_handle_t my_handle;
    esp_err_t err;
    // Open
    err = nvs_open(STORAGE_NAMESPACE, NVS_READONLY, &my_handle);
    if (err != ESP_OK)
        return ESP_FAIL;
    size_t required_size = 0; // value will default to 0, if not set yet in NVS
    err = nvs_get_blob(my_handle, "target_mac", NULL, &required_size);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND)
        return err;
    if (required_size == sizeof(espnow_peer_info_t)) {
        err = nvs_get_blob(my_handle, "target_mac", espnow_peer_info, &required_size);
        if (err != ESP_OK) {
            return err;
        }
    }
    // Close
    nvs_close(my_handle);
    return ESP_OK;
}
static esp_err_t espnow_lite_save_target_mac_to_nvs(espnow_peer_info_t* espnow_peer_info)
{
    nvs_handle_t my_handle;
    esp_err_t err;
    // Open
    err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &my_handle);
    if (err != ESP_OK)
        return err;
    err = nvs_set_blob(my_handle, "target_mac", espnow_peer_info, sizeof(espnow_peer_info_t));
    if (err != ESP_OK)
        return err;
    // Commit
    err = nvs_commit(my_handle);
    if (err != ESP_OK)
        return err;
    nvs_close(my_handle);
    return ESP_OK;
}
static bool is_in_peerlist(const espnow_peer_info_t* espnow_peer_info, const uint8_t* peer_mac)
{
    for (int i = 0; i < espnow_peer_info->peer_num; i++) {
        if (0 == strncmp((char*)espnow_peer_info->peer_mac[i], (char*)peer_mac, ESP_NOW_ETH_ALEN)) {
            return true;
        }
    }
    return false;
}

static esp_err_t add_peer_by_mac(espnow_peer_info_t* espnow_peer_info, const uint8_t* peer_mac)
{
    esp_err_t err = ESP_FAIL;
    esp_now_peer_info_t peer = {
        .channel = 0,
        .ifidx = ESP_IF_WIFI_STA,
        .encrypt = false,
    };
    if (is_in_peerlist(espnow_peer_info, peer_mac) == true) {
        ESP_LOGI(TAG, "espnow_peer_mac: " MACSTR " already in list.", MAC2STR(peer_mac));
        return ESP_FAIL;
    }
    if (espnow_peer_info->peer_num < ESP_NOW_PEER_MAX) {

        memcpy(peer.peer_addr, peer_mac, ESP_NOW_ETH_ALEN);
        ESP_ERROR_CHECK(esp_now_add_peer(&peer));

        memcpy(espnow_peer_info->peer_mac[espnow_peer_info->peer_num++], peer_mac, ESP_NOW_ETH_ALEN);
        err = espnow_lite_save_target_mac_to_nvs(espnow_peer_info);
    } else {
        ESP_LOGI(TAG, "espnow_peer_mac full");
    }
    return err;
}
esp_err_t clear_target_mac(espnow_peer_info_t* espnow_peer_info)
{
    esp_err_t err;
    memset(espnow_peer_info, 0, sizeof(espnow_peer_info_t));
    err = espnow_lite_save_target_mac_to_nvs(espnow_peer_info);
    return err;
}
void peer_info_print(const espnow_peer_info_t* espnow_peer_info)
{
    const uint8_t* mac_addr;
    ESP_LOGI(TAG, "espnow_peer_info: %d", espnow_peer_info->peer_num);
    for (int i = 0; i < espnow_peer_info->peer_num; i++) {
        mac_addr = espnow_peer_info->peer_mac[i];
        ESP_LOGI(TAG, "espnow_peer_mac: " MACSTR " ", MAC2STR(mac_addr));
    }
}
/* Prepare ESPNOW data to be sent. */
static void espnow_data_prepare(espnow_send_param_t* send_param, uint8_t* payload, uint16_t loadlen)
{
    espnow_data_t* buf = (espnow_data_t*)send_param->buffer;
    assert(send_param->len >= sizeof(espnow_data_t));
    buf->casttype = send_param->casttype;
    buf->crc = 0;
    memcpy(buf->payload, payload, loadlen);
    buf->crc = esp_crc16_le(UINT16_MAX, (uint8_t const*)buf, send_param->len);
}

/* Parse received ESPNOW data. */
static esp_err_t espnow_data_parse(const uint8_t* data, uint16_t data_len, espnow_payload_t* res)
{
    if (data_len != ESPNOW_SENDLEN) {
        ESP_LOGE(TAG, "data len error");
        return ESP_FAIL;
    }
    espnow_data_t* buf = (espnow_data_t*)data;
    uint16_t crc, crc_cal = 0;
    crc = buf->crc;
    buf->crc = 0;
    crc_cal = esp_crc16_le(UINT16_MAX, (uint8_t const*)data, data_len);
    if (crc == crc_cal) {
        memcpy(res, buf->payload, data_len - ESPNOW_HEADLEN);
        return ESP_OK;
    }
    return ESP_FAIL;
}

static esp_err_t espnow_send(const uint8_t* dest_mac, uint8_t* payload, uint16_t payloadlen)
{
    memcpy(send_param->dest_mac, dest_mac, ESP_NOW_ETH_ALEN);
    send_param->casttype = ISBROADCAST(send_param->dest_mac);
    send_param->len = ESPNOW_HEADLEN + payloadlen;
    if (send_param->len > ESPNOW_PAYLOADMAX) {
        ESP_LOGE(TAG, "Too large payloadlen");
        return ESP_FAIL;
    }

    send_param->buffer = malloc(send_param->len);
    if (send_param->buffer == NULL) {
        ESP_LOGE(TAG, "Malloc send buffer fail");
        return ESP_FAIL;
    }
    espnow_data_prepare(send_param, payload, payloadlen);
    if (esp_now_send(send_param->dest_mac, send_param->buffer,
            send_param->len)
        != ESP_OK) {
        ESP_LOGE(TAG, "Send error");
    }
    free(send_param->buffer);
    return ESP_OK;
}

static void espnow_recv_cb(const uint8_t* mac_addr, const uint8_t* data, int len)
{
    espnow_event_t espnow_evt;
    espnow_payload_t* payload = &espnow_evt.payload;
    if ((espnow_data_parse(data, len, payload) == ESP_OK)) {
        memcpy(espnow_evt.mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
        if (xQueueSend(s_espnow_queue, &espnow_evt, portMAX_DELAY) != pdTRUE) {
            ESP_LOGW(TAG, "Send receive queue fail");
        }
    }
}

#ifdef CONFIG_ESPNOW_SWITCH
static void espnow_main_task(void* pvParameter)
{
    espnow_event_t* espnow_evt = malloc(sizeof(espnow_event_t));
    espnow_payload_t* recload = &espnow_evt->payload;
    uint8_t* mac_addr = (uint8_t*)&espnow_evt->mac_addr;

    espnow_payload_t* payload = malloc(sizeof(espnow_payload_t));
    int resend_count = 0;
    uint8_t current_channel1;
    wifi_second_chan_t current_channel2 = WIFI_SECOND_CHAN_NONE;
    esp_wifi_get_channel(&current_channel1, &current_channel2);
    esp_sleep_enable_timer_wakeup(30 * 1000);
    task_state = SEND_SATRT;

    for (;;) {
        switch (task_state) {
        case SEND_SATRT:
            payload->magic = (uint16_t)esp_random();
            payload->device_type = KEY1;
            payload->evt = SINGLE_CLICK;
            resend_count = 0;
            if (espnow_peer_info.peer_num > 0) {
                task_state = SEND_INFO;
            } else {
                task_state = BIND_GAP_SLEEP;
            }
            break;
        case SEND_INFO:
            if (resend_count++ < RESEND_MAX_COUNT) {
                board_sleep_set(SEND_GAP_TIME, true);
                espnow_send(send_param->dest_mac, (uint8_t*)payload, sizeof(espnow_payload_t));
                if (xQueueReceive(s_espnow_queue, espnow_evt, SEND_WIATCALL_DELAY) == pdTRUE) {
                    if (is_in_peerlist(&espnow_peer_info, mac_addr) && recload->evt == SWITCH_ACK) {
                        task_state = BIND_GAP_SLEEP;
                        ESP_LOGI(TAG, "SEND_INFO Recive ack succeed");
                    } else {
                        ESP_LOGI(TAG, "Recive ack fail");
                    }
                } else {
                    ESP_LOGI(TAG, "SEND_INFO Recive ack timeout");
                }

            } else {
                task_state = SEND_INFO_SCAN;
                resend_count = 0;
            }
            break;
        case SEND_INFO_SCAN:
            if (resend_count++ < WIFI_MAX_CHANNEL * RESEND_MAX_SCAN) {
                current_channel1 >= WIFI_MAX_CHANNEL ? (current_channel1 = 1) : (current_channel1 += 1);
                esp_wifi_set_channel(current_channel1, current_channel2);
                ESP_LOGI(TAG, "esp_wifi_chanage_channel %d", current_channel1);
                board_sleep_set(SEND_GAP_TIME, true);
                espnow_send(send_param->dest_mac, (uint8_t*)payload, sizeof(espnow_payload_t));
                if (xQueueReceive(s_espnow_queue, espnow_evt, SEND_WIATCALL_DELAY) == pdTRUE) {
                    if (is_in_peerlist(&espnow_peer_info, mac_addr) && recload->evt == SWITCH_ACK) {
                        uint8_t target_channel;
                        wifi_second_chan_t second_channel;
                        esp_wifi_get_channel(&target_channel, &second_channel);
                        espnow_lite_save_target_channel_to_nvs(target_channel);
                        task_state = BIND_GAP_SLEEP;
                        ESP_LOGI(TAG, "SEND_INFO Recive ack succeed,save channel %d", target_channel);

                    } else {
                        ESP_LOGI(TAG, "Recive ack fail");
                    }
                }
            } else {
                resend_count = 0;
                task_state = BIND_GAP_SLEEP;
            }
            break;
        case BIND_GAP_SLEEP:
            board_sleep_set(BIND_GAP_TIME, false);
            ESP_LOGI(TAG, "Wake from BIND_GAP_SLEEP");
            ESP_LOGI(TAG, "Start broadcast ");
            task_state = BIND_START;
            break;
        case BIND_START: {
            payload->magic = (uint16_t)esp_random();
            payload->device_type = OVERALL;
            payload->evt = SWITCH_BIND;
            resend_count = 0;
            task_state = BIND_DEVICE;
            break;
        }
        case BIND_DEVICE:
            if (resend_count++ < WIFI_MAX_CHANNEL * RESEND_MAX_SCAN) {
                espnow_send(send_param->dest_mac, (uint8_t*)payload, sizeof(espnow_payload_t));
                if (xQueueReceive(s_espnow_queue, espnow_evt, SEND_WIATCALL_DELAY) == pdTRUE) {
                    if (recload->evt == SWITCH_ACK) {
                        uint8_t target_channel;
                        wifi_second_chan_t second_channel;
                        esp_wifi_get_channel(&target_channel, &second_channel);
                        espnow_lite_save_target_channel_to_nvs(target_channel);

                        add_peer_by_mac(&espnow_peer_info, mac_addr);
                        task_state = TASK_STATE_DONE;
                        ESP_LOGI(TAG, "BIND Recive succeed");

                    } else {
                        ESP_LOGI(TAG, "Recive ack fail");
                    }
                } else {
                    ESP_LOGI(TAG, "current_channel1 %d", current_channel1);
                    current_channel1 >= WIFI_MAX_CHANNEL ? (current_channel1 = 1) : (current_channel1 += 1);
                    ESP_LOGI(TAG, "chanage channel to %d", current_channel1);
                    esp_wifi_set_channel(current_channel1, current_channel2);
                    board_sleep_set(SEND_GAP_TIME, true);
                }

            } else {
                resend_count = 0;
                task_state = TASK_STATE_DONE;
            }
            break;
        case TASK_STATE_DONE:
            ESP_LOGI(TAG, "TASK_STATE_DONE");
            board_sleep_set(BIND_GAP_TIME, false);
            break;
        default:
            ESP_LOGI(TAG, "task_state %d", task_state);
            vTaskDelay(50);
            break;
        }
        ESP_LOGI(TAG, "task_state =  %d", task_state);
    }
}
#endif
#ifdef CONFIG_ESPNOW_BULB
static void espnow_main_task(void* pvParameter)
{
    espnow_event_t* espnow_evt = malloc(sizeof(espnow_event_t));
    espnow_payload_t* sendload = malloc(sizeof(espnow_payload_t));
    espnow_payload_t* recload = &espnow_evt->payload;
    uint8_t* mac_addr = (uint8_t*)&espnow_evt->mac_addr;
    uint16_t magic_last = 0;
    while (xQueueReceive(s_espnow_queue, espnow_evt, portMAX_DELAY) == pdTRUE) {
        if (recload->magic != magic_last) {
            magic_last = recload->magic;
            switch (recload->evt) {
            case SWITCH_ACK:
                ESP_LOGI(TAG, "recload->evt:SWITCH_ACK,device: %d", recload->device_type);
                break;
            case SINGLE_CLICK:
                ESP_LOGI(TAG, "recload->evt:SINGLE_CLICK,device: %d", recload->device_type);
                sendload->magic = (uint16_t)esp_random();
                sendload->device_type = recload->device_type;
                sendload->evt = SWITCH_ACK;
                for (int i = 0; i < ACKNUMBER; i++) {
                    espnow_send(send_param->dest_mac, sendload, sizeof(espnow_payload_t));
                    vTaskDelay(1 / portTICK_PERIOD_MS);
                }
                board_bulb_toggle();
                break;
            case DOUBLE_CLICK:
                ESP_LOGI(TAG, "recload->evt:DOUBLE_CLICK,device: %d", recload->device_type);
                sendload->magic = (uint16_t)esp_random();
                sendload->device_type = recload->device_type;
                sendload->evt = SWITCH_ACK;
                for (int i = 0; i < ACKNUMBER; i++) {
                    espnow_send(send_param->dest_mac, sendload, sizeof(espnow_payload_t));
                    vTaskDelay(1 / portTICK_PERIOD_MS);
                }
                break;
            case SWITCH_BIND:
                ESP_LOGI(TAG, "recload->evt:SWITCH_BIND,device: %d", recload->device_type);
                sendload->magic = (uint16_t)esp_random();
                sendload->device_type = recload->device_type;
                sendload->evt = SWITCH_ACK;
                for (int i = 0; i < ACKNUMBER; i++) {
                    espnow_send(send_param->dest_mac, sendload, sizeof(espnow_payload_t));
                    vTaskDelay(1 / portTICK_PERIOD_MS);
                }
                add_peer_by_mac(&espnow_peer_info, mac_addr);
                break;
            case SWITCH_UNBIND:
                ESP_LOGI(TAG, "recload->evt:SWITCH_UNBIND,device: %d", recload->device_type);
                sendload->magic = (uint16_t)esp_random();
                sendload->device_type = recload->device_type;
                sendload->evt = SWITCH_ACK;
                for (int i = 0; i < ACKNUMBER; i++) {
                    espnow_send(send_param->dest_mac, sendload, sizeof(espnow_payload_t));
                    vTaskDelay(1 / portTICK_PERIOD_MS);
                }
                break;
            default:
                ESP_LOGI(TAG, "recload->evt:unknow %d,device: %d", recload->evt, recload->device_type);
                break;
            }
        } else {
            ESP_LOGI(TAG, "Receiving duplicate messages.");
            for (int i = 0; i < ACKNUMBER; i++) {
                espnow_send(send_param->dest_mac, sendload, sizeof(espnow_payload_t));
                vTaskDelay(1 / portTICK_PERIOD_MS);
            }
        }
    }
}

#endif
/* WiFi should start before using ESPNOW */
esp_err_t espnow_wifi_init(void)
{
    uint8_t channel;
    wifi_second_chan_t second_channel;
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    board_sleep_set(SEND_GAP_TIME, true);
    ESP_ERROR_CHECK(esp_wifi_start());

#if CONFIG_ESPNOW_ENABLE_LONG_RANGE
    ESP_ERROR_CHECK(esp_wifi_set_protocol(
        ESPNOW_WIFI_IF, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N | WIFI_PROTOCOL_LR));
#endif
#ifdef CONFIG_ESPNOW_SWITCH
    espnow_lite_read_target_channel_from_nvs(&channel);
    ESP_LOGI(TAG, "esp_wifi_set_channel target_channel %d ", channel);
    esp_wifi_get_channel(NULL, &second_channel);
    esp_wifi_set_channel(channel, second_channel);

#endif
#ifdef CONFIG_ESPNOW_BULB
    esp_wifi_get_channel(NULL, &second_channel);
    esp_wifi_set_channel(1, second_channel);
#endif
    return ESP_OK;
}
esp_err_t espnow_init(void)
{
    send_param = malloc(sizeof(espnow_send_param_t));
    memset(send_param, 0, sizeof(espnow_send_param_t));
    if (send_param == NULL) {
        ESP_LOGE(TAG, "Malloc send parameter fail");
        return ESP_FAIL;
    }

    /* Initialize ESPNOW and register sending and receiving callback function. */
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));
#if CONFIG_ESP_WIFI_STA_DISCONNECTED_PM_ENABLE
    ESP_ERROR_CHECK(esp_now_set_wake_window(65535));
#endif

    /* Set primary master key. */
    ESP_ERROR_CHECK(esp_now_set_pmk((uint8_t*)CONFIG_ESPNOW_PMK));
    /* Add broadcast peer information to peer list. */
    esp_now_peer_info_t* peer = malloc(sizeof(esp_now_peer_info_t));
    if (peer == NULL) {
        ESP_LOGE(TAG, "Malloc peer information fail");
        esp_now_deinit();
        return ESP_FAIL;
    }
    memset(peer, 0, sizeof(esp_now_peer_info_t));
    peer->channel = 0;
    peer->ifidx = ESP_IF_WIFI_STA;
    peer->encrypt = false;
    memcpy(peer->peer_addr, broadcast_mac, ESP_NOW_ETH_ALEN);
    ESP_ERROR_CHECK(esp_now_add_peer(peer));

    espnow_lite_read_target_mac_from_nvs(&espnow_peer_info);
    for (int i = 0; i < espnow_peer_info.peer_num; i++) {
        memcpy(peer->peer_addr, espnow_peer_info.peer_mac[i], ESP_NOW_ETH_ALEN);
        ESP_LOGI(TAG, "esp_now_add_peer " MACSTR "", MAC2STR(espnow_peer_info.peer_mac[i]));
        ESP_ERROR_CHECK(esp_now_add_peer(peer));
    }

    free(peer);
    send_param->casttype = ESPNOW_UNICAST;
    memcpy(send_param->dest_mac, broadcast_mac, ESP_NOW_ETH_ALEN);
    s_espnow_queue = xQueueCreate(ESPNOW_QUEUE_SIZE, sizeof(espnow_event_t));
    if (s_espnow_queue == NULL) {
        ESP_LOGE(TAG, "Create mutex fail");
        return ESP_FAIL;
    }
    xTaskCreate(espnow_main_task, "espnow_main_task", 2048, NULL, 15, NULL);
    return ESP_OK;
}