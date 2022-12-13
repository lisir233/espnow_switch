idf_component_register(SRCS "espnow_lite.c" 
        INCLUDE_DIRS include
        PRIV_REQUIRES driver esp_wifi esp_netif nvs_flash esp32_c2_wireless_board)