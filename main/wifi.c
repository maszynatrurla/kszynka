/*
 * connect.c
 *
 *  Created on: 13 lut 2021
 *      Author: andrzej
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_wifi.h"

#include "lwip/err.h"
#include "lwip/sys.h"

/* Must define wifi credential strings:
 *  CRED_MY_SSID and CRED_MY_PWD */
#include "credentials.h"

#define MAXIMUM_RETRY   8

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static const char *TAG = "WIFI";

static EventGroupHandle_t s_connect_event_group;
static int s_retry_num = 0;
static uint32_t s_myip = 0;

static void wifi_handler(void *arg, esp_event_base_t event_base,
        int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        if (s_retry_num < MAXIMUM_RETRY)
        {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        }
        else
        {
            xEventGroupSetBits(s_connect_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGW(TAG,"connect to the AP fail");
    }
}

static void got_ip_handler(void *arg, esp_event_base_t event_base,
        int32_t event_id, void *event_data)
{
    if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip: "IPSTR, IP2STR(&event->ip_info.ip));
        s_myip = event->ip_info.ip.addr;
        s_retry_num = 0;
        xEventGroupSetBits(s_connect_event_group, WIFI_CONNECTED_BIT);
    }
}


int wifi_connect(void)
{
    int res = 0;
    EventBits_t bits;
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = CRED_MY_SSID,
            .password = CRED_MY_PWD,
        },
    };

    if (s_connect_event_group != NULL)
    {
        // already initialized
        return -1;
    }

    s_connect_event_group = xEventGroupCreate();
    tcpip_adapter_init();
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));


    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &got_ip_handler, NULL));

    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start());

    bits = xEventGroupWaitBits(s_connect_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            10000 / portTICK_RATE_MS);


    if (bits & WIFI_CONNECTED_BIT)
    {
        ESP_LOGI(TAG, "connected to wifi");
    }
    else if (bits & WIFI_FAIL_BIT)
    {
        ESP_LOGE(TAG, "Failed to connect to wifi");
        res = -4;
    }
    else
    {
        ESP_LOGE(TAG, "TIMEOUT?");
        res = -3;
    }

    return res;
}

const char * wifi_getIpAddress(void)
{
    return ip4addr_ntoa((ip4_addr_t *) &s_myip);
}

int wifi_disconnect(void)
{
    // not connected yet
    if (s_connect_event_group == NULL) {
        return -1;
    }

    ESP_ERROR_CHECK(esp_event_handler_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_handler));
    ESP_ERROR_CHECK(esp_event_handler_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, &got_ip_handler));

    vEventGroupDelete(s_connect_event_group);
    s_connect_event_group = NULL;

    esp_wifi_stop();
    ESP_ERROR_CHECK(esp_wifi_deinit());

    ESP_LOGI(TAG, "Disconnected from wifi");
    return 0;

}

int wifi_scan(int * rssi)
{
    int32_t ret;
    wifi_scan_config_t scan_config = {0};

    scan_config.ssid = (uint8_t *) CRED_MY_SSID;
    scan_config.show_hidden = true;

    ret = esp_wifi_scan_start(&scan_config, true);

    if (ESP_OK == ret)
    {
        uint16_t found_count;
        wifi_ap_record_t * records;


        esp_wifi_scan_get_ap_num(&found_count);
        printf("Scan found %d stations\n", found_count);

        records = malloc(found_count * sizeof(*records));

        if (NULL == records)
        {
            ESP_LOGE(TAG, "Ups! nie mam pamieci\n");
            found_count = 0;
            esp_wifi_scan_get_ap_records(&found_count, records); // free wifi memory?
        }
        else
        {
            esp_wifi_scan_get_ap_records(&found_count, records);

            for (int i = 0; i < found_count; ++i)
            {
                printf("%d.\n", i);
                printf("    BSSID : %02X:%02X:%02X:%02X:%02X:%02X\n", (unsigned) records[i].bssid[0],
                        (unsigned) records[i].bssid[1], (unsigned) records[i].bssid[2],
                        (unsigned) records[i].bssid[3], (unsigned) records[i].bssid[4],
                        (unsigned) records[i].bssid[5]);
                printf("    SSID : %s\n", records[i].ssid);
                printf("    RSSI : %d dB\n", (int) records[i].rssi);
                *rssi = (int) records[i].rssi;
            }

            return 0;
        }
    }
    else
    {
        ESP_LOGE(TAG, "scan returned %d\n", ret);
    }

    return -1;
}
