/*
 * storage.c
 *
 *  Created on: 13 lut 2021
 *      Author: andrzej
 */

#include <string.h>

#include "nvs.h"
#include "nvs_flash.h"
#include "storage.h"
#include "credentials.h"

#define STO_NAMESPACE   "gniot"

#define STO_KEY_MY_ID                "my_id"
#define STO_KEY_SLEEP                "sleep"

#define DEFAULT_SLEEP_LENGTH        15


static GniotConfig_t s_config;


void config_init(void)
{
    nvs_handle handle;
    uint32_t tmp32;

    strcpy(s_config.server_address, CRED_DEFAULT_SERVER);
    s_config.server_port = CRED_DEFAULT_SERVER_PORT;
    strcpy(s_config.fallback_server_address, CRED_DEFAULT_FALLBACK_SERVER);
    s_config.fallback_server_port = CRED_DEFAULT_FALLBACK_PORT;

    ESP_ERROR_CHECK(nvs_open(STO_NAMESPACE, NVS_READWRITE, &handle));

    if (ESP_OK == nvs_get_u32(handle, STO_KEY_MY_ID, &tmp32))
    {
        s_config.my_id = tmp32;
    }
    else
    {
        s_config.my_id = 0;
    }

    if (ESP_OK == nvs_get_u32(handle, STO_KEY_SLEEP, &tmp32))
    {
        s_config.sleep_length = (uint16_t) tmp32;
    }
    else
    {
        s_config.sleep_length = DEFAULT_SLEEP_LENGTH;
    }

    nvs_close(handle);
}

void storage_init(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
}

const GniotConfig_t * config_get(void)
{
    return &s_config;
}

int config_set_myid(uint32_t my_id)
{
    nvs_handle handle;
    esp_err_t err;

    s_config.my_id = my_id;

    ESP_ERROR_CHECK(nvs_open(STO_NAMESPACE, NVS_READWRITE, &handle));
    err = nvs_set_u32(handle, STO_KEY_MY_ID, my_id);

    nvs_commit(handle);
    nvs_close(handle);

    return (int) err;
}

int config_set_sleep(uint16_t sleep_length)
{
    nvs_handle handle;
    esp_err_t err;

    s_config.sleep_length = sleep_length;

    ESP_ERROR_CHECK(nvs_open(STO_NAMESPACE, NVS_READWRITE, &handle));
    err = nvs_set_u32(handle, STO_KEY_SLEEP, sleep_length);

    nvs_commit(handle);
    nvs_close(handle);

    return (int) err;
}
