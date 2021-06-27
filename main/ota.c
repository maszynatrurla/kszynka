/*
 * ota.c
 *
 *  Created on: 21 lut 2021
 */

#include "client.h"
#include "storage.h"

#include "esp_system.h"
#include "esp_log.h"
#include "esp_ota_ops.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef enum esp_ota_firm_state {
    ESP_OTA_INIT = 0,
    ESP_OTA_PREPARE,
    ESP_OTA_START,
    ESP_OTA_RECVED,
    ESP_OTA_FINISH,
} esp_ota_firm_state_t;

typedef struct esp_ota_firm {
    uint8_t             ota_num;
    uint8_t             update_ota_num;

    esp_ota_firm_state_t    state;

    size_t              content_len;

    size_t              read_bytes;
    size_t              write_bytes;

    size_t              ota_size;
    size_t              ota_offset;

    const char          *buf;
    size_t              bytes;
} esp_ota_firm_t;

static const char * TAG = "ota";

static esp_ota_firm_t s_ota_firm;
static esp_ota_handle_t s_update_handle;
static char ota_write_buf[1501];
static int s_written = 0;

static int read_until(const char *buffer, char delim, int len)
{
//  /*TODO: delim check,buffer check,further: do an buffer length limited*/
    int i = 0;
    while (buffer[i] != delim && i < len) {
        ++i;
    }
    return i + 1;
}

bool _esp_ota_firm_parse_http(esp_ota_firm_t *ota_firm, const char *text, size_t total_len, size_t *parse_len)
{
    /* i means current position */
    int i = 0, i_read_len = 0;
    char *ptr = NULL, *ptr2 = NULL;
    char length_str[32];

    while (text[i] != 0 && i < total_len) {
        if (ota_firm->content_len == 0 && (ptr = (char *)strstr(text, "Content-Length")) != NULL) {
            ptr += 16;
            ptr2 = (char *)strstr(ptr, "\r\n");
            memset(length_str, 0, sizeof(length_str));
            memcpy(length_str, ptr, ptr2 - ptr);
            ota_firm->content_len = atoi(length_str);
            ota_firm->ota_size = ota_firm->content_len / ota_firm->ota_num;
            ota_firm->ota_offset = ota_firm->ota_size * ota_firm->update_ota_num;
            printf("parse Content-Length:%d, ota_size %d\n", ota_firm->content_len, ota_firm->ota_size);
        }

        i_read_len = read_until(&text[i], '\n', total_len - i);

        if (i_read_len > total_len - i) {
            ESP_LOGE(TAG, "recv malformed http header");
        }

        // if resolve \r\n line, http header is finished
        if (i_read_len == 2) {
            if (ota_firm->content_len == 0) {
                ESP_LOGE(TAG, "did not parse Content-Length item");
            }

            *parse_len = i + 2;

            return true;
        }

        i += i_read_len;
    }

    return false;
}

static size_t esp_ota_firm_do_parse_msg(esp_ota_firm_t *ota_firm, const char *in_buf, size_t in_len)
{
    size_t tmp;
    size_t parsed_bytes = in_len;

    switch (ota_firm->state) {
        case ESP_OTA_INIT:
            if (_esp_ota_firm_parse_http(ota_firm, in_buf, in_len, &tmp)) {
                ota_firm->state = ESP_OTA_PREPARE;
                printf("Http parse %d bytes\n", tmp);
                parsed_bytes = tmp;
            }
            break;
        case ESP_OTA_PREPARE:
            ota_firm->read_bytes += in_len;

            if (ota_firm->read_bytes >= ota_firm->ota_offset) {
                ota_firm->buf = &in_buf[in_len - (ota_firm->read_bytes - ota_firm->ota_offset)];
                ota_firm->bytes = ota_firm->read_bytes - ota_firm->ota_offset;
                ota_firm->write_bytes += ota_firm->read_bytes - ota_firm->ota_offset;
                ota_firm->state = ESP_OTA_START;
                printf("Receive %d bytes and start to update\n", ota_firm->read_bytes);
                //printf("Write %d total %d", ota_firm->bytes, ota_firm->write_bytes);
            }

            break;
        case ESP_OTA_START:
            if (ota_firm->write_bytes + in_len >= ota_firm->ota_size) {
                ota_firm->bytes = ota_firm->ota_size - ota_firm->write_bytes;
                ota_firm->state = ESP_OTA_RECVED;
            } else
                ota_firm->bytes = in_len;

            ota_firm->buf = in_buf;

            ota_firm->write_bytes += ota_firm->bytes;

            //printf("Write %d total %d\n", ota_firm->bytes, ota_firm->write_bytes);

            break;
        case ESP_OTA_RECVED:
            parsed_bytes = 0;
            ota_firm->state = ESP_OTA_FINISH;
            break;
        default:
            parsed_bytes = 0;
            ESP_LOGD(TAG, "State is %d", ota_firm->state);
            break;
    }

    return parsed_bytes;
}

static void esp_ota_firm_parse_msg(esp_ota_firm_t *ota_firm, const char *in_buf, size_t in_len)
{
    size_t parse_bytes = 0;

    ESP_LOGD(TAG, "Input %d bytes", in_len);

    do {
        size_t bytes = esp_ota_firm_do_parse_msg(ota_firm, in_buf + parse_bytes, in_len - parse_bytes);
        ESP_LOGD(TAG, "Parse %d bytes", bytes);
        if (bytes)
            parse_bytes += bytes;
    } while (parse_bytes != in_len);
}

static inline int esp_ota_firm_is_finished(esp_ota_firm_t *ota_firm)
{
    return (ota_firm->state == ESP_OTA_FINISH || ota_firm->state == ESP_OTA_RECVED);
}

static inline int esp_ota_firm_can_write(esp_ota_firm_t *ota_firm)
{
    return (ota_firm->state == ESP_OTA_START || ota_firm->state == ESP_OTA_RECVED);
}

static inline const char* esp_ota_firm_get_write_buf(esp_ota_firm_t *ota_firm)
{
    return ota_firm->buf;
}

static inline size_t esp_ota_firm_get_write_bytes(esp_ota_firm_t *ota_firm)
{
    return ota_firm->bytes;
}

static void esp_ota_firm_init(esp_ota_firm_t *ota_firm, const esp_partition_t *update_partition)
{
    memset(ota_firm, 0, sizeof(esp_ota_firm_t));
    ota_firm->state = ESP_OTA_INIT;
    ota_firm->ota_num = 2;
    ota_firm->update_ota_num = update_partition->subtype - ESP_PARTITION_SUBTYPE_APP_OTA_0;

    ESP_LOGI(TAG, "Totoal OTA number %d update to %d part", ota_firm->ota_num, ota_firm->update_ota_num);

}

static int ota_response_handler(const char * data, int length)
{
    if (length > 0)
    {
        esp_ota_firm_parse_msg(&s_ota_firm, data, length);

        if (esp_ota_firm_can_write(&s_ota_firm))
        {
            esp_err_t err;
            int write_len = esp_ota_firm_get_write_bytes(&s_ota_firm);
            memset(ota_write_buf, 0, sizeof(ota_write_buf));
            memcpy(ota_write_buf, esp_ota_firm_get_write_buf(&s_ota_firm), write_len);

            err = esp_ota_write(s_update_handle, (const void *)ota_write_buf, write_len);
            if (err != ESP_OK)
            {
                ESP_LOGE(TAG, "Error: esp_ota_write failed! err=0x%x", err);
                return 1;
            }
            else
            {
                s_written += write_len;
            }
        }
    }
    else if (length < 0)
    {
        ESP_LOGE(TAG, "Error while receiving OTA %d\n", length);
    }

    if (esp_ota_firm_is_finished(&s_ota_firm))
    {
        return 1;
    }

    return 0;

}

int do_ota_upgrade(const char * endpoint)
{
    esp_err_t err;
    int r;
    Request_t req;
    const char * reqs;
    const esp_partition_t *configured = esp_ota_get_boot_partition();
    const esp_partition_t *partition = esp_ota_get_running_partition();

    if (configured != partition)
    {
        ESP_LOGE(TAG, "Partition configured is different than current!\n"
                "    CFG:%08X RUN:%08X\n", configured->address, partition->address);

        return -1;
    }

    printf("Currently running from partition @ %08X\n", partition->address);

    partition = esp_ota_get_next_update_partition(NULL);
    assert(partition != NULL);
    printf("New firmware goes to partition subtype %d at offset %08X\n",
            partition->subtype, partition->address);

    s_written = 0;

    r = client_open();
    if (r)
    {
        ESP_LOGE(TAG, "Failed to open connection %d\n", r);
        return r;
    }

    request_new(&req, endpoint);
    reqs = request_make(&req);

    r = client_request(reqs, strlen(reqs));

    if (r)
    {
        ESP_LOGE(TAG, "Failed to create request: %d\n", r);
        client_close();
        return r;
    }

    err = esp_ota_begin(partition, OTA_SIZE_UNKNOWN, &s_update_handle);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_ota_begin failed, error=%d", err);
        client_close();
        return -2;
    }

    esp_ota_firm_init(&s_ota_firm, partition);
    client_response_hdl(ota_response_handler);
    client_close();

    printf("Written %d bytes\n", s_written);

    if (esp_ota_end(s_update_handle) != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_ota_end failed!");
    }

    if (esp_ota_firm_is_finished(&s_ota_firm))
    {
        err = esp_ota_set_boot_partition(partition);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "esp_ota_set_boot_partition failed! err=0x%x", err);
        }
        else
        {
            ESP_LOGI(TAG, "Prepare to restart system!");
            esp_restart();
        }

    }

    return (int) err;
}
