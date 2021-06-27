/*
 * HTTP client implementation.
 * client.c
 *
 *  Created on: 13 lut 2021
 *      Author: andrzej
 */
#include <string.h>

#include "esp_log.h"

#include "client.h"
#include "storage.h"
#include "credentials.h"

#include <netdb.h>
#include <sys/socket.h>

// Label for log
static const char * TAG = "client";

/**
 * Our representation for server address.
 */
typedef struct {
    const char * address;
    char port [6];
} ServerAddress_t;

/**
 * Tokenizer temporary buffer.
 */
typedef struct {
    char buf [32];
    int idx;
} TokBuf;

/**
 * Addresses to try ordered by priority.
 */
static ServerAddress_t s_servers [2];
/**
 * Index of currently used server.
 */
static int s_server_index = 0;

/**
 * Socket handle.
 */
static int s_socket = -1;

static char big_rcv_buf [1024];

static char big_snd_buf [512];

/**
 * (Re)init tokenizer buffer.
 */
static void tok_zero(TokBuf * tb)
{
    tb->idx = 0;
}

/**
 * Add character to token, check if token ready.
 */
static int tok_append(TokBuf * tb, char c)
{
    /* non-blank printable character check
     *   these can be a part of token */
    if ((c > 33) && (c < 127))
    {
        /* check for space in tmp buffer, reserve one char for null-termination */
        if (tb->idx < sizeof(tb->buf) - 1)
        {
            tb->buf[tb->idx] = c;
            ++(tb->idx);
        }
        else
        {
            /* remember that buffer if overflown */
            tb->idx = sizeof(tb->buf);
        }
    }
    else if (0 == tb->idx)
    {
        // token have not started yet. ignore
    }
    else if (tb->idx < sizeof(tb->buf))
    {
        // token ended,
        // auto-reset itself
        tb->buf[tb->idx] = 0;
        tb->idx = 0;
        return 1;
    }
    else
    {
        // previous token, was too big to fit into
        // our buffer, ignore it and try to capture another one
        tb->idx = 0;
    }

    return 0;
}

/**
 * Read addresses from configuration.
 */
static void init_addresses(void)
{
    const GniotConfig_t * config = config_get();

    s_servers[0].address = config->server_address;
    sprintf(s_servers[0].port, "%d", (int) config->server_port);

    s_servers[1].address = config->fallback_server_address;
    sprintf(s_servers[1].port, "%d", (int) config->fallback_server_port);
}

int client_open(void)
{
    const struct addrinfo hints = {
            .ai_family = AF_INET,
            .ai_socktype = SOCK_STREAM,
    };

    int srv_idx;
    int result = -2;

    if (s_socket >= 0)
    {
        return -1;
    }

    // get server address (it might've changed since last call)
    init_addresses();

    for (srv_idx = 0; srv_idx < 2; ++srv_idx)
    {
        struct addrinfo *res;
        const ServerAddress_t * web_server = &s_servers[srv_idx];

        /* DNS lookup is not really needed since we use local (numerical) addresses
         * only. But it might come in handy in the future. */
        int err = getaddrinfo(web_server->address, web_server->port, &hints, &res);

        if(err != 0 || res == NULL)
        {
            ESP_LOGE(TAG, "DNS lookup of %s failed err=%d res=%p", web_server->address, err, res);
            continue;
        }

        s_socket = socket(res->ai_family, res->ai_socktype, 0);
        if (s_socket < 0)
        {
            ESP_LOGE(TAG, "... Failed to allocate socket.");
            freeaddrinfo(res);
            continue;
        }

        if (connect(s_socket, res->ai_addr, res->ai_addrlen) != 0)
        {
            ESP_LOGE(TAG, "... socket connect failed errno=%d", errno);
            close(s_socket);
            s_socket = -1;
            freeaddrinfo(res);
            continue;
        }

        ESP_LOGI(TAG, "... connected");
        freeaddrinfo(res);
        result = 0;
        s_server_index = srv_idx;
        break;
    }

    return result;
}

int client_close(void)
{
    if (s_socket < 0)
    {
        return -1;
    }

    close(s_socket);
    s_socket = -1;
    ESP_LOGI(TAG, "closed connection\n");
    return 0;
}

int client_request(const char * request, int length)
{
    struct timeval receiving_timeout;

    if (s_socket < 0)
    {
        return -1;
    }

    if (write(s_socket, request, length) < 0)
    {
        ESP_LOGE(TAG, "... socket send failed");
        close(s_socket);
        s_socket = -1;
        return -2;
    }

    receiving_timeout.tv_sec = 5;
    receiving_timeout.tv_usec = 0;

    if (setsockopt(s_socket, SOL_SOCKET, SO_RCVTIMEO, &receiving_timeout,
            sizeof(receiving_timeout)) < 0)
    {
        ESP_LOGE(TAG, "... failed to set socket receiving timeout");
        close(s_socket);
        return -3;
    }

    return 0;
}

int client_response(void)
{
    int r;
    char recv_buf[64];

    /* Read HTTP response */
    do {
        bzero(recv_buf, sizeof(recv_buf));
        r = read(s_socket, recv_buf, sizeof(recv_buf)-1);
        for(int i = 0; i < r; i++) {
            putchar(recv_buf[i]);
        }
    } while (r > 0);

    return r == 0;
}



int client_response_hdl(response_buf_handler handler)
{
    int r;

    /* Read HTTP response */
    do {
        bzero(big_rcv_buf, sizeof(big_rcv_buf));
        r = read(s_socket, big_rcv_buf, sizeof(big_rcv_buf)-1);
        if (0 != handler(big_rcv_buf, r))
        {
            r = -1;
        }
    }
    while (r > 0);

    return r == 0;
}

int client_response_iterate(reponse_handler handler)
{
    int r;
    char recv_buf[96];
    TokBuf tok_key;
    TokBuf tok_val;
    TokBuf * ptok;
    int status = -2;
    enum {
        ERI_BEGIN,
        ERI_HTTP_CODE,
        ERI_HEAD,
        ERI_KEY,
        ERI_VALUE,
    } state = ERI_BEGIN;

    tok_zero(&tok_key);
    tok_zero(&tok_val);
    ptok = &tok_key;

    do {
        bzero(recv_buf, sizeof(recv_buf));
        r = read(s_socket, recv_buf, sizeof(recv_buf)-1);
        if (r < 0)
        {
            return -1;
        }
        else
        {
            for (int i = 0; i < r; ++i)
            {
                if (tok_append(ptok, recv_buf[i]))
                {
                    if (ERI_BEGIN == state)
                    {
                        /* looking for HTTP header */
                        if (0 == strncmp("HTTP/", tok_key.buf, 5))
                        {
                            state = ERI_HTTP_CODE;
                        }
                    }
                    else if (ERI_HTTP_CODE == state)
                    {
                        /* checking response code from server */
                        /* we want 200 - OK */
                        if (0 == strcmp("200", tok_key.buf))
                        {
                            status = 0;
                            state = ERI_HEAD;
                        }
                        else
                        {
                            ESP_LOGE(TAG, "HTTP error status %s\n", tok_key.buf);
                            return -3;
                        }
                    }
                    else if (ERI_HEAD == state)
                    {
                        /* looking for magic word, which starts data */
                        if (0 == strcmp(">gnIOT<", tok_key.buf))
                        {
                            state = ERI_KEY;
                        }
                    }
                    else if (ERI_KEY == state)
                    {
                        /* key copied, now find value */
                        ptok = &tok_val;
                        state = ERI_VALUE;
                    }
                    else if (ERI_VALUE == state)
                    {
                        /* pass key and value to handler callback
                         * start reading another key */
                        handler(tok_key.buf, tok_val.buf);
                        ptok = &tok_key;
                        state = ERI_KEY;
                    }
                }
            }
        }
    } while (r > 0);

    return status;
}


int request_new(Request_t * request, const char * endpoint)
{
    const GniotConfig_t * cfg = config_get();
    int printed = snprintf(big_snd_buf, sizeof(big_snd_buf) - 1, "GET %s?id=%u", endpoint, cfg->my_id);
    request->ptr = &big_snd_buf[printed];

    return 0;
}

int request_sets(Request_t * request, const char * key, const char * value)
{
    char * out = (char *) request->ptr;
    int capacity = sizeof(big_snd_buf) - (((unsigned) out) - ((unsigned) big_snd_buf));

    if ((out > big_snd_buf) && (out < &big_snd_buf[sizeof(big_snd_buf) - 1]))
    {
        int printed = snprintf(out, capacity - 1, "&%s=%s", key, value);
        request->ptr = &out[printed];
        return 0;
    }
    return -1;
}

int request_seti(Request_t * request, const char * key, int32_t value)
{
    char * out = (char *) request->ptr;
    int capacity = sizeof(big_snd_buf) - (((unsigned) out) - ((unsigned) big_snd_buf));

    if ((out > big_snd_buf) && (out < &big_snd_buf[sizeof(big_snd_buf) - 1]))
    {
        int printed = snprintf(out, capacity - 1, "&%s=%d", key, value);
        request->ptr = &out[printed];
        return 0;
    }
    return -1;
}

int request_setu(Request_t * request, const char * key, uint32_t value)
{
    char * out = (char *) request->ptr;
    int capacity = sizeof(big_snd_buf) - (((unsigned) out) - ((unsigned) big_snd_buf));

    if ((out > big_snd_buf) && (out < &big_snd_buf[sizeof(big_snd_buf) - 1]))
    {
        int printed = snprintf(out, capacity - 1, "&%s=%u", key, value);
        request->ptr = &out[printed];
        return 0;
    }
    return -1;
}

int request_setf(Request_t * request, const char * key, float value)
{
    char * out = (char *) request->ptr;
    int capacity = sizeof(big_snd_buf) - (((unsigned) out) - ((unsigned) big_snd_buf));

    if ((out > big_snd_buf) && (out < &big_snd_buf[sizeof(big_snd_buf) - 1]))
    {
        int printed = snprintf(out, capacity - 1, "&%s=%f", key, value);
        request->ptr = &out[printed];
        return 0;
    }
    return -1;
}

const char * request_make(Request_t * request)
{
    if (s_server_index < 2)
    {
        char * out = (char *) request->ptr;
        int capacity = sizeof(big_snd_buf) - (((unsigned) out) - ((unsigned) big_snd_buf));

        if ((out > big_snd_buf) && (out < &big_snd_buf[sizeof(big_snd_buf) - 1]))
        {
            snprintf(out, capacity - 1, " HTTP/1.0\r\n"
                    "Host: %s:%s\r\n"
                    "User-Agent: esp-idf/1.0 esp32\r\n"
                    "\r\n", s_servers[s_server_index].address, s_servers[s_server_index].port);
            return big_snd_buf;
        }
    }
    return NULL;
}

