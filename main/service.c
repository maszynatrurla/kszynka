/**
 * High level code controlling passing of data between gniot and server.
 * service.c
 *
 *  Created on: 14 lut 2021
 *      Author: andrzej
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#include "service.h"
#include "client.h"
#include "storage.h"
#include "ota.h"

enum {
    S_CMD_DUMP_CFG,
    S_CMD_OTA,
};

#define CMD_SET(C)      (s_cmd.flags |= (1 << (C)))
#define IS_ANY_CMD_SET  (s_cmd.flags != 0)
#define IS_CMD_SET(C)   ((s_cmd.flags & (1 << (C))) != 0)
#define CMD_CLEAR(C)    (s_cmd.flag &= ~(1UL << (C)))
#define CMD_CLEAR_ALL() (s_cmd.flags = 0)

typedef struct {
    uint32_t flags;
} Cmd_t;

static Cmd_t s_cmd = {0};

void command_handler(const char * key, const char * val)
{
    if (0 == strcmp("timestamp", key))
    {
        printf("Timestamp: %s\n", val);
    }
    else if (0 == strcmp("your_id", key))
    {
        int iv = atoi(val);
        printf("%s -> %s\n", key, val);
        config_set_myid((uint32_t) iv);
    }
    else if (0 == strcmp("sleep_length", key))
    {
        int iv = atoi(val);
        printf("%s -> %s\n", key, val);
        config_set_sleep((uint16_t) iv);
    }
    else if (0 == strcmp("do_ota", key))
    {
        printf("do_ota requested\n");
        CMD_SET(S_CMD_OTA);
    }
}

void service_perform(int connection_status, const StationData_t * data)
{
    if (connection_status)
    {

    }
    else
    {
        int r;
        Request_t request;
        const char * rs;

        CMD_CLEAR_ALL();

        r = client_open();
        if (!r)
        {
            request_new(&request, "/kszynka");

            if (data->bosz_ok)
            {
                request_setf(&request, "bosz_T", data->temperature);
                request_setf(&request, "pressure", data->pressure);
                request_setf(&request, "humidity", data->humidity);
            }

            if (data->smog_ok)
            {
                request_setu(&request, "pm1_0_cf1", data->smog.pm1_0_cf1);
                request_setu(&request, "pm2_5_cf1", data->smog.pm2_5_cf1);
                request_setu(&request, "pm10_cf1",  data->smog.pm10_cf1 );
                request_setu(&request, "pm1_0",     data->smog.pm1_0    );
                request_setu(&request, "pm2_5",     data->smog.pm2_5    );
                request_setu(&request, "pm10",      data->smog.pm10     );
                request_setu(&request, "part0_3",   data->smog.part0_3  );
                request_setu(&request, "part0_5",   data->smog.part0_5  );
                request_setu(&request, "part1_0",   data->smog.part1_0  );
                request_setu(&request, "part2_5",   data->smog.part2_5  );
                request_setu(&request, "part5_0",   data->smog.part5_0  );
                request_setu(&request, "part10",    data->smog.part10   );
            }

            request_setu(&request, "lux", data->solar_mv);

            rs = request_make(&request);

            r = client_request(rs, strlen(rs));
            if (!r)
            {

                (void) client_response_iterate(command_handler);

            }
            client_close();
        }

        if (IS_CMD_SET(S_CMD_OTA))
        {
            printf("Perform OTA\n");
//            do_ota_upgrade("/ota");
            return;
        }

        CMD_CLEAR_ALL();
    }
}

