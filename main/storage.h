/*
 * storage.h
 *
 *  Created on: 13 lut 2021
 *      Author: andrzej
 */

#ifndef MAIN_STORAGE_H_
#define MAIN_STORAGE_H_

#include <stdint.h>
#include <stdbool.h>


typedef struct
{
    int my_id;

    char server_address [32];
    uint16_t server_port;

    char fallback_server_address [32];
    uint16_t fallback_server_port;

    uint16_t sleep_length;
} GniotConfig_t;


void storage_init(void);
void config_init(void);
const GniotConfig_t * config_get(void);

int config_set_myid(uint32_t my_id);
int config_set_sleep(uint16_t sleep_length);


#endif /* MAIN_STORAGE_H_ */
