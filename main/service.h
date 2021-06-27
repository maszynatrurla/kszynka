/*
 * High level code controlling passing of data between gniot and server.
 * service.h
 *
 *  Created on: 14 lut 2021
 *      Author: andrzej
 */

#ifndef MAIN_SERVICE_H_
#define MAIN_SERVICE_H_

#include <stdint.h>

#include "smog.h"

#define NO_MEASUREMENT  0xFFFFFFFFUL

typedef struct {
    bool bosz_ok;
    float bosz_temperature;
    float pressure;

    bool dte_ok;
    float dte_temperature;
    float humidity;

    int32_t solar_mv;

    bool smog_ok;
    SmogData_t smog;
} StationData_t;

void service_perform(int connection_status, const StationData_t * data);

#endif /* MAIN_SERVICE_H_ */
