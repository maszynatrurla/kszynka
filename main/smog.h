/*
 * smog.h
 *
 *  Created on: 27 mar 2021
 *      Author: andrzej
 */

#ifndef MAIN_SMOG_H_
#define MAIN_SMOG_H_

typedef struct
{
    uint16_t pm1_0_cf1;
    uint16_t pm2_5_cf1;
    uint16_t pm10_cf1;
    uint16_t pm1_0;
    uint16_t pm2_5;
    uint16_t pm10;
    uint16_t part0_3;
    uint16_t part0_5;
    uint16_t part1_0;
    uint16_t part2_5;
    uint16_t part5_0;
    uint16_t part10;
} SmogData_t;

void smog_init(void);
int smog_measure(void);
void smog_print(void);
void smog_get_data(SmogData_t * sdata);

void smog_test(void);
void smog_read_pm(void);
void smog_set_mode(bool active);
void smog_set_sleep(bool wakeup);


#endif /* MAIN_SMOG_H_ */
