/*
 * bosz.h
 *
 *  Created on: 7 kwi 2021
 *      Author: andrzej
 */

#ifndef _BOSH_H_
#define _BOSH_H_

#include <stdint.h>

void bosz_test(void);

int bosz_init(void);
int bosz_read(int32_t * temperature, uint32_t * pressure, uint32_t * humidity);

#endif // _BOSH_H_
