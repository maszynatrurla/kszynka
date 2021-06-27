/*
 * connect.h
 *
 *  Created on: 13 lut 2021
 *      Author: andrzej
 */

#ifndef MAIN_WIFI_H_
#define MAIN_WIFI_H_

int wifi_connect(void);
const char * wifi_getIpAddress(void);
int wifi_disconnect(void);

int wifi_scan(int * rssi);

#endif /* MAIN_WIFI_H_ */
