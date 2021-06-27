/**
 * Humidity and temperature reading interface.
 *
 * humtemp.h
 *
 *  Created on: 8 lut 2021
 *      Author: andrzej
 */

#ifndef MAIN_HUMTEMP_H_
#define MAIN_HUMTEMP_H_

#include <stdint.h>

#define HT_SUCCESS      0   /**< Successful reading. */
#define HT_E_TIMEOUT    1   /**< Error - timeout when waiting for data. */
#define HT_E_CHECKSUM   2   /**< Error - checksum error, data corrupted. */
#define HT_E_UNKNOWN    3   /**< Error - something else. */

typedef uint16_t Humidity_t;      /**< Humidity reading type. */
typedef int16_t  Temperature_t;   /**< Temperature reading type. */

/**
 * Initialize module.
 * WARNING: sleeps for 2 seconds.
 */
void humtemp_init(void);
/**
 * Perform reading.
 * @param humidity pointer to where store humidity reading
 * @param temperature pointer to where store temperature reading
 * @returns result code (0 on success)
 */
int humtemp_read(Humidity_t * humidity, Temperature_t * temperature);

void humtemp_test(void);


#endif /* MAIN_HUMTEMP_H_ */
