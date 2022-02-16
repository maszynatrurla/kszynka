/*
 * bosz.c
 *
 *  Created on: 7 kwi 2021
 *      Author: andrzej
 */

#include "bosz.h"

#include "driver/spi_master.h"
#include "driver/gpio.h"

#include <stdint.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define BOSZ_SPI    SPI3_HOST

#define BOSZ_READ   1
#define BOSZ_WRITE  0

#define BOSZ_A_CALIB        0x88
#define BOSZ_A_ID           0xD0
#define BOSZ_A_RESET        0xE0
#define BOSZ_A_MORE_CALIB   0xE1
#define BOSZ_A_CTRL_HUM     0xF2
#define BOSZ_A_STATUS       0xF3
#define BOSZ_A_CTRL_MEAS    0xF4
#define BOSZ_A_CONFIG       0xF5
#define BOSZ_A_PRESS_MSB    0xF7
#define BOSZ_A_PRESS_LSB    0xF8
#define BOSZ_A_PRESS_XLSB   0xF9
#define BOSZ_A_TEMP_MSB     0xFA
#define BOSZ_A_TEMP_LSB     0xFB
#define BOSZ_A_TEMP_XLSB    0xFC
#define BOSZ_A_HUM_MSB      0xFD
#define BOSZ_A_HUM_LSB      0xFE

#define BOSZ_CHIP_ID        0x60

typedef union {
    struct {
        uint8_t mode : 2;
        uint8_t osrs_p : 3;
        uint8_t osrs_t : 3;
    };
    uint8_t value;
} CtrlMeas_t;

typedef struct __attribute__((packed))
{
    uint8_t press_msb;
    uint8_t press_lsb;
    uint8_t press_xlsb;
    uint8_t temp_msb;
    uint8_t temp_lsb;
    uint8_t temp_xlsb;
    uint8_t hum_msb;
    uint8_t hum_lsb;
} BoszMeas_t;

#define BOSZ_CALIB_PART_1 \
    uint16_t dig_T1;    \
    int16_t  dig_T2;    \
    int16_t  dig_T3;    \
    uint16_t dig_P1;    \
    int16_t  dig_P2;    \
    int16_t  dig_P3;    \
    int16_t  dig_P4;    \
    int16_t  dig_P5;    \
    int16_t  dig_P6;    \
    int16_t  dig_P7;    \
    int16_t  dig_P8;    \
    int16_t  dig_P9;    \
    uint8_t  reserved;  \
    uint8_t  dig_H1;    \


typedef struct __attribute__((packed)) BoszCalib_tTag
{
    BOSZ_CALIB_PART_1
} BoszCalib_t;

typedef struct __attribute__((packed)) BoszMoreCalib_tTag
{
    int16_t  dig_H2;
    uint8_t  dig_H3;
    uint8_t  bosh_are_you_freaking_high_E4;
    uint8_t  bosh_are_you_freaking_high_E5;
    uint8_t  bosh_are_you_freaking_high_E6;
    int8_t   dig_H6;
} BoszMoreCalib_t;


static struct __attribute__((packed))
{
    BOSZ_CALIB_PART_1
    int16_t  dig_H2;
    uint8_t  dig_H3;
    int16_t  dig_H4;
    int16_t  dig_H5;
    int8_t   dig_H6;
} s_calib;


static spi_device_handle_t s_spi;

static int32_t s_t_fine;

int32_t bme280_compensate_T_int32(int32_t adc_T)
{
    int32_t var1, var2, T;
    var1 = ((((adc_T>>3) - ((int32_t)s_calib.dig_T1<<1))) * ((int32_t)s_calib.dig_T2)) >> 11;
    var2 = (((((adc_T>>4) - ((int32_t)s_calib.dig_T1)) * ((adc_T>>4) - ((int32_t)s_calib.dig_T1))) >> 12)
            * ((int32_t)s_calib.dig_T3)) >> 14;
    s_t_fine = var1 + var2;
    T = (s_t_fine * 5 + 128) >> 8;
    return T;
}

uint32_t bme280_compensate_P_int64(int32_t adc_P)
{
    int64_t var1, var2, p;
    var1 = ((int64_t)s_t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)s_calib.dig_P6;
    var2 = var2 + ((var1*(int64_t)s_calib.dig_P5)<<17);
    var2 = var2 + (((int64_t)s_calib.dig_P4)<<35);
    var1 = ((var1 * var1 * (int64_t)s_calib.dig_P3)>>8) + ((var1 * (int64_t)s_calib.dig_P2)<<12);
    var1 = (((((int64_t)1)<<47)+var1))*((int64_t)s_calib.dig_P1)>>33;
    if (var1 == 0)
    {
        return 0; // avoid exception caused by division by zero
    }
    p = 1048576-adc_P;
    p = (((p<<31)-var2)*3125)/var1;
    var1 = (((int64_t)s_calib.dig_P9) * (p>>13) * (p>>13)) >> 25;
    var2 = (((int64_t)s_calib.dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)s_calib.dig_P7)<<4);
    return (uint32_t)p;
}


uint32_t bme280_compensate_H_int32(int32_t adc_H)
{
    int32_t v_x1_u32r;
    v_x1_u32r = (s_t_fine - ((int32_t)76800));
    v_x1_u32r = (((((adc_H << 14) - (((int32_t)s_calib.dig_H4) << 20) - (((int32_t)s_calib.dig_H5) *
            v_x1_u32r)) + ((int32_t)16384)) >> 15) * (((((((v_x1_u32r *
                    ((int32_t)s_calib.dig_H6)) >> 10) * (((v_x1_u32r * ((int32_t)s_calib.dig_H3)) >> 11) +
                            ((int32_t)32768))) >> 10) + ((int32_t)2097152)) * ((int32_t)s_calib.dig_H2) +
                    8192) >> 14));
    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) *
            ((int32_t)s_calib.dig_H1)) >> 4));
    v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
    v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
    return (uint32_t)(v_x1_u32r>>12);
}

int bosz_init(void)
{
    esp_err_t ret;
    spi_bus_config_t buscfg = {
            .miso_io_num = 19,
            .mosi_io_num = 23,
            .sclk_io_num = 18,
            .quadwp_io_num = -1,
            .quadhd_io_num = -1,
            .max_transfer_sz = 128,
    };
    spi_device_interface_config_t devcfg = {
            .command_bits = 1,
            .address_bits = 7,
            .dummy_bits = 0,
            .mode = 0,
            .clock_speed_hz = 100*1000,               //Clock out at 100 kHz
            .spics_io_num = 22,
            .queue_size = 2,
    };
    spi_transaction_t t;

    //Initialize the SPI bus
    ret=spi_bus_initialize(BOSZ_SPI, &buscfg, 0);
    ESP_ERROR_CHECK(ret);
    //Attach the LCD to the SPI bus
    ret=spi_bus_add_device(BOSZ_SPI, &devcfg, &s_spi);
    ESP_ERROR_CHECK(ret);

    memset(&t, 0, sizeof(t));
    t.cmd = BOSZ_READ;
    t.addr = BOSZ_A_ID;
    t.length = 8;
    t.flags = SPI_TRANS_USE_RXDATA;
    t.user = (void *) 0;

    ret = spi_device_polling_transmit(s_spi, &t);

    if (ESP_OK == ret)
    {
        if (BOSZ_CHIP_ID == t.rx_data[0])
        {
            /* read calibration variables */
            memset(&t, 0, sizeof(t));
            t.cmd = BOSZ_READ;
            t.addr = BOSZ_A_CALIB;
            t.length = sizeof(BoszCalib_t) * 8;
            t.user = (void *) 0;
            t.rx_buffer = &s_calib;

            ret = spi_device_polling_transmit(s_spi, &t);

            if (ESP_OK == ret)
            {
                /* read more calibration variables */
                BoszMoreCalib_t moreCalib;

                memset(&t, 0, sizeof(t));
                t.cmd = BOSZ_READ;
                t.addr = BOSZ_A_MORE_CALIB;
                t.length = sizeof(BoszMoreCalib_t) * 8;
                t.user = (void *) 0;
                t.rx_buffer = &moreCalib;

                ret = spi_device_polling_transmit(s_spi, &t);

                if (ESP_OK == ret)
                {
                    s_calib.dig_H2 = moreCalib.dig_H2;
                    s_calib.dig_H3 = moreCalib.dig_H3;
                    s_calib.dig_H4 = (int16_t) ((((uint16_t) moreCalib.bosh_are_you_freaking_high_E4) << 4)
                            | (moreCalib.bosh_are_you_freaking_high_E5 & 0xF));
                    s_calib.dig_H5 = (int16_t) ((moreCalib.bosh_are_you_freaking_high_E5 >> 4)
                            | ( ((uint16_t) moreCalib.bosh_are_you_freaking_high_E6) << 4));

                    return 0;
                }

                return -4;
            }

            return -3;
        }

        return -2;
    }

    return -1;
}

int bosz_read(int32_t * temperature, uint32_t * pressure, uint32_t * humidity)
{
    CtrlMeas_t ctrl;
    esp_err_t ret;
    spi_transaction_t t;

    ctrl.mode = 1;    // forced mode
    ctrl.osrs_p = 3;  // oversampling x 4 -> standard resolution -> 18 bit / 0.66 Pa
    ctrl.osrs_t = 1;  // oversampling x 1 -> 16 bit / 0.005 C

    memset(&t, 0, sizeof(t));
    t.cmd = BOSZ_WRITE;
    t.addr = BOSZ_A_CTRL_HUM;
    t.length = 8;
    t.flags = SPI_TRANS_USE_TXDATA;
    t.user = (void *) 0;
    t.tx_data[0] = 2; // oversampling x 2

    ret = spi_device_polling_transmit(s_spi, &t);

    if (ESP_OK == ret)
    {

        memset(&t, 0, sizeof(t));
        t.cmd = BOSZ_WRITE;
        t.addr = BOSZ_A_CTRL_MEAS;
        t.length = 8;
        t.flags = SPI_TRANS_USE_TXDATA;
        t.user = (void *) 0;
        t.tx_data[0] = ctrl.value;

        ret = spi_device_polling_transmit(s_spi, &t);

        if (ESP_OK == ret)
        {
            BoszMeas_t meas;

            vTaskDelay(50 / portTICK_RATE_MS);

            memset(&t, 0, sizeof(t));
            t.cmd = BOSZ_READ;
            t.addr = BOSZ_A_PRESS_MSB;
            t.length = sizeof(meas) * 8;
            t.rx_buffer = &meas;

            ret = spi_device_polling_transmit(s_spi, &t);

            if (ESP_OK == ret)
            {
                int32_t adc_P = (int32_t) ( ((uint32_t) meas.press_xlsb & 15) | (((uint32_t) meas.press_lsb) << 4) | (((uint32_t) meas.press_msb) << 12) );
                int32_t adc_T = (int32_t) ( ((uint32_t) meas.temp_xlsb & 15) | (((uint32_t) meas.temp_lsb) << 4) | (((uint32_t) meas.temp_msb) << 12) );
                int32_t adc_H = (int32_t) ( ((uint32_t) meas.hum_lsb) | (((uint32_t) meas.hum_msb) << 8) );

                *temperature = bme280_compensate_T_int32(adc_T);
                *pressure = bme280_compensate_P_int64(adc_P);
                *humidity = bme280_compensate_H_int32(adc_H);

                return 0;
            }

            return -2;
        }
    }

    return -1;
}


void bosz_test(void)
{
    int res;
    if (0 == (res = bosz_init()))
    {
        printf("Bosz initialized. calibration values:\n");
        printf("uint16_t dig_T1 = %d\n", (int) s_calib.dig_T1);
        printf("int16_t  dig_T2 = %d\n", (int) s_calib.dig_T2);
        printf("int16_t  dig_T3 = %d\n", (int) s_calib.dig_T3);
        printf("uint16_t dig_P1 = %d\n", (int) s_calib.dig_P1);
        printf("int16_t  dig_P2 = %d\n", (int) s_calib.dig_P2);
        printf("int16_t  dig_P3 = %d\n", (int) s_calib.dig_P3);
        printf("int16_t  dig_P4 = %d\n", (int) s_calib.dig_P4);
        printf("int16_t  dig_P5 = %d\n", (int) s_calib.dig_P5);
        printf("int16_t  dig_P6 = %d\n", (int) s_calib.dig_P6);
        printf("int16_t  dig_P7 = %d\n", (int) s_calib.dig_P7);
        printf("int16_t  dig_P8 = %d\n", (int) s_calib.dig_P8);
        printf("int16_t  dig_P9 = %d\n", (int) s_calib.dig_P9);
        printf("uint8_t  dig_H1 = %d\n", (int) s_calib.dig_H1);
        printf("int16_t  dig_H2 = %d\n", (int) s_calib.dig_H2);
        printf("uint8_t  dig_H3 = %d\n", (int) s_calib.dig_H3);
        printf("int16_t  dig_H4 = %d\n", (int) s_calib.dig_H4);
        printf("int16_t  dig_H5 = %d\n", (int) s_calib.dig_H5);
        printf("\n\n");

        vTaskDelay(1000 / portTICK_RATE_MS);

        for (int i = 0; i < 100; ++i)
        {
            int32_t t;
            uint32_t p;
            uint32_t h;

            if (0 == (res = bosz_read(&t, &p, &h)))
            {
                printf("T = %f C  P = %f hPa  RH = %.2f %%\n", ((double) t) / 100., ((double) (p / 256)) / 100., ((double) h / 1024.));
            }
            else
            {
                printf("Oh noes read failed - %d\n", res);
            }

            vTaskDelay(1000 / portTICK_RATE_MS);
        }
    }
    else
    {
        printf("Oh noes init failed - %d\n", res);
    }
}

