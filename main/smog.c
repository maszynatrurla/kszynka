/*
 * smog.c
 *
 *  Created on: 27 mar 2021
 *      Author: andrzej
 */

#include <stdint.h>
#include <stdbool.h>

#include "smog.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/uart.h"

#define RX_BUF_SIZE 64
#define EXPECTED_FRAME_LEN 0x1C

#define TXD_PIN (GPIO_NUM_17)
#define RXD_PIN (GPIO_NUM_16)

#define SMOG_DATA_COUNT 13

#define SMOG_ON_PIN    4

static const char * LEGEND [] = {
        "PM 1.0 CF=1 [ug/m3]",
        "PM 2.5 CF=1 [ug/m3]",
        "PM 10  CF=1 [ug/m3]",
        "PM 1.0 [ug/m3]",
        "PM 2.5 [ug/m3]",
        "PM 10  [ug/m3]",
        "0.3 um particles / 100ml",
        "0.5 um particles / 100ml",
        "1.0 um particles / 100ml",
        "2.5 um particles / 100ml",
        "5.0 um particles / 100ml",
        "10  um particles / 100ml",
};

static enum {
    E_SMOGS_START,
    E_SMOGS_PREAMBLE,
    E_SMOGS_FLENHI,
    E_SMOGS_FLENLO,
    E_SMOGS_DATAHI,
    E_SMOGS_DATALO,
    E_SMOGS_CHSUMHI,
    E_SMOGS_CHSUMLO,
} s_state;

uint16_t s_data[SMOG_DATA_COUNT];
uint16_t s_flen;
uint16_t s_idx;
uint16_t s_checksum;
uint16_t s_checksum_calc;

static bool smog_parse(uint8_t din);
static void smog_send_cmd(uint8_t cmd, uint8_t arg);


void smog_init(void)
{
    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO15/16
    io_conf.pin_bit_mask = (1ULL << SMOG_ON_PIN);
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

    gpio_set_level(SMOG_ON_PIN, 1);

    const uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_2, RX_BUF_SIZE * 4, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_2, &uart_config);
    uart_set_pin(UART_NUM_2, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    s_state = E_SMOGS_START;
}

bool smog_parse(uint8_t din)
{
    switch (s_state)
    {
    case E_SMOGS_START:
        s_checksum_calc = din;
        if (0x42 == din) s_state = E_SMOGS_PREAMBLE;
        break;
    case E_SMOGS_PREAMBLE:
        s_checksum_calc += din;
        if (0x4D == din) s_state = E_SMOGS_FLENHI;
        else s_state = E_SMOGS_START;
        s_flen = 0;
        break;
    case E_SMOGS_FLENHI:
        s_checksum_calc += din;
        s_flen = ((uint16_t) din) << 8;
        s_state = E_SMOGS_FLENLO;
        break;
    case E_SMOGS_FLENLO:
        s_checksum_calc += din;
        s_flen |= (uint16_t) din;
        if (EXPECTED_FRAME_LEN == s_flen) s_state = E_SMOGS_DATAHI;
        else s_state = E_SMOGS_START;
        s_idx = 0;
        break;
    case E_SMOGS_DATAHI:
        s_checksum_calc += din;
        s_data[s_idx] = ((uint16_t) din) << 8;
        s_state = E_SMOGS_DATALO;
        break;
    case E_SMOGS_DATALO:
        s_checksum_calc += din;
        s_data[s_idx] |= (uint16_t) din;
        ++s_idx;
        if (s_idx < SMOG_DATA_COUNT)
            s_state = E_SMOGS_DATAHI;
        else
            s_state = E_SMOGS_CHSUMHI;
        break;
    case E_SMOGS_CHSUMHI:
        s_checksum = ((uint16_t) din) << 8;
        s_state = E_SMOGS_CHSUMLO;
        break;
    case E_SMOGS_CHSUMLO:
        s_checksum |= (uint16_t) din;
        s_state = E_SMOGS_START;
        if (s_checksum == s_checksum_calc)
        {
            return true;
        }
        else
        {
            printf("checksum fail expected %04X received %04X\n",
                    s_checksum_calc, s_checksum);
        }
        break;
    default:
        s_state = E_SMOGS_START;
        break;
    }
    return false;
}

void smog_print(void)
{
    for (int i = 0; i < SMOG_DATA_COUNT - 1; ++i)
    {
        printf("%6d %s\n", (int) s_data[i], LEGEND[i]);
    }
    printf("\n\n");
}

void smog_get_data(SmogData_t * sdata)
{
    int i = 0;
    sdata->pm1_0_cf1 = s_data[i++];
    sdata->pm2_5_cf1 = s_data[i++];
    sdata->pm10_cf1  = s_data[i++];
    sdata->pm1_0     = s_data[i++];
    sdata->pm2_5     = s_data[i++];
    sdata->pm10      = s_data[i++];
    sdata->part0_3   = s_data[i++];
    sdata->part0_5   = s_data[i++];
    sdata->part1_0   = s_data[i++];
    sdata->part2_5   = s_data[i++];
    sdata->part5_0   = s_data[i++];
    sdata->part10    = s_data[i++];
}

static void smog_send_cmd(uint8_t cmd, uint8_t arg)
{
    char cmd_buf [] = {0x42, 0x4D, 0, 0, 0, 0, 0};
    uint16_t chsum = 0x42 + 0x4D;

    chsum += cmd;
    chsum += arg;

    cmd_buf[2] = (char) cmd;
    cmd_buf[4] = (char) arg;
    cmd_buf[5] = (char) (chsum >> 8);
    cmd_buf[6] = (char) (chsum & 255);

    uart_write_bytes(UART_NUM_2, cmd_buf, sizeof(cmd_buf));
}

void smog_read_pm(void)
{
    smog_send_cmd(0xE2, 0);
}

void smog_set_mode(bool active)
{
    smog_send_cmd(0xE1, active ? 0x01 : 0x00);
}

void smog_set_sleep(bool wakeup)
{
    smog_send_cmd(0xE4, wakeup ? 0x01 : 0x00);
}

int smog_measure(void)
{
    int rxBytes;
    uint8_t rxbuf [RX_BUF_SIZE];
    int reads_done = 0, fails = 0;

    // flush uart
    do {
        rxBytes = uart_read_bytes(UART_NUM_2, rxbuf, RX_BUF_SIZE, 10 / portTICK_RATE_MS);
    } while (rxBytes > 0);

    // wake up sensor
    smog_set_sleep(true);
    smog_set_sleep(true);
    smog_set_sleep(true);

    // wait for just over a 10 measurements (needs some time to get valid reading)
    for (reads_done = 0; reads_done < 14 && fails < 20;)
    {
        rxBytes = uart_read_bytes(UART_NUM_2, rxbuf, RX_BUF_SIZE, 200 / portTICK_RATE_MS);

        if (rxBytes > 0)
        {
            for (int i = 0; i < rxBytes; ++i)
            {
                if (smog_parse(rxbuf[i]))
                {
                    fails = 0;
                    printf("%d. smog\n", reads_done);
                    ++reads_done;
//                    smog_print();
                }
            }
        }
        else
        {
            ++fails;
        }
    }

    // put sensor back to sleep
    // (otherwise it uses lots of power to run fan
    smog_set_sleep(false);

    return (reads_done >= 14) ? 0 : -1;
}

void smog_test(void)
{
    uint8_t rxbuf [RX_BUF_SIZE];
    int reads_done = 0;
//    int kupska = 0;

    smog_init();

    while (1)
    {
        int rxBytes;

//        smog_set_mode(false);
        smog_set_sleep(false);

        vTaskDelay(6000 / portTICK_RATE_MS);

        smog_set_sleep(true);
//        smog_set_mode(false);

        for (reads_done = 0; reads_done < 20;)
        {
            rxBytes = uart_read_bytes(UART_NUM_2, rxbuf, RX_BUF_SIZE, 200 / portTICK_RATE_MS);

            if (rxBytes > 0)
            {
                for (int i = 0; i < rxBytes; ++i)
                {
                    if (smog_parse(rxbuf[i]))
                    {
                        printf("%d.\n", reads_done);
                        ++reads_done;
                        smog_print();
                    }
                }
            }
        }
    }
//
//    vTaskDelay(6000 / portTICK_RATE_MS);
//
//
//    vTaskDelay(2000 / portTICK_RATE_MS);
//
//    while (1)
//    {
//        const int rxBytes = uart_read_bytes(UART_NUM_2, rxbuf, RX_BUF_SIZE, 200 / portTICK_RATE_MS);
//
//        if (rxBytes > 0)
//        {
//            for (int i = 0; i < rxBytes; ++i)
//            {
//                if (smog_parse(rxbuf[i]))
//                {
//                    printf("%d.\n", reads_done);
//                    ++reads_done;
//                    smog_print();
//                }
//            }
//            kupska = 0;
//        }
//        else
//        {
//            ++kupska;
//            printf("X| %d\n", kupska);
//        }
//
//        if (reads_done > 30)
//        {
////            smog_set_sleep(false);
//            printf("SMOG_OFF\n");
//            gpio_set_level(SMOG_ON_PIN, 0);
//            reads_done = 0;
//        }
//
//        if (kupska > 100)
//        {
////            smog_set_sleep(true);
//            printf("SMOG_ON\n");
//            gpio_set_level(SMOG_ON_PIN, 1);
//            vTaskDelay(2000 / portTICK_RATE_MS);
//            kupska = 0;
//        }
//    }
}

