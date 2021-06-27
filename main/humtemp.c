/**
 * Driver for reading from <del>DHT-11</del> DHT-22 temperature and humidity sensor.
 * humtemp.c
 *
 *  Created on: 8 lut 2021
 *      Author: andrzej
 */

#include <stdbool.h>
#include <string.h>

#include "humtemp.h"

#include "driver/gpio.h"
#include "esp_timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"

/**
 * GPIO used for data line to DHT22 chip.
 */
#define DHT_DATA_PIN    13

/**
 * How many clock ticks (in CCOUNT register) per microsecond?
 */
#define CLOCK_TICKS_PER_MICROSECOND 80 // WARNING! Assumption that we run 80MHz clock!

/**
 * Prefix for E_LOG.
 */
static const char *TAG = "HumTemp";

/**
 * Queue for passing events from IRQ to task.
 */
static xQueueHandle evt_queue = NULL;

/**
 * Read state machine states.
 */
static enum {
    E_DHT_RD_IDLE,          /**< Nothing to be done. */
    E_DHT_RD_START,         /**< Start bit. */
    E_DHT_RD_START_HIGH,    /**< Start bit continuation. */
    E_DHT_RD_BIT_A,         /**< Low part of data bit. */
    E_DHT_RD_BIT_B,         /**< High part of data bit. */
} s_dht_read_state;

static int64_t s_prev_time;    /**< Store clock ticks of previous state. */

/**
 * Temporary buffer for storing bits read from DHT.
 */
static struct
{
    uint8_t bit;    /**< Index of current bit. */
    uint8_t dat[5]; /**< Values read. */
} s_read;


/**
 * Set pin connected to DHT-11 data as output.
 * Prepare for first phase of DHT data read - read request sending.
 */
static void set_output(void)
{
    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO15/16
    io_conf.pin_bit_mask = (1ULL << DHT_DATA_PIN);
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);
}

/**
 * Set pin connected to DHT-11 data as input.
 * Prepare for second phase of DHT data read -
 * capture of data.
 */
static void set_input(void)
{
    gpio_config_t io_conf;
    // enable interrupt for both edges of signal
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    // set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << DHT_DATA_PIN);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
}

/**
 * Update temporary buffer of DHT data with single bit.
 * @return true if all data collected
 */
static int add_bit(int b)
{
    if (b)
    {
        s_read.dat[s_read.bit >> 3] |= (1 << (7 - (s_read.bit & 7)));
    }

    ++s_read.bit;
    return (s_read.bit >= 40);
}

/**
 * Interrupt handler for both edges of DHT-11 data pin.
 * Reads data and appends it to temporary buffer.
 * hen all data is read it sends message to queue.
 */
static void gpio_isr_handler(void * arg)
{
    // Get accurate CPU "timestamp" of given moment.
    int64_t now_cc = esp_timer_get_time();
    // Get how many microseconds since previous read state
    uint32_t dt_us = (uint32_t) (now_cc - s_prev_time);
    // Get state of DATA pin (high/low?)
    int data_state = gpio_get_level(DHT_DATA_PIN);

    /*
     * FSM-like logic for reading data.
     * Interrupt should be called for every change of DATA signal,
     * in case it is called more often, we check with clock count
     * to see where we are in expected waveform.
     */
    if (E_DHT_RD_START == s_dht_read_state)
    {
        if (data_state && (dt_us > 80))
        {
            /* at least 80 microseconds from start of transaction
             * and DATA is HIGH, that means we are in start bit
             * and can continue */
            s_dht_read_state = E_DHT_RD_START_HIGH;
        }
    }
    else if (E_DHT_RD_START_HIGH == s_dht_read_state)
    {
        if (!data_state && (dt_us > 160))
        {
            /* at lease 160 microseconds from start of transaction
             * and DATA is LOW, the next DATA transition will be a
             * first data bit */
            s_dht_read_state = E_DHT_RD_BIT_A;
            s_prev_time = now_cc;
            s_read.bit = 0;
            memset(s_read.dat, 0, sizeof(s_read.dat));
        }
    }
    else if (E_DHT_RD_BIT_A == s_dht_read_state)
    {
        if (data_state && (dt_us > 30))
        {
            s_dht_read_state = E_DHT_RD_BIT_B;
            s_prev_time = now_cc;
        }
    }
    else if (E_DHT_RD_BIT_B == s_dht_read_state)
    {
        if (!data_state && (dt_us > 10))
        {
            int done;

            s_dht_read_state = E_DHT_RD_BIT_A;
            s_prev_time = now_cc;

            /* We determine the type of bit sent by DHT (1 or zero)
             * by the duty cycle of wave (how long it is high)*/
            if (dt_us > 50)
            {
                done = add_bit(1);
            }
            else
            {
                done = add_bit(0);
            }

            if (done)
            {
                /* we have 40 bits already.
                 * inform task and stop reading */
                s_dht_read_state = E_DHT_RD_IDLE;
                xQueueSendFromISR(evt_queue, (uint32_t *) &s_prev_time, NULL);
            }
        }
    }
}

void humtemp_init(void)
{
    /* enable per-pin GPIO interrupts */
    gpio_install_isr_service(0);
    /* create queue. we only need it for waking main
     * task when, DHT read is completed */
    evt_queue = xQueueCreate(2, sizeof(uint32_t));
    /* minimal delay from POWER ON till DHT ready
     * can be deleted if this delay is insured by logic
     * somewhere else */
//    vTaskDelay(2200 / portTICK_RATE_MS);
}


int humtemp_read(Humidity_t * humidity, Temperature_t * temperature)
{
    int result = HT_E_UNKNOWN;
    uint32_t dummy;

    s_dht_read_state = E_DHT_RD_START;

    /* request transmission, by forcing DHT's DATA pin LOW */
    set_output();
    gpio_set_level(DHT_DATA_PIN, 0);
    /* transmission request must last at least 18 milliseconds */
    vTaskDelay(30 / portTICK_RATE_MS);

    /* prepare for interrupt-based data capture from DHT */
    gpio_isr_handler_add(DHT_DATA_PIN, gpio_isr_handler, NULL);
    /* save time when transaction started */
    s_prev_time = esp_timer_get_time();
    set_input(); // reconfiguring pin will bring it back HIGH (pull up)

    /* Wait for data read to be complete.
     * Wait for maximum of 1 second. */
    if (xQueueReceive(evt_queue, &dummy, 1000 / portTICK_PERIOD_MS))
    {
        /* verify read data by checksum */
        if ((255 & (s_read.dat[0] + s_read.dat[1] + s_read.dat[2] + s_read.dat[3])) == s_read.dat[4])
        {
            int T_sign = s_read.dat[2] & 0x80;

            *humidity = (((uint16_t) s_read.dat[0]) << 8) | (uint16_t) s_read.dat[1];
            *temperature = (((uint16_t) (s_read.dat[2] & 0x7F)) << 8) | (uint16_t) s_read.dat[3];
            if (T_sign)
            {
                *temperature = 0 - *temperature;
            }

            ESP_LOGI(TAG, "RH=%02X %02X  T=%02X %02X  \n", (unsigned) s_read.dat[0], (unsigned) s_read.dat[1],
                    (unsigned) s_read.dat[2], (unsigned) s_read.dat[3]);

            result = HT_SUCCESS;
        }
        else
        {
            ESP_LOGW(TAG, "CHECKSUM ERROR: %d+%d+%d+%d=%d NOT %d",(int) s_read.dat[0],
                    (int) s_read.dat[1], (int) s_read.dat[2], (int) s_read.dat[3],
                    (int) (155 & (s_read.dat[0] + s_read.dat[1] + s_read.dat[2] + s_read.dat[3])),
                    (int) s_read.dat[4]);
            result = HT_E_CHECKSUM;
        }
    }
    else
    {
        ESP_LOGW(TAG, "Read timed out.");
        result = HT_E_TIMEOUT;
    }

    /* Interrupt can be disabled now */
    gpio_isr_handler_remove(DHT_DATA_PIN);
    gpio_set_intr_type(DHT_DATA_PIN, GPIO_INTR_DISABLE);

    return result;
}

void humtemp_test(void)
{
    humtemp_init();

    for (int i = 0; i < 40; ++i)
    {
        int res;
        Humidity_t h;
        Temperature_t t;

        if (0 == (res = humtemp_read(&h,  &t)))
        {
            printf("T = %f C  RH = %f %%\n", ((double) t) / 10., ((double) h) / 10. );
        }
        else
        {
            printf("oj failed %d\n", res);
        }

        vTaskDelay(3000 / portTICK_RATE_MS);
    }
}

