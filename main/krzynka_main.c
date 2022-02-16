/* 
*/
#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "esp_sleep.h"
#include "nvs.h"
#include "nvs_flash.h"

#include "smog.h"
#include "bosz.h"
#include "lux.h"
#include "wifi.h"
#include "storage.h"
#include "service.h"

void app_main(void)
{
    /* Print chip information */
    const GniotConfig_t * cfg;
    StationData_t weather_data = {
            .bosz_ok = false,
            .smog_ok = false,
    };

    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);

    printf("***** ***\n");

    storage_init();
    config_init();

    smog_init();
    smog_set_sleep(false);
    bosz_init();
    lux_init();

    cfg = config_get();

    for (int i = 0; i < 3; ++i)
    {
        int32_t t;
        uint32_t p;
        uint32_t h;
        if (0 == bosz_read(&t, &p, &h))
        {
            double ft = ((double) t) / 100.;
            double fp = ((double) (p / 256)) / 100.;
            double fh = ((double) h) / 1024.;
            printf("T = %f C  P = %f hPa  RH = %.2f %%\n", ft, fp, fh);
            weather_data.temperature = (float) ft;
            weather_data.pressure = (float) fp;
            weather_data.humidity = (float) fh;
            weather_data.bosz_ok = true;
            break;
        }
    }

    {
        uint32_t lux = lux_read();
        printf("Sun: %d mV\n", lux);
        weather_data.solar_mv = lux;
    }

    for (int i = 0; i < 3; ++i)
    {
        if (0 == smog_measure())
        {
            smog_print();
            weather_data.smog_ok = true;
            smog_get_data(&weather_data.smog);
            break;
        }
    }


    int conn_result = wifi_connect();

    if (0 == conn_result)
    {
        printf("Connected to wifi. IP address = %s\n", wifi_getIpAddress());
    }

    service_perform(conn_result, &weather_data);

    wifi_disconnect();


    const int wakeup_time_sec = 60 * cfg->sleep_length;
    printf("Enabling timer wakeup, %ds\n", wakeup_time_sec);
    esp_sleep_enable_timer_wakeup(wakeup_time_sec * 1000000);
    esp_deep_sleep_start();



    for (int i = 10; i >= 0; i--) {
        printf("Restarting in %d seconds...\n", i);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    printf("Restarting now.\n");
    fflush(stdout);
    esp_restart();
}
