#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"

#define configTICK_RATE_HZ 10000
#define DIR GPIO_NUM_26
#define STEP GPIO_NUM_25

void stepper_task(void *params)
{
    int spr = 200;
    int rpm  = 100;
    int microsteps = 1;

    float delay_ms = (60.0 * 1000) / (rpm * spr * microsteps * 2);
    int delay_ticks = (int)(delay_ms / portTICK_PERIOD_MS);

    gpio_set_direction(DIR, GPIO_MODE_OUTPUT);
    gpio_set_direction(STEP, GPIO_MODE_OUTPUT);
    gpio_set_level(DIR, 1);

    while(true) {
        for(int i = 0; i < 200; i++) {  // 200 steps for a full rotation (adjust as needed)
            gpio_set_level(STEP, 1);
            esp_rom_delay_us(1);  // Small delay to ensure the step pulse is registered
            
            gpio_set_level(STEP, 0);
            vTaskDelay(1);

            // Yield to other tasks to prevent watchdog triggers    
        }

        vTaskDelay(1);
    }
}

void app_main() {
    xTaskCreate(stepper_task, "stepper_task", 2048, NULL, 5, NULL);
}