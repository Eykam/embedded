#include <stdio.h>
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include <string.h>

#include "vl53l1x_api.h"
#include "ssd1306.h"


#define tag "range_finder"

#define MAX_INT_DIGITS 11  // Enough for 32-bit int (-2147483648 to 2147483647)
#define sensorDev (uint16_t) 0x52

char* int_to_str(int32_t num, char* buffer) {
    char* ptr = buffer + MAX_INT_DIGITS - 1;
    uint32_t unum = (num < 0) ? -num : num;
    
    *ptr = '\0';
    
    do {
        *--ptr = '0' + (unum % 10);
        unum /= 10;
    } while (unum > 0);
    
    if (num < 0) {
        *--ptr = '-';
    }
    
    return ptr;
}


void display_range(void *parameters){
    SSD1306_t dev;

    QueueHandle_t xQueue = (QueueHandle_t)parameters;  
    uint16_t receivedValue;
    int screen_width = 128;  // Width of the SSD1306 display in pixels
    int char_width = 6;      // Width of each character in pixels

    // ======================================= Setup =======================================
    #if CONFIG_I2C_INTERFACE
        ESP_LOGI(tag, "Configuring LCD I2C");
        ESP_LOGI(tag, "LCD SDA => %d",CONFIG_SDA_GPIO);
        ESP_LOGI(tag, "LCD SCL => %d",CONFIG_SCL_GPIO);
        i2c_master_init(&dev, CONFIG_SDA_GPIO, CONFIG_SCL_GPIO, CONFIG_RESET_GPIO);
    #endif // CONFIG_I2C_INTERFACE

    #if CONFIG_SSD1306_128x64
        ESP_LOGI(tag, "Panel is 128x64");
        ssd1306_init(&dev, 128, 64);
    #endif // CONFIG_SSD1306_128x64
    #if CONFIG_SSD1306_128x32
        ESP_LOGI(tag, "Panel is 128x32");
        ssd1306_init(&dev, 128, 32);
    #endif // CONFIG_SSD1306_128x32
    

    gpio_set_direction(GPIO_NUM_5, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_5, 1);            
    
    ssd1306_clear_screen(&dev,false);

    // ======================================= Main =======================================
    char *l1 = "Distance";
    int l1_len = strlen(l1);

    char l1_buffer[screen_width];     
    l1_buffer[0] = '\0';            

    for (int i = 1; i < (screen_width - (l1_len * 6)); i++) {
        l1_buffer[i] = ' ';
    }

    // Concatenate the text
    memcpy(l1_buffer,l1, l1_len);
    ESP_LOGI(tag, "l1_buffer: %s", l1_buffer);    
    ssd1306_display_text(&dev, 0, l1_buffer, screen_width, false);

    while(true){
        if (xQueueReceive(xQueue, &receivedValue, 100) == pdPASS) {
            ESP_LOGI(tag, "Received value: %d", receivedValue);               
            
            char l2_buffer [screen_width];
            snprintf(l2_buffer, sizeof(l2_buffer), 
                                "%d mm\n\t\t%d cm\n", 
                                receivedValue, 
                                receivedValue / 10);

            char l3_buffer [screen_width];
            snprintf(l3_buffer, sizeof(l3_buffer), 
                                "%.2f in\n%.2f ft", 
                                (float)receivedValue / 25.4, 
                                (float)receivedValue / 304.8);


            ssd1306_display_text(&dev, 1, l2_buffer, screen_width, false);
            ssd1306_display_text(&dev, 2, l3_buffer, screen_width, false);
        }
    }
}

void find_range(void *parameters) {
    QueueHandle_t xQueue = (QueueHandle_t)parameters;


    #if CONFIG_I2C_INTERFACE
        ESP_LOGI(tag, "Configuring RangeFinder I2C");
        ESP_LOGI(tag, "RangeFinder SDA => %d", VL53L1X_SDA_IO);
        ESP_LOGI(tag, "RangeFinder SCL => %d", VL53L1X_SCL_IO);
        VL53L1X_I2C_init(VL53L1X_SDA_IO, VL53L1X_SCL_IO, CONFIG_RESET_GPIO);
    #endif // CONFIG_I2C_INTERFACE


    ESP_LOGI(tag, "Booting RangeFinder...");
    uint8_t state = 0;
    int boot_status;

    while(state == 0) {
        
        boot_status = VL53L1X_BootState(sensorDev, &state);
        ESP_LOGI(tag, "RangeFinder boot_status: %d   state: %d", boot_status, state);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }

    ESP_LOGI(tag, "RangeFinder Booted! Intializing...");
    /* Sensor Initialization */
    VL53L1X_SensorInit(sensorDev);
    ESP_LOGI(tag, "RangeFinder Intialized!");
    
    // /* Modify the default configuration */
    // state = VL53L1X_SetInterMeasurementPeriod();
    // state = VL53l1X_SetOffset();
    /* enable the ranging*/

    VL53L1X_StartRanging(sensorDev);
    ESP_LOGI(tag, "Started Ranging!");
    /* ranging loop */
    while(true){
        uint8_t ready_signal = 0;
        uint8_t range_signal;
        uint16_t distance;

        // int read_check_status;
        // int distance_status;
        // int range_status;
        // int interrupt_status;

        ESP_LOGI(tag, "Waiting for Data...");
        
        while( ready_signal == 0){
             VL53L1X_CheckForDataReady(sensorDev, &ready_signal);
        }


        ready_signal = 0;
        VL53L1X_GetRangeStatus(sensorDev, &range_signal);
        VL53L1X_GetDistance(sensorDev, &distance);
        VL53L1X_ClearInterrupt(sensorDev);
        
        ESP_LOGI(tag, "Data Found! => %d", distance);

        if (xQueueSend(xQueue, &distance, (uint32_t) 100) != pdPASS){
            ESP_LOGE(tag, "Failed to send data to display");
        };

        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
}



void app_main(void)
{
    // Setup

    QueueHandle_t xQueue;
    xQueue = xQueueCreate(10, sizeof(uint16_t));  


    // Create the find_range task
    xTaskCreate(
        find_range,         // Function that implements the task
        "find_range_task",  // Text name for the task
        4096,               // Stack size in words
        (void *)xQueue,               // Task input parameter
        5,                  // Task priority
        NULL                // Task handle
    );

    // Create the display_range task
    xTaskCreate(
        display_range,      // Function that implements the task
        "display_range_task", // Text name for the task
        4096,               // Stack size in words
        (void *)xQueue,               // Task input parameter
        5,                  // Task priority
        NULL                // Task handle
    );
}
 