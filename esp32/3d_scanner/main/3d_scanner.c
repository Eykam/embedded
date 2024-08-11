#include <stdio.h>
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "ssd1306.h"
#include <string.h>

#define configTICK_RATE_HZ 10000

#define HORIZONTAL_DIR GPIO_NUM_26
#define HORIZONTAL_STEP GPIO_NUM_25
#define HORIZONTAL_STEPPER_SWITCH GPIO_NUM_23

#define VERICAL_DIR GPIO_NUM_14
#define VERTICAL_STEP GPIO_NUM_27
#define VERTICAL_STEPPER_SWITCH GPIO_NUM_22

#define VERTICAL_STEP_LENGTH 360

#define tag "3d_scanner"

void display_scan_status(void *parameters){
    SSD1306_t dev;

    // QueueHandle_t xQueue = (QueueHandle_t)parameters;  
    // uint16_t receivedValue;
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
            
    
    ssd1306_clear_screen(&dev,false);

    // ======================================= Main =======================================
    char *l1 = "Status";
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

    int prev_vert_status = 0;
    int prev_horiz_status = 0;
    
    while(true){

        int vert_status = gpio_get_level(VERTICAL_STEPPER_SWITCH);
        int horiz_status = gpio_get_level(HORIZONTAL_STEPPER_SWITCH);

        if (vert_status != prev_vert_status || horiz_status != prev_horiz_status) {

            char l2_buffer [screen_width];
            char l3_buffer [screen_width];

            if (vert_status == 1){
                memcpy(l2_buffer,"Motor 1: On", 12);
            }else{
                memcpy(l2_buffer,"Motor 1: Off", 13);
            }

            if (horiz_status == 1){
                memcpy(l3_buffer,"Motor 2: On", 12);
            }else{
                memcpy(l3_buffer,"Motor 2: Off", 13);
            }

            ssd1306_display_text(&dev, 1, l2_buffer, screen_width, false);
            ssd1306_display_text(&dev, 2, l3_buffer, screen_width, false);
        }

        vTaskDelay(100);
    }
}

void stepper_horizontal(void *params)
{
    // int spr = 200;
    // int rpm  = 100;
    // int microsteps = 1;

    // float delay_ms = (60.0 * 1000) / (rpm * spr * microsteps * 2);
    // int delay_ticks = (int)(delay_ms / portTICK_PERIOD_MS);

    gpio_set_direction(HORIZONTAL_DIR, GPIO_MODE_OUTPUT);
    gpio_set_direction(HORIZONTAL_STEP, GPIO_MODE_OUTPUT);
    gpio_set_level(HORIZONTAL_DIR, 1);

    while(true) {
        if (gpio_get_level(HORIZONTAL_STEPPER_SWITCH) == 1) {
            gpio_set_level(HORIZONTAL_STEP, 1);
            esp_rom_delay_us(1);  // Small delay to ensure the step pulse is registered
                
            gpio_set_level(HORIZONTAL_STEP, 0);
        }
        
        vTaskDelay(1);
    }
}


void stepper_vertical(void *params)
{
    int height = 160;

    gpio_set_direction(VERICAL_DIR, GPIO_MODE_OUTPUT);
    gpio_set_direction(VERTICAL_STEP, GPIO_MODE_OUTPUT);
    gpio_set_level(VERICAL_DIR, 1);

    while(true && height < VERTICAL_STEP_LENGTH) {
      
      if (gpio_get_level(VERTICAL_STEPPER_SWITCH) == 1) {
        gpio_set_level(VERTICAL_STEP, 1);
        esp_rom_delay_us(1);  // Small delay to ensure the step pulse is registered
            
        gpio_set_level(VERTICAL_STEP, 0);
        height++;
      }
     
        // Yield to other tasks to prevent watchdog triggers    
        vTaskDelay(100);
    }
}

void app_main(void)
{
 // Create the display_range task
    xTaskCreate(
        display_scan_status,      // Function that implements the task
        "display_scan_status_task", // Text name for the task
        4096,               // Stack size in words
        NULL,               // Task input parameter
        5,                  // Task priority
        NULL                // Task handle
    );

    xTaskCreate(stepper_vertical, "stepper_vertical_task", 2048, NULL, 6, NULL);
    xTaskCreate(stepper_horizontal, "stepper_horizontal_task", 2048, NULL, 6, NULL);

}
