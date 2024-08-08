#include <stdio.h>
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "ssd1306.h"


#define tag "SSD1306"

#define MAX_INT_DIGITS 11  // Enough for 32-bit int (-2147483648 to 2147483647)

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



void app_main(void)
{
    SSD1306_t dev;

    // ======================================= Setup =======================================
    #if CONFIG_I2C_INTERFACE
        ESP_LOGI(tag, "INTERFACE is i2c");
        ESP_LOGI(tag, "CONFIG_SDA_GPIO=%d",CONFIG_SDA_GPIO);
        ESP_LOGI(tag, "CONFIG_SCL_GPIO=%d",CONFIG_SCL_GPIO);
        ESP_LOGI(tag, "CONFIG_RESET_GPIO=%d",CONFIG_RESET_GPIO);
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
    


    // ======================================= Main =======================================

    gpio_set_direction(GPIO_NUM_5, GPIO_MODE_OUTPUT);

    int count = 0;
    
    while(true){
        char buffer [MAX_INT_DIGITS];
        count++;

        ssd1306_display_text(&dev, 0,  int_to_str(count, buffer), MAX_INT_DIGITS, false);
        
        gpio_set_level(GPIO_NUM_5, 1);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        
        gpio_set_level(GPIO_NUM_5, 0);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        
        ssd1306_clear_screen(&dev, false);
    }

}
 