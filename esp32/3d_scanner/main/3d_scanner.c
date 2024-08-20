#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "esp_attr.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_err.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include <string.h>
#include "freertos/portable.h"
#include "esp_timer.h"

#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "lwip/err.h"
#include "lwip/sys.h"

#include "vl53l1x_api.h"
#include "ssd1306.h"

#define ESP_WIFI_SSID      CONFIG_ESP_WIFI_SSID
#define ESP_WIFI_PASS      CONFIG_ESP_WIFI_PASSWORD
#define ESP_MAXIMUM_RETRY  CONFIG_ESP_MAXIMUM_RETRY

#define tag "3d_scanner"

#define MAX_INT_DIGITS 11  // Enough for 32-bit int (-2147483648 to 2147483647)
#define sensorDev (uint16_t) 0x52

#define VL53L1X_SDA_IO 17
#define VL53L1X_SCL_IO 16

#define DOWN 0
#define UP 1

#define HORIZONTAL_STEP_PIN GPIO_NUM_25
#define HORIZONTAL_DIR_PIN GPIO_NUM_26

#define VERTICAL_STEP_PIN GPIO_NUM_27
#define VERTICAL_DIR_PIN GPIO_NUM_32

#define TIMER_BASE_CLK   80000000  // 80 MHz APB clock frequency
#define TIMER_DIVIDER    80        // Hardware timer clock divider
#define TIMER_SCALE      (TIMER_BASE_CLK / TIMER_DIVIDER)  // Convert timer counter value to ticks per second

#define HORIZONTAL_INTERVAL_SEC (0.018)  // 10 ms interval for Horizontal Motor
#define HORIZONTAL_STEPS 100  // Number of steps for the vertical motor
#define HORIZONTAL_STEPS_PER_REV 200  // Full revolution for horizontal motor

#define VERTICAL_MAX_STEPS 3300
#define VERTICAL_STEPS 96  // Number of steps for the vertical motor
#define VERTICAL_INTERVAL_SEC (0.001)  // 1 ms per vertical step


/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1


static volatile bool vertical_done = false;
static gptimer_handle_t horizontal_timer = NULL;
static gptimer_handle_t vertical_timer = NULL;
volatile bool load_position_flag = false;
// volatile bool sync_with_tof = false;

void init_nvs() {
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    printf("Did err? => (%s)\n", esp_err_to_name(err));
}


void save_vertical_rail_position(int32_t position) {
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
        return;
    }

    int32_t truncated_position = position;

    if (position >= VERTICAL_MAX_STEPS) {
        truncated_position = VERTICAL_MAX_STEPS;
    }

    // Write the position to NVS
    err = nvs_set_i32(my_handle, "vertical_pos", truncated_position);
    if (err != ESP_OK) {
        printf("Failed to write position to NVS! => (%s)\n", esp_err_to_name(err));
    }

    // Commit the written value
    err = nvs_commit(my_handle);
    if (err != ESP_OK) {
        printf("Failed to commit position to NVS!\n");
    }

    // Close NVS handle
    nvs_close(my_handle);
}



int32_t load_vertical_rail_position() {
    nvs_handle_t my_handle;
    int32_t position = 0; // Default position (e.g., 0, if not found)
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err == ESP_OK) {
        // Read the position from NVS
        err = nvs_get_i32(my_handle, "vertical_pos", &position);
        switch (err) {
            case ESP_OK:
                printf("vertical rail position = %ld\n", position);
                break;
            case ESP_ERR_NVS_NOT_FOUND:
                printf("vertical rail position not found, defaulting to %ld\n", position);
                break;
            default :
                printf("Error (%s) reading!\n", esp_err_to_name(err));
                break;
        }
        nvs_close(my_handle);
    } else {
        printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    }
    
    return position;
}

void recalibrate_vertical_rail() {
    int32_t current_position = load_vertical_rail_position();

    printf("Recalibrating from: (%ld) => 0\n", current_position);
    gpio_set_level(VERTICAL_DIR_PIN, DOWN);

    while (current_position > 0) {
        printf("Current Position: %ld\n", current_position);
        
        gpio_set_level(VERTICAL_STEP_PIN, 1);
        esp_rom_delay_us(10); 
        
        gpio_set_level(VERTICAL_STEP_PIN, 0);
        esp_rom_delay_us(10); 
        
        current_position--;
    }

    save_vertical_rail_position(0);
    gpio_set_level(VERTICAL_DIR_PIN, UP);
}

// void reset_vertical_rail_position(){
//     gpio_set_level(VERTICAL_DIR_PIN, DOWN);
    
//     int count = 0;
//     while (count < VERTICAL_MAX_STEPS){
//         gpio_set_level(VERTICAL_STEP_PIN, 1);
//         esp_rom_delay_us(10); 
        
//         gpio_set_level(VERTICAL_STEP_PIN, 0);
//         esp_rom_delay_us(10); 
//         count++;
//     }
// }



static bool IRAM_ATTR horizontal_timer_isr_callback(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data) {
    static int horizontal_step_count = 0;

    gpio_set_level(HORIZONTAL_STEP_PIN, 1);
    esp_rom_delay_us(10);
    gpio_set_level(HORIZONTAL_STEP_PIN, 0);

    horizontal_step_count++;
    if (horizontal_step_count >= HORIZONTAL_STEPS_PER_REV) {
        horizontal_step_count = 0;
        gptimer_stop(timer);  // Stop horizontal motor after one full revolution
        gptimer_start(vertical_timer);  // Start vertical motor steps
    }
    return true;  // Return true to continue firing the timer
}

void update_vertical_position_task(void *pvParameter) {
    while (1) {
        if (load_position_flag) {
            int32_t prev_position = load_vertical_rail_position();
            load_position_flag = false;
            // Perform the NVS operation or logging
            prev_position += VERTICAL_STEPS;
            save_vertical_rail_position(prev_position);
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

static bool IRAM_ATTR vertical_timer_isr_callback(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data) {
    static int vertical_step_count = 0;

    gpio_set_level(VERTICAL_STEP_PIN, 1);
    esp_rom_delay_us(10);
    gpio_set_level(VERTICAL_STEP_PIN, 0);

    vertical_step_count++;
    if (vertical_step_count >= VERTICAL_STEPS) {
        vertical_step_count = 0;
        vertical_done = true;
        load_position_flag = true;  

        gptimer_stop(timer);  // Stop vertical motor after 96 steps

        // Ensure the user_data is valid and correctly points to horizontal_timer
        if (user_data != NULL) {
            if (horizontal_timer != NULL) {
                esp_err_t err = gptimer_start(horizontal_timer);
                if (err != ESP_OK) {
                    printf("Failed to start horizontal timer: %d\n", err);
                }
            } else {
                printf("Error: horizontal_timer is NULL\n");
            }
        } else {
            printf("Error: user_data is NULL\n");
        }
    }

    return true;  // Return true to let the timer auto-reload and continue firing
}


void init_gptimer(gptimer_handle_t *timer, double timer_interval_sec, gptimer_alarm_cb_t isr_callback, void *user_data) {
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = TIMER_SCALE,  // Resolution based on the clock and divider
    };
    
    esp_err_t err = gptimer_new_timer(&timer_config, timer);
    if (err != ESP_OK || timer == NULL) {
        printf("Failed to initialize timer: %d\n", err);
        return;
    }

    gptimer_event_callbacks_t cbs = {
        .on_alarm = isr_callback,
    };
    err = gptimer_register_event_callbacks(*timer, &cbs, user_data);
    if (err != ESP_OK) {
        printf("Failed to register timer callbacks: %d\n", err);
        return;
    }

    gptimer_alarm_config_t alarm_config = {
        .reload_count = 0,  // Start counting from zero
        .alarm_count = (uint64_t)(timer_interval_sec * TIMER_SCALE),  // Set the alarm count based on the interval
        .flags.auto_reload_on_alarm = true,  // Auto-reload the timer on alarm
    };
    err = gptimer_set_alarm_action(*timer, &alarm_config);
    if (err != ESP_OK) {
        printf("Failed to set timer alarm action: %d\n", err);
        return;
    }

    err = gptimer_enable(*timer);
    if (err != ESP_OK) {
        printf("Failed to enable timer: %d\n", err);
        return;
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
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    ESP_LOGI(tag, "RangeFinder Booted! Intializing...");
    /* Sensor Initialization */
    uint8_t init_status = VL53L1X_SensorInit(sensorDev);
    ESP_LOGI(tag, "RangeFinder Intialized! %d", init_status);
    
    // Set a faster timing budget for higher frequency measurements
    VL53L1X_SetDistanceMode(sensorDev, 1);
    VL53L1X_SetTimingBudgetInMs(sensorDev, 15); // 10ms timing budget
    VL53L1X_SetInterMeasurementInMs(sensorDev, 18); // 10ms between measurements

    VL53L1X_StartRanging(sensorDev);
    ESP_LOGI(tag, "Started Ranging at 100Hz!");

    TickType_t xLastWakeTime;
    const TickType_t xFrequency = 1; // 10ms interval for 100Hz

    xLastWakeTime = xTaskGetTickCount();
    int counter = 0;

    /* ranging loop */
    while(true){
        uint8_t dataReady = 0;
        uint8_t rangeStatus;
        uint16_t distance;

        // uint32_t start_time = esp_timer_get_time();
        
        while (dataReady == 0) {
            VL53L1X_CheckForDataReady(sensorDev, &dataReady);
            
            if (dataReady == 0) {
                vTaskDelayUntil(&xLastWakeTime, 1); // Short delay if data not ready
            }
        }

  
        VL53L1X_GetRangeStatus(sensorDev, &rangeStatus);
        VL53L1X_GetDistance(sensorDev, &distance);
        VL53L1X_ClearInterrupt(sensorDev);
      
        
        if (rangeStatus == 0) { // 0 indicates a valid measurement
            if (counter >= 200){
                counter = 0;
                ESP_LOGI(tag,"Read 200 values from sensor!");
            }
            
            counter++;
            // ESP_LOGI(tag, "Distance: %d mm", distance);
            // if (xQueueSend(xQueue, &distance, 0) != pdPASS) {
            //     ESP_LOGW(tag, "Failed to send data to display queue");
            // }
        } else {
            ESP_LOGW(tag, "Invalid measurement, status: %d", rangeStatus);
        }


        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void display_range(void *parameters){
    SSD1306_t dev;

    QueueHandle_t xQueue = (QueueHandle_t)parameters;  
    uint16_t receivedValue;
    // uint16_t displayValue = 0;
    // int updateCounter = 0;
    // const int updateFrequency = 10;

    int screen_width = 128;  // Width of the SSD1306 display in pixels
    // int char_width = 6;      // Width of each character in pixels

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
    int counter = 0;

    while(true){
        if (xQueueReceive(xQueue, &receivedValue, 100) == pdPASS) {
            // ESP_LOGI(tag, "Received value: %d", receivedValue);               
            
            // char l2_buffer [screen_width];
            // snprintf(l2_buffer, sizeof(l2_buffer), 
            //                     "%d mm\n\t\t%d cm\n", 
            //                     receivedValue, 
            //                     receivedValue / 10);

            // char l3_buffer [screen_width];
            // snprintf(l3_buffer, sizeof(l3_buffer), 
            //                     "%.2f in\n%.2f ft", 
            //                     (float)receivedValue / 25.4, 
            //                     (float)receivedValue / 304.8);


            // ssd1306_display_text(&dev, 1, l2_buffer, screen_width, false);
            // ssd1306_display_text(&dev, 2, l3_buffer, screen_width, false);
            counter++;
        }


        if (counter >= 200) {
            ESP_LOGI(tag, "Counter reached 200");
            counter = 0;  // Reset the counter after logging
        }

        vTaskDelay(pdMS_TO_TICKS(10));  // Delay of 10 ms
    }
}


static int s_retry_num = 0;

static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(tag, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(tag,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(tag, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = ESP_WIFI_SSID,
            .password = ESP_WIFI_PASS,
            /* Setting a password implies station will connect to all security modes including WEP/WPA.
             * However these modes are deprecated and not advisable to be used. Incase your Access point
             * doesn't support WPA2, these mode can be enabled by commenting below line */
	     .threshold.authmode = WIFI_AUTH_WPA2_PSK,

            .pmf_cfg = {
                .capable = true,
                .required = false
            },
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(tag, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(tag, "connected to ap SSID:%s",
                 ESP_WIFI_SSID);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(tag, "Failed to connect to SSID:%s",
                 ESP_WIFI_SSID);
    } else {
        ESP_LOGE(tag, "UNEXPECTED EVENT");
    }

    ESP_ERROR_CHECK(esp_event_handler_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler));
    ESP_ERROR_CHECK(esp_event_handler_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler));
    vEventGroupDelete(s_wifi_event_group);
}




void app_main() {
    ESP_LOGI(tag, "[APP] Startup..");
    ESP_LOGI(tag, "[APP] Free memory: %ld bytes", esp_get_free_heap_size());
    ESP_LOGI(tag, "[APP] IDF version: %s", esp_get_idf_version());

    // Initialize GPIO pins
    gpio_set_direction(HORIZONTAL_STEP_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(HORIZONTAL_DIR_PIN, GPIO_MODE_OUTPUT);

    gpio_set_direction(VERTICAL_STEP_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(VERTICAL_DIR_PIN, GPIO_MODE_OUTPUT);

    // Set motor directions (1 or 0 depending on desired direction)
    gpio_set_level(HORIZONTAL_DIR_PIN, 1);

    init_nvs();
    recalibrate_vertical_rail();
    
    // ESP_LOGI(tag, "ESP_WIFI_MODE_STA");
    // wifi_init_sta();

    QueueHandle_t xQueue;
    xQueue = xQueueCreate(100, sizeof(uint16_t)); 
    
    xTaskCreate(update_vertical_position_task, "vertical_calibration", 2048, NULL, 5, NULL);

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
    xTaskCreatePinnedToCore(
        display_range,      // Function that implements the task
        "display_range_task", // Text name for the task
        4096,               // Stack size in words
        (void *)xQueue,               // Task input parameter
        4,                  // Task priority
        NULL,
        0               // Task handle
    );

    init_gptimer(&horizontal_timer, HORIZONTAL_INTERVAL_SEC, horizontal_timer_isr_callback, &horizontal_timer);
    init_gptimer(&vertical_timer, VERTICAL_INTERVAL_SEC, vertical_timer_isr_callback, &horizontal_timer);

    gptimer_start(horizontal_timer);  // Start horizontal motor first
}