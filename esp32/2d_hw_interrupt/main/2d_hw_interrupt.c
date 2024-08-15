#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/gptimer.h"
#include "esp_attr.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_err.h"

#define HORIZONTAL_STEP_PIN GPIO_NUM_25
#define HORIZONTAL_DIR_PIN GPIO_NUM_26

#define VERTICAL_STEP_PIN GPIO_NUM_27
#define VERTICAL_DIR_PIN GPIO_NUM_32

#define TIMER_BASE_CLK   80000000  // 80 MHz APB clock frequency
#define TIMER_DIVIDER    80        // Hardware timer clock divider
#define TIMER_SCALE      (TIMER_BASE_CLK / TIMER_DIVIDER)  // Convert timer counter value to ticks per second

#define HORIZONTAL_INTERVAL_SEC (0.01)  // 10 ms interval for Horizontal Motor
#define HORIZONTAL_STEPS_PER_REV 200  // Full revolution for horizontal motor
#define VERTICAL_STEPS 96  // Number of steps for the vertical motor
#define VERTICAL_INTERVAL_SEC (0.001)  // 1 ms per vertical step

static volatile bool vertical_done = false;
static gptimer_handle_t vertical_timer = NULL;
volatile bool load_position_flag = false;

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

    // Write the position to NVS
    err = nvs_set_i32(my_handle, "vertical_rail_pos", position);
    if (err != ESP_OK) {
        printf("Failed to write position to NVS!\n");
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
        err = nvs_get_i32(my_handle, "vertical_rail_pos", &position);
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

    while (current_position > 0) {
        gpio_set_direction(VERTICAL_DIR_PIN, 0);
        
        gpio_set_level(VERTICAL_STEP_PIN, 1);
        esp_rom_delay_us(10);
        gpio_set_level(VERTICAL_STEP_PIN, 0);

        current_position--;
    } 
    
    gpio_set_direction(VERTICAL_DIR_PIN, 1);
}


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

void track_vertical_position(void *pvParameter) {
    while (1) {
        if (load_position_flag) {
            load_position_flag = false;
            // Perform the NVS operation or logging
            int prev_position = load_vertical_rail_position();
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
        gptimer_start(*(gptimer_handle_t*) user_data);  // Resume horizontal motor
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

void app_main() {
    // Initialize GPIO pins
    gpio_set_direction(HORIZONTAL_STEP_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(HORIZONTAL_DIR_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(VERTICAL_STEP_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(VERTICAL_DIR_PIN, GPIO_MODE_OUTPUT);

    // Set motor directions (1 or 0 depending on desired direction)
    gpio_set_level(HORIZONTAL_DIR_PIN, 1);

    init_nvs();
    recalibrate_vertical_rail();

    xTaskCreate(track_vertical_position, "vertical_calibration", 2048, NULL, 5, NULL);

    // Initialize timers for both motors
    gptimer_handle_t horizontal_timer = NULL;

    init_gptimer(&horizontal_timer, HORIZONTAL_INTERVAL_SEC, horizontal_timer_isr_callback, &horizontal_timer);
    init_gptimer(&vertical_timer, VERTICAL_INTERVAL_SEC, vertical_timer_isr_callback, &horizontal_timer);

    gptimer_start(horizontal_timer);  // Start horizontal motor first
}