#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_attr.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_err.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include <string.h>
#include "freertos/portable.h"

#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

#include "vl53l1x_api.h"
#include "ssd1306.h"

#define ESP_WIFI_SSID      CONFIG_ESP_WIFI_SSID
#define ESP_WIFI_PASS      CONFIG_ESP_WIFI_PASSWORD
#define ESP_MAXIMUM_RETRY  CONFIG_ESP_MAXIMUM_RETRY

#define PORT 8035
#define BUFFER_LENGTH 1500

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

#define HORIZONTAL_INTERVAL_SEC (0.018)  // 10 ms interval for Horizontal Motor
#define HORIZONTAL_STEPS 100  // Number of steps for the vertical motor
#define HORIZONTAL_STEPS_PER_REV 200  // Full revolution for horizontal motor

#define VERTICAL_MAX_STEPS 3300
#define VERTICAL_STEPS 96  // Number of steps for the vertical motor
#define VERTICAL_INTERVAL_SEC (0.001)  // 1 ms per vertical step


/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;
static EventGroupHandle_t event_group;

#define STATUS_QUEUE_BIT (1 << 0)
#define POINT_QUEUE_BIT (1 << 1)


/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define CLIENT_IP <CLIENT_IP_HERE> // IP of client you'd like to connect to
#define HOST_IP 

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1


enum ScannerStatus {
    OFFLINE,
    INITIALIZING,
    READY,
    SCANNING,
    PAUSED,
    DONE,
    RESTARTING,
};


volatile static int SAMPLES = 1;
volatile static bool vertical_done = false;
volatile static bool load_position_flag = false;
volatile static uint8_t current_vertical_rail_position = 0;
volatile static enum ScannerStatus Status = OFFLINE;
// volatile bool sync_with_tof = false;

QueueHandle_t point_queue;
QueueHandle_t status_queue;

#define QUEUE_SIZE 200
#define PACKET_SIZE 4  // 64 bits = 8 bytes

typedef struct {
    uint16_t horizontal_steps;  // 8 bits (8 unused)
    uint16_t vertical_steps;    // 6 bits (10 bits unused)
    uint32_t distance_mm;      // 18 bits (14 bits unused)
} ScannerData;


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

    current_vertical_rail_position = truncated_position;

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
    ESP_LOGI(tag, "Started Ranging at 100Hz!\n");

    TickType_t xLastWakeTime;
    // const TickType_t xFrequency = 1; // 10ms interval for 100Hz

    xLastWakeTime = xTaskGetTickCount();

    ScannerData data;
    int data_count = 0;
    int rotations = 0;
    int sample_count = 0;
    int total_rotations =  VERTICAL_MAX_STEPS / VERTICAL_STEPS;

    while(true) {
        uint8_t dataReady = 0;
        uint8_t rangeStatus;
        uint16_t distance;
        uint16_t avg_dist = 0;

        switch (Status){

            case SCANNING:
                // ESP_LOGI(tag,"Starting Sampling for => %d\n", data_count);
                while (sample_count < SAMPLES  && rotations < total_rotations){

                    // Wait for data to be ready
                    while (dataReady == 0) {
                        VL53L1X_CheckForDataReady(sensorDev, &dataReady);
                        vTaskDelay(1);
                    }

                    VL53L1X_GetRangeStatus(sensorDev, &rangeStatus);
                    VL53L1X_GetDistance(sensorDev, &distance);
                    VL53L1X_ClearInterrupt(sensorDev);

                    if (rangeStatus == 0 || rangeStatus == 4) {
                        // Signal to step the motor
                        // ESP_LOGI(tag, "Valid Measurement => [%d] : %d\n", sample_count, distance);
                        avg_dist += distance;
                        sample_count++;
                    } else {
                        ESP_LOGW(tag, "Invalid measurement, status: %d\n", rangeStatus);
                    }

                    vTaskDelay(pdMS_TO_TICKS(10)); // Adjust this delay as needed
                }

                if (rotations == total_rotations){
                    uint8_t updated_status = (uint8_t) DONE;

                    if (xQueueSend(status_queue, &updated_status, 0) == pdPASS) { 
                        xEventGroupSetBits(event_group, STATUS_QUEUE_BIT);
                    }
                }

                // Obtained Valid Samples
                if (sample_count == SAMPLES){

                    // Populate data structure
                    data.horizontal_steps = ++data_count;
                    data.vertical_steps = rotations * VERTICAL_STEPS;
                    data.distance_mm = avg_dist / SAMPLES;

                    sample_count = 0;
                    avg_dist = 0;

                    // Send data to queues
                    if (xQueueSend(point_queue, &data, 0) == pdPASS) { 
                        xEventGroupSetBits(event_group, POINT_QUEUE_BIT);
                    }
                    if (xQueueSend(xQueue, &distance, 0) != pdPASS) {
                        ESP_LOGW(tag, "Failed to send data to display queue");
                    }


                    // Step the motor after each measurement
                    gpio_set_level(HORIZONTAL_STEP_PIN, 1);
                    esp_rom_delay_us(10);
                    gpio_set_level(HORIZONTAL_STEP_PIN, 0);
                
                }

            
                if (data_count == HORIZONTAL_STEPS_PER_REV) {
                    rotations++;
                    data_count = 0;

                    // Trigger vertical step here
                    int vertical_step_count = 0;
                    gpio_set_level(VERTICAL_DIR_PIN, 1);

                    while (vertical_step_count < VERTICAL_STEPS){
            
                        gpio_set_level(VERTICAL_STEP_PIN, 1);
                        esp_rom_delay_us(10);

                        gpio_set_level(VERTICAL_STEP_PIN, 0);
                        vertical_step_count++;
                        vTaskDelay(1);
                    }

                    load_position_flag = true;
                }
                break;

            case INITIALIZING:
                ESP_LOGI(tag,"Initializing...\n");

                uint8_t updated_status = (uint8_t) INITIALIZING;
                if (xQueueSend(status_queue, &updated_status, 0) == pdPASS) { 
                    xEventGroupSetBits(event_group, STATUS_QUEUE_BIT);
                }

                vTaskDelay(500);
                break;

            case READY:
                ESP_LOGI(tag,"Ready for Start Command...\n");
                vTaskDelay(500);
                break;

            case PAUSED:
                ESP_LOGI(tag,"Paused...\n");
                vTaskDelay(500);
                break;

            case DONE:
                ESP_LOGI(tag,"Done! Restart to continue...\n");
                vTaskDelay(500);
                break;

            case RESTARTING:
                ESP_LOGI(tag,"Restarting...\n");
                data_count = 0;
                rotations = 0;
                sample_count = 0;
                recalibrate_vertical_rail();
                
                uint8_t currStatus = (uint8_t) READY; 
                if (xQueueSend(status_queue, &currStatus, 0) == pdPASS) {
                    xEventGroupSetBits(event_group, STATUS_QUEUE_BIT);
                }

                break;

            case OFFLINE:
                ESP_LOGI(tag,"Offline...\n");
                vTaskDelay(500);
                break;
            }
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

            if (counter % 10 == 0) {

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

void wifi_transmission_task(void *pvParameters) {
    ScannerData data;
    uint8_t received_status;
    
    // Create a UDP socket
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);

    struct sockaddr_in dest_addr;
    dest_addr.sin_addr.s_addr = inet_addr(CLIENT_IP);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(8035);

    while (1) {
        if (sock < 0) {
            ESP_LOGE(tag, "Unable to create socket: errno %d", errno);
            sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
            continue;
        }

        // Check both queues without blocking
        EventBits_t bits = xEventGroupWaitBits(
            event_group,
            STATUS_QUEUE_BIT | POINT_QUEUE_BIT,
            pdTRUE,  // Clear bits before returning
            pdFALSE,  // Don't wait for all bits
            0  // Don't block
        );

        if (bits & STATUS_QUEUE_BIT) {
            if (xQueueReceive(status_queue, &received_status, 0) == pdPASS) {
                uint8_t packet[1];
                packet[0] = received_status;
                Status = received_status;
            
                ESP_LOGI(tag, "Sending status to backend => %d", Status);
                int err = sendto(sock, packet, sizeof(packet), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));

                if (err < 0) {
                    ESP_LOGE(tag, "Error occurred during sending done command: errno %d", errno);
                }
            }
        }

        if (bits & POINT_QUEUE_BIT) {
            if (data.vertical_steps < VERTICAL_MAX_STEPS && xQueueReceive(point_queue, &data, 0) == pdPASS) {
                uint16_t packet[PACKET_SIZE];
                // Pack data into the packet
                packet[0] = data.horizontal_steps;
                packet[1] = data.vertical_steps;  // Only use 6 bits
                packet[2] = (data.distance_mm >> 8) & 0xFF;  // High byte
                packet[3] = data.distance_mm & 0xFF;  // Low byte

                int err = sendto(sock, packet, sizeof(packet), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
            
                if (err < 0) {
                    ESP_LOGE(tag, "Error occurred during sending: errno %d", errno);
                }
            }
        }

        // If neither queue had data, add a small delay to prevent tight-looping
        if ((bits & (STATUS_QUEUE_BIT | POINT_QUEUE_BIT)) == 0) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }

    shutdown(sock, 0);
    close(sock);
}



void wifi_receiving_task(void *pvParameters){
    int sock;
    struct sockaddr_in dest_addr;
    bool wifi_connected = false;

    // Function to create and bind a socket
    bool create_bind_socket(int *sock, struct sockaddr_in *addr) {
        *sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
        if (*sock < 0) {
            ESP_LOGE(tag, "Unable to create socket: errno %d", errno);
            return false;
        }
        ESP_LOGI(tag, "Socket created");

        addr->sin_addr.s_addr = htonl(INADDR_ANY);
        addr->sin_family = AF_INET;
        addr->sin_port = htons(PORT);

        int err = bind(*sock, (struct sockaddr *)addr, sizeof(*addr));
        if (err < 0) {
            ESP_LOGE(tag, "Socket unable to bind: errno %d", errno);
            close(*sock);
            *sock = -1;
            return false;
        }
        ESP_LOGI(tag, "Socket bound, port %d", PORT);
        return true;
    }


    // Initially create and bind the socket
    if (!create_bind_socket(&sock, &dest_addr)) {
        vTaskDelete(NULL); // Failed to create socket, delete the task
    }

    while (1) {
        wifi_ap_record_t ap_info;
        
        if (esp_wifi_sta_get_ap_info(&ap_info) == 0) {
           if (!wifi_connected) {
                ESP_LOGI(tag, "Wi-Fi reconnected");
                wifi_connected = true;
                
                // Re-create and bind the socket if it was previously closed
                if (sock == -1) {
                    if (!create_bind_socket(&sock, &dest_addr)) {
                        ESP_LOGE(tag, "Failed to rebind socket after Wi-Fi reconnection");
                        vTaskDelay(pdMS_TO_TICKS(250)); // Retry after a delay
                        continue;
                    }
                }
            }
        } else {
            ESP_LOGW(tag, "Wi-Fi disconnected");
            wifi_connected = false;

            // Close the socket when disconnected
            if (sock != -1) {
                ESP_LOGI(tag, "Closing socket due to Wi-Fi disconnection");
                shutdown(sock, 0);
                close(sock);
                sock = -1;
            }

            vTaskDelay(pdMS_TO_TICKS(250)); // Wait before retrying
            continue;
        }


        if (sock != -1) {
            struct sockaddr_in source_addr;
            socklen_t socklen = sizeof(source_addr);
            uint8_t buffer[BUFFER_LENGTH];

            int len = recvfrom(sock, buffer, sizeof(buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);

            if (len < 0) {
                ESP_LOGE(tag, "recvfrom failed: errno %d", errno);
                if (errno == EBADF) {
                    // Invalid socket, set sock to -1 and clean up
                    ESP_LOGE(tag, "Bad file descriptor, resetting socket");
                    close(sock);
                    sock = -1;
                    continue; // Retry after cleanup
                }
                if (errno == EAGAIN || errno == EWOULDBLOCK) {
                    continue; // Retry in case of non-critical errors
                }
                break; // Exit loop for critical errors
            } else {
                buffer[len] = 0; // Null-terminate whatever we received and treat like a string
                char addr_str[128];
                inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr.s_addr, addr_str, sizeof(addr_str) - 1);
                
                if (strcmp(addr_str, CLIENT_IP) == 0) {
                    // ESP_LOGI(tag, "Received %d bytes from %s:", len, addr_str);
                    // ESP_LOGI(tag, "%s", buffer);
                    
                    // If you need to print as integers:
                    // ESP_LOGI(tag, "Message as integers:");
                    for (int i = 0; i < len; i++) {
                        // printf("%d ", buffer[i]);
                        if (buffer[i] == 255){
                           ESP_LOGI(tag, "Received KeepAlive");
                        }else{
                            Status = (enum ScannerStatus) buffer[i];
                        }
                    }

                    printf("\n");
                }
            }

        } else {
            ESP_LOGW(tag, "Socket is not valid, skipping recvfrom");
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
        
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
    
    ESP_LOGI(tag, "ESP_WIFI_MODE_STA");
    wifi_init_sta();

    status_queue = xQueueCreate(100, sizeof(uint8_t));
    event_group = xEventGroupCreate();

    if (status_queue == NULL) {
        ESP_LOGE(tag, "Failed to create status queue");
        return;
    }

    uint8_t updated_status = (uint8_t) INITIALIZING;

    if (xQueueSend(status_queue, &updated_status, 0) == pdPASS) {
        xEventGroupSetBits(event_group, STATUS_QUEUE_BIT);
    }
    
    recalibrate_vertical_rail();

    QueueHandle_t xQueue;
    xQueue = xQueueCreate(100, sizeof(uint16_t)); 

    point_queue = xQueueCreate(QUEUE_SIZE, sizeof(ScannerData));
    if (point_queue == NULL) {
        ESP_LOGE(tag, "Failed to create data queue");
        return;
    }

    xTaskCreatePinnedToCore(
        wifi_receiving_task,
        "wifi_receiving_task",
        4096,
        NULL,
        5,
        NULL,
        1  // Pin to Core 1
    );

    xTaskCreatePinnedToCore(
        wifi_transmission_task,
        "wifi_transmission_task",
        4096,
        NULL,
        5,
        NULL,
        1  // Pin to Core 1
    );

    
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
        1               // Task handle
    );
}