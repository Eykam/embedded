
/* 
* This file is part of VL53L1 Platform 
* 
* Copyright (c) 2016, STMicroelectronics - All Rights Reserved 
* 
* License terms: BSD 3-clause "New" or "Revised" License. 
* 
* Redistribution and use in source and binary forms, with or without 
* modification, are permitted provided that the following conditions are met: 
* 
* 1. Redistributions of source code must retain the above copyright notice, this 
* list of conditions and the following disclaimer. 
* 
* 2. Redistributions in binary form must reproduce the above copyright notice, 
* this list of conditions and the following disclaimer in the documentation 
* and/or other materials provided with the distribution. 
* 
* 3. Neither the name of the copyright holder nor the names of its contributors 
* may be used to endorse or promote products derived from this software 
* without specific prior written permission. 
* 
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE 
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, 
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. 
* 
*/

#include "vl53l1_platform.h"
#include "vl53l1x_api.h"
#include "driver/gpio.h"
#include "driver/i2c_types.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#define TAG "VL53L1X"

#define I2C_NUM I2C_NUM_1
#define I2C_MASTER_FREQ_HZ 400000 // I2C clock of SSD1306 can run at 400 kHz max.
#define I2C_TICKS_TO_WAIT 100     // Maximum ticks to wait before issuing a timeout.


static i2c_master_bus_handle_t VL53L1X_i2c_bus = NULL;
static i2c_master_dev_handle_t VL53L1X_i2c_device = NULL;

void VL53L1X_I2C_init(int16_t sda, int16_t scl, int16_t reset)
{
	ESP_LOGI(TAG, "New i2c driver is used");
	i2c_master_bus_config_t i2c_mst_config = {
		.clk_source = I2C_CLK_SRC_DEFAULT,
		.glitch_ignore_cnt = 7,
		.i2c_port = I2C_NUM,
		.scl_io_num = scl,
		.sda_io_num = sda,
        .flags.enable_internal_pullup = true,
	};


	ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &VL53L1X_i2c_bus));

	i2c_device_config_t dev_cfg = {
		.dev_addr_length = I2C_ADDR_BIT_LEN_7,
		.device_address = VL53L1X_address,
		.scl_speed_hz = I2C_MASTER_FREQ_HZ,
	};

	ESP_ERROR_CHECK(i2c_master_bus_add_device(VL53L1X_i2c_bus, &dev_cfg, &VL53L1X_i2c_device));

}

int8_t VL53L1_WriteMulti(uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count) {
    uint8_t *buffer = malloc(count + 2);
    if (!buffer) return -1;
    
    buffer[0] = (index >> 8) & 0xFF;  // MSB of index
    buffer[1] = index & 0xFF;         // LSB of index
    memcpy(buffer + 2, pdata, count);
    
    esp_err_t ret = i2c_master_transmit(VL53L1X_i2c_device, buffer, count + 2, I2C_TICKS_TO_WAIT);
    free(buffer);
    return (ret == ESP_OK) ? 0 : -1;
}

int8_t VL53L1_ReadMulti(uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count) {
    uint8_t write_buffer[2];
    write_buffer[0] = (index >> 8) & 0xFF;  // MSB of index
    write_buffer[1] = index & 0xFF;         // LSB of index

    esp_err_t ret = i2c_master_transmit_receive(VL53L1X_i2c_device, 
                                                write_buffer, 2,  // Write 2 bytes (16-bit index)
                                                pdata, count, I2C_TICKS_TO_WAIT);



    return (ret == ESP_OK) ? 0 : -1;
}

int8_t VL53L1_WrByte(uint16_t dev, uint16_t index, uint8_t data) {
    uint8_t buffer[3] = {(index >> 8) & 0xFF, index & 0xFF, data};
    esp_err_t ret = i2c_master_transmit(VL53L1X_i2c_device, buffer, 3, I2C_TICKS_TO_WAIT);
    return (ret == ESP_OK) ? 0 : -1;
}

int8_t VL53L1_WrWord(uint16_t dev, uint16_t index, uint16_t data) {
    uint8_t buffer[4] = {(index >> 8) & 0xFF, index & 0xFF, (data >> 8) & 0xFF, data & 0xFF};
    esp_err_t ret = i2c_master_transmit(VL53L1X_i2c_device, buffer, 4, I2C_TICKS_TO_WAIT);
    return (ret == ESP_OK) ? 0 : -1;
}

int8_t VL53L1_WrDWord(uint16_t dev, uint16_t index, uint32_t data) {
    uint8_t buffer[6] = {(index >> 8) & 0xFF, index & 0xFF, 
                         (data >> 24) & 0xFF, (data >> 16) & 0xFF, 
                         (data >> 8) & 0xFF, data & 0xFF};
    esp_err_t ret = i2c_master_transmit(VL53L1X_i2c_device, buffer, 6, I2C_TICKS_TO_WAIT);
    return (ret == ESP_OK) ? 0 : -1;
}

int8_t VL53L1_RdByte(uint16_t dev, uint16_t index, uint8_t *data) {
    return VL53L1_ReadMulti(dev, index, data, 1);
}

int8_t VL53L1_RdWord(uint16_t dev, uint16_t index, uint16_t *data) {
    uint8_t buffer[2];
    int8_t status = VL53L1_ReadMulti(dev, index, buffer, 2);
    if (status == 0) {
        *data = ((uint16_t)buffer[0] << 8) | buffer[1];
    }
    return status;
}

int8_t VL53L1_RdDWord(uint16_t dev, uint16_t index, uint32_t *data) {
    uint8_t buffer[4];
    int8_t status = VL53L1_ReadMulti(dev, index, buffer, 4);
    if (status == 0) {
        *data = ((uint32_t)buffer[0] << 24) | ((uint32_t)buffer[1] << 16) | 
                ((uint32_t)buffer[2] << 8) | buffer[3];
    }
    return status;
}

int8_t VL53L1_WaitMs(uint16_t dev, int32_t wait_ms) {
    vTaskDelay(wait_ms / portTICK_PERIOD_MS);
    return 0;
}