/*
 * SHT31.c
 *
 *  Created on: 30 wrz 2023
 *      Author: macie
 */
#include "SHT31.h"

#include "stm32l432xx.h"
#include "stm32l4xx_hal.h"

#define SHT31_ADDR 0x44 << 1 // SHT31 I2C address shifted left by 1 bit
#define CMD_MEASURE_TEMP 0x2C06 // Command to measure temperature
#define CMD_MEASURE_HUMIDITY 0x2C10 // Command to measure humidity

extern I2C_HandleTypeDef hi2c1;

void SHT31_ReadTempHumidity(float* temp, float* humidity)
{
    uint8_t data[6];
	uint8_t dt[2];
	uint8_t dh[2];
	dt[0] = 0x06;
	dt[1] = 0x2C;
	dh[0] = 0x10;
	dh[1] = 0x2C;
    uint16_t temp_raw, humidity_raw;
    // Send command to measure temperature
    uint8_t command_temp_buffer[2] = {(CMD_MEASURE_TEMP & 0xff00u) >> 8u, CMD_MEASURE_TEMP & 0xffu};
    uint8_t command_humid_buffer[2] = {(CMD_MEASURE_HUMIDITY & 0xff00u) >> 8u, CMD_MEASURE_HUMIDITY & 0xffu};
    HAL_I2C_Master_Transmit(&hi2c1, SHT31_ADDR, command_temp_buffer, sizeof(command_temp_buffer), 100);
    HAL_Delay(50);
    // Read temperature data
    HAL_I2C_Master_Receive(&hi2c1, SHT31_ADDR, data, 3, 100);
    temp_raw = data[0] << 8 | data[1];
    *temp = ((float)temp_raw * 175.0f / 65535.0f) - 45.0f;
    // Send command to measure humidity
    HAL_I2C_Master_Transmit(&hi2c1, SHT31_ADDR, command_humid_buffer, sizeof(command_humid_buffer), 100);
    HAL_Delay(50);
    // Read humidity data
    HAL_I2C_Master_Receive(&hi2c1, SHT31_ADDR, data, 3, 100);
    humidity_raw = data[0] << 8 | data[1];
    *humidity = ((float)humidity_raw * 100.0f / 65535.0f);
}
