/**
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

#include "vl53l1_platform.h"
#include <string.h>
#include <time.h>
#include <math.h>
#include <main.h>
extern I2C_HandleTypeDef hi2c2;

int8_t VL53L1_WriteMulti( uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count) {
	uint8_t addr=dev << 1;
	uint8_t buf[2+count];
	buf[0]=index >> 8;
	buf[1]=index & 0xFF;
	memcpy(&buf[2],pdata,count);
	if(HAL_I2C_Master_Transmit(&hi2c2,addr,buf,2+count,1000) != HAL_OK)
		return -1;
	return 0;
}

int8_t VL53L1_ReadMulti(uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count){
	uint8_t addr=dev << 1;
  uint8_t buf[2]={index >> 8 , index & 0xFF };
	if(HAL_I2C_Master_Transmit(&hi2c2,addr,buf,2,1000) != HAL_OK)
		return -1;
	if(HAL_I2C_Master_Receive(&hi2c2,addr,pdata,count,1000) != HAL_OK)
    return -1;
	return 0;
}

int8_t VL53L1_WrByte(uint16_t dev, uint16_t index, uint8_t data) {
	
	return VL53L1_WriteMulti(dev, index, &data, 1);
}

int8_t VL53L1_WrWord(uint16_t dev, uint16_t index, uint16_t data) {
	uint8_t buf[2];
	buf[0] = data >> 8;
	buf[1] = data & 0xFF;
	return VL53L1_WriteMulti(dev, index, buf, 2);
}

int8_t VL53L1_WrDWord(uint16_t dev, uint16_t index, uint32_t data) {
	uint8_t buf[4];
	buf[0] = (data >> 24) & 0xFF;
	buf[1] = (data >> 16) & 0xFF;
	buf[2] = (data >> 8) & 0xFF;
	buf[3] = data & 0xFF;
	return VL53L1_WriteMulti(dev, index, buf, 4);
}

int8_t VL53L1_RdByte(uint16_t dev, uint16_t index, uint8_t *data) {
	return VL53L1_ReadMulti(dev, index, data, 1);
}

int8_t VL53L1_RdWord(uint16_t dev, uint16_t index, uint16_t *data) {
	uint8_t buf[2];
	VL53L1_ReadMulti(dev, index, buf, 2);
	*data = (buf[0] << 8) | buf[1];
	return 0;
}

int8_t VL53L1_RdDWord(uint16_t dev, uint16_t index, uint32_t *data) {
	uint8_t buf[4];
	VL53L1_ReadMulti(dev, index, buf, 4);
	*data = (buf[0] << 24) | (buf[1] << 16) | (buf[2] << 8) | buf[3];
	return 0;
}

int8_t VL53L1_WaitMs(uint16_t dev, int32_t wait_ms){
	HAL_Delay(wait_ms);
	return 0;
}
