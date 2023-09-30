/*
 * SHT31.h
 *
 *  Created on: 30 wrz 2023
 *      Author: macie
 */

#ifndef INC_SHT31_H_
#define INC_SHT31_H_

#include "stm32l4xx_hal.h"


void SHT31_ReadTempHumidity(float* temp, float* humidity);


#endif /* INC_SHT31_H_ */
