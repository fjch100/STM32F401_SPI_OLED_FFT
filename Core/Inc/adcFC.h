/*
 * adcFC.h
 *
 *  Created on: Dec 30, 2022
 *      Author: fjch1
 */

#ifndef INC_ADCFC_H_
#define INC_ADCFC_H_

#include "stm32f4xx_hal.h"
#include "main.h"

#define FMULT 330.00/4095.00
#define FMULT2 3.3/4095.00

float AdcRead(uint32_t canal, uint8_t muestrasAverage);






#endif /* INC_ADCFC_H_ */
