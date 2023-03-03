/*
 * adcFC.c
 *
 *  Created on: Dec 30, 2022
 *      Author: fjch1
 */

#include "adcFC.h"

extern ADC_HandleTypeDef hadc1;

float AdcRead(uint32_t canal, uint8_t numAverage){
	ADC_ChannelConfTypeDef cConfig = {0};
	cConfig.Channel = canal;
	cConfig.Rank = 1;
	cConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	  if (HAL_ADC_ConfigChannel(&hadc1, &cConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }

	uint32_t adcr = 0;

	for(uint8_t i=0; i<numAverage; i++){
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, 1);
		adcr += HAL_ADC_GetValue(&hadc1);
	}
	//HAL_ADC_Stop(&hadc1);

	return (float)adcr/(float)numAverage;
}
