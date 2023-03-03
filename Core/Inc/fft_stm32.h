/*
	FFT libray
	Copyright (C) 2010 Didier Longueville
	Copyright (C) 2014 Enrique Condes
	Modified for STM32 by Felix Centeno, 2022

	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

#ifndef INC_FFT_STM32_H_
#define INC_FFT_STM32_H_

	#include <stdlib.h>
	#include <stdio.h>
	#include <math.h>

	#define FFT_LIB_REV 0x14
	/* Custom constants */
	#define FFT_FORWARD 0x01
	#define FFT_REVERSE 0x00

	#define SCL_INDEX 0x00
	#define SCL_TIME 0x01
	#define SCL_FREQUENCY 0x02
	#define SCL_PLOT 0x03

	/* Windowing type */
	#define FFT_WIN_TYP_RECTANGLE 0x00 /* rectangle (Box car) */
	#define FFT_WIN_TYP_HAMMING 0x01 /* hamming */
	#define FFT_WIN_TYP_HANN 0x02 /* hann */
	#define FFT_WIN_TYP_TRIANGLE 0x03 /* triangle (Bartlett) */
	#define FFT_WIN_TYP_NUTTALL 0x04 /* nuttall */
	#define FFT_WIN_TYP_BLACKMAN 0x05 /* blackman */
	#define FFT_WIN_TYP_BLACKMAN_NUTTALL 0x06 /* blackman nuttall */
	#define FFT_WIN_TYP_BLACKMAN_HARRIS 0x07 /* blackman harris*/
	#define FFT_WIN_TYP_FLT_TOP 0x08 /* flat top */
	#define FFT_WIN_TYP_WELCH 0x09 /* welch */

	/*Mathematial constants*/
	#define PI	3.14159265359
	#define twoPi 6.28318531
	#define fourPi 12.56637061
	#define sixPi 18.84955593



	//Math
	#define sq(x) ((x)*(x))
	#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))


	 struct Struct_FFT
	 {
		double *vReal;
		double *vImag;
		uint16_t samples;	//This value MUST ALWAYS be a power of 2
		double samplingFrequency;
	 };

	/* Functions */
	//uint8_t Revision(void);
	uint8_t Exponent(uint16_t value);
	void ComputeFFT(double *vReal, double *vImag, uint16_t samples, uint8_t dir);
	void DCRemoval(double *vData, uint16_t samples);
	void DCRemovalST(void);
	double MajorPeakST(void);
	double MajorPeak(double *vD, uint16_t samples, double samplingFrequency);
	void Windowing(double *vData, uint16_t samples, uint8_t windowType, uint8_t dir);
	void ComplexToMagnitude(double *vReal, double *vImag, uint16_t samples);
	void ComplexToMagnitudeST();

	//void Windowing(uint8_t windowType, uint8_t dir);

	void Swap(double *x, double *y);
	void PrintVector(double *vData, uint16_t bufferSize, uint8_t scaleType, uint16_t samples, double samplingFrequency);

#endif /* INC_FFT_STM32_H_ */
