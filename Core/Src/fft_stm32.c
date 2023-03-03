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

#include "fft_stm32.h"
#include <string.h>
#include <stdio.h>
#include "usbd_cdc_if.h"


// MIN/MAX/ABS macros
	#ifndef MIN
	#define MIN(a,b)			((a<b)?(a):(b))
	#endif

	#ifndef MAX
	#define MAX(a,b)			((a>b)?(a):(b))
	#endif
	#define ABS(x)				((x>0)?(x):(-x))

/* Variables */
uint16_t _samples; //This value MUST ALWAYS be a power of 2
double _samplingFrequency;
double *_vReal;
double *_vImag;
uint8_t _power;


//#define USE_TABLES
#ifdef USE_TABLES

double _c1[]  = {0.0000000000, 0.7071067812, 0.9238795325, 0.9807852804,
															0.9951847267, 0.9987954562, 0.9996988187, 0.9999247018,
															0.9999811753, 0.9999952938, 0.9999988235, 0.9999997059,
															0.9999999265, 0.9999999816, 0.9999999954, 0.9999999989,
															0.9999999997};
double _c2[] = {1.0000000000, 0.7071067812, 0.3826834324, 0.1950903220,
															0.0980171403, 0.0490676743, 0.0245412285, 0.0122715383,
															0.0061358846, 0.0030679568, 0.0015339802, 0.0007669903,
															0.0003834952, 0.0001917476, 0.0000958738, 0.0000479369,
															0.0000239684};
#endif

struct Struct_FFT St_fft;

//St_fft.Compute = &ComputeFFT;

 /******   ComputeFFT ******************************
 * double *vReal: array con muestras reales
 *
 * double *vImag: array con muestras imaginarias
 *
 * uint16_t sample: numero de muestras
 *
 * uint8_t power:
 *
 * uint8_t dir:  FFT_FORWARD = 0x01,  FFT_REVERSE = 0x00
 */
void ComputeFFT(double *vReal, double *vImag, uint16_t samples, uint8_t dir)
{	// Computes in-place complex-to-complex FFT
	// Reverse bits

	uint16_t j = 0;
	uint8_t power = Exponent(samples);
	for (uint16_t i = 0; i < (samples - 1); i++) {
		if (i < j) {
			Swap(&vReal[i], &vReal[j]);
			if(dir==FFT_REVERSE)
				Swap(&vImag[i], &vImag[j]);
		}
		uint16_t k = (samples >> 1);
		while (k <= j) {
			j -= k;
			k >>= 1;
		}
		j += k;
	}
	// Compute the FFT
#ifdef USE_TABLES
	uint8_t indx = 0;
	double *p2, *p1;
#endif
	double c1 = -1.0;
	double c2 = 0.0;
	uint16_t l2 = 1;
	for (uint8_t l = 0; (l < power); l++) {
		uint16_t l1 = l2;
		l2 <<= 1;
		double u1 = 1.0;
		double u2 = 0.0;
		for (j = 0; j < l1; j++) {
			 for (uint16_t i = j; i < samples; i += l2) {
					uint16_t i1 = i + l1;
					double t1 = u1 * vReal[i1] - u2 * vImag[i1];
					double t2 = u1 * vImag[i1] + u2 * vReal[i1];
					vReal[i1] = vReal[i] - t1;
					vImag[i1] = vImag[i] - t2;
					vReal[i] += t1;
					vImag[i] += t2;
			 }
			 double z = ((u1 * c1) - (u2 * c2));
			 u2 = ((u1 * c2) + (u2 * c1));
			 u1 = z;
		}
#ifdef USE_TABLES
		p2 = &_c2[indx];
		p1 = &_c1[indx];
		c2 = *p2;
		c1 = *p1;
		indx++;
#else
		c2 = sqrt((1.0 - c1) / 2.0);
		c1 = sqrt((1.0 + c1) / 2.0);
#endif
		if (dir == FFT_FORWARD) {
			c2 = -c2;
		}
	}
	// Scaling for reverse transform
	if (dir != FFT_FORWARD) {
		for (uint16_t i = 0; i < samples; i++) {
			 vReal[i] /= samples;
			 vImag[i] /= samples;
		}
	}
}


/********************** ComplexToMagnitude *******************************
 *
 * double *vReal:
 *
 * double *vImag:
 *
 * uint16_t samples:
 *
 */
void ComplexToMagnitude(double *vReal, double *vImag, uint16_t samples)
{	// vM is half the size of vReal and vImag

	for (uint16_t i = 0; i < samples; i++) {
		vReal[i] = sqrt(sq(vReal[i]) + sq(vImag[i]));
	}
}

void ComplexToMagnitudeST(void)
{
	// vM is half the size of vReal and vImag
	for (uint16_t i = 0; i < St_fft.samples; i++) {
		St_fft.vReal[i] = sqrt(sq(St_fft.vReal[i]) + sq(St_fft.vImag[i]));
	}
}

/********************* DCRemoval() ***************************
 *
 * double *vData: Array con valores reales
 *
 * uint16_t samples: numero de muestras
 */
void DCRemovalST(void)
{
	// calculate the mean of vData
	double mean = 0;
	for (uint16_t i = 0; i < St_fft.samples; i++)
	{
		mean += St_fft.vReal[i];
	}
	mean /= St_fft.samples;
	// Subtract the mean from vData
	for (uint16_t i = 0; i < St_fft.samples; i++)
	{
		St_fft.vReal[i] -= mean;
	}
}

void DCRemoval(double *vData, uint16_t samples)
{
	// calculate the mean of vData
	double mean = 0;
	for (uint16_t i = 0; i < samples; i++)
	{
		mean += vData[i];
	}
	mean /= samples;
	// Subtract the mean from vData
	for (uint16_t i = 0; i < samples; i++)
	{
		vData[i] -= mean;
	}
}



/******************* Windowing(double *vData, uint16_t samples, uint8_t windowType, uint8_t dir) ******
 *
 * double *vData: array con data real
 *
 * uint16_t samples: numero de muetras
 *
 * uint8_t windowType: tipo de window a aplicar: FFT_WIN_TYP_RECTANGLE, FFT_WIN_TYP_HAMMING
 * 			FFT_WIN_TYP_HANN, FFT_WIN_TYP_TRIANGLE, FFT_WIN_TYP_NUTTALL, FFT_WIN_TYP_BLACKMAN
 *
 * uint8_t dir:
 */
void Windowing(double *vData, uint16_t samples, uint8_t windowType, uint8_t dir)
{
	// Weighing factors are computed once before multiple use of FFT
	// The weighing function is symetric; half the weighs are recorded
	double samplesMinusOne = (double)samples - 1.0;

	for (uint16_t i = 0; i < (samples >> 1); i++) {
		double indexMinusOne = (double)i;
		double ratio = (indexMinusOne / samplesMinusOne);
		double weighingFactor = 1.0;
		// Compute and record weighting factor
		switch (windowType) {
		case FFT_WIN_TYP_RECTANGLE: // rectangle (box car)
			weighingFactor = 1.0;
			break;
		case FFT_WIN_TYP_HAMMING: // hamming
			weighingFactor = 0.54 - (0.46 * cos(twoPi * ratio));
			break;
		case FFT_WIN_TYP_HANN: // hann
			weighingFactor = 0.54 * (1.0 - cos(twoPi * ratio));
			break;
		case FFT_WIN_TYP_TRIANGLE: // triangle (Bartlett)
			#if defined(ESP8266) || defined(ESP32)
			weighingFactor = 1.0 - ((2.0 * fabs(indexMinusOne - (samplesMinusOne / 2.0))) / samplesMinusOne);
			#else
			weighingFactor = 1.0 - ((2.0 * abs(indexMinusOne - (samplesMinusOne / 2.0))) / samplesMinusOne);
			#endif
			break;
		case FFT_WIN_TYP_NUTTALL: // nuttall
			weighingFactor = 0.355768 - (0.487396 * (cos(twoPi * ratio))) + (0.144232 * (cos(fourPi * ratio))) - (0.012604 * (cos(sixPi * ratio)));
			break;
		case FFT_WIN_TYP_BLACKMAN: // blackman
			weighingFactor = 0.42323 - (0.49755 * (cos(twoPi * ratio))) + (0.07922 * (cos(fourPi * ratio)));
			break;
		case FFT_WIN_TYP_BLACKMAN_NUTTALL: // blackman nuttall
			weighingFactor = 0.3635819 - (0.4891775 * (cos(twoPi * ratio))) + (0.1365995 * (cos(fourPi * ratio))) - (0.0106411 * (cos(sixPi * ratio)));
			break;
		case FFT_WIN_TYP_BLACKMAN_HARRIS: // blackman harris
			weighingFactor = 0.35875 - (0.48829 * (cos(twoPi * ratio))) + (0.14128 * (cos(fourPi * ratio))) - (0.01168 * (cos(sixPi * ratio)));
			break;
		case FFT_WIN_TYP_FLT_TOP: // flat top
			weighingFactor = 0.2810639 - (0.5208972 * cos(twoPi * ratio)) + (0.1980399 * cos(fourPi * ratio));
			break;
		case FFT_WIN_TYP_WELCH: // welch
			weighingFactor = 1.0 - sq((indexMinusOne - samplesMinusOne / 2.0) / (samplesMinusOne / 2.0));
			break;
		}
		if (dir == FFT_FORWARD) {
			vData[i] *= weighingFactor;
			vData[samples - (i + 1)] *= weighingFactor;
		}
		else {
			vData[i] /= weighingFactor;
			vData[samples - (i + 1)] /= weighingFactor;
		}
	}
}



/*********************** MajorPeak() *********************************
 *
 *
 *
 */
double MajorPeakST()
{
	double maxY = 0;
	uint16_t IndexOfMaxY = 0;
	//If sampling_frequency = 2 * max_frequency in signal,
	//value would be stored at position samples/2
	for (uint16_t i = 1; i < ((St_fft.samples >> 1) + 1); i++) {
		if ((St_fft.vReal[i-1] < St_fft.vReal[i]) && (St_fft.vReal[i] > St_fft.vReal[i+1])) {
			if (St_fft.vReal[i] > maxY) {
				maxY = St_fft.vReal[i];
				IndexOfMaxY = i;
			}
		}
	}
	double delta = 0.5 * ((St_fft.vReal[IndexOfMaxY-1] - St_fft.vReal[IndexOfMaxY+1]) / (St_fft.vReal[IndexOfMaxY-1] - (2.0 * St_fft.vReal[IndexOfMaxY]) + St_fft.vReal[IndexOfMaxY+1]));
	double interpolatedX = ((IndexOfMaxY + delta)  * St_fft.samplingFrequency) / (St_fft.samples-1);
	if(IndexOfMaxY==(St_fft.samples >> 1)) //To improve calculation on edge values
		interpolatedX = ((IndexOfMaxY + delta)  * St_fft.samplingFrequency) / (St_fft.samples);
	// returned value: interpolated frequency peak apex
	return(interpolatedX);
}


double MajorPeak(double *vD, uint16_t samples, double samplingFrequency)
{
	double maxY = 0;
	uint16_t IndexOfMaxY = 0;
	//If sampling_frequency = 2 * max_frequency in signal,
	//value would be stored at position samples/2
	for (uint16_t i = 1; i < ((samples >> 1) + 1); i++) {
		if ((vD[i-1] < vD[i]) && (vD[i] > vD[i+1])) {
			if (vD[i] > maxY) {
				maxY = vD[i];
				IndexOfMaxY = i;
			}
		}
	}
	double delta = 0.5 * ((vD[IndexOfMaxY-1] - vD[IndexOfMaxY+1]) / (vD[IndexOfMaxY-1] - (2.0 * vD[IndexOfMaxY]) + vD[IndexOfMaxY+1]));
	double interpolatedX = ((IndexOfMaxY + delta)  * samplingFrequency) / (samples-1);
	if(IndexOfMaxY==(samples >> 1)) //To improve calculation on edge values
		interpolatedX = ((IndexOfMaxY + delta)  * samplingFrequency) / (samples);
	// returned value: interpolated frequency peak apex
	return(interpolatedX);
}




uint8_t Exponent(uint16_t value)
{
	// Calculates the base 2 logarithm of a value
	uint8_t result = 0;
	while (((value >> result) & 1) != 1) result++;
	return(result);
}




void Swap(double *x, double *y)
{
	double temp = *x;
	*x = *y;
	*y = temp;
}



void PrintVector(double *vData, uint16_t bufferSize, uint8_t scaleType, uint16_t samples, double samplingFrequency)
{
  char bufferPrt[50];
  memset(bufferPrt, 0, 50);
  for (uint16_t i = 0; i < bufferSize; i++)
  {
    double abscissa;
    /* Print abscissa value */
    switch (scaleType)
    {
      case SCL_INDEX:
        abscissa = (i * 1.0);
	break;
      case SCL_TIME:
        abscissa = ((i * 1.0) / samplingFrequency);
	break;
      case SCL_FREQUENCY:
        abscissa = ((i * 1.0 * samplingFrequency) / samples);
	break;
    }

    if(scaleType==SCL_FREQUENCY){
    	sprintf(bufferPrt,"%f Hz: %f \r\n", abscissa, vData[i]);
    }else{
    	sprintf(bufferPrt,"%f : %f \r\n", abscissa, vData[i]);
    }
    CDC_Transmit_FS(bufferPrt, strlen(bufferPrt));
  }
  memset(bufferPrt, 0, 50);
  sprintf(bufferPrt,"%s","----------------------- \r\n");
  CDC_Transmit_FS(bufferPrt, strlen(bufferPrt));
}


