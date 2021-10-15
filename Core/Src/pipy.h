/*
 * pipy.h
 *
 *  Created on: Jan 24, 2021
 *      Author: Dell
 */

#ifndef SRC_PIPY_H_
#define SRC_PIPY_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>



float* FIRFilter_Init(); // clears the output array

float* FIRFilter_Filter(float filter[], float raw_data[]);

// parameters
static const float pi = 3.141592;
static const int len_signal = 32;
static const int half_period = 8;

float Theta(float phi); // computes theta from phi

float CrossCorrelation(float signal_1[], float signal_2[],float p_sum[], float sum[]);  // outputs phi

float DoSum(int r, float p_sum[]);  // part of CC

float FindMax(float sum[]); //part of CC

int IndexFind(float sum[], float max_so_far);  //part of CC


#endif /* SRC_PIPY_H_ */
