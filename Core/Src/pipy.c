/*
 * pipy.c
 *
 *  Created on: Jan 24, 2021
 *      Author: Dell
 */



#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "pipy.h"
#include <stdint.h>

//when changing the size of the buffer, make sure you change the size of the arrays in FIRFilter_Init and
//FIRFilter_Filter, in pipy.h change the half_period

float* FIRFilter_Init() {

	static float filtered_output[32];

	for (uint8_t n=0; n< 32; n++) { // clear filter buffer
		filtered_output[n] = 0.0f;
	}
	return filtered_output;
}


float* FIRFilter_Filter(float filter[], float raw_data[]) {

	int n = 32;
	static float filtered_output[32];

	for (uint8_t j = 0; j< 32; j++) {

		filtered_output[j] = filter[j] * raw_data[n-j];
		n--;
	}
	return filtered_output;
}


float Theta(float phi) {

	return asin(phi/pi);
}

// cross correlation
float CrossCorrelation(float signal_1[], float signal_2[],float p_sum[], float sum[]) {
	int n = 0;
	int r = 0;
	float part_total=0;     // product of two samples
	float total = 0;        // sum of all products in a batch
	float phase_shift = 0; //phase shift in rads
	float samples_p_cycle = 2 * half_period; //samples per cycle (2* half_period)
	int maxIdx = 0;

	printf(" %s \n", "Here are the totals");
	while (r <= (len_signal - half_period )) {   // shift the first signal and cross-correlate
		while (n <=  half_period) {  // cycle over each pair of samples, multiply, and store
			part_total = signal_1[n] * signal_2[n+r];
			p_sum[n+r*(half_period+1)] = part_total;
			n++;
		}

		total = DoSum(r, p_sum); // adds all the terms in p_sum
		sum[r] = total ;// store the total

		total = 0;
		n = 0;
		r++;
		}

	float max_so_far = FindMax(sum);

	maxIdx = IndexFind(sum, max_so_far );  // find its index

	phase_shift = 2*pi- (maxIdx * 2 * pi)/samples_p_cycle;
	return phase_shift;
}


float DoSum(int r, float p_sum[]) {
	float total =0;
	int n = 0;
	while ( n <= half_period) {
		total = total + p_sum[n+r*(half_period+1)];
		n++;
	}
	return total;
}

float FindMax(float sum[]){
	int n = 0;
	float max_so_far = sum[0];

	while ( n < len_signal - half_period+1 ) {

				if ( max_so_far < sum[n]) { // compares this node with max_so_far
					max_so_far = sum[n];  //if this node is superior, it becomes max_sofar
				}
				n++;
			}
	return max_so_far;
}

int IndexFind(float sum[], float max_so_far) { // @suppress("No return")

	int i = 0;
	while ( i < len_signal - half_period+1) { // traverse through the tree to find the index matching this value
		if (sum[i] == max_so_far) {
			return i;
		}
		i++;
	}
	return 0;
}
