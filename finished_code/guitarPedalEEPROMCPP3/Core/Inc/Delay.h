/*
 * MOD_Flanger_1.h
 *
 *  Created on: Sep 20, 2024
 *      Author: x1_user
 */

#ifndef INC_DELAY_1_H_
#define INC_DELAY_1_H_

#include <math.h>
#include <stdint.h>

typedef struct{

	/* Sampling rate (Hz) */
	float fs;

	/* Effect depth */

	int maxN;
	int delay;

	int N;
	/* LFO Parameters (Hz) */
	float g;
	float amount;



	/* Delay Lines for samples */
	float* x_ptr;
	float* y_ptr;


	/* Index for Linear buffering */
	int ind;


} delay;

void delay_init(delay *filt, float fs, float delay, float g,float amount,  float *x_ptr, float *y_ptr, int maxN);
void delay_Set_Params(delay *filt, float delay, float g,float amount);
float delay_Update(delay *filt, float x);

#endif /* INC_MOD_FLANGER_1_H_ */
