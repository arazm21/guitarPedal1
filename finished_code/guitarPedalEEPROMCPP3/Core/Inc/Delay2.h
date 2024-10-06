/*
 * MOD_Flanger_1.h
 *
 *  Created on: Sep 20, 2024
 *      Author: x1_user
 */

#ifndef INC_DELAY2_1_H_
#define INC_DELAY2_1_H_

#include <math.h>
#include <stdint.h>

typedef struct{

	/* Sampling rate (Hz) */
	float fs;

	/* Effect depth */

	int maxNx;
	int maxNy;

	int delay;

	int N;
	/* LFO Parameters (Hz) */
	float g;
	float amount;



	/* Delay Lines for samples */
	float* x_ptr;
	float* y_ptr;


	/* Index for Linear buffering */
	int indx;
	int indy;


} delay2;

void delay2_init(delay2 *filt, float fs, float delay, float g,float amount,  float *x_ptr, float *y_ptr, int maxNx, int maxNy);
void delay2_Set_Params(delay2 *filt, float delay, float g,float amount);
float delay2_Update(delay2 *filt, float x);

#endif /* INC_MOD_FLANGER_1_H_ */
