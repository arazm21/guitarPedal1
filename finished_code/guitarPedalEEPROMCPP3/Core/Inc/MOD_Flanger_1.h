/*
 * MOD_Flanger_1.h
 *
 *  Created on: Sep 20, 2024
 *      Author: x1_user
 */

#ifndef INC_MOD_FLANGER_1_H_
#define INC_MOD_FLANGER_1_H_

#include <math.h>
#include <stdint.h>

typedef struct{

	/* Sampling rate (Hz) */
	float fs;

	/* Effect depth */
	float depth;

	/* LFO Parameters (Hz) */
	float f_lfo; /* LFO frequency */
	int n; /* Counter index for LFO */

	/* Feedback, FeedForward and Dry */
	float FB, FF, BL;

	/* Delay Lines for samples */
	float* x_ptr;
	float* y_ptr;

	/* Number of max Delays (Half) */
	int M0;

	/* Index for Linear buffering */
	int ind;


} Flanger_1;

void MOD_Flanger_1_init(Flanger_1 *filt, float fs, float depth, float rate, float FB, float FF, float BL, float *x_ptr, float *y_ptr, int len);
void MOD_Flanger_1_Set_Params(Flanger_1 *filt, float depth, float rate, float FB, float FF, float BL);
float MOD_Flanger_1_Update(Flanger_1 *filt, float x);

#endif /* INC_MOD_FLANGER_1_H_ */
