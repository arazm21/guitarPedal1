/*
 * MOD_Flanger_1.h
 *
 *  Created on: Sep 20, 2024
 *      Author: x1_user
 */

#ifndef INC_CF_1_H_
#define INC_CF_1_H_

#include <math.h>
#include <stdint.h>

typedef struct{

	/* Sampling rate (Hz) */
		float fs;

		/* Effect depth */

		int maxNy;

		int delay;

		int N;
		/* LFO Parameters (Hz) */
		float g;
		float amount;

		/* Delay Lines for samples */
		float* y_ptr;

		/* Index for Linear buffering */
		int indy;


} CF;

void CF_init(CF *filt, float fs, float delay, float g,float amount, float *y_ptr, int maxNy);
void CF_Set_Params(CF *filt, float delay, float g,float amount);
float CF_Update(CF *filt, float x);

#endif /* INC_MOD_FLANGER_1_H_ */
