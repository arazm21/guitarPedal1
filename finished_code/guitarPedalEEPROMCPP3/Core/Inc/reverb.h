/*
 * MOD_Flanger_1.h
 *
 *  Created on: Sep 20, 2024
 *      Author: x1_user
 */

#ifndef INC_REVERB_1_H_
#define INC_REVERB_1_H_

#include <math.h>
#include <stdint.h>
#include "CF.h"
#include "Delay2.h"

#define g1 0.805
#define g2 0.827
#define g3 0.783
#define g4 0.764
#define AP_g 0.7

typedef struct{

	/* Sampling rate (Hz) */
	float fs;

	/* Effect depth */

	int maxN;
	int delay_s;

	/* LFO Parameters (Hz) */
	float amount;



	/* Delay Lines for samples */
	float* x_ptr;

	float *yCF1, *yCF2,*yCF3,*yCF4;
	float *yAP1,*yAP2,*yAP3;

	CF CF1,CF2,CF3,CF4;

	delay2 AP1,AP2,AP3;


} reverb;

void reverb_init(reverb *filt, float fs, float delay_s,float amount,  float *x_ptr, int maxN,CF* CF1,
		CF* CF2,CF* CF3,CF* CF4,delay2* AP1,delay2* AP2,delay2* AP3, float *yCF1, float *yCF2, float *yCF3,
		float *yCF4, float *yAP1, float *yAP2, float *yAP3);
void reverb_Set_Params(reverb *filt, float delay,float amount);
float reverb_Update(reverb *filt, float x);

#endif /* INC_MOD_FLANGER_1_H_ */
