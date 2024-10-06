/*
 * MOD_Flanger_1.c
 *
 *  Created on: Sep 20, 2024
 *      Author: x1_user
 */

#include "MOD_Flanger_1.h"

/* User Provides Array Large Enough for Maximum Delay (1 ms) - 192 for Fs = 96kHz */
void MOD_Flanger_1_init(Flanger_1 *filt, float fs, float depth, float rate, float FB, float FF, float BL, float *x_ptr, float *y_ptr, int len){

	/* Set up Delays Lines */
	filt->x_ptr = x_ptr;
	filt->y_ptr = y_ptr;

	/* Set Parameters */
	filt->fs = fs;
	MOD_Flanger_1_Set_Params(filt, depth, rate, FB, FF, BL);

	/* Clear First Samples */
	filt->n = 0;
	filt->ind = 0;
	for(int i=0; i<len; i++){
			x_ptr[i] = 0.0f;
			y_ptr[i] = 0.0f;
	}
}

/* Every Input is assumed to be in the range [0 - 100] */
void MOD_Flanger_1_Set_Params(Flanger_1 *filt, float depth, float rate, float FB, float FF, float BL){


	/* mapped to [0.1 - 1] Hz */
	filt->f_lfo = 0.1 + 0.009*rate;


	/* mapped to [0 - 2] ms */
	filt->depth = depth * 0.02f;

	/* Factor for LFO max delay in discrete domain */
	filt->M0 = roundf( .0005f * filt->depth * filt->fs );

}

float MOD_Flanger_1_Update(Flanger_1 *filt, float x){

	/* Keep track of n for LFO generation */
	if( filt->f_lfo*filt->n >= filt->fs ){
		filt->n = 0;
	}

	/* Get Number of Delays */
	float O = filt->M0 * ( 1 + sinf(2*M_PI*filt->f_lfo*filt->n/filt->fs) );
	int O_floor = (int)floorf(O);
	int O_ceil = (int)ceilf(O);
	float frac = O - O_floor;

	/* Calculate indexes For Circ Buffer */
	int ind_ceil = (filt->ind - O_ceil + 2*filt->M0) % (2*filt->M0);
	int ind_floor = (filt->ind - O_floor + 2*filt->M0) % (2*filt->M0);

	/* Calculate Fractional Delays Using Linear Inperpolation */
	float x_frac_del = (1.0f - frac)* filt->x_ptr[ind_floor] + frac*filt->x_ptr[ind_ceil];
	float y_frac_del = (1.0f - frac)* filt->y_ptr[ind_floor] + frac*filt->y_ptr[ind_ceil];

    /* Difference Equation: y(i) = BL*x(i) + (FF-BL*FB)*x_frac_delay + FB*y_frac_delay; */
	float y = filt->BL*x + (filt->FF - filt->BL*filt->FB)*x_frac_del + filt->FB*y_frac_del;

	/* Save Samples in Delay Lines */
	filt->x_ptr[filt->ind] = x;
	filt->y_ptr[filt->ind] = y;

	/* Keep Track of n for LFO Generation and ind for Circ Buffer */
	filt->n++;
	filt->ind++;
	if(filt->ind == 2*filt->M0)
		filt->ind = 0;

	return y;
}


