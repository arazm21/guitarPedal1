/*
 * MOD_Flanger_1.c
 *
 *  Created on: Sep 20, 2024
 *      Author: x1_user
 */

#include "Delay.h"

/* User Provides Array Large Enough for Maximum Delay (1 ms) - 192 for Fs = 96kHz */
void delay_init(delay *filt, float fs, float delay, float g,float amount,  float *x_ptr, float *y_ptr, int maxN){

	/* Set up Delays Lines */
	filt->x_ptr = x_ptr;
	filt->y_ptr = y_ptr;

	/* Set Parameters */
	filt->fs = fs;
	filt->maxN = maxN;
	delay_Set_Params(filt, delay, g, amount);

	/* Clear First Samples */
	filt->ind = 0;

}

/* Every Input is assumed to be in the range [0 - 100] */
void delay_Set_Params(delay *filt, float delay, float g,float amount){


	/* mapped to [0-200ms]  */

    filt->delay = delay/500.f;
	/* mapped to [0 - 2] ms */
    filt->N = filt->delay*filt->fs;
    filt->g = g/50-1;
    filt->amount = amount/100;

	/* Factor for LFO max delay in discrete domain */


}


float delay_Update(delay *filt, float x){





	/* Calculate indexes For delay for Circ Buffer */


	int charIndex = (filt->ind + filt->maxN - filt->N)%filt->maxN;
	/* Difference Equation */
    float y = filt->g * x + *(filt->x_ptr+charIndex) + *(filt->y_ptr+charIndex) * filt->g;


	/* Save Samples in Delay Lines */
    filt->x_ptr[filt->ind] = x;
	filt->y_ptr[filt->ind] = y;

	/* Keep Track of n for LFO Generation and ind for Circ Buffer */
	filt->ind++;
	if(filt->ind == filt->maxN)
		filt->ind = 0;

	return y;
}


