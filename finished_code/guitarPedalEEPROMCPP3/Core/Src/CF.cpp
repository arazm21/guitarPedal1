/*
 * MOD_Flanger_1.c
 *
 *  Created on: Sep 20, 2024
 *      Author: x1_user
 */

#include "CF.h"

/* User Provides Array Large Enough for Maximum Delay (1 ms) - 192 for Fs = 96kHz */
void CF_init(CF *filt, float fs, float delay, float g,float amount, float *y_ptr, int maxNy){

	/* Set up Delays Lines */
		filt->y_ptr = y_ptr;

		/* Set Parameters */
		filt->fs = fs;
		filt->maxNy = maxNy;

		CF_Set_Params(filt, delay, g, amount);

		/* Clear First Samples */
		filt->indy = 0;

}

/* Every Input is assumed to be in the range [0 - 100] */
void CF_Set_Params(CF *filt, float delay, float g,float amount){


	/* mapped to [0-200ms]  */

	    filt->delay = delay/500.f;
		/* mapped to [0 - 2] ms */
	    filt->N = filt->delay*filt->fs;
	    filt->g = g/50-1;
	    filt->amount = amount/100;

		/* Factor for LFO max delay in discrete domain */



}

float CF_Update(CF *filt, float x){





	/* Calculate indexes For delay for Circ Buffer */


	int charIndexY = (filt->indy + filt->maxNy - filt->N)%filt->maxNy;

	/* Difference Equation */
    float y = x + *(filt->y_ptr+charIndexY) * filt->g;


	/* Save Samples in Delay Lines */
	filt->y_ptr[filt->indy] = y;

	/* Keep Track of n for LFO Generation and ind for Circ Buffer */
	filt->indy++;
	if(filt->indy >= filt->maxNy)
			filt->indy = 0;

	return y;
}


