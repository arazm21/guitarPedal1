/*
 * MOD_Tremolo.c
 *
 *  Created on: Jun 29, 2024
 *      Author: admin
 */

#include <MOD_Tremolo.h>

void MOD_Tremolo_init(Tremolo *trem, float fs, float depth, float rate){

	/* Set up filter parameters */
	trem->fs = fs;
	MOD_Tremolo_Set_Params(trem, depth, rate);

	/* Clear first samples */
	trem->n = 0;
}

void MOD_Tremolo_Set_Params(Tremolo *trem, float depth, float rate){
	// depth and rate inputs are assumed to be [0 - 100]

	// depth gets mapped to [0 - 1.0f], f_lfo to [0.1 - 14] Hz
	trem->depth = 0.01f*depth;
	trem->f_lfo = 0.1 + 0.139f*rate;
}


float MOD_Tremolo_Update(Tremolo *trem, float x){

	if(trem->fs <= trem->f_lfo * trem->n){
		trem->n = 0;
	}

	float y = (1 - trem->depth)*x + trem->depth*x*cosf(2.0f*M_PI* trem->f_lfo * trem->n / trem->fs);
	trem->n++;

	return y;
}
