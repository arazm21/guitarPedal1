/*
 * EQ_Peaking.c
 *
 *  Created on: Sep 19, 2024
 *      Author: x1_user
 */

#include "EQ_Peaking.h"

void EQ_Peaking_Init(Peaking *filt, float fs, float fc, float f_bw, float g){

	/* Set up filter parameters */
	filt->fs = fs;
	filt->fc = fc;
	filt->f_bw = f_bw;
	filt->g = g;
	EQ_Peaking_Set_Params(filt, filt->fc, filt->f_bw, filt->g);

	/* Clear first samples */
	filt->x_1=0.0f;
	filt->x_2=0.0f;
	filt->y_1=0.0f;
	filt->y_2=0.0f;
}

void EQ_Peaking_Set_Params(Peaking *filt, float fc, float f_bw, float g){
	/* Input fc, f_bw and g are assumed to be [0 - 100] */

	/* fc is mapped to [80 - 2,000] Hz, g to [0.01 - 4.0], f_bw is relative to fc */
	filt->fc = 80.0f + 19.2f*fc;
	filt->f_bw = f_bw/100.0f * filt->fc*0.5f + 0.1f; // +0.1 to avoid bandwidth being zero.
	filt->g = 0.05f + 0.0395f*g;

	float Q = filt->fc/filt->f_bw;
	float wc_T = 2*tanf(M_PI*filt->fc/filt->fs);
	float wc_T2 = wc_T*wc_T;

	filt->b0 = 4.0f + 2.0f * (filt->g/Q) * wc_T + wc_T2;
	filt->b1 = 2.0f * wc_T2 - 8.0f;
	filt->b2 = 4.0f - 2.0f * (filt->g/Q) * wc_T + wc_T2;
	filt->a0 = 4.0f + 2.0f * 1/Q * wc_T + wc_T2;
	filt->a1 = 2.0f * wc_T2 - 8.0f;
	filt->a2 = 4.0f - 2.0f * 1.0f/Q * wc_T + wc_T2;

}

float EQ_Peaking_Update(Peaking *filt, float x){

	/* Calculate output */
	float y = filt->b0*x + filt->b1*filt->x_1 + filt->b2*filt->x_2 - filt->a1*filt->y_1 - filt->a2*filt->y_2;
	y = y/filt->a0;

	/* Update previous samples */
	filt->x_2 = filt->x_1;
	filt->x_1 = x;
	filt->y_2 = filt->y_1;
	filt->y_1 = y;

	/* Return output */
	return y;
}



