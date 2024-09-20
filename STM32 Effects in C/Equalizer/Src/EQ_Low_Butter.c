/*
 * EQ_Low_Butter.c
 *
 *  Created on: Sep 19, 2024
 *      Author: x1_user
 */

#include "EQ_Low_Butter.h"

void EQ_Low_Butter_Init(LP_Butter *filt, float fs, float fc){


	/* Set up filter parameters */
	filt->fc = fc;
	filt->fs = fs;
	EQ_Low_Butter_Set_Params(filt, filt->fc);

	/* Clear first samples */
	filt->x_1=0.0f;
	filt->x_2=0.0f;
	filt->y_1=0.0f;
	filt->y_2=0.0f;
}


void EQ_Low_Butter_Set_Params(LP_Butter *filt, float fc){

	/* Input fc is assumed to be [0 - 100] */
	/* fc [0-100] is mapped to [80 - 4,000] Hz */
	filt->fc = 80.0f + 39.2f*fc;

	float wc_T = 2*tanf(M_PI*filt->fc/filt->fs);
	float wc_T_2 = wc_T*wc_T;

	filt->b0 = wc_T_2;
	filt->b1 = filt->b0*2.0f;
	filt->b2 = filt->b0;
	filt->a0 = 4.0f + 2.0f*wc_T/Q + wc_T_2;
	filt->a1 = -8.0f + 2.0f*wc_T_2;
	filt->a2 =  4.0f - 2.0f*wc_T/Q + wc_T_2;
}


float EQ_Low_Butter_Update(LP_Butter *filt, float x){

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

