/*
 * First_Order_High_Pass.c
 *
 *  Created on: Jun 11, 2024
 *      Author: admin
 */
#include "First_Order_High_Pass.h"

void First_Order_High_Init(High_1st *filt, float fc, float fs){

	First_Order_High_Set_Parameters(filt, fc, fs);

	/* Clear buffers */
	for(uint8_t i=0; i<2; i++){
		filt->x[i]=0;
		filt->y[i]=0;
	}

}

void First_Order_High_Set_Parameters(High_1st *filt, float fc, float fs){

	/* set fc - (0 - 100). cutoff frequency  (20 - 2k) Hz */
	fc = 20 + 198*fc/100;

	/* Warping */
	float wc_T = 2.0f * tanf(M_PI*fc/fs);

	/* Calculate up filter Coefficients */
	filt->b[0] = 2.0f;
	filt->b[1] = -2.0f;

	filt->a[0] = 2.0f + wc_T;
	filt->a[1] = -2.0f + wc_T;
}

float First_Order_High_Update(High_1st *filt, float x){

	/* Shift samples */
	filt->x[1] = filt->x[0];
	filt->x[0] = x;
	filt->y[1] = filt->y[0];

	/* Compute Output */
	filt->y[0] = filt->b[0]*filt->x[0] + filt->b[1]*filt->x[1] - filt->a[1]*filt->y[1];
	filt->y[0] /= filt->a[0];

	return filt->y[0];
}

