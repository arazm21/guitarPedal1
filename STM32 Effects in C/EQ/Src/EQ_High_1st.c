/*
 * First_Order_High_Pass.c
 *
 *  Created on: Jun 11, 2024
 *      Author: admin
 */
#include <EQ_High_1st.h>

void EQ_High_1st_Init(HP_1st *filt, float fs, float fc){

	/* Set up filter parameters */
	filt->fs = fs;
	filt->fc = fc;
	EQ_High_1st_Set_Parameters(filt, filt->fc);

	/* Clear buffers */
	for(uint8_t i=0; i<2; i++){
		filt->x[i]=0;
		filt->y[i]=0;
	}
}

void EQ_High_1st_Set_Parameters(HP_1st *filt, float fc){
	// input fc is assumed to be [0 - 100]

	// fc is mapped to [20 - 2,000] Hz
	filt->fc = 20.0f + 19.8f*fc;

	/* Warping */
	float wc_T = 2.0f * tanf(M_PI*filt->fc/filt->fs);

	/* Calculate up filter Coefficients */
	filt->b[0] = 2.0f;
	filt->b[1] = -2.0f;

	filt->a[0] = 2.0f + wc_T;
	filt->a[1] = -2.0f + wc_T;
}

float EQ_High_1st_Update(HP_1st *filt, float x){

	/* Shift samples */
	filt->x[1] = filt->x[0];
	filt->x[0] = x;
	filt->y[1] = filt->y[0];

	/* Compute Output */
	filt->y[0] = filt->b[0]*filt->x[0] + filt->b[1]*filt->x[1] - filt->a[1]*filt->y[1];
	filt->y[0] /= filt->a[0];

	return filt->y[0];
}

