/*
 * EQ_Low_Shelving.c
 *
 *  Created on: Jun 19, 2024
 *      Author: admin
 */

#include "EQ_Low_Shelving.h"


void EQ_Low_Shelving_Init(LS *filt, float fs, float fc, float V0){

	/* Check bounds for inputs */
//	EQ_Low_Shelving_Check_Bounds(filt, fc, V0);

	/* Set up filter parameters */
	filt->fc = fc;
	filt->V0 = V0;
	EQ_Low_Shelving_Set_Params(filt, fs, filt->fc, filt->V0);

	/* Clear first samples */
	filt->x_1=0.0f;
	filt->x_2=0.0f;
	filt->y_1=0.0f;
	filt->y_2=0.0f;
}

// No longer in use because of [0-100] inputs
void EQ_Low_Shelving_Check_Bounds(LS *filt, float fc, float V0){

	if(V0 > 10.0f) V0 = 10.0f;
	if(V0 < 0.1f) V0 = 0.1f;
	if(fc > 300.0f) fc = 300.0f;
	if(fc < 20.0f) fc = 20.0f;

	filt->V0 = V0;
	filt->fc = fc;

}


void EQ_Low_Shelving_Set_Params(LS *filt, float fs, float fc, float V0){

	//EQ_Low_Shelving_Check_Bounds(filt, fc, V0);

	// fc, V0 = [0 - 100] input get mapped to [80-300] Hz and [0.1-4] respectively
	fc = 80.0f + 2.2f*fc;
	V0 = 0.1f + 0.039f*V0;


	float K = tanf(M_PI*filt->fc/fs);
	float K_2 = K*K;

	if(filt->V0 >= 1){ // BOOST

		float den = 1.0f + sqrt(2.0f)*K + K_2;
	    filt->b0 = (1 + sqrtf(2*filt->V0)*K + filt->V0*K_2)/den;
	    filt->b1 = 2.0f*(filt->V0*K_2 - 1)/den;
	    filt->b2 = (1 - sqrtf(2*filt->V0)*K + filt->V0*K_2)/den;
	    filt->a1 = 2.0f*(K_2 - 1)/den;
	    filt->a2 = (1 - sqrtf(2)*K + K_2)/den;

	} else { // CUT

		float den = filt->V0 + sqrtf(2*filt->V0)*K + K_2;
		filt->b0 = filt->V0*(1 + sqrt(2)*K + K_2)/den;
		filt->b1 = 2*filt->V0*(K_2 - 1)/den;
		filt->b2 = filt->V0*(1 - sqrt(2)*K + K_2)/den;
		filt->a1 = 2*(K_2 - filt->V0)/den;
		filt->a2 = (filt->V0 - sqrt(2*filt->V0)*K + K_2)/den;

	}
}

float EQ_Low_Shelving_Update(LS *filt, float x){
	/* Calculate output */
	float y = filt->b0*x + filt->b1*filt->x_1 + filt->b2*filt->x_2 - filt->a1*filt->y_1 - filt->a2*filt->y_2;

	/* Update previous samples */
	filt->x_2 = filt->x_1;
	filt->x_1 = x;
	filt->y_2 = filt->y_1;
	filt->y_1 = y;

	/* Return output */
	return y;
}

