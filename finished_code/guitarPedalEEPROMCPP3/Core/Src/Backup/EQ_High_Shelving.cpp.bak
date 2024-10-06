/*
 * EQ_High_Shelving.c
 *
 *  Created on: Jun 29, 2024
 *      Author: admin
 */

#include <EQ_High_Shelving.h>

void EQ_High_Shelving_Init(HS *filt, float fs, float fc, float V0){

	/* Check bounds for inputs */
//	EQ_High_Shelving_Check_Bounds(filt, fc, V0);

	/* Set up filter parameters */
	filt->fs = fs;
	filt->fc = fc;
	filt->V0 = V0;
	EQ_High_Shelving_Set_Params(filt, filt->fc, filt->V0);

	/* Clear first samples */
	filt->x_1=0.0f;
	filt->x_2=0.0f;
	filt->y_1=0.0f;
	filt->y_2=0.0f;
}

// No longer in use because of [0-100] inputs
void EQ_High_Shelving_Check_Bounds(HS *filt, float fc, float V0){

	if(V0 > GAIN_MAX) V0 = 10.0f;
	if(V0 < GAIN_MIN) V0 = 0.1f;
	if(fc > FC_MAX) fc = 300.0f;
	if(fc < FC_MIN) fc = 20.0f;

	filt->V0 = V0;
	filt->fc = fc;
}

void EQ_High_Shelving_Set_Params(HS *filt, float fc, float V0){

	//EQ_High_Shelving_Check_Bounds(filt, fc, V0);


	// fc, V0 = [0 - 100] input get mapped to [1,000-10,000] Hz and [0.1-4] respectively
	fc = 1000.0f + 90.0f*fc;
	V0 = 0.1f + 0.039f*V0;


	float K = tanf(M_PI*filt->fc/filt->fs);
	float K_2 = K*K;

	if(filt->V0 >= 1){ // BOOST

		float den = 1.0f + sqrt(2.0f)*K + K_2;
		filt->b0 = (filt->V0 + sqrtf(2.0f*filt->V0)*K + K_2)/den;
		filt->b1 = 2.0f*(K_2 - filt->V0)/den;
		filt->b2 = (filt->V0 - sqrtf(2*filt->V0)*K + K_2)/den;
		filt->a1 = 2.0f*(K_2 - 1)/den;
		filt->a2 = (1.0f - sqrtf(2)*K + K_2)/den;

	} else { // CUT

		float den = 1.0f + sqrtf(2.0f*filt->V0)*K + K_2;
		filt->b0 = filt->V0*(1.0f + sqrt(2)*K + K_2)/den;
		filt->b1 = 2.0f*filt->V0*(K_2 - 1)/den;
		filt->b2 = filt->V0*(1.0f - sqrt(2.0f)*K + K_2)/den;
		filt->a1 = 2.0f*(filt->V0*K_2 - 1.0f)/den;
		filt->a2 = (1.0f - sqrt(2.0f*filt->V0)*K + filt->V0*K_2)/den;

	}
}

float EQ_High_Shelving_Update(HS *filt, float x){

	/* Calculate output */
	float y = filt->b0*x + filt->b1*filt->x_1 + filt->b2*filt->x_2 - filt->a1*filt->y_1 - filt->a2*filt->y_2;

	/* Update stored samples */
	filt->x_2 = filt->x_1;
	filt->x_1 = x;
	filt->y_2 = filt->y_1;
	filt->y_1 = y;

	/* Return output */
	return y;
}
