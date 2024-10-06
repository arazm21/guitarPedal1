/*
 * cubic_overdrive.c
 *
 *  Created on: May 16, 2024
 *      Author: admin
 */

#include "cubic_overdrive.h"


void cubic_overdrive_init(cubic_overdrive* cubic_OD, float level, float fact_ind)
{
	for(uint8_t i=0; i<FACTORS_SIZE; i++)
		cubic_OD->factors[i] = 3 + 2*i;
	cubic_OD->level = level/100;
	cubic_OD->fact_ind = roundf( fact_ind / 100.0f * 5.0f );
}

float cubic_overdrive_update(cubic_overdrive* cubic_OD, float x)
{
	/*float x_pow = x;
	for(int i=0; i<cubic_OD->factors[cubic_OD->factor_ind]-1; i++)
		x_pow = x_pow*x;*/

	uint8_t factor = cubic_OD->factors[(int) cubic_OD->fact_ind];

	float x_pow = pow(x, factor);
	float scaler = 1.0f/factor;

	return x - cubic_OD->level * scaler * x_pow;
}
