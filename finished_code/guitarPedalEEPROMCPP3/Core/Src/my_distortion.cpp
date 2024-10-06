/*
 * my_distortion.c
 *
 *  Created on: May 16, 2024
 *      Author: admin
 */

#include "my_distortion.h"

void my_disortion_init(my_distortion_struct *mds)
{
	/*
	mds->thresh_low = 1.0f/3.0f;
	mds->thresh_high = 2.0f/3.0f;

	mds->alpha = 2.0f;
	mds->A0 = 1.0f;
	mds->pre_gain = 1.0f;
	mds->gain = 1.0f;

	mds->a1 = -3.0f;
	mds->b1 = 4.0f;
	mds->c1 = -1.0f/3.0f;

	mds->a2 = -mds->a1;
	mds->b2 = mds->b1;
	mds->c2 = -mds->c1;
	*/


	mds->thresh_low = 0.15f;
	mds->thresh_high = 0.8f;

	mds->alpha = 4.0f;
	mds->A0 = 1.7f;
	mds->pre_gain = 1.0f;
	mds->gain = 0.5f;

	mds->a1 = -3.5503f;
	mds->b1 = 5.0651f;
	mds->c1 = -0.0799f;

	mds->a2 = -mds->a1;
	mds->b2 = mds->b1;
	mds->c2 = -mds->c1;
}



float my_distortion_update(my_distortion_struct *mds ,float x)
{
	x = x*mds->pre_gain;

	float abs_x = fabsf(x);
	float y;

	if ( abs_x <= mds->thresh_low ){
		y = mds->alpha * x;
	} else if ( abs_x >= mds->alpha*mds->thresh_low ){
		if(x>0)
			y = mds->A0 * x;
		else
			y = -mds->A0 * x;
		//y = mds->A0*__signbitf(x);
	} else if ( x > mds->thresh_low && x <= mds->alpha*mds->thresh_high ){
		y = mds->a1*x*x + mds->b1*x + mds->c1;
	} else if ( x < -mds->thresh_low && x > -mds->alpha*mds->thresh_high ) {
		y = mds->a2*x*x + mds->b2*x + mds->c2;
	}

	return y*mds->gain;
}


