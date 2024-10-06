/*
 * my_distortion.h
 *
 *  Created on: May 16, 2024
 *      Author: admin
 */

#ifndef INC_MY_DISTORTION_H_
#define INC_MY_DISTORTION_H_

#include <math.h>
#include <stdint.h>

typedef struct {
	float thresh_low;
	float thresh_high;

	float alpha;
	float A0;
	float pre_gain;
	float gain;

	float a1, b1, c1;
	float a2, b2, c2;

} my_distortion_struct;

void my_disortion_init(my_distortion_struct *mds);
float my_distortion_update(my_distortion_struct *mds ,float x);


#endif /* INC_MY_DISTORTION_H_ */
