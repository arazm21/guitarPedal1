/*
 * EQ_Peaking.h
 *
 *  Created on: Sep 19, 2024
 *      Author: x1_user
 */

#ifndef INC_EQ_PEAKING_H_
#define INC_EQ_PEAKING_H_

#include <math.h>
#include <stdint.h>


typedef struct{

	float x_1, x_2;
	float y_1, y_2;

	float b0, b1, b2, a0, a1, a2;

	float fs, fc, f_bw, g;

} Peaking;

void EQ_Peaking_Init(Peaking *filt, float fs, float fc, float f_bw, float g);
void EQ_Peaking_Set_Params(Peaking *filt, float fc, float f_bw, float g);
float EQ_Peaking_Update(Peaking *filt, float x);

#endif /* INC_EQ_PEAKING_H_ */
