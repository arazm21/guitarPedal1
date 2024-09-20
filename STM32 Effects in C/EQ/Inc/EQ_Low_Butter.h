/*
 * EQ_Low_Butter_2nd.h
 *
 *  Created on: Sep 19, 2024
 *      Author: x1_user
 */

#ifndef INC_EQ_LOW_BUTTER_H_
#define INC_EQ_LOW_BUTTER_H_

#include <math.h>
#include <stdint.h>

#define Q 0.7071f

typedef struct{

	float x_1, x_2;
	float y_1, y_2;

	float b0, b1, b2, a0, a1, a2;

	float fc;
	float fs;

} LP_Butter;

void EQ_Low_Butter_Init(LP_Butter *filt, float fs, float fc);
void EQ_Low_Butter_Set_Params(LP_Butter *filt, float fc);
float EQ_Low_Butter_Update(LP_Butter *filt, float x);

#endif /* INC_EQ_LOW_BUTTER_H_ */
