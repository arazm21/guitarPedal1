/*
 * EQ_Low_Shelving.h
 *
 *  Created on: Jun 19, 2024
 *      Author: admin
 */

#ifndef INC_EQ_LOW_SHELVING_H_
#define INC_EQ_LOW_SHELVING_H_

#include <math.h>
#include <stdint.h>

typedef struct{

	float x_1, x_2;
	float y_1, y_2;

	float b0, b1, b2, a1, a2;

	float V0;
	float fc;
	float fs;

} LS;

void EQ_Low_Shelving_Init(LS *filt, float fs, float fc, float V0);
void EQ_Low_Shelving_Set_Params(LS *filt, float fc, float V0);
void EQ_Low_Shelving_Check_Bounds(LS *filt, float fc, float V0);
float EQ_Low_Shelving_Update(LS *filt, float x);

#endif /* INC_EQ_LOW_SHELVING_H_ */
