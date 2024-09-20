/*
 * EQ_High_Shelving.h
 *
 *  Created on: Jun 29, 2024
 *      Author: admin
 */

#ifndef INC_EQ_HIGH_SHELVING_H_
#define INC_EQ_HIGH_SHELVING_H_

#include <math.h>
#include <stdint.h>

#define GAIN_MAX 10.0f
#define GAIN_MIN 0.1f
#define FC_MAX 20000.0f
#define FC_MIN 3000.0f

typedef struct{

	float x_1, x_2;
	float y_1, y_2;

	float b0, b1, b2, a1, a2;

	float V0;
	float fc;
	float fs;

} HS;

void EQ_High_Shelving_Init(HS *filt, float fs, float fc, float V0);
void EQ_High_Shelving_Check_Bounds(HS *filt, float fc, float V0);
void EQ_High_Shelving_Set_Params(HS *filt, float fc, float V0);
float EQ_High_Shelving_Update(HS *filt, float x);


#endif /* INC_EQ_HIGH_SHELVING_H_ */
