/*
 * First_Order_High_Pass.h
 *
 *  Created on: Jun 11, 2024
 *      Author: admin
 */

#ifndef INC_EQ_HIGH_1ST_H_
#define INC_EQ_HIGH_1ST_H_

#include <math.h>
#include <stdint.h>

typedef struct{

	float x[2];
	float y[2];

	float b[2];
	float a[2];

	float fs, fc;

} HP_1st;

void EQ_High_1st_Init(HP_1st *filt, float fs, float fc);
void EQ_High_1st_Set_Parameters(HP_1st *filt, float fc);
float EQ_High_1st_Update(HP_1st *filt, float x);


#endif /* INC_EQ_HIGH_1ST_H_ */
