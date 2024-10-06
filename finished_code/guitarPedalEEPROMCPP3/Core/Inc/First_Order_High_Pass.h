/*
 * First_Order_High_Pass.h
 *
 *  Created on: Jun 11, 2024
 *      Author: admin
 */

#ifndef INC_FIRST_ORDER_HIGH_PASS_H_
#define INC_FIRST_ORDER_HIGH_PASS_H_

#include <math.h>
#include <stdint.h>

typedef struct{

	float x[2];
	float y[2];

	float b[2];
	float a[2];

} High_1st;

void First_Order_High_Init(High_1st *filt, float fc, float fs);
void First_Order_High_Set_Parameters(High_1st *filt, float fc, float fs);
float First_Order_High_Update(High_1st *filt, float x);


#endif /* INC_FIRST_ORDER_HIGH_PASS_H_ */
