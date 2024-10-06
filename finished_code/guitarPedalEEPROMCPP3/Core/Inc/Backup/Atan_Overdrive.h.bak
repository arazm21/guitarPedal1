/*
 * arctan_overdrive.h
 *
 *  Created on: May 16, 2024
 *      Author: admin
 */

#ifndef INC_ATAN_OVERDRIVE_H_
#define INC_ATAN_OVERDRIVE_H_

#include <math.h>
#include <stdint.h>

typedef struct {
	float gain; // (0 - 100)
} Atan_OD;


void Atan_Overdrive_init(Atan_OD* atan_od, float gain);
void Atan_Overdrive_Set_Gain(Atan_OD* atan_od, float gain);
float Atan_Overdrive_update(Atan_OD* atan_od, float x);

#endif /* INC_ATAN_OVERDRIVE_H_ */
