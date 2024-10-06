/*
 * arctan_overdrive.c
 *
 *  Created on: May 16, 2024
 *      Author: admin
 */
#include "Atan_Overdrive.h"

void Atan_Overdrive_init(Atan_OD* atan_od, float gain){
	Atan_Overdrive_Set_Gain(atan_od, gain);
}

void Atan_Overdrive_Set_Gain(Atan_OD* atan_od, float gain){
	// gain is mapped from 1 to 128
	float exp = (7.0f / 100.0f) * gain/4.0f;
	atan_od->gain = powf(2, exp);
}

float Atan_Overdrive_update(Atan_OD* atan_od, float x){
	return atan_od->gain * (2/M_PI) * atanf(x*atan_od->gain);
}
