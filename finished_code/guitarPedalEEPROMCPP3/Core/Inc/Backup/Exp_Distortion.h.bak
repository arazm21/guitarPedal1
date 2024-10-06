/*
 * Exp_Distortion.h
 *
 *  Created on: Jun 11, 2024
 *      Author: admin
 */

#ifndef INC_EXP_DISTORTION_H_
#define INC_EXP_DISTORTION_H_

#include <math.h>
#include <stdint.h>

typedef struct {
	/*
	 * g - Gain, Ammount of Distortion
	 * range 1.0 - inf
	 * 1) 1.0 - 80.0 Overdrive
	 * 2) 80.0-2000.0 Distortion
	 * 3) 2000.0-inf Fuzz
	*/

	/* Gain */
	float g;

	/* Ratio of Symmetry. Default value of s = 1 */
	float s;

}Exp_Dist;

#endif /* INC_EXP_DISTORTION_H_ */

void Exp_Distortion_Init(Exp_Dist *ed, float g, float s);
void Exp_Distortion_Init_Default(Exp_Dist *ed, float g);

void Exp_Distortion_Set_Symmetry(Exp_Dist *ed, float s);
void Exp_Distortion_Set_Gain(Exp_Dist *ed, float g);

float Exp_Distortion_Update(Exp_Dist *ed, float x);
