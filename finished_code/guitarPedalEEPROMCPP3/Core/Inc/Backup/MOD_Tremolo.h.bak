/*
 * MOD_Tremolo.h
 *
 *  Created on: Jun 29, 2024
 *      Author: admin
 */

#ifndef INC_MOD_TREMOLO_H_
#define INC_MOD_TREMOLO_H_

#define LFO_MAX_FREQUENCY 15.0f
#define LFO_MIN_FREQUENCY 0.1f

#include <math.h>
#include <stdint.h>

typedef struct{

	/* Sampling rate (Hz) */
	float fs;

	/* Effect depth */
	float depth;

	/* LFO Parameters (Hz) */
	float f_lfo; /* LFO frequency */
	int n; /* Counter index for LFO */



} MOD_Tremolo;

void MOD_Tremolo_init(MOD_Tremolo *trem, float fs, float depth, float rate);
void MOD_Tremolo_Set_Params(MOD_Tremolo *trem, float depth, float rate);
float MOD_Tremolo_Update(MOD_Tremolo *trem, float x);



#endif /* INC_MOD_TREMOLO_H_ */
