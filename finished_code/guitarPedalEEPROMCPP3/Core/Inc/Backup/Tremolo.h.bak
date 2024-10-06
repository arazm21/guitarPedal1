/*
 * tremolo.h
 *
 *  Created on: May 30, 2024
 *      Author: admin
 */

#ifndef INC_TREMOLO_H_
#define INC_TREMOLO_H_

typedef struct{

	/* Effect depth */
	float depth;

	/* LFO parameters (Hz) */
	float lfo_dir;
	float lfo_count;
	float lfo_count_limit;

	/* Effect sample rate (Hz) */
	float sample_rate_Hz;

	/* Output processed signal with effect*/
	float out;

} Tremolo;

void Tremolo_init(Tremolo *trem, float depth, float lfo_frequency_Hz, float sample_rate_Hz);

void Tremolo_set_depth(Tremolo *trem, float depth);
void Tremolo_set_LFO_frequency(Tremolo *trem, float lfo_frequency_Hz);

float Tremolo_update(Tremolo *trem, float in);

#endif /* INC_TREMOLO_H_ */
