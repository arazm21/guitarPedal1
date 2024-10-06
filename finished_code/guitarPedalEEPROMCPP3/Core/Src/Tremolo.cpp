/*
 * Tremolo.c
 *
 *  Created on: May 30, 2024
 *      Author: admin
 */

#include "Tremolo.h"

void Tremolo_init(Tremolo *trem, float depth, float lfo_frequency_Hz, float sample_rate_Hz){

	/* Store Tremolo Depth */
	Tremolo_set_depth(trem, depth);


	/* Store sample rate */
	trem->sample_rate_Hz = sample_rate_Hz;

	/* Compute LFO parameters */
	Tremolo_set_LFO_frequency(trem, lfo_frequency_Hz);

	/* Clear count and set LFO 'direction' */
	trem->lfo_count = 0.0f;
	trem->lfo_dir = 1.0f;


	/* Clear output */
	trem->out = 0.0f;
}

void Tremolo_set_depth(Tremolo *trem, float depth){

	/* Check bounds */
	if( depth > 1.0f )
		depth = 1.0f;
	else if( depth < 0.0f )
		depth = 0.0f;

	trem->depth = depth;
}

void Tremolo_set_LFO_frequency(Tremolo *trem, float lfo_frequency_Hz){

	/* Check LFO frequency bounds */
	if( lfo_frequency_Hz < 0.1f )
		lfo_frequency_Hz = 0.1f;
	else if( lfo_frequency_Hz > 0.5f * trem->sample_rate_Hz )
		lfo_frequency_Hz = 0.5f * trem->sample_rate_Hz;

	/* Compute counter limit based on LFO frequency */
	trem->lfo_count_limit = 0.25f * (trem->sample_rate_Hz / lfo_frequency_Hz);

	/* Check LFO counter bounds */
	if( trem->lfo_count > trem->lfo_count_limit )
		trem->lfo_count = trem->lfo_count_limit;
	else if( trem->lfo_count < -trem->lfo_count_limit )
		trem->lfo_count = -trem->lfo_count_limit;

}

float Tremolo_update(Tremolo *trem, float in){

	/* Effect difference equation: y[n] = x[n] * ( (1-d) + d*g[n] )  */
	trem->out = in * ( (1.0f - trem->depth) + trem->depth*( trem->lfo_count / trem->lfo_count_limit ) );

	/* Check counter and change direction of counting */
	if( trem->lfo_count >= trem->lfo_count_limit )
		trem->lfo_dir = -1.0f;
	if( trem->lfo_count >= -trem->lfo_count_limit )
			trem->lfo_dir = 1.0f;

	/* Increment counter */
	trem->lfo_count += trem->lfo_dir;

	/* Return processed output */
	return trem->out;

}
