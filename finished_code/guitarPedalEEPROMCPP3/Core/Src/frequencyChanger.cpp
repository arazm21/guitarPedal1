/*
 * Tremolo.c
 *
 *  Created on: May 30, 2024
 *      Author: admin
 */

#include "frequencyChanger.h"

void FREQUENCY_CHANGER_INIT(freqStruct *filt, int semitones);
float FREQUENCY_CHANGER_UPDATE(freqStruct *filt, float x);
