#ifndef INC_FREQUENCY_CHANGER_H_
#define INC_FREQUENCY_CHANGER_H_

#include <math.h>
#include <stdint.h>

typedef struct{

	float semitones;
    float frequencies[12];
} freqStruct;

void FREQUENCY_CHANGER_INIT(freqStruct *filt, int semitones);
float FREQUENCY_CHANGER_UPDATE(freqStruct *filt, float x);

#endif /* INC_EQ_LOW_SHELVING_H_ */
