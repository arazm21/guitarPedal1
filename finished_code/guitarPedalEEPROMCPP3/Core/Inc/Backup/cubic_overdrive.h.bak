/*
 * cubic_overdrive.h
 *
 *  Created on: May 16, 2024
 *      Author: admin
 */

#ifndef INC_CUBIC_OVERDRIVE_H_
#define INC_CUBIC_OVERDRIVE_H_

#include <math.h>
#include <stdint.h>

#define FACTORS_SIZE 5

typedef struct{
	uint8_t factors[FACTORS_SIZE];
	//uint8_t factor_ind;
	float level; // (0 - 100)
	float fact_ind; // (0 - 100)
} cubic_overdrive;

void cubic_overdrive_init(cubic_overdrive* cubic_OD, float level, float fact_ind);
float cubic_overdrive_update(cubic_overdrive* cubic_OD, float x);

#endif /* INC_CUBIC_OVERDRIVE_H_ */
