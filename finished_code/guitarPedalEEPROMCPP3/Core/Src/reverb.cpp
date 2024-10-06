/*
 * MOD_Flanger_1.c
 *
 *  Created on: Sep 20, 2024
 *      Author: x1_user
 */

#include "reverb.h"

void reverb_init(reverb *filt, float fs, float delay_s,float amount,  float *x_ptr, int maxN,CF* CF1,
		CF* CF2,CF* CF3,CF* CF4,delay2* AP1,delay2* AP2,delay2* AP3, float *yCF1, float *yCF2, float *yCF3,
		float *yCF4, float *yAP1, float *yAP2, float *yAP3){

	/* Initialize Helpers */
	CF_init(CF1, fs, -1, -1, 100, yCF1, 1900);
	CF_init(CF2, fs, -1, -1, 100, yCF2, 1600);
	CF_init(CF3, fs, -1, -1, 100, yCF3, 2100);
	CF_init(CF4, fs, -1, -1, 100, yCF4, 2300);

	delay2_init(AP1, fs, -1, -1, 100, x_ptr, yAP1, maxN,300);
	delay2_init(AP2, fs, -1, -1, 100, x_ptr, yAP2,maxN, 100);
	delay2_init(AP3, fs, -1, -1, 100, x_ptr, yAP3,maxN, 50);


	/* Set Parameters */
	filt->AP1=*AP1;
	filt->AP2=*AP2;
	filt->AP3=*AP3;

	filt->CF1=*CF1;
	filt->CF2=*CF2;
	filt->CF3=*CF3;
	filt->CF4=*CF4;

	filt->yAP1=yAP1;
	filt->yAP2=yAP2;
	filt->yAP3=yAP3;

	filt->yCF1=yCF1;
	filt->yCF2=yCF2;
	filt->yCF3=yCF3;
	filt->yCF4=yCF4;

    filt->x_ptr=x_ptr;
	filt->fs = fs;
	filt->maxN = maxN;
	reverb_Set_Params(filt, delay_s, amount);

}

void reverb_Set_Params(reverb *filt, float delay,float amount){
	filt->delay_s = delay;
	filt->amount = amount;

	/* re-adjust delay2s in AP and CF */
	filt->CF1.N = .03604f * filt->fs * filt->delay_s;
	filt->CF2.N = .03112f * filt->fs * filt->delay_s;
	filt->CF3.N = .04044f * filt->fs * filt->delay_s;
	filt->CF4.N = .04492f * filt->fs * filt->delay_s;

	filt->AP1.N =  .005f * filt->fs * filt->delay_s;
	filt->AP2.N =  .00168f * filt->fs * filt->delay_s;
	filt->AP3.N =  .00048f * filt->fs * filt->delay_s;

	filt->CF1.g = g1;
	filt->CF2.g = g2;
	filt->CF3.g = g3;
	filt->CF4.g = g4;

	filt->AP1.g = AP_g;
	filt->AP2.g = AP_g;
	filt->AP3.g = AP_g;



}

float reverb_Update(reverb *filt, float x){

	float x_CF1 = CF_Update( &(filt->CF1) , x );
	float x_CF2 = CF_Update( &(filt->CF2) , x );
	float x_CF3 = CF_Update( &(filt->CF3) , x );
	float x_CF4 = CF_Update( &(filt->CF4) , x );

	float y = 0.25f*(x_CF1 + x_CF2 + x_CF3 + x_CF4);
	//float y=x;
	y = delay2_Update( &(filt->AP1), y);
	y = delay2_Update( &(filt->AP2), y);
	y = delay2_Update( &(filt->AP3), y);

	y = filt->amount*y + (1-filt->amount)*x;
    filt->x_ptr[filt->AP1.indx] = x;

	return y;

}




