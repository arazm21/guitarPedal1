/*
 * Exp_Distortion.c
 *
 *  Created on: Jun 11, 2024
 *      Author: admin
 */
#include "Exp_Distortion.h"

void Exp_Distortion_Init_Default(Exp_Dist *ed, float g){
	Exp_Distortion_Init(ed, g, 1);
}

void Exp_Distortion_Init(Exp_Dist *ed, float g, float s){
	Exp_Distortion_Set_Gain(ed, g);
	Exp_Distortion_Set_Symmetry(ed, s);
}

void Exp_Distortion_Set_Gain(Exp_Dist *ed, float g){
	// gain g is mapped from 2^0 to 2^12
	float exponent = (12.0f / 100.0f ) * g;
	ed->g = powf(2, exponent);

	//if(g<1.0f)
	//	g = 1.0f;
	//ed->g = g;
}

void Exp_Distortion_Set_Symmetry(Exp_Dist *ed, float s){
	// symmetry is 1 as default for now
	ed->s = 1;

	//if(s>3*ed->g)
	//	s = 3*ed->g;
	//ed->s = s;
}


float Exp_Distortion_Update(Exp_Dist *ed, float x){
	float y = 0.0f;

	if( x > 0.0f )
		y =  1 - expf(-ed->s * ed->g *x);
	else
		y = -1 + expf(ed->g *x);

	return y;
}
