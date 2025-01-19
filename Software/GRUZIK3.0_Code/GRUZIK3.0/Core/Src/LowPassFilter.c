/*
 * LowPassFilter.c
 *
 *  Created on: Dec 22, 2024
 *      Author: Szymon
 */
#include "main.h"
#include"LowPassFilter.h"

void LowPassFilter_Init(LowPassFilter_t *LPF, float alpha)
{
	LPF->alpha = alpha;
}
float LowPassFilter_Update(LowPassFilter_t *LPF, float input)
{
	/*Standard low pass filter*/
	LPF->output = LPF->alpha * input + (1.0f - LPF->alpha) * LPF->output;

	/*IIR filter*/
	//LPF->output = (1.0f - LPF->alpha) * input + LPF->alpha * LPF->output;

	return LPF->output;
}
