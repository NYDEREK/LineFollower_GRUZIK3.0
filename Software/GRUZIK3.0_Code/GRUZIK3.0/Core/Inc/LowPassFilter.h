/*
 * LowPassFilter.h
 *
 *  Created on: Dec 22, 2024
 *      Author: Szymon
 */

#ifndef INC_LOWPASSFILTER_H_
#define INC_LOWPASSFILTER_H_

typedef struct {

	float alpha;
	float output;

} LowPassFilter_t;
float LowPassFilter_Update(LowPassFilter_t *LPF, float input);
void LowPassFilter_Init(LowPassFilter_t *LPF, float alpha);
#endif /* INC_LOWPASSFILTER_H_ */
