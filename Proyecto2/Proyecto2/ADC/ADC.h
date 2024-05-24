/*
 * ADC.h
 *
 * Created: 5/05/2024 13:15:16
 *  Author: yemo0
 */ 


#ifndef ADC_H_
#define ADC_H_

#include <avr/io.h>
#include <stdint.h>

void initADC(void);

uint8_t ADC_CONVERT(uint8_t canal);


#endif /* ADC_H_ */