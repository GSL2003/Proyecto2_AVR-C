/* -----------------------------------------------
 Universidad del Valle de Guatemala
 PWM0.h
 Autor: Guillermo José Schwartz López
 Ultima modificacion: 23/05/2024
 -------------------------------------------------
 */


#ifndef PWM0_H_
#define PWM0_H_

//->Se definen las librerias
#include <stdalign.h>
#include <avr/io.h>			//Se declara para usar los registros

//Definimos etiquetas/Variables que se pueden usar mas adelante
#define invertido 1
#define no_invertido 0

//Declarar una función de mapeo para convertir el valor del potenciómetro en el ciclo de trabajo del PWM
float map2(float x, float in_min, float in_max, float out_min, float out_max);	

//->Funciones para configurar PWM0 - TIMER0- modo FAST

//Prototipo de Función del PWM0 modo fast usando TIMER0 - Canal A
void initPWM0FastA(uint8_t inverted, uint16_t prescaler);	

void updateDutyCycleA(uint8_t duty);	//Canal A


#endif /* PWM0_H_ */