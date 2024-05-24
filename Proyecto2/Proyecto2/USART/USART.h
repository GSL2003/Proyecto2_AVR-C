/* -----------------------------------------------
 Universidad del Valle de Guatemala
 USART.h
 Autor: Guillermo Jos� Schwartz L�pez
 Ultima modificacion: 23/05/2024
 -------------------------------------------------
 */


#ifndef USART_H_
#define USART_H_

#include <avr/io.h>	//Liberia para los registros
#include <stdint.h> //Libreria para enteros
#include <avr/interrupt.h> //Libreria para interrupciones

//Definir prototipo de funciones
void initUART9600(void);			//El microcontrolador solo tiene un modulo UART
void writeUART(char caractrer);		//Funci�n para escribir caracteres
void writeNUM(uint8_t ADC_Datos);	//Funci�n para escribir valores enteros -- ADAFRUIT

#endif /* USART_H_ */