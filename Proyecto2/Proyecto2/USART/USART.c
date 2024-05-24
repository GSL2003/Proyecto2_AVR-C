/* -----------------------------------------------
 Universidad del Valle de Guatemala
 USART.c
 Autor: Guillermo José Schwartz López
 Ultima modificacion: 23/05/2024
 -------------------------------------------------
 */

#include "USART.h"

//Inicializar la función del UART
void initUART9600(void){
	//Paso1: Configurar pines TX y RX
	DDRD &= ~(1<<DDD0);		//Se configura el RX (PD0) como entrada
	DDRD |= (1<<DDD1);		//Se configura el TX (PD1) como salida
	
	//Paso2: Configurar registro A ---> Modo FAST U2X0 = 1
	UCSR0A = 0;
	UCSR0A |= (1<<U2X0);
	
	//Paso3: Configurar registro B --> Habilitar ISR RX, se habilita RX y TX
	//->>Se configura para tener interrupciones
	UCSR0B = 0;		//Se pone en 0 para mayor facilidad de configurar posteriomente
	UCSR0B |= (1<<RXCIE0)|(1<<RXEN0)|(1<<TXEN0);
	
	//Paso4: Configurar C > Frame (Se define el frame): 8 bits datos, no paridad, 1 bit de stop
	UCSR0C = 0;    //--> Se configura si se desea modo: asincrono o sincrono
	UCSR0C |= (1<<UCSZ01)|(1<<UCSZ00);	//Se configura el tamaño del caracter --> 8 bits
	
	//Paso5: Baudrate = 9600
	UBRR0 = 207;
}

void writeUART(char caractrer){
	while(!(UCSR0A &(1<<UDRE0)));		//La función se queda en espera hasta que UDR este en 1
	UDR0 = caractrer;
}



void writeNUM(uint8_t ADC_Datos){
	//Envío de caracter por caracter a la UART
	uint8_t CE = 0; //Centenas
	uint8_t DE = 0; //Decenas
	uint8_t UN = 0; //Unidades
	while(ADC_Datos > 99){
		CE++;
		ADC_Datos -= 100;
	}
	writeUART((char) CE + 48);
	while(ADC_Datos > 9){
		DE++;
		ADC_Datos -= 10;
	}
	writeUART((char) DE + 48);
	while(ADC_Datos != 0){
		UN++;
		ADC_Datos -= 1;
	}
	writeUART((char) UN + 48);
	writeUART('\n');
}

uint8_t StrToNum(char* str){		//Función para convertir string a valores enteros
	//Cadena de 4 caracteres
	uint8_t i = 0;		//Valor de caracteres
	uint8_t RETV = 0;	//Regresar el valor almacenado
	uint8_t UN = 0;	//UNIDADES
	uint8_t DE = 0;	//DECENAS
	uint8_t CE = 0;	//CENTENAS
	for(i = 0; str[i] != '\0'; i++){
		//Mientras el caracter no sea nulo, se procesa
		switch(i){
			case 1: //Valor de las Unidades
			UN = ((uint8_t) str[i]) - 48;
			break;
			case 2: // Valor de las Decenas
			DE = 10*(((uint8_t) str[i]) - 48);
			break;
			case 3: //Valor de las Centenas
			CE = 100*(((uint8_t) str[i]) - 48);
			break;
		}
	}
	RETV = UN+DE+CE;
	return RETV;
}