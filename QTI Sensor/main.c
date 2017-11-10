/*
 * QTI Sensor.c
 *
 * Created: 11/8/2017 2:38:43 PM
 * Author : CIT-Labs
 */ 

#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "serial.h"

int main(void)
{
	DDRB &= 0b11111110; //set PB0 to input
	
	init_uart();
	
    while (1) 
    {
		int val = PINB & 0b00000001;
		// printf("Hello World!");
		printf("The current value of the QTI sensor is %u\n", val);
		_delay_ms(1000);
    }
}

