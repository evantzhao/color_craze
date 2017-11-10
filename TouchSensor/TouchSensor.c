/*
 * TouchSensor.c
 *
 * Created: 10/16/2013 3:26:23 PM
 *  Author: dah322
 */ 


#include <avr/io.h>			// This includes the C library for I/O devices
#define F_CPU 16000000		// This sets the clock to the crystal on the UNO
#include <util/delay.h>		// This includes the C library for the _ms_delay() method
#include <avr/interrupt.h> // This includes the C library for interrupts

// You need to insert your source code HERE (or after main)
ISR(INT0_vect){
	 if (PIND & 0b00000100) //shifts PIND to check the 3rd pin is low
	 {
		 PORTB &= 0b11011111; //set LED to low
	 }
	 else {
		 PORTB |= 0b00100000; //set LED to high
		 }
}
int main(void)
{
	DDRB |= 0b00100000; //set PB5 as output to LED
	DDRD &= 0b11111011; //set PD4 as touch sensor input
	EIMSK |= 0b00000001; //enable INT0
	EICRA |= 0b00000001; //enable ISC00, set to toggle
	EICRA &= 0b00000001; //make sure the last two digits are 01
	PORTB = 0b11011111; //LED Off to start
	sei(); //enable interrupts
    while(1)
    {
/*		if (PIND & 0b00010000) //shifts PIND to check the 4th pin is low
		{
			PORTB &= 0b11011111; //set LED to low
			_delay_ms(1000);
		}
		else {
			PORTB |= 0b00100000; //set LED to high
			_delay_ms(1000);
		}*/
			_delay_ms(1000);
    }
}