/*
 * ultrasonic.c
 *
 * Created: 11/1/2017 2:12:29 PM
 * Author : CIT-Labs
 * 
*/

#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "serial.h"
#define ABS(x) ((x > 0) ? x : -x)
#define PRESCALER 1
#define SET_PRESCALER() (TCCR1B |= 0B00000001)
#define TOLERANCE 100
#define TARGET_PERIOD 5000

int period = 0;
int distance = 0;

int ultrasonic_value = 0;
int timer_value = 0;

int uss_times = 0;

/* interrupt for color sensor output */
ISR(PCINT0_vect) //interrupt for color sensor output
{
	timer_value = TCNT1 * PRESCALER;  //read timer1 with prescaler adjustment
	TCNT1 = 0;   //reset timer
}

ISR(INT0_vect) {
	ultrasonic_value = TCNT1 * PRESCALER;	// update the ultrasonic value.
	TCNT1 = 0;
}

void initSonar() {
	DDRD |= 0b00000100; // Set INTO to be an output initially

	EICRA |= 0b00000001;
	EICRA &= 0b11111101; // Set INT0 to trigger on all logical changes.

	EIMSK &= 0b11111110; // Disable INT0 initially

	TCCR1A = 0x0;   //set timer to normal operation
	TCCR1B = 0x0;   //set timer to normal operation
	SET_PRESCALER();   //set prescaler to 1
}

int getDistance() {
	PORTD |= 0b00000100;	// Turn on the ultrasonic sensor
	_delay_ms(0.005);		// Delay for the pulse
	PORTD &= 0b11111011;	// Turn off the ultrasonic sensor

	DDRD &= 0b11111011;		// Enable listening mode on the sensor.

	EIMSK |= 0b00000001;	// Enable interrupts on INT0
	_delay_ms(0.02);
	EIMSK &= 0b11111110; // Disable INT0 interrupts

	DDRD |= 0b11111011;		// Return to output mode for the sensor.

	return ultrasonic_value;
}

/* interrupt for color sensor and time */
void initColor() //initializes the color sensor and timer
{
	DDRB       &= 0b11111110;     //set PB0 to input for color sensor output
	PCICR      |=   0b00000001;     //enable PCI0
	PCMSK0  |=   0b00000001;    //enable PCINT0 interrupt

	DDRB     |=   0b00000010;   //set PB1 to output, for LED power output
	PORTB   |=   0b00000010;   //set PB1 to high to return on color sensor LED
	DDRD     |=   0b11110000;   //set PD5-PD8 to output for S0-S3 on color sensor
	PORTD   |=   0b00010000;   //set PD5 to high for red filter 20 % scaling factor

	TCCR1A = 0x0;   //set timer to normal operation
	TCCR1B = 0x0;   //set timer to normal operation
	SET_PRESCALER();   //set prescaler to 1
}


/* reads color sensor. Returns an int. */
int getColor()
{
	PCMSK0  |=   0b00000001;    //enable PCINT0 interrupt
	_delay_ms(1);       //wait for ISR to calculate period. Max period is 250us < 1ms
	PCMSK0 &= 0b11111110; //disable interrupt
	return timer_value << 1;  //timer_value is half of period. double for full period
}


/* initializes the external LED output */
void initLED()
{
	DDRB     |=   0b00000100;  //sets PB2 to output for LED
}

/*turns on external LED */
void enableLED()
{
	PORTB     |=   0b00000100;  //sets PB2 to high
}

/*turns off external LED */
void disableLED()
{
	PORTB     &=   0b11111011;  //sets PB2 to low
}

int main(void)
{
	init_uart();    //initialize serial
	// initColor();   //initialize color sensor
	// initLED();     //initialize external LED
	
	initSonar();	// initialize sonar.

	sei();      //enable global interrupts
	while(1)
	{
		// period = getColor();    //read color sensor
		// printf("Period: %u[microseconds]\n", period/16);

		distance = getDistance();
		printf("Distance: %u[meters]\n", distance / 74 / 2);

		// if (ABS(period - TARGET_PERIOD) < TOLERANCE)
		// {
		// 	disableLED();
		// }
		// else
		// {
		// 	enableLED();
		// }
		_delay_ms(500);   //loop every 500ms
	}
}

