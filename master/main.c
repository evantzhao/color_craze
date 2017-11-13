/*
 * colorSensor.c
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
#define ALL 2
#define LEFT 1
#define RIGHT 0
#define FORWARD 1
#define BACKWARD 0
int period = 0;
int timer_value = 0;

// Setting the motors. Assumes they are on PB2-5. Upper most is leftp0. then leftp1.
void initMotors() {
	DDRB |= 0b00111100;
}

void stopMotors(int arg) {
	switch(arg) {
		case RIGHT:
			PORTB &= 0b11110011;
			break; /* optional */

		case LEFT:
			PORTB &= 0b11001111;
			break; /* optional */

		case ALL:
			PORTB &= 0b11000011;
			break;
	}
	return;
}

void runLeftMotors(int direction) {
	switch(direction) {
		case FORWARD:
			PORTB |= 0b00100000;
			PORTB &= 0b11101111;
		case BACKWARD:
			PORTB |= 0b00010000;
			PORTB &= 0b11011111;
	}
}

void runRightMotors(int direction) {
	switch(direction) {
		case FORWARD:
			PORTB |= 0b00001000;
			PORTB &= 0b11111011;
		case BACKWARD:
			PORTB |= 0b00000100;
			PORTB &= 0b11110111;
	}	
}

/* interrupt for color sensor output */
ISR(PCINT0_vect) //interrupt for color sensor output
{
	timer_value = TCNT1 * PRESCALER;  //read timer1 with prescaler adjustment
	TCNT1 = 0;   //reset timer
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

void moveForwardAndBack() {
	runLeftMotors(FORWARD);
	runRightMotors(FORWARD);

	_delay_ms(3000);

	stopMotors(ALL);

	runLeftMotors(BACKWARD);
	runRightMotors(BACKWARD);

	_delay_ms(3000);
}

void readColor() {
	period = getColor();    //read color sensor
	printf("Period: %u[microseconds]\n", period/16);
	if (ABS(period - TARGET_PERIOD) < TOLERANCE) {
		disableLED();
	} else {
		enableLED();
	}
}

int main(void)
{
	init_uart();    //initialize serial
	initMotors();	// Initialize the motor systems.
	initColor();   //initialize color sensor
	initLED();     //initialize external LED
	sei();      //enable global interrupts
	while(1) {
		moveForwardAndBack();
	}
}
