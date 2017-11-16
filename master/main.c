/*
 * main.c
 *
 * Created: 11/1/2017 2:12:29 PM
 * Author : Evan Zhao
 * 
 * The master code for MAE something competition!
 *
 * 
 * ************************************************
 * Wiring configurations:
 * ************************************************
 * 
 * Left Motor P0		->	PB2
 * Left Motor P1		->	PB3		(PWM)
 * Right Motor P0		->	PB4
 * Right Motor P1		->	PB5
 *
 * --------------------------------
 *
 * Color Sensor Input	->	PB0
 * Color Sensor S0-S3	->	PD4-PD7
 *
 * --------------------------------
 *
 * Random LED			->	PB1
 *
 * ************************************************
*/

#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "serial.h"

#define ABS(x) ((x > 0) ? x : -x)
#define SET_PRESCALER() (TCCR1B |= 0B00000001)

#define ALL 2
#define LEFT 1
#define RIGHT 0
#define BACKWARD 0
#define FORWARD 1

#define PRESCALER 1
#define BLUE_LOW 150
#define BLUE_HIGH 500

enum states { FIND_YELLOW, MOVE_BACK, MOVE_F_THEN_B, NONE }

int period = 0;
int timer_value = 0;

// Setting the motors. Assumes they are on PB2-5. Upper most is leftp0. then leftp1. mirrored config for right.
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
	if(direction == FORWARD) {
		PORTB |= 0b00100000;
		PORTB &= 0b11101111;
	} else if(direction == BACKWARD) {
		PORTB |= 0b00010000;
		PORTB &= 0b11011111;
	}
}

void runRightMotors(int direction) {
	if(direction == FORWARD) {
		PORTB |= 0b00001000;
		PORTB &= 0b11111011;
	} else if(direction == BACKWARD) {
		PORTB |= 0b00000100;
		PORTB &= 0b11110111;
	}	
}

/* 
 *Interrupt for color sensor output
*/
ISR(PCINT0_vect) {
	timer_value = TCNT1;
	TCNT1 = 0;   //reset timer
}

/* 
 * Interrupt for color sensor and time
 * Also initializes the color sensor and timer
*/
void initColor() {
	DDRB	&=	0b11111110;		//set PB0 to input for color sensor output
	// DDRB &= (1 << DDB0);
	PCICR	|=	0b00000001;		//enable PCI0
	PCMSK0	|=	0b00000001;		//enable PCINT0 interrupt

	DDRB	|=	0b00000010;		//set PB1 to output, for LED power output
	PORTB	|=	0b00000010;		//set PB1 to high to return on color sensor LED
	DDRD	|=	0b11110000;		//set PD5-PD8 to output for S0-S3 on color sensor
	PORTD	|=	0b00010000;		//set PD5 to high for red filter 20 % scaling factor

	TCCR1A	=	0x0;			//set timer to normal operation
	TCCR1B	=	0x0;			//set timer to normal operation
	SET_PRESCALER();			//set prescaler to 1
}


/* 
 * Reads color sensor. Returns an int.
*/
int getColor() {
	PCMSK0 |= 0b00000001;						// enable PCINT0 interrupt
	_delay_ms(1);								// wait for ISR to calculate period. Max period is 250us < 1ms
	PCMSK0 &= 0b11111110;						// disable interrupt
	int doubled = timer_value * 2;				// timer_value is half of period. double for full period
	int measured_period = doubled / 16;			// Scale the number of ticks we have counted by the amount of time each one is. 

	return measured_period;
}


/* initializes the external LED output */
void initLED() {
	DDRB |= 0b00000010;  //sets PB1 to output for LED
}

/*turns on external LED */
void enableLED() {
	PORTB |= 0b00000010;  //sets PB1 to high
}

/*turns off external LED */
void disableLED() {
	PORTB &= 0b11111101;  //sets PB1 to low
}

int isBlue(int input) {
	return input >= BLUE_LOW && input <= BLUE_HIGH;
}

void readColor() {
	period = getColor();    //read color sensor

	printf("Period: %u[microseconds]\n", period);

	if(isBlue(period)) {
		enableLED();
	} else {
		disableLED();
	}
}

/************************************************
* PWM Generation Code *
*************************************************/

void initPWM() {
	OCR2A = 128;							// Set PWM for 50% duty cycle

	TCCR2A |= (1 << COM2A1);					// Set non-inverting mode
	TCCR2A &= 0b10111111;

	TCCR2A |= (1 << WGM21) | (1 << WGM20);	// Set fast PWM mode

	TCCR2B |= (1 << CS21);					// Set prescaler to 8 and start PWM. Write this to all zeros to kill the PWM signal.
}

void moveForwardAndBack() {
	while(1) {
		enableLED();
		runRightMotors(FORWARD);
		runLeftMotors(FORWARD);

		_delay_ms(3000);

		stopMotors(ALL);
		disableLED();
		
		_delay_ms(1000);
		
		enableLED();
		runRightMotors(BACKWARD);
		runLeftMotors(BACKWARD);

		_delay_ms(3000);
		
		disableLED();
		stopMotors(ALL);
		
		_delay_ms(1000);
	}
}

void move_back() {
	while(1) {
		runRightMotors(BACKWARD);
		runLeftMotors(BACKWARD);		
	}
}

void find_yellow() {
	readColor();
	while(isBlue(period)) {
		runRightMotors(FORWARD);
		runLeftMotors(FORWARD);
		_delay_ms(100);
		readColor();
	}
	
	stopMotors(ALL);

	while(1) {
		runRightMotors(FORWARD);
		runLeftMotors(BACKWARD);	
	}
}

int main(void)
{
	init_uart();    //initialize serial
	initMotors();	// Initialize the motor systems.
	initColor();   //initialize color sensor
	initLED();     //initialize external LED
	sei();      //enable global interrupts
	
	_delay_ms(2000);

	states state = MOVE_BACK;

	while(1) {
		switch(state) {

		case MOVE_BACK:
			move_back();
			break;
		case FIND_YELLOW:
			find_yellow();
			break;
		case MOVE_F_THEN_B:
			moveForwardAndBack();
			break;
		default:
			_delay_ms(100);
			break;
		}		
	}
}

