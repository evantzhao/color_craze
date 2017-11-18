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
 * Left Motor P0 (green)		->	PB5
 * Left Motor P1 (blue)			->	PB3		(PWM)
 * Right Motor P0 (blue)		->	PB4
 * Right Motor P1 (green)		->	PD3		(PWM)	(OCR2B)
 *
 * --------------------------------
 *
 * Color Sensor Input			->	PB0
 * Color Sensor S0-S3			->	PD4-PD7
 *
 * --------------------------------
 *
 * Random LED					->	PB1
 *
 * --------------------------------
 * QTI Sensor Configuration
 *
 * Forward Right				->	PC0
 * Forward Left					->	PC1
 * Backside						->	PC2
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

#define BLUE 0
#define YELLOW 1

#define PRESCALER 1
#define BLUE_LOW 150
#define BLUE_HIGH 500

enum states { MOVE_FORWARD, FIND_YELLOW, MOVE_BACK, MOVE_F_THEN_B, NONE, TESTING, UTURN };

int start_side = YELLOW;
int period = 0;
int timer_value = 0;

// Globals that keep track of the QTI values. Note that 
// you must manually poll these values. 
int qtileft = 0;
int qtiright = 0;
int qtibot = 0;

// Initializing the QTI sensors in the configuration specified in the header.
void initQTI() {
	DDRC &= 0b11111000;		// Setting as inputs
}

// Updates the values held within the QTI global variables. 
void updateQTI() {
	qtiright	= PINC & 0b00000001;
	qtileft		= (PINC & 0b00000010) >> 1;
	qtibot		= (PINC & 0b00000100) >> 2;
}

// Setting the motors. Assumes they are on PB3-5, and PD3.
void initMotors() {
	DDRB |= 0b00111000;
	DDRD |= 0b00001000;
	PORTB &= 0b11001111;
}

// Note that these PWM's only run in one direction. 
void runPWM(int side, int magnitude) {
	if(side == LEFT) {
		OCR2A = magnitude;			// Set PWM for 50% duty cycle
	} else if(side == RIGHT) {
		OCR2B = magnitude;
	}
}

void stopMotors(int arg) {
	switch(arg) {
		case RIGHT:
			PORTB &= 0b11101111;
			runPWM(RIGHT, 0);
			break; /* optional */

		case LEFT:
			PORTB &= 0b11011111;
			runPWM(LEFT, 0);
			break; /* optional */

		case ALL:
			PORTB &= 0b11001111;
			runPWM(LEFT, 0);
			runPWM(RIGHT, 0);
			break;
	}
	return;
}

void runLeftMotors(int direction, int magnitude) {
	if(direction == FORWARD) {
		PORTB &= 0b11011111;
		runPWM(LEFT, magnitude);
	} else if(direction == BACKWARD) {
		runPWM(LEFT, 0);
		PORTB |= 0b00100000;
	}
}

void runRightMotors(int direction, int magnitude) {
	if(direction == FORWARD) {
		PORTB &= 0b11101111;
		runPWM(RIGHT, magnitude);
	} else if(direction == BACKWARD) {
		runPWM(RIGHT, 0);
		PORTB |= 0b00010000;
	}	
}

void uturn() {
	runLeftMotors(FORWARD, 255);
	runRightMotors(BACKWARD, 255);

	_delay_ms(1260);

	stopMotors(ALL);

	return;
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
	OCR2A = 0;			// Set PWM for 0% duty cycle
	OCR2B = 0;			// Set PWM for 0% duty cycle

	// TCCR2A |= (1 << COM2A1);					// Set non-inverting mode
	TCCR2A |= 0b10000000;
	TCCR2A &= 0b10111111;	// Set the first PWM to be non-inverting mode.

	TCCR2A |= 0b00100000;
	TCCR2A &= 0b11101111;	// Set the second PWM to be non-inverting mode. 

	TCCR2A |= 0b00000011;	// Set fast PWM mode
	TCCR2B &= 0b11110111;

	TCCR2B |= 0b00000010;
	TCCR2B &= 0b11111010;	// Set prescaler to 8 and start PWM. Write this to all zeros to kill the PWM signal.
}

void moveForwardAndBack() {
	while(1) {
		enableLED();
		runRightMotors(FORWARD, 255);
		runLeftMotors(FORWARD, 255);

		_delay_ms(3000);

		stopMotors(ALL);
		disableLED();
		
		_delay_ms(1000);
		
		enableLED();
		runRightMotors(BACKWARD, 0);
		runLeftMotors(BACKWARD, 0);

		_delay_ms(3000);
		
		disableLED();
		stopMotors(ALL);
		
		_delay_ms(1000);
	}
}

void move_back() {
	while(1) {
		runRightMotors(BACKWARD, 255);
		runLeftMotors(BACKWARD, 255);		
	}
}

void find_yellow() {
	readColor();
	while(isBlue(period)) {
		runRightMotors(FORWARD, 255);
		runLeftMotors(FORWARD, 255);
		_delay_ms(100);
		readColor();
	}
	
	stopMotors(ALL);

	while(1) {
		runRightMotors(FORWARD, 255);
		runLeftMotors(BACKWARD, 0);	
	}
}

int main(void)
{
	init_uart();    //initialize serial
	initMotors();	// Initialize the motor systems.
	initPWM();
	initQTI();
	initColor();   //initialize color sensor
	initLED();     //initialize external LED
	sei();      //enable global interrupts
	
	_delay_ms(1000);

	enum states state = MOVE_FORWARD;
	
	readColor();
	start_side = isBlue(period) ? BLUE : YELLOW;

	while(1) {
		switch(state) {
			
		case MOVE_FORWARD:	// This is actually a QTI follow the line routine.
			updateQTI();
			while(1) {
				while(qtileft == 1 && qtiright == 1) {
					runLeftMotors(FORWARD, 255);
					runRightMotors(FORWARD, 245);
					updateQTI();
					_delay_ms(10);
				}
				
				stopMotors(ALL);
				
				if(qtileft == 0 && qtiright == 0) {
					while(1) {
						// we have reached the end mang.
						_delay_ms(1000);
					}
				}
				
				while(qtileft == 0) {
					runRightMotors(FORWARD, 255);
					updateQTI();
					_delay_ms(10);
				}
				
				stopMotors(ALL);
				
				while(qtiright == 0) {
					runLeftMotors(FORWARD, 255);
					updateQTI();
					_delay_ms(10);
				}
			}
			
			break;
		case MOVE_BACK:
			while(1) {
				updateQTI();
				printf("The value of the right QTI is %u, left QTI: %u, rear: %u\n", qtiright, qtileft, qtibot);
				_delay_ms(500);
			}
			// move_back();
			break;
		case FIND_YELLOW:
			find_yellow();
			break;
		case MOVE_F_THEN_B:
			readColor();
			runLeftMotors(FORWARD, 255);
			runRightMotors(FORWARD, 245);
			while(isBlue(period)) {
				readColor();
				_delay_ms(100);
			}
			stopMotors(ALL);
			
			runLeftMotors(BACKWARD, 255);
			runRightMotors(BACKWARD, 245);
			
			_delay_ms(3000);
			
			stopMotors(ALL);
			
			while(1){
				_delay_ms(100);
			}
			
			break;
		case UTURN:
			uturn();
			while(1) {
				_delay_ms(1000);
			}
			break;
		case TESTING:
			runLeftMotors(FORWARD, 255);
			runRightMotors(FORWARD, 245);
			
			_delay_ms(6000);
			
			stopMotors(ALL);
			
			_delay_ms(1000);
			
			uturn();
			
			_delay_ms(1000);
			
			runLeftMotors(FORWARD, 200);
			runRightMotors(FORWARD, 200);
			
			_delay_ms(9000);
			
			stopMotors(ALL);
			
			while(1) {
				_delay_ms(1000);
			}
			break;
		default:
			break;
		}		
	}
}

