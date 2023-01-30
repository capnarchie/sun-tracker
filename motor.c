
#define F_CPU 3333333
#define USART0_BAUD_RATE(BAUD_RATE) ((float)(F_CPU * 64 / (16 * (float)BAUD_RATE)) + 0.5)
#include <avr/io.h>
#include <stdio.h>
#include <util/delay.h>
#include <string.h>
#include <avr/interrupt.h>

#define ENCODER_CH_1A_PIN PIN2_bm
#define ENCODER_CH_1B_PIN PIN3_bm
#define ANGLE_PER_COUNT 0.03

volatile int16_t encoder_position = 0;
volatile int16_t encoder_position_buf = 0;
volatile double angle = 0;
void encoder_init() {
	//PORTD.DIR |= (ENCODER_CH_1A_PIN | ENCODER_CH_1B_PIN);
	PORTD.PIN2CTRL |= PORT_PULLUPEN_bm;
	PORTD.PIN3CTRL |= PORT_PULLUPEN_bm;
	PORTD.OUT |= ENCODER_CH_1A_PIN | ENCODER_CH_1B_PIN; 
	PORTD.PIN2CTRL |= PORT_ISC_RISING_gc; // trigger interrupt on rising
	PORTD.PIN3CTRL |= PORT_ISC_RISING_gc;
	sei();
}

uint8_t prev_state = 0;
ISR(PORTD_PORT_vect){

	uint8_t new_state = (PORTD.IN & 0xC)>>2;
	encoder_position++;
	if(new_state != prev_state)//if ((new_state == 0x02) || (new_state == 0x00)) //check for encoder state 01 00 11 10
	{
		if ((prev_state == 0x00 && new_state == 0x01) || (prev_state = 0x03 && new_state == 0x02))
		{
			encoder_position++;
		}

	}
	else if ((prev_state == 0x01 && new_state == 0x00) || (prev_state == 0x02 && new_state == 0x03))
	{// ((new_state == 0x03) || (new_state == 0x01)){
		encoder_position--;
	}
	angle = encoder_position * 0.03;
	encoder_position_buf = encoder_position;
	if (angle >= 45)
	{
		PORTD.OUT = PORTD.OUT & 0xFE;
	}
	//prev_state = new_state;
	PORTD.INTFLAGS  |= 1<<2; // interrupt has been handled for pin2
	PORTD.INTFLAGS |= 1<<3; // interrupt has been handled for pin3
}




int main(void)
{
	encoder_init();
	USART0_init();
	///* Replace with your application code */
	PORTD.DIR |= PIN0_bm; // Motor 1 Forward
	PORTD.DIR |= PIN1_bm; // Motor 1 Backward
	//PORTD.DIR |= PIN2_bm; // Motor 1 Encoder A
	//PORTD.DIR |= ~PIN3_bm; // Motor 1 Encoder B
	PORTA.DIR |= PIN2_bm | PIN3_bm; // microswitch pins
	
	PORTD.DIR |= PIN4_bm; // Motor 2 Forward
	PORTD.DIR |= PIN5_bm; // Motor 2 Backward
	//PORTD.DIR |= PIN6_bm; // Motor 2 Encoder A
	//PORTD.DIR |= ~PIN7_bm; // Motor 2 Encoder B
	//PORTD.OUT = 0b00010000;
	PORTD.OUT |= 0b00000001;
	char buffer[20];
	while (1)
	{
		while (PORTA.IN & (1<<2) || PORTA.IN & (1<<3)) // Check if either of the switch has been pressed
		{
			PORTD.OUT = 0; // kill everything
		}
	}
}
