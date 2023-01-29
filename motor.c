
#define F_CPU 3333333
#define USART0_BAUD_RATE(BAUD_RATE) ((float)(F_CPU * 64 / (16 * (float)BAUD_RATE)) + 0.5)
#include <avr/io.h>
#include <stdio.h>
#include <util/delay.h>
#include <string.h>
#include <avr/interrupt.h>

#define ENCODER_CH_1A_PIN PIN2_bm
#define ENCODER_CH_1B_PIN PIN3_bm

void USART0_init(void)
{
	PORTA.DIR &= ~PIN1_bm;
	PORTA.DIR |= PIN0_bm;
	
	USART0.BAUD = (uint16_t)USART0_BAUD_RATE(9600);

	USART0.CTRLB |= USART_TXEN_bm;
}

void USART0_sendChar(char c)
{
	while (!(USART0.STATUS & USART_DREIF_bm))
	{
		;
	}
	USART0.TXDATAL = c;
}

void USART0_sendString(char *str)
{
	for(size_t i = 0; i < strlen(str); i++)
	{
		USART0_sendChar(str[i]);
	}
}

int16_t encoder_position = 0;
void encoder_init() {
	PORTD.DIR |= (ENCODER_CH_1A_PIN | ENCODER_CH_1B_PIN);
	//PORTD.PIN2CTRL |= PORT_PULLUPEN_bm;
	//PORTD.PIN3CTRL |= PORT_PULLUPEN_bm;
	PORTD.OUT |= ENCODER_CH_1A_PIN | ENCODER_CH_1B_PIN;
	sei();
	PORTD.PIN2CTRL |= PORT_ISC_RISING_gc;
	PORTD.PIN3CTRL |= PORT_ISC_RISING_gc;
}

ISR(PORTD_PORT_vect){
	char buffer[20];
	uint8_t new_state = PORTD.IN & 0x03;
	if ((new_state == 0x02) || (new_state == 0x00)) //check for encoder state
	{
		encoder_position++;
	}
	else if ((new_state == 0x03) || (new_state == 0x01)){
		encoder_position--;
	}
	sprintf(buffer, "Encoder: %4d \n", encoder_position);
	USART0_sendString(buffer);
	PORTD_INTFLAGS  |= 1<<2; // interrupt has been handled for pin2
	PORTD_INTFLAGS |= 1<<3; // interrupt has been handled for pin3
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

	while (1)
	{
		while (PORTA.IN & (1<<2) || PORTA.IN & (1<<3)) // Check if either of the switch has been pressed
		{
			PORTD.OUT = 0; // stop motor
		}
		PORTD.OUT = PORTD.OUT ^ 0x02; // invert bit 1
		PORTD.OUT |= 0b00000001; // turn on motor forward
		_delay_ms(1000);
		PORTD.OUT = PORTD.OUT ^ 0x01; // invert bit 0
		PORTD.OUT |= 0x02; // turn on motor backward
		_delay_ms(1000);
		
		//USART0_sendString("main");
		////_delay_ms(2000);
		//PORTD.OUT = 0b00000010;
		//_delay_ms(2000);
		
		//PORTD.OUT = 0b00010000;
		//_delay_ms(2000);
		//encoder_read();
		//sprintf(buffer, "%2d", encoder_count);
		//USART0_sendString(buffer);
		//PORTD.OUT = 0b00100000;
		//_delay_ms(2000);
		////encoder_read();
	}
}
