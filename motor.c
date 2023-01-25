
#include <avr/io.h>
#include <util/delay.h>


int main(void)
{
    ///* Replace with your application code */
	PORTD.DIR |= PIN0_bm; //
	PORTD.DIR |= ~PIN1_bm;
	//PORTD.DIR |= PIN2_bm; //
	//PORTD.DIR |= ~PIN3_bm;	
	
	
	PORTD.DIR |= PIN4_bm; //
	PORTD.DIR |= ~PIN5_bm;
	
	
	
    while (1) 
    {
			PORTD.OUT = 0b00000001;
			_delay_ms(2000);
			PORTD.OUT = 0b00000010;
			_delay_ms(2000);
			PORTD.OUT = 0b00010000;
			_delay_ms(2000);
			PORTD.OUT = 0b00100000;
			_delay_ms(2000);
    }
}

