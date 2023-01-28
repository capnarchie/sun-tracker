/*
 * main.c
 *
 * Created: 1/25/2023 9:42:10 AM
 *  Author: Uku
 */ 

#include <xc.h>


#define F_CPU 3333333
#define USART0_BAUD_RATE(BAUD_RATE) ((float)(F_CPU * 64 / (16 * (float)BAUD_RATE)) + 0.5)

#include <avr/io.h>
#include <stdio.h>
#include <util/delay.h>
#include <string.h>
#include <avr/interrupt.h>

#define RTCC_SECONDS_REG 0x01
#define RTCC_MINUTES_REG 0x02
#define RTCC_HOURS_REG 0x03
#define RTCC_DAY_REG 0x04
#define RTCC_DATE_REG 0x05
#define RTCC_MONTH_REG 0x06
#define RTCC_YEAR_REG 0x07
#define RTCC_CONTROL_REG 0x08

//prototypes
static void SPI0_init(void);
static void clientSelect(void);
static void clientDeselect(void);
static uint8_t SPI0_exchangeData(uint8_t data);
static void rtcc_write_reg(uint8_t reg, uint8_t data);
uint8_t rtcc_read_reg(uint8_t reg);
void rtcc_init();


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



static void SPI0_init(void)
{
	PORTA.DIR |= PIN4_bm; /* Set MOSI pin direction to output */
	PORTA.DIR &= ~PIN5_bm; /* Set MISO pin direction to input */
	PORTA.DIR |= PIN6_bm; /* Set SCK pin direction to output */
	PORTA.DIR |= PIN2_bm; /* Set SS pin direction to output */

	SPI0_CTRLB |= SPI_SSD_bm;

	SPI0.CTRLA = SPI_CLK2X_bm           /* Enable double-speed */
	| 0//SPI_DORD_bm             /* MSB is transmitted first */
	| SPI_ENABLE_bm          /* Enable module */
	| SPI_MASTER_bm          /* SPI module in Master mode */
	| SPI_PRESC_DIV4_gc;    /* System Clock divided by 4 */
	
	SPI0.CTRLA |= SPI_MASTER_bm;
}
//https://github.com/microchip-pic-avr-examples/atmega4809-getting-started-with-spi-studio/tree/master/Sending_Data_as_Host
static uint8_t SPI0_exchangeData(uint8_t data)
{
	SPI0.DATA = data;

	while (!(SPI0.INTFLAGS & SPI_IF_bm))  /* waits until data is exchanged*/
	{
		;
	}
	//SPI0.INTFLAGS = SPI_IF_bm;
	return SPI0.DATA;
}

static void rtcc_write_reg(uint8_t reg, uint8_t data)
{
	uint8_t temp;
	//clientSelect();//select rtcc
	SPI0_exchangeData(0b00010010); // write command
	SPI0_exchangeData(reg);// send register address
	SPI0_exchangeData(data); // send the data to write to register
	//clientDeselect();// deselect the rtcc
}

uint8_t rtcc_read_reg(uint8_t reg)
{
	uint8_t data;
	uint8_t temp;
	//clientSelect();//select rtcc
	SPI0_exchangeData(0b00010011); //read command
	SPI0_exchangeData(reg); // send register address
	data = SPI0_exchangeData(0x00); //send dummy byte to receive
	//clientDeselect();//deselect rtcc
	return data;
	
}

void rtcc_init(){
	clientSelect();
	// enable oscillator and battery backup
	////SPI0_exchangeData(0b01010100);
	rtcc_write_reg(RTCC_SECONDS_REG, 0); //Write ST to 0 aka disable oscillator(bit7)
	rtcc_write_reg(RTCC_CONTROL_REG, ~(1<<3)); // Disable EXTOSC
	while (rtcc_read_reg(RTCC_DAY_REG) & ~(1<<5)); // wait OSCRUN to clear

	
	//sets year to jan 1 2000 clock: 00:00:00
	rtcc_write_reg(RTCC_SECONDS_REG, 0x00);
	rtcc_write_reg(RTCC_MINUTES_REG, 0x00);
	rtcc_write_reg(RTCC_HOURS_REG, 0x00);
	rtcc_write_reg(RTCC_DAY_REG, 0x01);
	rtcc_write_reg(RTCC_DATE_REG, 0x01);
	rtcc_write_reg(RTCC_MONTH_REG, 0x01);
	rtcc_write_reg(RTCC_YEAR_REG, 0x01);
	//
	rtcc_write_reg(RTCC_CONTROL_REG, 1<<3); // Enable EXTOSC
	rtcc_write_reg(RTCC_SECONDS_REG, 1<<7); //Write ST to 1 aka Start oscillator(bit7)

	clientDeselect();
}

void get_time(uint8_t* seconds, uint8_t* minutes, uint8_t* hours, uint8_t* day, uint8_t* date, uint8_t* month, uint8_t* year)
{
	clientSelect();
	*seconds = rtcc_read_reg(RTCC_SECONDS_REG);
	*minutes = rtcc_read_reg(RTCC_MINUTES_REG);
	*hours = rtcc_read_reg(RTCC_HOURS_REG);
	*day = rtcc_read_reg(RTCC_DAY_REG);
	*date = rtcc_read_reg(RTCC_DATE_REG);
	*month = rtcc_read_reg(RTCC_MONTH_REG);
	*year = rtcc_read_reg(RTCC_YEAR_REG);
	clientDeselect();
}

static void clientSelect(void)
{
	PORTC.DIR |= PIN2_bm; // set SS pin as output
	PORTC.OUT &= ~PIN2_bm; // Set SS pin value to LOW(RTCC)
	PORTC.PIN2CTRL |= PORT_PULLUPEN_bm;
}

static void clientDeselect(void)
{
	PORTC.DIR &= ~PIN2_bm; // set SS pin as input
	PORTC.OUT |= PIN2_bm; // Set SS pin value to HIGH(RTCC)
}

void get_time_string(char* time_string)
{
	uint8_t seconds=30, minutes, hours, day, date, month, year;
	get_time(&seconds, &minutes, &hours, &day, &date, &month, &year);
	//*time_string = seconds;
	sprintf(time_string, "Aeg: %02d:%02d:%02d %02d-%02d-%02d\n", hours, minutes, seconds, date, month, year);
}

void send_time_string_over_uart(void)
{
	char time_string[20];
	get_time_string(time_string);
	USART0_sendString(time_string);
}

int main(void)
{
	uint8_t data = 0;
	sei();
	SPI0_init();
	USART0_init();
	rtcc_init();
	//
	//sei();
	while (1)
	{
		send_time_string_over_uart();
		_delay_ms(1000);
		//clientSelect();
		//rtcc_init();
		//SPI0_exchangeData(data);
		//clientDeselect();
	}
}
