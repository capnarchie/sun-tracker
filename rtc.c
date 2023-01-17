
#define F_CPU 3333333
#define USART0_BAUD_RATE(BAUD_RATE) ((float)(F_CPU * 64 / (16 * (float)BAUD_RATE)) + 0.5)

#include <avr/io.h>
#include <stdio.h>
#include <util/delay.h>
#include <string.h>

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

	SPI0.CTRLA = SPI_CLK2X_bm           /* Enable double-speed */
	| SPI_DORD_bm            /* LSB is transmitted first */
	| SPI_ENABLE_bm          /* Enable module */
	| SPI_MASTER_bm          /* SPI module in Master mode */
	| SPI_PRESC_DIV16_gc;    /* System Clock divided by 16 */
}
//https://github.com/microchip-pic-avr-examples/atmega4809-getting-started-with-spi-studio/tree/master/Sending_Data_as_Host
static uint8_t SPI0_exchangeData(uint8_t data)
{
	SPI0.DATA = data;

	while (!(SPI0.INTFLAGS & SPI_IF_bm))  /* waits until data is exchanged*/
	{
		;
	}

	return SPI0.DATA;
}

static void rtcc_write_reg(uint8_t reg, uint8_t data)
{
	uint8_t temp;
	clientSelect();//select rtcc
	temp = SPI0_exchangeData(reg);// send register address
	temp = SPI0_exchangeData(data); // send the data to write to register
	clientDeselect();// deselect the rtcc
}

uint8_t rtcc_read_reg(uint8_t reg)
{
	uint8_t data;
	uint8_t temp;
	clientSelect();//select rtcc
	temp = SPI0_exchangeData(reg); // send register address
	data = SPI0_exchangeData(0x00); //send dummy byte to receive
	clientDeselect();//deselect rtcc
	return data;
	
}

void rtcc_init(){
	
	//sets year to jan 1 2000 clock: 00:00:00
	rtcc_write_reg(RTCC_SECONDS_REG, 0x00);
    rtcc_write_reg(RTCC_MINUTES_REG, 0x00);
    rtcc_write_reg(RTCC_HOURS_REG, 0x00);
    rtcc_write_reg(RTCC_DAY_REG, 0x01);
    rtcc_write_reg(RTCC_DATE_REG, 0x01);
    rtcc_write_reg(RTCC_MONTH_REG, 0x01);
    rtcc_write_reg(RTCC_YEAR_REG, 0x00);
	
	// enable oscillator and battery backup
	rtcc_write_reg(RTCC_SECONDS_REG, 128); //Write ST to 1 aka Start oscillator(bit7)
	rtcc_write_reg(RTCC_DAY_REG, 16); // Write VBATEN to 1 aka enable battery backup to access rtcc features(bit3)
}

void get_time(uint8_t* seconds, uint8_t* minutes, uint8_t* hours, uint8_t* day, uint8_t* date, uint8_t* month, uint8_t* year)
{
	*seconds = rtcc_read_reg(RTCC_SECONDS_REG);
	*minutes = rtcc_read_reg(RTCC_MINUTES_REG);
	*hours = rtcc_read_reg(RTCC_HOURS_REG);
	*day = rtcc_read_reg(RTCC_DAY_REG);
	*date = rtcc_read_reg(RTCC_DATE_REG);
	*month = rtcc_read_reg(RTCC_MONTH_REG);
	*year = rtcc_read_reg(RTCC_YEAR_REG);
}

static void clientSelect(void)
{
	PORTA.OUT &= ~PIN2_bm; // Set SS pin value to LOW(RTCC)
}

static void clientDeselect(void)
{
	PORTA.OUT |= PIN2_bm; // Set SS pin value to HIGH(RTCC)
}

void get_time_string(char* time_string)
{
	uint8_t seconds, minutes, hours, day, date, month, year;
	get_time(&seconds, &minutes, &hours, &day, &date, &month, &year);
	sprintf(time_string, "%02d:%02d:%02d %02d-%02d-%02d", hours, minutes, seconds, date, month, year);
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
	SPI0_init();
	USART0_init();
	rtcc_init();
	while (1)
	{
		send_time_string_over_uart();
		//clientSelect();
		//rtcc_init();
		//SPI0_exchangeData(data);
		//clientDeselect();
	}
}
