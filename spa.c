//#include <math.h>
//
//#define pi 3.14159265
//
//double get_declination(int day, int month)
//{
  //double epsilon = pi/180 * 23.44 * cos(2 * pi * (day + 284) / 365);
  //double lambda = 2 * pi * (day - 81) / 365;
  //double delta = asin(sin(epsilon) * sin(lambda));
  //return delta;
//}
//
//double get_hour_angle(int hour, int minute)
//{
  //double t = hour + minute / 60.0 - 12;
  //double ha = pi / 12 * t;
  //return ha;
//}
//
//void get_sun_angles(double lat, double lon, int day, int month, int hour, int minute, double *elevation, double *azimuth)
//{
  //double delta = get_declination(day, month);
  //double ha = get_hour_angle(hour, minute);
  //double h = asin(sin(lat * pi / 180) * sin(delta) + cos(lat * pi / 180) * cos(delta) * cos(ha));
  //*elevation = h * 180 / pi;
  //double a = asin(-sin(ha) * cos(delta) / cos(h));
  //*azimuth = a * 180 / pi + 180;
//}
//
//int main()
//{
  //double lat = 58, lon = 26;
  //int day = 29, month = 1, hour = 18, minute = 30;
  //double elevation, azimuth;
  //get_sun_angles(lat, lon, day, month, hour, minute, &elevation, &azimuth);
  ////printf("Elevation: %f\nAzimuth: %f\n", elevation, azimuth);
  //return 0;
//}
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

	SPI0_CTRLB |= SPI_SSD_bm; // Disable Master slave transition on SS pin
	//SPI0_CTRLB |= ~SPI_MODE0_bm;
	SPI0.CTRLA = SPI_CLK2X_bm           /* Enable double-speed */
	| 0						 /* MSB is transmitted first */
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
	clientSelect();//select rtcc
	SPI0_exchangeData(0b00010010); // write command
	SPI0_exchangeData(reg);// send register address
	SPI0_exchangeData(data); // send the data to write to register
	clientDeselect();// deselect the rtcc
}

uint8_t rtcc_read_reg(uint8_t reg)
{
	uint8_t data;
	clientSelect();//select rtcc
	SPI0_exchangeData(0b00010011); //read command
	SPI0_exchangeData(reg); // send register address
	data = SPI0_exchangeData(0x00); //send dummy byte to receive
	clientDeselect();//deselect rtcc
	return data;
	
}

void rtcc_init(){
	//clientSelect();

	volatile uint8_t seconds = rtcc_read_reg(RTCC_SECONDS_REG);
	seconds = seconds & ~(1<<6);
	rtcc_write_reg(RTCC_SECONDS_REG, seconds); //Write ST to 0 aka disable oscillator(bit7)
	volatile uint8_t control = rtcc_read_reg(RTCC_CONTROL_REG);
	control = control & ~(1<<2);//disable extosc
	rtcc_write_reg(RTCC_CONTROL_REG, control);
	volatile uint8_t OSCRUNNING = rtcc_read_reg(RTCC_DAY_REG);
	while (rtcc_read_reg(RTCC_DAY_REG) & (1<<5)); //wait OSCRUN to clear after EXTOSC disable

	
	// Set time to 22:35:30 28-01-2023
	rtcc_write_reg(RTCC_SECONDS_REG, 0b00110000);//0b10110000);
	rtcc_write_reg(RTCC_MINUTES_REG, 0b00110101);
	rtcc_write_reg(RTCC_HOURS_REG, 0b00100010);
	rtcc_write_reg(RTCC_DAY_REG, 0b00100110);
	rtcc_write_reg(RTCC_DATE_REG, 0b00101000);
	rtcc_write_reg(RTCC_MONTH_REG, 0b00000001);
	rtcc_write_reg(RTCC_YEAR_REG, 0b00100011);
	
	seconds = rtcc_read_reg(RTCC_SECONDS_REG);
	seconds |= 1<<7;
	rtcc_write_reg(RTCC_SECONDS_REG, seconds);
	
	control = rtcc_read_reg(RTCC_CONTROL_REG);
	control |= 1<<3;
	rtcc_write_reg(RTCC_CONTROL_REG, control);

	// Wait for OSCRUN to clear
	while (rtcc_read_reg(RTCC_DAY_REG) & (1<<5));

	//clientDeselect();
}

void get_time(uint8_t* seconds, uint8_t* minutes, uint8_t* hours, uint8_t* day, uint8_t* date, uint8_t* month, uint8_t* year)
{
	//clientSelect();
	*seconds = rtcc_read_reg(RTCC_SECONDS_REG);
	*minutes = rtcc_read_reg(RTCC_MINUTES_REG);
	*hours = rtcc_read_reg(RTCC_HOURS_REG);
	*day = rtcc_read_reg(RTCC_DAY_REG);
	*date = rtcc_read_reg(RTCC_DATE_REG);
	*month = rtcc_read_reg(RTCC_MONTH_REG);
	*year = rtcc_read_reg(RTCC_YEAR_REG);
	
	// BCD decode
	*seconds = (*seconds & 0xF) + ((*seconds >> 4) & 0x7) * 10;
	*minutes = (*minutes & 0xF) + ((*minutes >> 4) & 0x7) * 10;
	*hours = ((*hours >> 4) & 0x03) * 10 + (*hours & 0b1111);
	*date = ( (*date & 0xF) + ((*date >> 4) * 10) );
	*month = ((*month & 0b00011000) >> 3) * 10 + (*month & 0b00001111);
	*year = ((*year & 0xF0) >> 4) * 10 + (*year & 0x0F);
	
	
	//clientDeselect();
}

void clientSelect(void)
{
	PORTC.DIR |= PIN2_bm; // set SS pin as output
	PORTC.OUT &= ~PIN2_bm; // Set SS pin value to LOW(RTCC)
	PORTC.PIN2CTRL |= PORT_PULLUPEN_bm;
}

void clientDeselect(void)
{
	PORTC.DIR &= ~PIN2_bm; // set SS pin as input
	PORTC.OUT |= PIN2_bm; // Set SS pin value to HIGH(RTCC)
	PORTC.PIN2CTRL &= ~PORT_PULLUPEN_bm;
}

int main(void)
{
	uint8_t seconds, minutes, hours, day, date, month, year;
	char time_string[20];
	sei();
	SPI0_init();
	USART0_init();
	rtcc_init();
	
	encoder_init();
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
	//
	//sei();
	while (1)
	{
		while (PORTA.IN & (1<<2) || PORTA.IN & (1<<3)) // Check if either of the switch has been pressed
		{
			PORTD.OUT = 0; // kill everything
		}
		
		get_time(&seconds, &minutes, &hours, &day, &date, &month, &year);
		sprintf(time_string, "Aeg: %02d:%02d:%02d %02d-%02d-%02d\n", hours, minutes, seconds, date, month, year);
		
		USART0_sendString(time_string);
		//_delay_ms(1000);
	}
}
