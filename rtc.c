#include <avr/io.h>

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

//https://github.com/microchip-pic-avr-examples/atmega4809-getting-started-with-spi-studio/tree/master/Sending_Data_as_Host
static void SPI0_init(void)
{
	PORTA.DIR |= PIN4_bm; /* Set MOSI pin direction to output */
	PORTA.DIR &= ~PIN5_bm; /* Set MISO pin direction to input */
	PORTA.DIR |= PIN6_bm; /* Set SCK pin direction to output */
	PORTA.DIR |= PIN7_bm; /* Set SS pin direction to output */

	SPI0.CTRLA = SPI_CLK2X_bm           /* Enable double-speed */
	| SPI_DORD_bm            /* LSB is transmitted first */
	| SPI_ENABLE_bm          /* Enable module */
	| SPI_MASTER_bm          /* SPI module in Master mode */
	| SPI_PRESC_DIV16_gc;    /* System Clock divided by 16 */
}

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
	//select rtcc
	SPI0_exchangeData(reg)// send register address
	SPI0_exchangeData(data) // send the data to write to register
	// deselect the rtcc
}

uint8_t rtcc_read_reg(uint8_t reg)
{
	uint8_t data;
	uint8_t temp;
	//select rtcc
	temp = SPI0_exchangeData(reg) // send register address
	data = SPI0_exchangeData(0x00) //send dummy byte to receive
	// deselect rtcc
	return data;
	
}

void rtcc_init(){
	// enable oscillator and battery backup
	rtcc_write_reg(RTCC_SECONDS_REG, 0x00);
    rtcc_write_reg(RTCC_MINUTES_REG, 0x00);
    rtcc_write_reg(RTCC_HOURS_REG, 0x00);
    rtcc_write_reg(RTCC_DAY_REG, 0x01);
    rtcc_write_reg(RTCC_DATE_REG, 0x01);
    rtcc_write_reg(RTCC_MONTH_REG, 0x01);
    rtcc_write_reg(RTCC_YEAR_REG, 0x00);
	//sets year to jan 1 2000 clock: 00:00:00
}

static void clientSelect(void)
{
	PORTA.OUT &= ~PIN7_bm; // Set SS pin value to LOW
}

static void clientDeselect(void)
{
	PORTA.OUT |= PIN7_bm; // Set SS pin value to HIGH
}

int main(void)
{
	uint8_t data = 0;
	SPI0_init();

	while (1)
	{
		clientSelect();
		SPI0_exchangeData(data);
		clientDeselect();
	}
}
