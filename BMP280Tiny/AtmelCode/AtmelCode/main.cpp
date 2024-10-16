/*
 * attiny88i2c.cpp
 *
 * Created: 19/09/2024 17:41:51
 * Author : Admin
 */ 

#include <avr/io.h>

// PIEZO_PINA PB1
// PIEZO_PINB PB2

#define SET_PIEZ0_A PORTB |= 0x02;
#define SET_PIEZ0_B PORTB |= 0x04;
#define CLR_PIEZ0_A PORTB &= 0xfd;
#define CLR_PIEZ0_B PORTB &= 0xfb;


void inline sleep_us(int8_t iD) {
	asm(
	"mov r25, %0 ; \n"
	"1: \n"
	"dec r25 \n"
	"nop  \n"
	"brne 1b \n"
	:
	: "r" (iD)
	: "r25" );
}

#define I2C_DELAY      sleep_us(5);


#define SDA_PIN PC4
#define SCL_PIN PC5

// Set SDA and SCL as inputs (high impedance) for HIGH state
#define I2C_SET_SDA     {DDRC &= 0xef;  PORTC |= 0x10;}// Set SDA as input
#define I2C_SET_SCL     {DDRC &= 0xdf;  PORTC |= 0x20;}  // Set SCL as input

// Set SDA and SCL as outputs and drive low
#define I2C_CLR_SDA      {DDRC |= 0x10; PORTC &= 0xef;}
#define I2C_CLR_SCL      {DDRC |= 0x20; PORTC &= 0xdf;}

// Generate a START condition
void i2c_start_condition( void )
{
	I2C_CLR_SDA
	I2C_DELAY
	I2C_CLR_SCL
	I2C_DELAY
}

// Generate a STOP condition
void i2c_stop_condition( void )
{
	I2C_CLR_SDA
	I2C_DELAY
	I2C_SET_SCL
	I2C_DELAY
	I2C_SET_SDA
	I2C_DELAY
}

bool i2c_write_byte( uint8_t B )
{
	uint8_t ack = 0;

	uint8_t i;
	for( i = 0; i < 8; i++ )
	{
		if(B & 0x80) I2C_SET_SDA
		else I2C_CLR_SDA
		B <<= 1;

		I2C_DELAY
		I2C_SET_SCL
		I2C_DELAY
		I2C_CLR_SCL
	}
	
	I2C_DELAY
	I2C_SET_SDA
	I2C_DELAY
	I2C_SET_SCL
	I2C_DELAY
	
	if( (PINC & 0x10) == 0x00 ) ack = 1;
	else ack = 0;

	I2C_CLR_SCL
	
	return ack;
}

uint8_t i2c_read_byte( bool ack )
{
	uint8_t B = 0;
	I2C_SET_SDA

	for( uint8_t i = 0; i < 8; i++ )
	{
		B <<= 1;
		do
		{
			I2C_SET_SDA
		} while ((PIND & 8) == 0); // clock stretching
		I2C_DELAY
		B |= (PIND & 1);
		I2C_DELAY
		I2C_CLR_SCL
	}

	if( ack ) I2C_CLR_SDA
	else I2C_SET_SDA
	I2C_SET_SCL
	I2C_DELAY
	I2C_CLR_SCL
	I2C_SET_SDA

	return B;
}

uint8_t calculate_crc(uint8_t* data, uint16_t length) {
	uint8_t crc = 0xFF;  // Initial CRC value
	for (uint16_t i = 0; i < length; i++) {
		crc ^= data[i];
		for (uint8_t j = 0; j < 8; j++) {
			if (crc & 0x80) {
				crc = (crc << 1) ^ 0x31;
				} else {
				crc <<= 1;
			}
		}
	}
	return crc;
}

int8_t i2c_write(uint8_t addr, const uint8_t* data, uint16_t count, bool appendCrc) {

	i2c_start_condition();
	
	if (!i2c_write_byte(0xc4)) {  // Write mode
		//if (!i2c_write_byte(addr << 1 | 1)) {  // Write mode
		i2c_stop_condition();
		return -1;  // Error: No ACK from sensor
	}
	
	
	
	
	
	i2c_stop_condition();
	return 0;  // Success






	for (uint16_t i = 0; i < count; i++) {
		if (!i2c_write_byte(data[i])) {  // Send each byte
			i2c_stop_condition();
			return -1;  // Error: No ACK from sensor
		}

		// If appendCrc is true, calculate and send the CRC after each block of data
		if (appendCrc) {
			uint8_t crc = calculate_crc((uint8_t*)&data[i], 1);  // CRC for one byte at a time
			if (!i2c_write_byte(crc)) {  // Send CRC byte
				i2c_stop_condition();
				return -1;  // Error: No ACK from sensor
			}
		}
	}

	i2c_stop_condition();
	return 0;  // Success
}

int8_t i2c_read(uint8_t addr, uint8_t* data, uint16_t count) {
	i2c_start_condition();
	if (!i2c_write_byte((addr << 1) | 0x01)) {  // Send address with read bit (last bit 1)
		i2c_stop_condition();
		return -1;  // Error: No ACK from sensor
	}

	for (uint16_t i = 0; i < count - 1; i++) {
		data[i] = i2c_read_byte(true);  // Read byte with ACK (acknowledge)
	}

	data[count - 1] = i2c_read_byte(false);  // Read the last byte with NACK (no acknowledge)

	i2c_stop_condition();
	return 0;  // Success
}

void beep(int8_t s)
{
	for(uint32_t i = 0; i<100;i++)
	{
		CLR_PIEZ0_B
		SET_PIEZ0_A
		sleep_us(s);
		sleep_us(s);
		sleep_us(s);
		sleep_us(s);
		sleep_us(s);
		sleep_us(s);
		sleep_us(s);
		sleep_us(s);
		sleep_us(s);
		sleep_us(s);
		CLR_PIEZ0_A
		SET_PIEZ0_B
		sleep_us(s);
		sleep_us(s);
		sleep_us(s);
		sleep_us(s);
		sleep_us(s);
		sleep_us(s);
		sleep_us(s);
		sleep_us(s);
		sleep_us(s);
		sleep_us(s);
	}
}

uint8_t bmp_init()
{
	i2c_start_condition();
	
	if (!i2c_write_byte(0x76 << 1)) {  // Write mode
		//if (!i2c_write_byte(addr << 1 | 1)) {  // Write mode
		i2c_stop_condition();
		return 0;  // Error: No ACK from sensor
	}
	
	i2c_stop_condition();
	return 1;  // Success
}

int main(void)
{
	DDRB = 0x06; // set piezo pins as outputs
	DDRC = 0x00; // all inputs
	PORTC = 0x30; // pullups enabled
	
	if(bmp_init())
		beep(100);
	else
		beep(255);
}

