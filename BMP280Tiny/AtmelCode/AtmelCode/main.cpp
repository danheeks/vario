// main.cpp Dan Heeks 20th October 2024
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>

// PIEZO_PINA PB1
// PIEZO_PINB PB2

#define SET_PIEZ0_A PORTB |= 0x02;
#define SET_PIEZ0_B PORTB |= 0x04;
#define CLR_PIEZ0_A PORTB &= 0xfd;
#define CLR_PIEZ0_B PORTB &= 0xfb;

#define F_CPU 8000000UL  // Set clock frequency to 8 MHz

#define BMP_280_ADDR 0x76

int delay_a = 0;
void delay(uint16_t ms)
{
	uint32_t i;
	uint32_t length = ms;
	length *= 600;
	
	for(i = 0; i<length; i++)
	{
		delay_a = 1;
	}
}

volatile uint16_t remaining_ms = 0;  // Variable to track the remaining delay time

// Watchdog Interrupt Service Routine (ISR)
ISR(WDT_vect) {
	if (remaining_ms >= 16) {
		remaining_ms -= 16;  // Decrease remaining time by 16 ms (WDT interval)
		} else {
		remaining_ms = 0;    // Ensure it doesn't go below zero
	}
}

void delay_ms(uint16_t ms) {
    remaining_ms = ms;

    // Set up the Watchdog Timer for 16 ms interrupts
    cli();  // Disable interrupts while configuring the WDT

    // Enable change of WDT settings
    wdt_reset();  // Reset the WDT
    MCUSR &= ~(1 << WDRF);  // Clear WDT reset flag
    WDTCSR |= (1 << WDCE) | (1 << WDE);  // Enable timed sequence for changing WDT settings

    // Set WDT to interrupt mode only, with the 16 ms timeout
    WDTCSR = (1 << WDIE);  // Set WDP0 = 0 (default), enabling the 16 ms timeout, enable WDT interrupt

    sei();  // Re-enable global interrupts

    // Enter low-power mode and wait for the delay to finish
    while (remaining_ms > 0) {
	    set_sleep_mode(SLEEP_MODE_IDLE);  // Power-down mode
	    sleep_mode();  // Go to sleep until WDT interrupt occurs
    }

    // Disable the Watchdog Timer after the delay is finished
    cli();
    wdt_reset();  // Reset the WDT again
    WDTCSR |= (1 << WDCE) | (1 << WDE);  // Enter WDT configuration mode
    WDTCSR = 0x00;  // Disable the WDT
    sei();  // Re-enable global interrupts
}

void led1_on()
{
	PORTB |= 0x20; // led1 on
}

void led1_off()
{
	PORTB &= 0xdf; // led1 off
}

void led2_on()
{
	PORTB |= 0x01; // led2 on
}

void led2_off()
{
	PORTB &= 0xfe; // led2 off
}

void flash_led1(uint16_t ms)
{
	led1_on();
	delay(ms);
	led1_off();	
}

void flash_led2(uint16_t ms)
{
	led2_on();
	delay(ms);
	led2_off();
}

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

void toneAC(uint16_t frequency) {
	// Calculate the value for the Output Compare Register
	uint16_t ocr_value = (F_CPU / (2 * frequency)) - 1;

	// Set PB1 and PB2 as output
	DDRB |= (1 << PB1) | (1 << PB2);
	PORTB |= (1 << PB1);

	// Set Timer1 in CTC mode
	TCCR1A = 0;  // Normal port operation, OC1A/OC1B disconnected
	TCCR1B = (1 << WGM12) | (1 << CS10);  // CTC mode, no prescaler

	// Set compare value
	OCR1A = ocr_value;

	// Enable Output Compare A Match Interrupt
	TIMSK1 |= (1 << OCIE1A);
}

void stop_tone() {
	TCCR1B &= ~(1 << CS10);  // Stop the timer (no clock source)
}

void beep(uint16_t frequency, uint16_t time_ms)
{
	toneAC(frequency);
	delay(time_ms);
	stop_tone();
}

uint8_t tone_count = 0;

ISR(TIMER1_COMPA_vect) {
	// Toggle PB1 and PB2 on every compare match to create the tone
	if(tone_count == 0)
	{
		PORTB |= 0b00000010;
		PORTB &= 0b11111110;
		tone_count = 1;
	}
	else
	{
		PORTB |= 0b00000001;
		PORTB &= 0b11111101;
		tone_count = 0;
	}
}

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

bool i2c_write_uint8_t( uint8_t B )
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
		I2C_DELAY
	}
	
	I2C_SET_SDA
	//I2C_DELAY
	I2C_SET_SCL
	I2C_DELAY
	
	if( (PINC & 0x10) == 0x00 ) ack = 1;
	else ack = 0;

	I2C_CLR_SCL
	
	return ack;
}

uint8_t i2c_read_uint8_t( bool ack )
{
	uint8_t data = 0;
	I2C_SET_SDA

	for( uint8_t i = 0; i < 8; i++ )
	{
		data <<= 1;
	    do{
		   I2C_SET_SCL
	    }while((PINC & 0x20) == 0);  //clock stretching	I2C_DELAY
		I2C_DELAY
		if((PINC & 0x10)!=0)data |= 1;
		I2C_DELAY
		I2C_CLR_SCL
	}

	if( ack ) I2C_CLR_SDA
	else I2C_SET_SDA
	I2C_SET_SCL
	I2C_DELAY
	I2C_CLR_SCL
	I2C_SET_SDA

	return data;
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

int8_t i2c_write(const uint8_t* data, uint16_t count, bool appendCrc) {

	i2c_start_condition();
	
	if (!i2c_write_uint8_t(BMP_280_ADDR << 1)) {  // Write mode
		flash_led2(1000);
		i2c_stop_condition();
		return -1;  // Error: No ACK from sensor
	}

	for (uint16_t i = 0; i < count; i++) {
		if (!i2c_write_uint8_t(data[i])) {  // Send each uint8_t
			i2c_stop_condition();
			flash_led2(100);
			return -1;  // Error: No ACK from sensor
		}

		// If appendCrc is true, calculate and send the CRC after each block of data
		if (appendCrc) {
			uint8_t crc = calculate_crc((uint8_t*)&data[i], 1);  // CRC for one uint8_t at a time
			if (!i2c_write_uint8_t(crc)) {  // Send CRC uint8_t
				i2c_stop_condition();
				flash_led2(2000);
				return -1;  // Error: No ACK from sensor
			}
		}
	}

	i2c_stop_condition();
	return 0;  // Success
}

int8_t i2c_read(uint8_t* data, uint16_t count) {
	
	i2c_start_condition();
	if (!i2c_write_uint8_t((BMP_280_ADDR << 1) | 1)) {  // Send address with read bit (last bit 1)
		i2c_stop_condition();
		flash_led2(100);
		delay(100);
		flash_led2(100);
		delay(100);
		flash_led2(100);
		delay(100);
		flash_led2(100);
		delay(100);
		flash_led2(100);
		delay(100);
		return -1;  // Error: No ACK from sensor
	}
	
	for (uint16_t i = 0; i < count - 1; i++) {
		data[i] = i2c_read_uint8_t(true);  // Read uint8_t with ACK (acknowledge)
	}

	data[count - 1] = i2c_read_uint8_t(false);  // Read the last uint8_t with NACK (no acknowledge)

	i2c_stop_condition();
	return 0;  // Success
}

void tone(uint16_t frequency, uint16_t duration_ms) {
	// Set PB1 and PB2 as outputs
	DDRB |= (1 << PB1) | (1 << PB2);

	// Set Fast PWM mode, non-inverted output on OC1A (PB1) and OC1B (PB2)
	TCCR1A |= (1 << COM1A0) | (1 << COM1B0);  // Toggle OC1A/OC1B on compare match
	TCCR1A |= (1 << WGM10);  // Fast PWM 8-bit
	TCCR1B |= (1 << WGM12) | (1 << CS10);  // Fast PWM, no prescaling

	// Calculate the OCR1A value for the desired frequency
	uint16_t ocr_value = (F_CPU / (2 * frequency)) - 1;
	OCR1A = ocr_value;  // Set the value to Timer1 for the frequency
	OCR1B = ocr_value;  // Set OC1B to mirror OC1A (for PB2)

	// Generate the tone for the specified duration
	for (uint16_t i = 0; i < duration_ms; i++) {
		delay(1);
	}

	// Stop the timer after the tone is played
	TCCR1A = 0;  // Clear the Timer1 control registers
	TCCR1B = 0;
	PORTB &= ~((1 << PB1) | (1 << PB2));  // Turn off the buzzer
}


uint8_t bmp_init()
{
	i2c_start_condition();
	
	if (!i2c_write_uint8_t(0x76 << 1)) {  // Write mode
		i2c_stop_condition();
		return 0;  // Error: No ACK from sensor
	}
	
	i2c_stop_condition();
	return 1;  // Success
}


/*!
 * Registers available on the sensor.
 */
enum {
  BMP280_REGISTER_DIG_T1 = 0x88,
  BMP280_REGISTER_DIG_T2 = 0x8A,
  BMP280_REGISTER_DIG_T3 = 0x8C,
  BMP280_REGISTER_DIG_P1 = 0x8E,
  BMP280_REGISTER_DIG_P2 = 0x90,
  BMP280_REGISTER_DIG_P3 = 0x92,
  BMP280_REGISTER_DIG_P4 = 0x94,
  BMP280_REGISTER_DIG_P5 = 0x96,
  BMP280_REGISTER_DIG_P6 = 0x98,
  BMP280_REGISTER_DIG_P7 = 0x9A,
  BMP280_REGISTER_DIG_P8 = 0x9C,
  BMP280_REGISTER_DIG_P9 = 0x9E,
  BMP280_REGISTER_CHIPID = 0xD0,
  BMP280_REGISTER_VERSION = 0xD1,
  BMP280_REGISTER_SOFTRESET = 0xE0,
  BMP280_REGISTER_CAL26 = 0xE1, /**< R calibration = 0xE1-0xF0 */
  BMP280_REGISTER_STATUS = 0xF3,
  BMP280_REGISTER_CONTROL = 0xF4,
  BMP280_REGISTER_CONFIG = 0xF5,
  BMP280_REGISTER_PRESSUREDATA = 0xF7,
  BMP280_REGISTER_TEMPDATA = 0xFA,
};

  /** Oversampling rate for the sensor. */
  enum sensor_sampling {
    /** No over-sampling. */
    SAMPLING_NONE = 0x00,
    /** 1x over-sampling. */
    SAMPLING_X1 = 0x01,
    /** 2x over-sampling. */
    SAMPLING_X2 = 0x02,
    /** 4x over-sampling. */
    SAMPLING_X4 = 0x03,
    /** 8x over-sampling. */
    SAMPLING_X8 = 0x04,
    /** 16x over-sampling. */
    SAMPLING_X16 = 0x05
  };

  /** Operating mode for the sensor. */
  enum sensor_mode {
    /** Sleep mode. */
    MODE_SLEEP = 0x00,
    /** Forced mode. */
    MODE_FORCED = 0x01,
    /** Normal mode. */
    MODE_NORMAL = 0x03,
    /** Software reset. */
    MODE_SOFT_RESET_CODE = 0xB6
  };

  /** Filtering level for sensor data. */
  enum sensor_filter {
    /** No filtering. */
    FILTER_OFF = 0x00,
    /** 2x filtering. */
    FILTER_X2 = 0x01,
    /** 4x filtering. */
    FILTER_X4 = 0x02,
    /** 8x filtering. */
    FILTER_X8 = 0x03,
    /** 16x filtering. */
    FILTER_X16 = 0x04
  };

  /** Standby duration in ms */
  enum standby_duration {
    /** 1 ms standby. */
    STANDBY_MS_1 = 0x00,
    /** 62.5 ms standby. */
    STANDBY_MS_63 = 0x01,
    /** 125 ms standby. */
    STANDBY_MS_125 = 0x02,
    /** 250 ms standby. */
    STANDBY_MS_250 = 0x03,
    /** 500 ms standby. */
    STANDBY_MS_500 = 0x04,
    /** 1000 ms standby. */
    STANDBY_MS_1000 = 0x05,
    /** 2000 ms standby. */
    STANDBY_MS_2000 = 0x06,
    /** 4000 ms standby. */
    STANDBY_MS_4000 = 0x07
  };

  /** Encapsulates the config register */
  struct config {
    /** Initialize to power-on-reset state */
    config() : t_sb(STANDBY_MS_1), filter(FILTER_OFF), none(0), spi3w_en(0) {}
    /** Inactive duration (standby time) in normal mode */
    unsigned int t_sb : 3;
    /** Filter settings */
    unsigned int filter : 3;
    /** Unused - don't set */
    unsigned int none : 1;
    /** Enables 3-wire SPI */
    unsigned int spi3w_en : 1;
    /** Used to retrieve the assembled config register's uint8_t value. */
    unsigned int get() { return (t_sb << 5) | (filter << 2) | spi3w_en; }
  };

  /** Encapsulates trhe ctrl_meas register */
  struct ctrl_meas {
    /** Initialize to power-on-reset state */
    ctrl_meas()
        : osrs_t(SAMPLING_NONE), osrs_p(SAMPLING_NONE), mode(MODE_SLEEP) {}
    /** Temperature oversampling. */
    unsigned int osrs_t : 3;
    /** Pressure oversampling. */
    unsigned int osrs_p : 3;
    /** Device mode */
    unsigned int mode : 2;
    /** Used to retrieve the assembled ctrl_meas register's uint8_t value. */
    unsigned int get() { return (osrs_t << 5) | (osrs_p << 2) | mode; }
  };

typedef struct {
  uint16_t dig_T1; /**< dig_T1 cal register. */
  int16_t dig_T2;  /**<  dig_T2 cal register. */
  int16_t dig_T3;  /**< dig_T3 cal register. */

  uint16_t dig_P1; /**< dig_P1 cal register. */
  int16_t dig_P2;  /**< dig_P2 cal register. */
  int16_t dig_P3;  /**< dig_P3 cal register. */
  int16_t dig_P4;  /**< dig_P4 cal register. */
  int16_t dig_P5;  /**< dig_P5 cal register. */
  int16_t dig_P6;  /**< dig_P6 cal register. */
  int16_t dig_P7;  /**< dig_P7 cal register. */
  int16_t dig_P8;  /**< dig_P8 cal register. */
  int16_t dig_P9;  /**< dig_P9 cal register. */
} bmp280_calib_data;

int32_t t_fine;
bmp280_calib_data _bmp280_calib;
float prev_p = 0;
config _configReg;
ctrl_meas _measReg;
  
uint8_t write_then_read(const uint8_t *write_buffer, uint8_t write_len, uint8_t *read_buffer, uint8_t read_len) {
	if (i2c_write(write_buffer, write_len, false) != 0) {
		beep(300,500);
		return false;
	}

	return i2c_read(read_buffer, read_len);
}


uint8_t read8(uint8_t reg) {
  uint8_t buffer[1];
  buffer[0] = uint8_t(reg);
  write_then_read(buffer, 1, buffer, 1);
  return buffer[0];
}

void write8(uint8_t reg, uint8_t value) {
  uint8_t buffer[2];
  buffer[1] = value;
  buffer[0] = reg;
  i2c_write(buffer, 2, false);
}

uint32_t read24(uint8_t reg) {
  uint8_t buffer[3];

  buffer[0] = uint8_t(reg);
  
  write_then_read(buffer, 1, buffer, 3);
  
  return uint32_t(buffer[0]) << 16 | uint32_t(buffer[1]) << 8 |
         uint32_t(buffer[2]);
}

uint16_t read16(uint8_t reg) {
  uint8_t buffer[2];

    buffer[0] = uint8_t(reg);
    write_then_read(buffer, 1, buffer, 2);
  return uint16_t(buffer[0]) << 8 | uint16_t(buffer[1]);
}

uint16_t read16_LE(uint8_t reg) {
  uint16_t temp = read16(reg);
  return (temp >> 8) | (temp << 8);
}

int16_t readS16_LE(uint8_t reg) {
  return (int16_t)read16_LE(reg);
}

void readCoefficients() {
  _bmp280_calib.dig_T1 = read16_LE(BMP280_REGISTER_DIG_T1);
  _bmp280_calib.dig_T2 = readS16_LE(BMP280_REGISTER_DIG_T2);
  _bmp280_calib.dig_T3 = readS16_LE(BMP280_REGISTER_DIG_T3);

  _bmp280_calib.dig_P1 = read16_LE(BMP280_REGISTER_DIG_P1);
  _bmp280_calib.dig_P2 = readS16_LE(BMP280_REGISTER_DIG_P2);
  _bmp280_calib.dig_P3 = readS16_LE(BMP280_REGISTER_DIG_P3);
  _bmp280_calib.dig_P4 = readS16_LE(BMP280_REGISTER_DIG_P4);
  _bmp280_calib.dig_P5 = readS16_LE(BMP280_REGISTER_DIG_P5);
  _bmp280_calib.dig_P6 = readS16_LE(BMP280_REGISTER_DIG_P6);
  _bmp280_calib.dig_P7 = readS16_LE(BMP280_REGISTER_DIG_P7);
  _bmp280_calib.dig_P8 = readS16_LE(BMP280_REGISTER_DIG_P8);
  _bmp280_calib.dig_P9 = readS16_LE(BMP280_REGISTER_DIG_P9);
}

void setSampling()
{
  _measReg.mode = MODE_NORMAL;
  _measReg.osrs_t = SAMPLING_X2;
  _measReg.osrs_p = SAMPLING_X16;

  _configReg.filter = FILTER_X16;
  _configReg.t_sb = STANDBY_MS_63;

  write8(BMP280_REGISTER_CONFIG, _configReg.get());
  write8(BMP280_REGISTER_CONTROL, _measReg.get());
}

float readTemperature() {
  int32_t var1, var2;

  int32_t adc_T = read24(BMP280_REGISTER_TEMPDATA);
  
  adc_T >>= 4;

  var1 = ((((adc_T >> 3) - ((int32_t)_bmp280_calib.dig_T1 << 1))) *
          ((int32_t)_bmp280_calib.dig_T2)) >>
         11;

  var2 = (((((adc_T >> 4) - ((int32_t)_bmp280_calib.dig_T1)) *
            ((adc_T >> 4) - ((int32_t)_bmp280_calib.dig_T1))) >>
           12) *
          ((int32_t)_bmp280_calib.dig_T3)) >>
         14;

  t_fine = var1 + var2;

  float T = (t_fine * 5 + 128) >> 8;
  return T / 100;
}

float readPressure() {
  int64_t var1, var2, p;

  // Must be done first to get the t_fine variable set up
  readTemperature();

  int32_t adc_P = read24(BMP280_REGISTER_PRESSUREDATA);
  adc_P >>= 4;

  var1 = ((int64_t)t_fine) - 128000;
  var2 = var1 * var1 * (int64_t)_bmp280_calib.dig_P6;
  var2 = var2 + ((var1 * (int64_t)_bmp280_calib.dig_P5) << 17);
  var2 = var2 + (((int64_t)_bmp280_calib.dig_P4) << 35);
  var1 = ((var1 * var1 * (int64_t)_bmp280_calib.dig_P3) >> 8) +
         ((var1 * (int64_t)_bmp280_calib.dig_P2) << 12);
  var1 =
      (((((int64_t)1) << 47) + var1)) * ((int64_t)_bmp280_calib.dig_P1) >> 33;

  if (var1 == 0) {
    return 0; // avoid exception caused by division by zero
  }
  p = 1048576 - adc_P;
  p = (((p << 31) - var2) * 3125) / var1;
  var1 = (((int64_t)_bmp280_calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
  var2 = (((int64_t)_bmp280_calib.dig_P8) * p) >> 19;

  p = ((p + var1 + var2) >> 8) + (((int64_t)_bmp280_calib.dig_P7) << 4);
  return (float)p / 256;
}

int main(void)
{
	DDRB = 0x27; // set piezo pins as outputs, also LEDs
	// B0 - 1 - LED2
	// B1 - 2 - Piezo
	// B2 - 4 - Piezo
	// B3 - 8 - MOSI
	// B4 - 10 - MISO
	// B5 - 20 - SCK and LED1
	// B6 - 40 - Not Used
	// B7 - 80 - Not Used
	
	PORTB &= 0xf7;
	PORTB = 0;

    // Enable global interrupts for the tone timer
    sei();
	
	beep(388, 70);
	delay_ms(1000);
	beep(590, 70);

	delay(500); // wait for sensor to settle

	// try init 10 times	
	for(int i = 0; i<10; i++)
	{
		if(bmp_init())
			break;

		if(i == 9)
		{
			beep(300,1000); // failed and giving up sound
			return 0;
		}
		beep(300,100); // failed sound
	}

	// i2c has acknowledged sensor's address
	beep(800, 100); // success sound
	
	readCoefficients();
	
	float prev_pressure = readPressure();

	setSampling();
  
	while(1)
	{
		float pressure = readPressure();
		float rate = (pressure - prev_pressure) * 15.0;
		prev_pressure = pressure;
		if(rate < -10)
		{
			float pitch = 500.0 - (rate * 10.0);
			if(pitch > 2000)pitch = 2000;
			toneAC((float)pitch);
			delay_ms(120);
			stop_tone();
			delay_ms(120);
		}
		else if(rate>30)
		{
			toneAC(300);
			delay_ms(250);
		}
		else
		{
			stop_tone();
			delay_ms(250);
		}		
	}
}

