/*
  Blink

  Turns an LED on for one second, then off for one second, repeatedly.

  Most Arduinos have an on-board LED you can control. On the UNO, MEGA and ZERO
  it is attached to digital pin 13, on MKR1000 on pin 6. LED_BUILTIN is set to
  the correct LED pin independent of which board is used.
  If you want to know what pin the on-board LED is connected to on your Arduino
  model, check the Technical Specs of your board at:
  https://www.arduino.cc/en/Main/Products

  modified 8 May 2014
  by Scott Fitzgerald
  modified 2 Sep 2016
  by Arturo Guadalupi
  modified 8 Sep 2016
  by Colby Newman

  This example code is in the public domain.

  https://www.arduino.cc/en/Tutorial/BuiltInExamples/Blink
*/
#include <Wire.h>
#include <Adafruit_I2CDevice.h>
#include <toneAC.h>

#define VOLUME 1
// #define USE_CALIBRATION
#define USE_TONE

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
    /** Used to retrieve the assembled config register's byte value. */
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
    /** Used to retrieve the assembled ctrl_meas register's byte value. */
    unsigned int get() { return (osrs_t << 5) | (osrs_p << 2) | mode; }
  };

#ifdef USE_CALIBRATION
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
#endif

//Adafruit_BMP280 bmp; // use I2C interface
//Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();
Adafruit_I2CDevice *i2c_dev = NULL;
int led_delay = 1000;
int32_t _sensorID = 0;
#ifdef USE_CALIBRATION
  int32_t t_fine;
  bmp280_calib_data _bmp280_calib;
  int64_t prev_p = 0;
#else
  int32_t prev_p = 0;
#endif
  config _configReg;
  ctrl_meas _measReg;

uint8_t read8(byte reg) {
  uint8_t buffer[1];
     buffer[0] = uint8_t(reg);
    i2c_dev->write_then_read(buffer, 1, buffer, 1);
  return buffer[0];
}

void write8(byte reg, byte value) {
  byte buffer[2];
  buffer[1] = value;
  buffer[0] = reg;
  i2c_dev->write(buffer, 2);
}

uint32_t read24(byte reg) {
  uint8_t buffer[3];

    buffer[0] = uint8_t(reg);
    i2c_dev->write_then_read(buffer, 1, buffer, 3);

  return uint32_t(buffer[0]) << 16 | uint32_t(buffer[1]) << 8 |
         uint32_t(buffer[2]);
}
#ifdef USE_CALIBRATION
uint16_t read16(byte reg) {
  uint8_t buffer[2];

    buffer[0] = uint8_t(reg);
    i2c_dev->write_then_read(buffer, 1, buffer, 2);
  return uint16_t(buffer[0]) << 8 | uint16_t(buffer[1]);
}

uint16_t read16_LE(byte reg) {
  uint16_t temp = read16(reg);
  return (temp >> 8) | (temp << 8);
}

int16_t readS16_LE(byte reg) {
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
#endif

void setSampling()
{
  if (!_sensorID)
    return; // begin() not called yet
  _measReg.mode = MODE_NORMAL;
  _measReg.osrs_t = SAMPLING_X16;
  _measReg.osrs_p = SAMPLING_X16;

  _configReg.filter = FILTER_X16;
  _configReg.t_sb = STANDBY_MS_63;

  write8(BMP280_REGISTER_CONFIG, _configReg.get());
  write8(BMP280_REGISTER_CONTROL, _measReg.get());
}

// the setup function runs once when you press reset or power the board
void setup() {
#ifdef USE_TONE
   toneAC(388, 4);
  delay(70);
  toneAC(590, 4);
  delay(70);
  toneAC();
#endif
  delay(3000); // wait for sensor to settle
 
i2c_dev = new Adafruit_I2CDevice(0x76, &Wire);
if (i2c_dev->begin())
{
  delay(100);
  _sensorID = read8(BMP280_REGISTER_CHIPID);
}

if(_sensorID == 0x58)
{
// success
  led_delay = 100; 
}
else
{
  // to do, play error tune
  return;
}
#ifdef USE_CALIBRATION
  readCoefficients();
#endif

  setSampling();
  delay(100);


  // initialize digital pin LED_BUILTIN as an output.
  pinMode(PIN_PB0, OUTPUT);
}

#ifdef USE_CALIBRATION
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

int64_t readPressure() {
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
  return p;
}
#endif

// the loop function runs over and over again forever
void loop() {
  if(_sensorID != 0x58)
  {
    // setup failed
    return;
  }

#ifdef USE_CALIBRATION
  int64_t p = readPressure();
  if((p - prev_p) < -150)
    digitalWrite(PIN_PB0, HIGH);  // turn the LED on (HIGH is the voltage level)
  else
    digitalWrite(PIN_PB0, LOW);   // turn the LED off by making the voltage LOW
#else
  int32_t p = read24(BMP280_REGISTER_PRESSUREDATA);
  int32_t rate = prev_p - p;
  if(rate < -90)
  {
  int16_t pitch = 500 - rate;
  if(pitch > 2000)pitch = 2000;
#ifdef USE_TONE
  toneAC(pitch, VOLUME);
#endif
//  LowPower.idle(SLEEP_120MS, ADC_OFF, TIMER2_OFF, TIMER1_ON, TIMER0_OFF, SPI_OFF,USART0_OFF, TWI_OFF);
    delay(120);
#ifdef USE_TONE
  toneAC(0);
#endif
//  LowPower.idle(SLEEP_120MS, ADC_OFF, TIMER2_OFF, TIMER1_ON, TIMER0_OFF, SPI_OFF,USART0_OFF, TWI_OFF);
    delay(120);
  }
  else if(rate>360)
  {
  #ifdef USE_TONE
    toneAC(300, 1);
  #endif
//    LowPower.idle(SLEEP_250MS, ADC_OFF, TIMER2_OFF, TIMER1_ON, TIMER0_OFF, SPI_OFF,USART0_OFF, TWI_OFF);
    delay(250);
  }
  else
  {
#ifdef USE_TONE
    toneAC(0);
#endif
    //LowPower.idle(SLEEP_250MS, ADC_OFF, TIMER2_OFF, TIMER1_ON, TIMER0_OFF, SPI_OFF,USART0_OFF, TWI_OFF);
    delay(250);
  }
  #endif
  prev_p = p;
}
