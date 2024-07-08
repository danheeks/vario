#include <toneAC.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <LowPower.h>

Adafruit_BMP280 bmp; // use I2C interface
Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();
/*
Hardware connections:

- (GND) to GND
+ (VDD) to 3.3V
SCL to A5
SDA to A4

D9 and D10 to the Piezo Buzzer ( see toneAC.h )

*/

float old_pressure = 0;

void setup() {
  Serial.begin(9600);
// play start sound
   toneAC(388, 4);
  delay(70);
  unsigned status;
  status = bmp.begin(0x76);
  Serial.println(status);
  if(!status)
  {
    toneAC(250, 10);
    delay(150);
    toneAC(0);
    delay(150);
    toneAC(250, 10);
    delay(150);
    toneAC(0);
    delay(150);
    toneAC(250, 10);
    delay(150);
    toneAC(0);
    delay(150);
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_63); /* Standby time. */

  sensors_event_t pressure_event;
  bmp_pressure->getEvent(&pressure_event);
  
  old_pressure = pressure_event.pressure;
  
  toneAC(590, 4);
  delay(70);
  toneAC(0);
}

void loop(){
  sensors_event_t pressure_event;
  bmp_pressure->getEvent(&pressure_event);
  
  float new_pressure = pressure_event.pressure;
  float rate = (new_pressure - old_pressure) * 1000.0;
  old_pressure = new_pressure;

  Serial.println((int)rate);

  if(rate<-8)
  {
  float pitch = 500.0 - (rate * 10.0);
  if(pitch > 2000)pitch = 2000.0;
  toneAC((unsigned int)pitch);
//  LowPower.idle(SLEEP_120MS, ADC_OFF, TIMER2_OFF, TIMER1_ON, TIMER0_OFF, SPI_OFF,USART0_OFF, TWI_OFF);
    delay(120);
  toneAC(0);
//  LowPower.idle(SLEEP_120MS, ADC_OFF, TIMER2_OFF, TIMER1_ON, TIMER0_OFF, SPI_OFF,USART0_OFF, TWI_OFF);
    delay(120);
  }
  else if(rate>30)
  {
    toneAC(300, 1);
//    LowPower.idle(SLEEP_250MS, ADC_OFF, TIMER2_OFF, TIMER1_ON, TIMER0_OFF, SPI_OFF,USART0_OFF, TWI_OFF);
    delay(250);
  }
  else
  {
    toneAC(0);
    //LowPower.idle(SLEEP_250MS, ADC_OFF, TIMER2_OFF, TIMER1_ON, TIMER0_OFF, SPI_OFF,USART0_OFF, TWI_OFF);
    delay(250);
  }
}
