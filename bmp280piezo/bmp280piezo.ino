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
// play start sound
//  Serial.begin(9600);
   toneAC(388, 5);
  delay(70);
  unsigned status;
  status = bmp.begin(0x76);
  if(!status)
  {
    toneAC(250, 10);
    delay(150);
    toneAC(0);
  }

  toneAC(610, 3);
  delay(70);
  toneAC(0);

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_63); /* Standby time. */

  old_pressure = bmp.readPressure();
  
  delay(70);
  toneAC(610, 3);
  delay(70);
  toneAC(0);
}

void loop(){
  sensors_event_t pressure_event;
  bmp_pressure->getEvent(&pressure_event);
  
  float new_pressure = pressure_event.pressure;
  float rate = (new_pressure - old_pressure) * 1000.0;
  old_pressure = new_pressure;
  
  //Serial.print(F("rate = "));
  //Serial.println(rate);

  if(rate<-8)
  {
  float pitch = 500.0 - (rate * 10.0);
  if(pitch > 1000)pitch = 1000.0;
  toneAC((unsigned int)pitch);
  LowPower.idle(SLEEP_120MS, ADC_OFF, TIMER2_OFF, TIMER1_ON, TIMER0_OFF, SPI_OFF,USART0_OFF, TWI_OFF);
  toneAC(0);
  LowPower.idle(SLEEP_120MS, ADC_OFF, TIMER2_OFF, TIMER1_ON, TIMER0_OFF, SPI_OFF,USART0_OFF, TWI_OFF);
  }
  else if(rate>20)
  {
    toneAC(300, 1);
    LowPower.idle(SLEEP_250MS, ADC_OFF, TIMER2_OFF, TIMER1_ON, TIMER0_OFF, SPI_OFF,USART0_OFF, TWI_OFF);
  }
  else
  {
    toneAC(0);
    LowPower.idle(SLEEP_250MS, ADC_OFF, TIMER2_OFF, TIMER1_ON, TIMER0_OFF, SPI_OFF,USART0_OFF, TWI_OFF);
  }
}
