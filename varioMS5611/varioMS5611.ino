#include <toneAC.h>
#include <Wire.h>
#include "MS5611.h"
#include <LowPower.h>

/*
Hardware connections:

- (GND) to GND
+ (VCC) to 5V
SCL to A5
SDA to A4

D9 and D10 to the Piezo Buzzer ( see toneAC.h )

*/
 
MS5611 mySensor;

#define NUM_PRESSURES 16
#define NUM_TOTALS 4
uint32_t pressure[NUM_PRESSURES];
uint32_t old_total[NUM_TOTALS];
int pressure_index = 0;
int total_index = 0;
uint32_t total;
int current_tone = 0;
int beep_time = 0;
 
void setup() 
{
// play start sound
  toneAC(388, 4);
  delay(70);
  toneAC(0);
  delay(30);
  toneAC(450, 4);
  delay(70);
  toneAC(0);

  while(!mySensor.begin(MS5611_ULTRA_HIGH_RES))
  {
    delay(500);
  }
  uint32_t p = mySensor.readPressure();
  total = p*NUM_PRESSURES;
  for(int i = 0; i<NUM_PRESSURES; i++)
  {
    pressure[i] = p;
  }
  for(int i = 0; i<NUM_TOTALS; i++)
  {
    old_total[i] = total;
  }
}
  
void loop()
{
    total -= pressure[pressure_index];
    pressure[pressure_index] = mySensor.readPressure();
    total += pressure[pressure_index];
    int32_t rate = total - old_total[total_index];
    float frate = (float)rate;
    frate = 0.0;
    old_total[total_index] = total;
    pressure_index++;
    total_index++;
    if(pressure_index >= NUM_PRESSURES)pressure_index = 0;
    if(total_index >= NUM_TOTALS)total_index = 0;
    if(rate < -15){
      if(beep_time <3)
        toneAC(500 - rate*5);
      else
        toneAC(0);
    }
    else if(rate > 60)
    {
      unsigned long tone = 500 - rate * 4;
      if(tone < 70)tone = 70;
      toneAC(tone);
    }
    else
    {
      toneAC(0);
    }
    beep_time++;
    if(beep_time >= 6)beep_time = 0;
 
  LowPower.idle(SLEEP_15MS, ADC_OFF, TIMER2_OFF, TIMER1_ON, TIMER0_OFF, SPI_OFF,USART0_OFF, TWI_OFF);
}
