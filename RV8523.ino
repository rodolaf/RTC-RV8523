/*
  RV8523 RTC (Real-Time-Clock) Example

  Leonardo   2 (SDA),  3 (SCL)
  
  Note: To enable the I2C pull-up resistors on the RTC-Breakout, the jumper J1 has to be closed.
 */

#include "RV8523.h"
#include "I2C.h"

I2C I2cc;

RV8523::time  RTCtime;    // Time structure member defined in the class
RV8523  RTC;              // Create an instance of RV8523

volatile byte state = LOW;

void setup()
{
  //init Serial port
  Serial.begin(9600);
  while(!Serial);    // Wait for serial port to connect - needed for Leonardo only
  
  RTC.begin();            // Prepare Arduino internal configuration for I2C communication
  RTC.timeOut(500);
  
  delay (100);
  
  RTC.resetRTC();
  RTC.ReadAndPrintRegisters();
  
  //RTC.stopCounting();
  
  RTC.setTime(16, 43, 00);
  RTC.setDate(2022, 2, 11);
  RTC.offsetCalibration(0, -6);
  //RTC.setAlarm (30);
  //RTC.clearFlag (Control_2, AF);
  RTC.startCounting();

  RTC.countdownNsecTimerB(7);
//  RTC.countdownNsecTimerA(5);

//  attachInterrupt(digitalPinToInterrupt(1), test, FALLING);

  RTC.ReadAndPrintRegisters();

  
}

void loop()
{
  RTC.getTimeDate(&RTCtime);

  Serial.print("\n"); 
  Serial.print(RTCtime.day); Serial.print("."); Serial.print(RTCtime.month); Serial.print("."); Serial.print(RTCtime.year); 
  Serial.print("  "); Serial.print(RTCtime.hour); Serial.print(":"); Serial.print(RTCtime.min); Serial.print(":"); Serial.print(RTCtime.sec);

  //RTC.ReadAndPrintRegisters();
  delay(10000);

//  if( state = HIGH)
//    Serial.print("State: "); Serial.print(state); Serial.print("\n");
//  state = LOW;
}

//void test ()
//{
//  //I2cc.write(I2C_ADDR, 0x0F, 0xF79);
//  state = HIGH;
//  return;
//}
