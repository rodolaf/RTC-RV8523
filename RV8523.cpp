#include <inttypes.h>
#if defined(__AVR__)
#include <avr/io.h>
#endif
#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "I2C.h"
#include "RV8523.h"

I2C I2C;      // Important if you want to use the methods defined in the I2C class!!!

//-------------------- Constructor --------------------


RV8523::RV8523(void)
{
  return;
}


//-------------------- Public --------------------


void RV8523::begin()
{
  I2C.begin();        //init I2C lib
}

void RV8523::timeOut (uint16_t _timeOut)
{
  I2C.timeOut(_timeOut);
}

void RV8523::startCounting(void)
{
  uint8_t val;

  I2C.read(I2C_ADDR, 0x00, 1);
  val = I2C.receive();

  if( (val & (1<<5)) != 0 )     // If STOP bit is 1 (bit 5), then clear it to start counting
  {
    I2C.write(I2C_ADDR, 0x00, val & ~(1<<5));   // On Control Register 1, clear STOP bit to start counting
  }

  return;
}

void RV8523::stopCounting(void)
{
  uint8_t val;

  I2C.read(I2C_ADDR, 0x00, 1);
  val = I2C.receive();

  if( (val & (1<<5)) == 0)            // If STOP bit (bit 5) is 0, then set it to stop counting
  {
    I2C.write(I2C_ADDR, 0x00, val | (1<<5));   // On Control Register 1, set STOP to stop counting
  }

  return;
}

void RV8523::getTimeDate (time *_time)
{
  I2C.read(I2C_ADDR, 0x03, 7);
  _time->sec   = bcd2bin(I2C.receive() & 0x7F);
  _time->min   = bcd2bin(I2C.receive() & 0x7F);
  _time->hour  = bcd2bin(I2C.receive() & 0x3F);       //24 hour mode
  _time->day   = bcd2bin(I2C.receive() & 0x3F);
                     bcd2bin(I2C.receive() & 0x07);   //day of week
  _time->month = bcd2bin(I2C.receive() & 0x1F);
  _time->year  = bcd2bin(I2C.receive()) + 2000;
  
  return;
}

void RV8523::setTime (uint8_t hour, uint8_t min, uint8_t sec)
{
  //Array to hold the 3 bytes that will be written to the register
  uint8_t writeBytes[3];
  uint8_t returnStatus = 0;

  writeBytes[0] = bin2bcd(sec); //MSB
  writeBytes[1] = bin2bcd(min);
  writeBytes[2] = bin2bcd(hour);

  returnStatus = I2C.write(I2C_ADDR, 0x03, writeBytes, 3);
}

void RV8523::setDate (uint16_t year, uint8_t month, uint8_t day)
{
  if(year > 2000)
  {
    year -= 2000;
  }
  //Array to hold the 3 bytes that will be written to the register
  uint8_t writeBytes[4];
  uint8_t returnStatus = 0;

  writeBytes[0] = bin2bcd(day); //MSB
  writeBytes[1] = 0;
  writeBytes[2] = bin2bcd(month);
  writeBytes[3] = bin2bcd(year);

  returnStatus = I2C.write(I2C_ADDR, 0x06, writeBytes, 4);
  return;
}

void RV8523::countdownNsecTimerA (uint8_t Nsec)
{
  uint8_t writeBytes[3];
  uint8_t returnStatus = 0;

  uint8_t Reg_0Fh, Reg_01h;

  // Disable counter before writting the seconds to be counter
  I2C.read(I2C_ADDR, 0x0F, 1);                      // Read actual value of "Timer & CLKOUT" register
  Reg_0Fh = I2C.receive();
  I2C.read(I2C_ADDR, 0x01, 1);                     // Read actual value of "Control 2" register
  Reg_01h = I2C.receive();
  
  I2C.write(I2C_ADDR, 0x0F, Reg_0Fh & 0b11111001);                // Disable counter in "Timer & CLKOUT" register -> TAC[1:0] = 00

  returnStatus = I2C.write(I2C_ADDR, 0x11, Nsec);                 // Write number of seconds to be counted
  returnStatus = I2C.write(I2C_ADDR, 0x10, 0b00000010);           // Select 1Hz as the source for Timer A
  returnStatus = I2C.write(I2C_ADDR, 0x01, 0b00000010);           // Enable countdown timer A interrupts
  returnStatus = I2C.write(I2C_ADDR, 0x0F, Reg_0Fh | 0b10111010); // TAM -> pulse interrupt ; TAC[1:0] = 01 -> Start countdown .. 
                                                                  // .. and put CLKOUT in high impedance COF[2:0] = 111 otherwise the
                                                                  // interrupt in #INT1 may be masked (#INT1 is at the same time output
                                                                  // for both CLKOUT and Timer A interrupt!!)
}


void RV8523::countdownNsecTimerB (uint8_t Nsec)
{
  uint8_t writeBytes[3];
  uint8_t returnStatus = 0;

  uint8_t Reg_0Fh, Reg_01h;

  // Disable counter before writting the seconds to be counter
  I2C.read(I2C_ADDR, 0x0F, 1);                     // Read actual value of "Timer & CLKOUT" register before modifiying it
  Reg_0Fh = I2C.receive();
  I2C.read(I2C_ADDR, 0x01, 1);                     // Read actual value of "Control 2" register before modifiying it
  Reg_01h = I2C.receive();
  
  I2C.write(I2C_ADDR, 0x0F, Reg_0Fh & 0b11111110);     // Disable counter in "Timer & CLKOUT" register -> TBC = 0

  returnStatus = I2C.write(I2C_ADDR, 0x13, Nsec);                   // Write number of seconds to be counted
  returnStatus = I2C.write(I2C_ADDR, 0x12, 0b00000010);             // Select 1Hz as Timer B clock source -> TBQ[2:0] = 010 ...
                                                                    // .. select a pulse of 46 ms width -> BW[2:0] = 000
  returnStatus = I2C.write(I2C_ADDR, 0x01, Reg_01h | 0b00000001);   // Enable countdown timer B interrupts 
  returnStatus = I2C.write(I2C_ADDR, 0x0F, Reg_0Fh | 0b01000001);   // TBM -> pulse interrupt ; TBC = 1 -> Start countdown .. 
}

void RV8523::set24hMode (void)
{
  uint8_t val;

  I2C.read(I2C_ADDR, 0x00, 1);
  val = I2C.receive();

  if( (val & (1<<3)) == 0)            // If 12h mode is selected (bit 3 = 0), then set it to switch to 24h mode
  {
    I2C.write(I2C_ADDR, 0x00, val | (1<<3));   // Set 12_24 bit to switch to 24h mode
  }

  return;
}

void RV8523::set12hMode (void)
{
  uint8_t val;

  I2C.read(I2C_ADDR, 0x00, 1);
  val = I2C.receive();

  if( (val & (1<<3)) != 0 )         // If 24h mode is selected (bit 3 = 1), then clear it to switch to 12h mode
  {
    I2C.write(I2C_ADDR, 0x00, val & ~(1<<3));   // Clear bit 3 to switch to 12h mode
  }

  return;
}

// Gets all RV8523 registers and displays them in the serial monitor

void RV8523::ReadAndPrintRegisters(void)
{
  String RegNames [20] = {"         Control 1: ", "         Control 2: ", "         Control 3: ", "           Seconds: ", 
                        "           Minutes: ", "             Hours: ", "              Days: ", "          Weekdays: ", 
                        "            Months: ", "             Years: ", "        Min. alarm: ", "        Hour alarm: ", 
                        "         Day alarm: ", "     Weekday alarm: ", "      Freq. offset: ", "    Timer & CLKOUT: ", 
                        "     Timer A Clock: ", "           Timer A: ", "     Timer B Clock: ", "           Timer B: "};
                        
  uint8_t val = 0;

  //init Serial port
  Serial.begin(9600);
  while(!Serial);    // Wait for serial port to connect - needed for Leonardo only
  
  Serial.print("\n");
  I2C.read(I2C_ADDR, 0x00, 20);
  for (uint8_t i=0; i<20; i++)
  {
    val = I2C.receive();
    Serial.print(RegNames[i]);
    for(int j = 7; j>=0;j--) 
    {
      Serial.print((char)('0' + ( (val>>j) & 0x01) ) );
    }
    Serial.println();
  }
}


//-------------------- Private --------------------


uint8_t RV8523::bin2bcd(uint8_t val)
{
  return val + 6 * (val / 10);
}


uint8_t RV8523::bcd2bin(uint8_t val)
{
  return val - 6 * (val >> 4);
}
