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

// Prepares on the uC the I2C communication with the RV8523
//    · Input:
//    · Output:
void RV8523::begin()
{
  I2C.begin();        //init I2C lib
}


// 
//    · Input:
//    · Output:

void RV8523::timeOut (uint16_t _timeOut)
{
  I2C.timeOut(_timeOut);
}


// Starts RTC time circuits
//    · Input:
//    · Output:

void RV8523::startCounting(void)
{
  uint8_t val;

  I2C.read(I2C_ADDR, Control_1, 1);
  val = I2C.receive();

  if( (val & (1<<5)) != 0 )     // If STOP bit is 1 (bit 5), then clear it to start counting
  {
    I2C.write(I2C_ADDR, Control_1, val & ~(1<<5));   // On Control Register 1, clear STOP bit to start counting
  }

  return;
}


// Freezes RTC time circuits
//    · Input:
//    · Output:

void RV8523::stopCounting(void)
{
  uint8_t val;

  I2C.read(I2C_ADDR, Control_1, 1);
  val = I2C.receive();

  if( (val & (1<<5)) == 0)            // If STOP bit (bit 5) is 0, then set it to stop counting
  {
    I2C.write(I2C_ADDR, Control_1, val | (1<<5));   // On Control Register 1, set STOP to stop counting
  }

  return;
}


// Reads RTC's time and date
//    · Input:
//    · Output:

void RV8523::getTimeDate (time *_time)
{
  I2C.read(I2C_ADDR, Seconds, 7);
  _time->sec   = bcd2bin(I2C.receive() & 0x7F);
  _time->min   = bcd2bin(I2C.receive() & 0x7F);
  _time->hour  = bcd2bin(I2C.receive() & 0x3F);       //24 hour mode
  _time->day   = bcd2bin(I2C.receive() & 0x3F);
                     bcd2bin(I2C.receive() & 0x07);   //day of week
  _time->month = bcd2bin(I2C.receive() & 0x1F);
  _time->year  = bcd2bin(I2C.receive()) + 2000;
  
  return;
}


// Set the RTC's time
//    · Input:
//    · Output:

void RV8523::setTime (uint8_t hour, uint8_t min, uint8_t sec)
{
  //Array to hold the 3 bytes that will be written to the register
  uint8_t writeBytes[3];
  uint8_t returnStatus = 0;

  writeBytes[0] = bin2bcd(sec); //MSB
  writeBytes[1] = bin2bcd(min);
  writeBytes[2] = bin2bcd(hour);

  returnStatus = I2C.write(I2C_ADDR, Seconds, writeBytes, 3);
}


// Set the RTC's date
//    · Input:
//    · Output:

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

  returnStatus = I2C.write(I2C_ADDR, Days, writeBytes, 4);
  return;
}


// Countdown 'Nsec' on Timer A
//    · Input:
//    · Output:

void RV8523::countdownNsecTimerA (uint8_t Nsec)
{
  uint8_t writeBytes[3];
  uint8_t returnStatus = 0;

  uint8_t Reg_0Fh, Reg_01h;

  // Disable counter before writting the seconds to be counter
  I2C.read(I2C_ADDR, TimerCLKOUT, 1);                      // Read actual value of "Timer & CLKOUT" register
  Reg_0Fh = I2C.receive();
  I2C.read(I2C_ADDR, Control_2, 1);                         // Read actual value of "Control 2" register
  Reg_01h = I2C.receive();
  
  I2C.write(I2C_ADDR, TimerCLKOUT, Reg_0Fh & 0b11111001);                 // Disable counter in "Timer & CLKOUT" register -> TAC[1:0] = 00

  returnStatus = I2C.write(I2C_ADDR, TimerA, Nsec);                       // Write number of seconds to be counted
  returnStatus = I2C.write(I2C_ADDR, TimerAclock, 0b00000010);            // Select 1Hz as the source for Timer A
  returnStatus = I2C.write(I2C_ADDR, Control_2,  Reg_01h | 0b00000010);   // Enable countdown timer A interrupts
  returnStatus = I2C.write(I2C_ADDR, TimerCLKOUT, Reg_0Fh | 0b10111010);  // TAM -> pulse interrupt ; TAC[1:0] = 01 -> Start countdown .. 
                                                                          // .. and put CLKOUT in high impedance COF[2:0] = 111 otherwise the
                                                                          // interrupt in #INT1 may be masked (#INT1 is at the same time output
                                                                          // for both CLKOUT and Timer A interrupt!!)
}

// Countdown 'Nsec' on Timer B
//    · Input:
//    · Output:

void RV8523::countdownNsecTimerB (uint8_t Nsec)
{
  uint8_t writeBytes[3];

  uint8_t Reg_0Fh, Reg_01h;

  // Disable counter before writting the seconds to be counter
  I2C.read(I2C_ADDR, TimerCLKOUT, 1);                         // Read actual value of "Timer & CLKOUT" register before modifiying it
  Reg_0Fh = I2C.receive();
  I2C.read(I2C_ADDR, Control_2, 1);                           // Read actual value of "Control 2" register before modifiying it
  Reg_01h = I2C.receive();
  
  I2C.write(I2C_ADDR, TimerCLKOUT, Reg_0Fh & 0b11111110);     // Disable counter in "Timer & CLKOUT" register -> TBC = 0

  I2C.write(I2C_ADDR, TimerB, Nsec);                          // Write number of seconds to be counted
  I2C.write(I2C_ADDR, TimerBclock, 0b00000010);               // Select 1Hz as Timer B clock source -> TBQ[2:0] = 010 ...
                                                              // .. select a pulse of 46 ms width -> BW[2:0] = 000
  I2C.write(I2C_ADDR, Control_2, Reg_01h | 0b00000001);       // Enable countdown timer B interrupts 
  I2C.write(I2C_ADDR, TimerCLKOUT, Reg_0Fh | 0b01111001);     // TBM -> pulse interrupt ; TBC = 1 -> Start countdown .. 
                                                              // .. and put CLKOUT in high impedance COF[2:0] = 111 otherwise the
                                                              // interrupt in #INT1 may be masked (#INT1 is at the same time output
                                                              // for both CLKOUT and Timer A interrupt!!)
}


// Calibrate the RTC´s offset (page 33 of datasheet)
//    · Input:
//    · Output:

bool RV8523::offsetCalibration (bool mode, int8_t offset)
{
  if ( (offset < -64) || (offset > 63))                   // Check if the offset is within range [-64 ... +64]...
    return -1;                                            // ... if not, return error value -1
  else
  {
    if (mode == 0)
      I2C.write(I2C_ADDR, FreqOffset, offset &= 0b01111111);    // Clear "MODE" bit and write offset (negative offsets are written as 2´s complement)
    else
      I2C.write(I2C_ADDR, FreqOffset, offset |= 0b10000000);    // Set "MODE" bit and write offset (negative offsets are written as 2´s complement)
  }
}


// Set an alarm at a specific minute
//    · Input:
//    · Output:

void RV8523::setAlarm (uint8_t min)
{
  uint8_t Reg_0Fh;
  uint8_t Reg_00h;
  
  if ( (min < 0) || (min > 60))                       // Check if the offset is within range [0 ... +60]...
    return -1;                                        // ... if not, return error value -1
  else
  {
    I2C.read(I2C_ADDR, TimerCLKOUT, 1);                        // Read actual value of "Timer & CLKOUT" register before modifiying it
    Reg_0Fh = I2C.receive();
    I2C.read(I2C_ADDR, Control_1, 1);                           // Read actual value of "Control 2" register before modifiying it
    Reg_00h = I2C.receive();
    I2C.write(I2C_ADDR, TimerCLKOUT, Reg_0Fh | 0b00111000);    // Put CLKOUT in high impedance COF[2:0] = 111 otherwise the
                                                                // interrupt in #INT1 will be masked!! (#INT1 is at the same time output
                                                                // for both CLKOUT and Timer A interrupt!!)
    min = bin2bcd(min);                                         // Convert minutes into BCD format
    I2C.write(I2C_ADDR, MinAlarm, min &= 0b01111111);           // Clear AE_M (Bit 7) to enable "minute alarm" and write minute value
    I2C.write(I2C_ADDR, Control_1, Reg_00h | 0b00000010);       // Set AIE (Bit 1) to enable "alarm interrupt"
  } 
}


// Clear a specific flag in a register
//    · Input:
//    · Output:

void RV8523::clearFlag (uint8_t Register, uint8_t flag)
{
  uint8_t RegValue;
  I2C.read(I2C_ADDR, Register, 1);                        // Store target register value prior to modification
  RegValue = I2C.receive();

  I2C.write(I2C_ADDR, Register, RegValue &= flag);  // Clear flag
  
}


// Set 12 hours mode
//    · Input:
//    · Output:

void RV8523::set24hMode (void)
{
  uint8_t val;

  I2C.read(I2C_ADDR, Control_1, 1);
  val = I2C.receive();

  if( (val & (1<<3)) == 0)            // If 12h mode is selected (bit 3 = 0), then set it to switch to 24h mode
  {
    I2C.write(I2C_ADDR, Control_1, val | (1<<3));   // Set 12_24 bit to switch to 24h mode
  }

  return;
}


// Set 24 hours mode
//    · Input:
//    · Output:

void RV8523::set12hMode (void)
{
  uint8_t val;

  I2C.read(I2C_ADDR, Control_1, 1);
  val = I2C.receive();

  if( (val & (1<<3)) != 0 )         // If 24h mode is selected (bit 3 = 1), then clear it to switch to 12h mode
  {
    I2C.write(I2C_ADDR, Control_1, val & ~(1<<3));   // Clear bit 3 to switch to 12h mode
  }

  return;
}


// Read all RTC's registers and print them in the serial console (for debugging)
//    · Input:
//    · Output:

void RV8523::ReadAndPrintRegisters(void)
{
  String RegNames [20] = {"         Control 1 (00h): ", "         Control 2 (01h): ", "         Control 3 (02h): ", "           Seconds (03h): ", 
                        "           Minutes (04h): ", "             Hours (05h): ", "              Days (06h): ", "          Weekdays (07h): ", 
                        "            Months (08h): ", "             Years (09h): ", "        Min. alarm (0Ah): ", "        Hour alarm (0Bh): ", 
                        "         Day alarm (0Ch): ", "     Weekday alarm (0Dh): ", "      Freq. offset (0Eh): ", "    Timer & CLKOUT (0Fh): ", 
                        "     Timer A Clock (10h): ", "           Timer A (11h): ", "     Timer B Clock (12h): ", "           Timer B (13h): "};
                        
  uint8_t val = 0;

  //init Serial port
  Serial.begin(9600);
  while(!Serial);    // Wait for serial port to connect - needed for Leonardo only
  
  Serial.print("\n");
  I2C.read(I2C_ADDR, Control_1, 20);
  for (uint8_t i=0; i<20; i++)
  {
    val = I2C.receive();
    Serial.print(RegNames[i]);
    for(int j = 7; j>=0;j--) 
    {
      Serial.print((char)('0' + ( (val>>j) & Control_2) ) );
    }
    Serial.println();
  }
}


// Reset all RTC's registers to the default value
//    · Input:
//    · Output:

void RV8523::resetRTC (void)
{
  I2C.write(I2C_ADDR, Control_1, 0b01011000 );
}


// Activates battery switchover
//    · Input:
//    · Output:

void RV8523::batterySwitchOver(bool ON)
{
  uint8_t val;
  
  I2C.read(I2C_ADDR, Control_3, 1);
  val = I2C.receive();
  if(val & 0xE0)
  {
    if(ON)
      I2C.write(I2C_ADDR, Control_3, val & ~0xE0);    // Battery switchover in standard mode
    else
      I2C.write(I2C_ADDR, Control_3, val | 0xE0);     // Battery switchover disabled
  }
  
}


//-------------------- Private --------------------


// Transforms a number from binary to BCD format
//    · Input:
//    · Output:

uint8_t RV8523::bin2bcd(uint8_t val)
{
  return val + 6 * (val / 10);
}


// Transforms a number from BCD to binary format
//    · Input:
//    · Output:

uint8_t RV8523::bcd2bin(uint8_t val)
{
  return val - 6 * (val >> 4);
}
