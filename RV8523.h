#ifndef RV8523_h
#define RV8523_h

#include <inttypes.h>
#include "I2C.h"

//#define I2C_ADDR (0xD0>>1)
const uint8_t I2C_ADDR = (0xD0>>1);


// Flags: Masks to clear a specific flag with an AND logic operation
const uint8_t WTAF = 0b01111111;    // Mask to clear the watchdog timer A interrupt flag (WTAF) using an AND operation
const uint8_t CTAF = 0b10111111;    // Mask to clear the countdown timer A interrupt flag (CTAF) using an AND operation
const uint8_t CTBF = 0b11011111;    // Mask to clear the countdown timer B  interrupt flag (CTBF) using an AND operation
const uint8_t SF   = 0b11101111;    // Mask to clear the second interrupt flag (SF) using an AND operation
const uint8_t AF   = 0b11110111;    // Mask to clear the alarm interrupt flag (AF) using an AND operation

// Registers
const uint8_t Control_1   = 0x00;
const uint8_t Control_2   = 0x01;
const uint8_t Control_3   = 0x02;
const uint8_t Seconds     = 0x03;
const uint8_t Minutes     = 0x04;
const uint8_t Hours       = 0x05;
const uint8_t Days        = 0x06;
const uint8_t Weekdays    = 0x07;
const uint8_t Months      = 0x08;
const uint8_t Years       = 0x09;
const uint8_t MinAlarm    = 0x0A;
const uint8_t HourAlarm   = 0x0B;
const uint8_t DayAlarm    = 0x0C;
const uint8_t WdayAlarm   = 0x0D;
const uint8_t FreqOffset  = 0x0E;
const uint8_t TimerCLKOUT = 0x0F;
const uint8_t TimerAclock = 0x10;
const uint8_t TimerA      = 0x11;
const uint8_t TimerBclock = 0x12;
const uint8_t TimerB      = 0x13;



class RV8523
{
  public:
    struct time
    {
      uint8_t sec, min, hour, day, month;
      uint16_t year;
    };
    
  public:
    RV8523();                                                   // Constructor

    void begin (void);                                          // Prepares on the uC the I2C communication with the RV8523
    void timeOut (uint16_t);                                    // 
    void startCounting (void);                                  // Starts RTC time circuits
    void stopCounting (void);                                   // Freezes RTC time circuits
    void getTimeDate (time *);                                  // Reads RTC's time and date
    void setTime (uint8_t hour, uint8_t min, uint8_t sec);      // Set the RTC's time
    void setDate (uint16_t year, uint8_t month, uint8_t day);   // Set the RTC's date
    void countdownNsecTimerA (uint8_t Nsec);                    // Countdown 'Nsec' on Timer A 
    void countdownNminTimerA (uint8_t Nminutes);                // Countdown 'Nminutes' on Timer A
    void countdownNhourTimerA (uint8_t Nhours);                 // Countdown 'Nhours' on Timer A
    void countdownNsecTimerB (uint8_t Nsec);                    // Countdown 'Nsec' on Timer B 
    int offsetCalibration (bool mode, int8_t offset);           // Calibrate the RTC´s offset (page 33 of datasheet)
    uint8_t setAlarm (uint8_t min);                             // Set an alarm at a specific minute
    uint8_t setAlarm (uint8_t hour, uint8_t min);               // Set an alarm at a specific hour and minute
    void clearFlag (uint8_t Register, uint8_t flag);            // Clear a specific flag in a register
    void set12hMode (void);                                     // Set 12 hours mode 
    void set24hMode (void);                                     // Set 24 hours mode
    uint8_t readRTCRegister (uint8_t RTCregister);                 // Read a single RTC register
    void ReadAndPrintRegisters (void);                          // Read all RTC's registers and print them in the serial console (for debugging)
    void resetRTC (void);                                       // Reset all RTC's registers to the default value
    void EnableBatterySwitchover(void);                         // Activates battery switchover

  private:
    uint8_t bin2bcd(uint8_t val);                               // Transforms a number from binary to BCD format
    uint8_t bcd2bin(uint8_t val);                               // Transforms a number from BCD to binary format

  
};

#endif
