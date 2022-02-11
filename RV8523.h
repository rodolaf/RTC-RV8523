#ifndef RV8523_h
#define RV8523_h

#include <inttypes.h>
#include "I2C.h"

#define I2C_ADDR (0xD0>>1)

#define ON    TRUE
#define OFF   FALSE

// Flags: Masks to clear a specific flag with an AND logic operation
#define WTAF        0b01111111    // Mask to clear the watchdog timer A interrupt flag (WTAF)
#define CTAF        0b10111111    // Mask to clear the countdown timer A interrupt flag (CTAF)
#define CTBF        0b11011111    // Mask to clear the countdown timer B  interrupt flag (CTBF)
#define SF          0b11101111    // Mask to clear the second interrupt flag (SF)
#define AF          0b11110111    // Mask to clear the alarm interrupt flag (AF)

// Registers
#define Control_1     0x00
#define Control_2     0x01
#define Control_3     0x02
#define Seconds       0x03
#define Minutes       0x04
#define Hours         0x05
#define Days          0x06
#define Weekdays      0x07
#define Months        0x08
#define Years         0x09
#define MinAlarm      0x0A
#define HourAlarm     0x0B
#define DayAlarm      0x0C
#define WdayAlarm     0x0D
#define FreqOffset    0x0E
#define TimerCLKOUT   0x0F
#define TimerAclock   0x10
#define TimerA        0x11
#define TimerBclock   0x12
#define TimerB        0x13



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
    void countdownNsecTimerB (uint8_t Nsec);                    // Countdown 'Nsec' on Timer B 
    bool offsetCalibration (bool mode, int8_t offset);          // Calibrate the RTC´s offset (page 33 of datasheet)
    void setAlarm (uint8_t min);                                // Set an alarm at a specific minute
    void clearFlag (uint8_t Register, uint8_t flag);            // Clear a specific flag in a register
    void set12hMode (void);                                     // Set 12 hours mode 
    void set24hMode (void);                                     // Set 24 hours mode
    void ReadAndPrintRegisters (void);                          // Read all RTC's registers and print them in the serial console (for debugging)
    void resetRTC (void);                                       // Reset all RTC's registers to the default value
    void batterySwitchOver(bool Action);                        // Activates battery switchover

  private:
    uint8_t bin2bcd(uint8_t val);                               // Transforms a number from binary to BCD format
    uint8_t bcd2bin(uint8_t val);                               // Transforms a number from BCD to binary format

  
};

#endif
