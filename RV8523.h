#ifndef RV8523_h
#define RV8523_h

#include <inttypes.h>
#include "I2C.h"

#define I2C_ADDR (0xD0>>1)


class RV8523
{
  public:
    struct time
    {
      uint8_t sec, min, hour, day, month;
      uint16_t year;
    };
    
  public:
    RV8523();                           // Constructor

    void begin (void);                  // Method to prepare the I2C communication with the RV8523
    void startCounting (void);          // Method to start the clock on the RV8523
    void stopCounting (void);
    void getTimeDate (time *);
    void setTime (uint8_t hour, uint8_t min, uint8_t sec);
    void setDate (uint16_t year, uint8_t month, uint8_t day);
    void countdownNsecTimerA (uint8_t Nsec);
    void countdownNsecTimerB (uint8_t Nsec);
//    void setAlarm
    void timeOut (uint16_t);
    void set12hMode (void);
    void set24hMode (void);
    void ReadAndPrintRegisters (void);  //

  private:
    uint8_t bin2bcd(uint8_t val);
    uint8_t bcd2bin(uint8_t val);

  
};

#endif
