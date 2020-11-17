#ifndef ultrasonic_h
#define ultrasonic_h

#include "Arduino.h"

class HCSR04
{
  public:
   virtual void pins(uint8_t trigger, uint8_t echo);
   virtual void init();
   virtual uint32_t get_distance_mm();
   virtual uint32_t get_distance_cm();
  private:
    uint8_t TRIG = 32;
    uint8_t ECHO = 35;
};


#endif
