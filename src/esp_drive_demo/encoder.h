#ifndef encoder_h
#define encoder_h

#include "Arduino.h"
#define TICKS_PER_REV 40.0 // Ticks per revolution for a perferated disk

class Encoder
{
  public:
   virtual void init();
   virtual void pin_setup(uint8_t encoder_pin);
   virtual uint32_t get_encoder_ticks();
   virtual uint32_t get_delta_encoder_ticks();
   virtual float get_velocity(float diameter);
   virtual float get_driven_distance_mm(float diameter);
   virtual void update_ticks();
   virtual void reset_encoder_ticks();
   virtual void zero_time();
   virtual uint32_t get_delta_t();
   uint32_t Encoder_Ticks = 0;
  private:
    uint8_t ENCODER_IN = 0; 
    uint32_t Encoder_Ticks_Prev = 0;
    uint32_t Delta_Encoder_Ticks = 0;
    float Standard_D = 65.0;
    uint32_t delta_t = 0;
    uint32_t prev_time = 0;
};

#endif
