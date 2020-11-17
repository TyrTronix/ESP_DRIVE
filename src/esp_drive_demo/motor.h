#ifndef motor_h
#define motor_h

#include "Arduino.h"

class Motor
{
  public:
   virtual void init(uint8_t pinA, uint8_t pinB, uint8_t pwmch1, uint8_t pwmch2, bool encoder_support);
   virtual void pwm(int16_t pwm_signal);
   virtual void motor_pwm_control(uint16_t velocity_ticks, uint16_t current_velocity_ticks, bool direction);
   virtual void stop();
   uint8_t PWMCH1 = 0;
   uint8_t PWMCH2 = 0;
   int16_t current_pwm = 0;
   const int16_t max_pwm = 255;
   const int16_t min_pwm = -255;
   int16_t current_distance = 0;
   
  private:    
    // private
};

#endif
