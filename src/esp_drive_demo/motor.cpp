#include "motor.h"

static uint16_t frequency = 30000;
static uint8_t resolution = 8;

static uint16_t velocity = 0;
static uint16_t angle = 0;
static uint16_t distance = 0;

void Motor::init(uint8_t pinA, uint8_t pinB, uint8_t pwmch1, uint8_t pwmch2, bool encoder_support)
{
  PWMCH1 = pwmch1;
  PWMCH2 = pwmch2;
  
  pinMode(pinA, OUTPUT);
  pinMode(pinB, OUTPUT);
  ledcSetup(PWMCH1, frequency, resolution);
  ledcSetup(PWMCH2, frequency, resolution);
  ledcAttachPin(pinA, PWMCH1);
  ledcAttachPin(pinB, PWMCH2); 
}

void Motor::stop()
{ 
  ledcWrite(PWMCH1, 0); 
  ledcWrite(PWMCH2, 0); 
}

void Motor::pwm(int16_t pwm_signal)
{ 
  current_pwm = pwm_signal;
  if(pwm_signal>0)
  {
    ledcWrite(PWMCH1, abs(pwm_signal));
    ledcWrite(PWMCH2, 0);
    return;
  }
  else
  {
    ledcWrite(PWMCH1, 0);
    ledcWrite(PWMCH2, abs(pwm_signal));
  }
}

void Motor::motor_pwm_control(uint16_t velocity_ticks, uint16_t current_velocity_ticks, bool direction)
{
  if(direction) // positive pwm case
  {
    if (current_velocity_ticks < velocity_ticks) current_pwm++;
    else if(current_velocity_ticks > velocity_ticks) current_pwm--;

    // saturate pwm signals +/-
    if(current_pwm <  0) pwm(0);
    else if(current_pwm > max_pwm) pwm(max_pwm);
    else pwm(current_pwm);
  }
  else // negative pwm case
  {
    if (current_velocity_ticks < velocity_ticks) current_pwm--;
    else if(current_velocity_ticks > velocity_ticks) current_pwm++;

    // saturate pwm signals +/-
    if(current_pwm <  min_pwm) pwm(min_pwm);
    else if(current_pwm >  0) pwm(0);
    else  pwm(current_pwm);
  }
}
