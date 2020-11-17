#include "ultrasonic.h"

void HCSR04::pins(uint8_t trigger, uint8_t echo)
{
  TRIG = trigger;
  ECHO = echo;
}

void HCSR04::init()
{ 
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT); 
}

uint32_t HCSR04::get_distance_mm()
{
  long  duration_us;
  uint32_t distance_mm;
  
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  duration_us = pulseIn(ECHO, HIGH);
  distance_mm= duration_us*0.343/2.0;
  return(distance_mm);
}

uint32_t HCSR04::get_distance_cm()
{
  uint32_t distance_cm = (uint32_t)(((float)get_distance_mm())/10.0);
  return distance_cm;
}
