#include "encoder.h"

void Encoder::zero_time()
{
    prev_time = esp_timer_get_time();
    delta_t = 0;
}

// Calculate the change in time
uint32_t Encoder::get_delta_t()
{
    delta_t = esp_timer_get_time() - prev_time;
    prev_time = esp_timer_get_time();
} 

void Encoder::pin_setup(uint8_t encoder_pin)
{ 
  ENCODER_IN = encoder_pin;
} 

void Encoder::init()
{
  pinMode(ENCODER_IN, INPUT_PULLUP);
}

void Encoder::reset_encoder_ticks()
{
  Encoder_Ticks = 0;
  Delta_Encoder_Ticks = 0;
  Encoder_Ticks_Prev = 0;
}

uint32_t Encoder::get_encoder_ticks()
{
  return Encoder_Ticks;
}

uint32_t Encoder::get_delta_encoder_ticks()
{
  return Delta_Encoder_Ticks;
}

void Encoder::update_ticks()
{
  Delta_Encoder_Ticks = Encoder_Ticks - Encoder_Ticks_Prev;
  Encoder_Ticks_Prev = Encoder_Ticks;
}

float Encoder::get_velocity(float diameter)
{
  return (Delta_Encoder_Ticks/TICKS_PER_REV)*PI*diameter;
}

float Encoder::get_driven_distance_mm(float diameter)
{
  return (Encoder_Ticks/TICKS_PER_REV)*PI*diameter;
}
