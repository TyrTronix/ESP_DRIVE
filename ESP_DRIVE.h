#include "Arduino.h"

#ifndef ESP_DRIVE_h
#define ESP_DRIVE_h

// Motor Control Pins
#define M1_Pos 12
#define M1_Neg 13
#define M2_Pos 18
#define M2_Neg 23
#define M3_Pos 19
#define M3_Neg 27
#define M4_Pos 16
#define M4_Neg 17

// PWM Properties
#define frequency 30000 //80
#define resolution 8
#define M1_CH_P 0
#define M1_CH_N 1
#define M2_CH_P 2
#define M2_CH_N 3
#define M3_CH_P 4
#define M3_CH_N 5
#define M4_CH_P 6
#define M4_CH_N 7

#define ENCODER1 22
#define ENCODER2 39

#define TRIG 32
#define ECHO 35

#define DEBOUNCE_DELAY 11000
#define TIMER_PERIOD   200000

void init_Motors();
void right(int speed);
void left(int speed);
void forward(int speed);
void backward(int speed);
void stop();
int distance_mm();
int distance_cm();
void M1_PWM(int pwm);
void M2_PWM(int pwm);
void M3_PWM(int pwm);
void M4_PWM(int pwm);
void M1_Stop();
void M2_Stop();
void M3_Stop();
void M4_Stop();

uint32_t get_delta_ticks1();
uint32_t get_delta_ticks2();




#endif
