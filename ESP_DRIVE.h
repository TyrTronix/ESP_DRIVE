#include "Arduino.h"

#ifndef ESP_DRIVE_h
#define ESP_DRIVE_h

// Motor Control Pins
#define M1_Pos 27 
#define M1_Neg 33 
#define M2_Pos 21 
#define M2_Neg 19 
#define M3_Pos 26 
#define M3_Neg 13 
#define M4_Pos 16
#define M4_Neg 17

#define nSLEEP 14

// PWM Properties
#define frequency 80//250000//30000 //80
#define resolution 8
#define M1_CH_P 0
#define M1_CH_N 1
#define M2_CH_P 2
#define M2_CH_N 3
#define M3_CH_P 4
#define M3_CH_N 5
#define M4_CH_P 6
#define M4_CH_N 7

#define ENCODER1 39
#define ENCODER2 36

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
void brake();
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
void M1_BRAKE();
void M2_BRAKE();
void M3_BRAKE(); 
void M4_BRAKE();

uint32_t get_delta_ticks1();
uint32_t get_delta_ticks2();




#endif
