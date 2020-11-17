#include "Arduino.h"
#include "motor.h"
#include "encoder.h"
#include "ultrasonic.h"

#ifndef ESP_DRIVE_h
#define ESP_DRIVE_h

#define TIMER_PERIOD   500000.0
#define DEBOUNCE_DELAY 11000.0


#define PWMCH0 0
#define PWMCH1 1
#define PWMCH2 2
#define PWMCH3 3
#define PWMCH4 4
#define PWMCH5 5
#define PWMCH6 6
#define PWMCH7 7

typedef enum
{
  CONTROLLER_OFF = 0,
  CONTROLLER_SET_VELOCITY,
  CONTROLLER_DRIVE_DISTANCE,
  CONTROLLER_TURN
} motor_controller_states;

class ESP_DRIVE
{
  public:
   virtual void init();
   virtual void set_motor1_pins(uint8_t pinM1m, uint8_t pinM1p, bool encoding);
   virtual void set_motor2_pins(uint8_t pinM2m, uint8_t pinM2p, bool encoding);
   virtual void set_motor3_pins(uint8_t pinM3m, uint8_t pinM3p, bool encoding);
   virtual void set_motor4_pins(uint8_t pinM4m, uint8_t pinM4p, bool encoding);
   virtual void right(int16_t velocity);
   virtual void left(int16_t velocity);
   virtual void forward(int16_t velocity, int8_t sterring);
   virtual void backward(int16_t velocity, int8_t sterring);
   virtual void backward_vel(uint8_t velocity_pct, int8_t sterring);
   virtual void forward_vel(uint8_t velocity, int8_t steering);
   virtual void drive_backward(uint8_t velocity, uint16_t distance, int8_t steering);
   virtual void drive_forward(uint8_t velocity, uint16_t distance, int8_t steering);
   virtual void turn(uint8_t velocity, int16_t angle);
   virtual void stop();
   virtual void motor_test();
   virtual void drive_test();
   virtual void uSensor_stop();
   virtual void uSensor_start(float Hz);
   virtual uint16_t distance_mm();
   virtual uint32_t get_enc1_velocity_ticks();
   virtual uint32_t get_enc2_velocity_ticks();
   virtual void change_wheel_diameter(uint16_t wheel_diam);
  private:
    void distance_sensor_setup();
    uint16_t WHEEL_DIAMETER = 66.0; // mm
    uint8_t ENCODER1_PIN = 5;
    uint8_t ENCODER2_PIN = 39;
    uint8_t M1m = 13;//13
    uint8_t M1p = 12;//14
    uint8_t M2m = 18;//23
    uint8_t M2p = 23;//18
    uint8_t M3m = 27;//27
    uint8_t M3p = 19;//19
    uint8_t M4m = 16;//17
    uint8_t M4p = 17;//16
    bool M1_Encoding = 0;
    bool M2_Encoding = 0;
    bool M3_Encoding = 0;
    bool M4_Encoding = 0;
};


#endif
