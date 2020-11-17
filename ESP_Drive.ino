#include "ESP_DRIVE.h"

void Test()
{
  int delay_time = 2000;
  int speed_set = 100;
  delay(delay_time);
  stop();
  delay(delay_time);
  right(speed_set);
  delay(delay_time);
  stop();
  delay(delay_time);
  left(speed_set);
  delay(delay_time);
  stop();
  delay(delay_time);
  forward(speed_set);
  delay(delay_time);
  stop();
  delay(delay_time);
  backward(speed_set);
  delay(delay_time);
  stop();
}

void Motor_Test()
{
  int delay_time = 80;
  int test_pwm = 0;

  // Motor 1
  for(test_pwm = 0; test_pwm < 255; test_pwm += 5)
  {
    Serial.print("Motor1: ");
    Serial.println(test_pwm);
    M1_PWM(test_pwm);
    delay(delay_time);
  }
  for(test_pwm = 255; test_pwm > -255; test_pwm -= 5)
  {
    Serial.print("Motor1: ");
    Serial.println(test_pwm);
    M1_PWM(test_pwm);
    delay(delay_time);
  }
  for(test_pwm = -255; test_pwm < 0; test_pwm += 5)
  {
    Serial.print("Motor1: ");
    Serial.println(test_pwm);
    M1_PWM(test_pwm);
    delay(delay_time);
  }
  M1_Stop();
  // Motor 2
  for(test_pwm = 0; test_pwm < 255; test_pwm += 5)
  {
    Serial.print("Motor2: ");
    Serial.println(test_pwm);
    M2_PWM(test_pwm);
    delay(delay_time);
  }
  for(test_pwm = 255; test_pwm > -255; test_pwm -= 5)
  {
    Serial.print("Motor2: ");
    Serial.println(test_pwm);
    M2_PWM(test_pwm);
    delay(delay_time);
  }
  for(test_pwm = -255; test_pwm < 0; test_pwm += 5)
  {
    Serial.print("Motor2: ");
    Serial.println(test_pwm);
    M2_PWM(test_pwm);
    delay(delay_time);
  }
  M2_Stop();
  // Motor 3
  for(test_pwm = 0; test_pwm < 255; test_pwm += 5)
  {
    Serial.print("Motor3: ");
    Serial.println(test_pwm);
    M3_PWM(test_pwm);
    delay(delay_time);
  }
  for(test_pwm = 255; test_pwm > -255; test_pwm -= 5)
  {
    Serial.print("Motor3: ");
    Serial.println(test_pwm);
    M3_PWM(test_pwm);
    delay(delay_time);
  }
  for(test_pwm = -255; test_pwm < 0; test_pwm += 5)
  {
    Serial.print("Motor3: ");
    Serial.println(test_pwm);
    M3_PWM(test_pwm);
    delay(delay_time);
  }
  M3_Stop();
  // Motor 4
  for(test_pwm = 0; test_pwm < 255; test_pwm += 5)
  {
    Serial.print("Motor4: ");
    Serial.println(test_pwm);
    M4_PWM(abs(test_pwm));
    delay(delay_time);
  }
  for(test_pwm = 255; test_pwm > -255; test_pwm -= 5)
  {
    Serial.print("Motor4: ");
    Serial.println(test_pwm);
    M4_PWM(test_pwm);
    delay(delay_time);
  }
  for(test_pwm = -255; test_pwm < 0; test_pwm += 5)
  {
    Serial.print("Motor4: ");
    Serial.println(test_pwm);
    M4_PWM(test_pwm);
    delay(delay_time);
  }
  M4_Stop();
}

void setup() 
{
  Serial.begin(115200);
  init_Motors();
  //Test();
  //Motor_Test();
  
  forward(85);
}


void loop() {
  distance_mm();
  Serial.println("_______");
  Serial.println(get_delta_ticks1());
  Serial.println(get_delta_ticks2());
  Serial.println("_______");
  delay(1000);
}
