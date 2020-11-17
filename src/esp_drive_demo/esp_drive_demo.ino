#include "ESP_DRIVE.h"

ESP_DRIVE Robot;

void setup() 
{
  Serial.begin(115200);
  Robot.init();
  
  //Robot.motor_test();
  //Robot.drive_test();
  
  //Robot.forward(255);
  //Robot.forward_vel(50, 0);
  Robot.uSensor_start(5);
  Robot.drive_forward(10, 40, 0);
}

void loop() 
{
  Serial.println("_______");
  Serial.println(Robot.distance_mm());
  Serial.println("_______");
  delay(100);
}
