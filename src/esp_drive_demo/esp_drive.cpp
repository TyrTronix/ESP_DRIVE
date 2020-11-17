#include "ESP_DRIVE.h"

TaskHandle_t xCTRL_Handle = NULL;
TaskHandle_t xSensor_Handle = NULL;
hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

static uint16_t MAX_VELOCITY = 100; // Ticks per second
static uint16_t current_velocity_right = 0;
static uint16_t current_velocity_left = 0;
static uint16_t current_distance_mm = 0;
static uint16_t uSensor_period = 200;
static int16_t Velocity_Target = 0;
static int16_t Distance_Target = 0;
static int16_t Angle_Target = 0;
static int16_t Steering = 0;
static uint8_t CONTROLLER_STATE = CONTROLLER_OFF;
static bool MOTOR_STARTUP = false;
static bool uSensor_on = false;

HCSR04 Distance_Sensor;
Encoder ENC1;
Encoder ENC2;
Motor M1;
Motor M2;
Motor M3;
Motor M4;

void IRAM_ATTR onTimer()
{
  portENTER_CRITICAL_ISR(&timerMux);
  ENC1.update_ticks();
  ENC2.update_ticks();
  portEXIT_CRITICAL_ISR(&timerMux);
}

void IRAM_ATTR encoder1_tick() 
{ 
  if(ENC1.get_delta_t() >= DEBOUNCE_DELAY) ENC1.Encoder_Ticks++;
  ENC1.zero_time();
}

void IRAM_ATTR encoder2_tick() 
{ 
  if(ENC2.get_delta_t() >= DEBOUNCE_DELAY) ENC2.Encoder_Ticks++;
  ENC2.zero_time();
}


//***************************************CONTROL FUNCTIONS***************************************

void pwm_forward_case()
{
    current_velocity_right = ENC1.get_delta_encoder_ticks()/(TIMER_PERIOD/1000000.0);
    current_velocity_left = ENC2.get_delta_encoder_ticks()/(TIMER_PERIOD/1000000.0);
    
    M1.motor_pwm_control(Velocity_Target, current_velocity_left, 0);
    M3.motor_pwm_control(Velocity_Target, current_velocity_left, 0);
    
    M2.motor_pwm_control(Velocity_Target, current_velocity_right, 1); 
    M4.motor_pwm_control(Velocity_Target, current_velocity_right, 1);
    
    Serial.print("M1 PWM = ");
    Serial.println(M1.current_pwm);
    Serial.print("M2 PWM = ");
    Serial.println(M2.current_pwm);
    Serial.print("M3 PWM = ");
    Serial.println(M3.current_pwm);
    Serial.print("M4 PWM = ");
    Serial.println(M4.current_pwm);
}

void pwm_backward_case()
{
    current_velocity_right = ENC1.get_delta_encoder_ticks()/(TIMER_PERIOD/1000000.0);
    current_velocity_left = ENC2.get_delta_encoder_ticks()/(TIMER_PERIOD/1000000.0);
    
    M1.motor_pwm_control(Velocity_Target, current_velocity_left, 1);
    M3.motor_pwm_control(Velocity_Target, current_velocity_left, 1);
    
    M2.motor_pwm_control(Velocity_Target, current_velocity_right, 0);
    M4.motor_pwm_control(Velocity_Target, current_velocity_right, 0);

    Serial.print("M1 PWM = ");
    Serial.println(M1.current_pwm);
    Serial.print("M2 PWM = ");
    Serial.println(M2.current_pwm);
    Serial.print("M3 PWM = ");
    Serial.println(M3.current_pwm);
    Serial.print("M4 PWM = ");
    Serial.println(M4.current_pwm);
}

void Control_Velocity()
{
  // Forward Case
  if(Velocity_Target > 0)
  {
    if(MOTOR_STARTUP)
    {
      while(current_velocity_right == 0 || current_velocity_left == 0)
      {
        pwm_forward_case();
        yield();
      }
      MOTOR_STARTUP = false;
    }
    pwm_forward_case();
  }
  // Backwards case
  else if(Velocity_Target < 0)
  {
    if(MOTOR_STARTUP)
    {
      while(current_velocity_right == 0 || current_velocity_left == 0)
      {
        pwm_backward_case();
        yield();
      }
      MOTOR_STARTUP = false; 
    }
    pwm_backward_case();
  }
}

void break_motors()
{
  uint8_t break_delay = 75;
  M1.pwm(-M1.current_pwm);
  M2.pwm(-M2.current_pwm);
  M3.pwm(-M3.current_pwm);
  M4.pwm(-M4.current_pwm);
  vTaskDelay(break_delay / portTICK_PERIOD_MS);
  M1.stop();
  M2.stop();
  M3.stop();
  M4.stop();
}

void Control_Drive()
{
  uint16_t current_distance_right = ENC1.get_encoder_ticks();
  uint16_t current_distance_left = ENC2.get_encoder_ticks();
  Control_Velocity();
  if(current_distance_right >= Distance_Target || current_distance_left >= Distance_Target) 
  {
    break_motors();
    CONTROLLER_STATE = CONTROLLER_OFF;
  }
  Serial.print("Left Side Ticks = ");
  Serial.println(current_distance_left);
  Serial.print("Right Side Ticks = ");
  Serial.println(current_distance_right);
}

void vTask_uSensor_sampler(void *mode)
{
  uint16_t off_delay = 1000;
  while(1)
  {
    if(uSensor_on)
    {
      current_distance_mm = Distance_Sensor.get_distance_mm();
      vTaskDelay(uSensor_period / portTICK_PERIOD_MS);
    }
    else vTaskDelay(off_delay / portTICK_PERIOD_MS);
  }
}

void vTask_motor_controller(void *mode)
{
  uint16_t off_delay = 1000;
  uint16_t active_delay = 100;
  while(1)
  {
    switch(CONTROLLER_STATE)
    {
      case CONTROLLER_OFF:
      {
        vTaskDelay(off_delay / portTICK_PERIOD_MS);
        continue;
      } break;
      case CONTROLLER_SET_VELOCITY:
      {
        Control_Velocity();
        
      } break;
      case CONTROLLER_DRIVE_DISTANCE:
      {
        Control_Drive();
      } break;
      case CONTROLLER_TURN:
      {
        //Control_Turn();
      } break;
    }
    vTaskDelay(active_delay / portTICK_PERIOD_MS);
  }
}

void create_uSensor_task(void)
{
  xTaskCreate (vTask_uSensor_sampler, "uSensor_controller", 1024, NULL, 1, &xSensor_Handle);
  configASSERT (xSensor_Handle);
}

void delete_uSensor_task(void)
{
  vTaskDelete(xSensor_Handle);
}

void create_controller_task(void)
{
  xTaskCreate (vTask_motor_controller, "motor_controller", 1024, NULL, 1, &xCTRL_Handle);
  configASSERT (xCTRL_Handle);
}

void delete_controller_task(void)
{
  vTaskDelete(xCTRL_Handle);
}

//***************************************SETUP FUNCTIONS***************************************

uint16_t ESP_DRIVE::distance_mm()
{
  return current_distance_mm;
}

void ESP_DRIVE::uSensor_start(float Hz)
{
  uSensor_on = true;
  uSensor_period = 1000.00/Hz;
  // Take a measurement so it is available right away
  current_distance_mm = Distance_Sensor.get_distance_mm(); 
}

void ESP_DRIVE::uSensor_stop()
{
  uSensor_on = false;
}

void ESP_DRIVE::change_wheel_diameter(uint16_t wheel_diam)
{
  WHEEL_DIAMETER = wheel_diam;
}

void ESP_DRIVE::set_motor1_pins(uint8_t pinM1m, uint8_t pinM1p, bool encoding)
{
  M1m = pinM1m;
  M1p = pinM1p;
  M1_Encoding = encoding;
}

void ESP_DRIVE::set_motor2_pins(uint8_t pinM2m, uint8_t pinM2p, bool encoding)
{
  M2m = pinM2m;
  M2p = pinM2p;
  M2_Encoding = encoding;
}

void ESP_DRIVE::set_motor3_pins(uint8_t pinM3m, uint8_t pinM3p, bool encoding)
{
  M3m = pinM3m;
  M3p = pinM3p;
  M3_Encoding = encoding;
}

void ESP_DRIVE::set_motor4_pins(uint8_t pinM4m, uint8_t pinM4p, bool encoding)
{
  M4m = pinM4m;
  M4p = pinM4p;
  M4_Encoding = encoding;
}

uint32_t ESP_DRIVE::get_enc1_velocity_ticks()
{
  return ENC1.get_delta_encoder_ticks()/(TIMER_PERIOD/1000000.0); // Ticks per second
}

uint32_t ESP_DRIVE::get_enc2_velocity_ticks()
{
  return ENC2.get_delta_encoder_ticks()/(TIMER_PERIOD/1000000.0); // Ticks per second
}

//***************************************INIT FUNCTION***************************************

void ESP_DRIVE::init()
{
  CONTROLLER_STATE = CONTROLLER_OFF;
  M1.init(M1m, M1p, PWMCH0, PWMCH1, M1_Encoding);
  M2.init(M2m, M2p, PWMCH2, PWMCH3, M2_Encoding);
  M3.init(M3m, M3p, PWMCH4, PWMCH5, M3_Encoding);
  M4.init(M4m, M4p, PWMCH6, PWMCH7, M4_Encoding);

  ENC1.pin_setup(ENCODER1_PIN);
  ENC2.pin_setup(ENCODER2_PIN);
  ENC1.init();
  ENC2.init();

  Distance_Sensor.init();

  attachInterrupt(ENCODER1_PIN, encoder1_tick, CHANGE); 
  attachInterrupt(ENCODER2_PIN, encoder2_tick, CHANGE); 

  timer = timerBegin(0, 80, true); // 80 is the prescaler needed to set microsends
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, TIMER_PERIOD, true);
  timerAlarmEnable(timer);

  create_controller_task();
  create_uSensor_task();
  stop();
}

//***************************************MOTOR FUNCTIONS***************************************
void ESP_DRIVE::turn(uint8_t velocity, int16_t angle)
{
  Velocity_Target = MAX_VELOCITY*((float)velocity/100.00);
  Serial.print("Turning with velocity: ");
  Serial.println(Velocity_Target);
  Serial.print("To and Angle of: ");
  Serial.println(Distance_Target);

  if(angle > 0) right(velocity); // start the motors
  else if(angle < 0) left(velocity); // start the motors
  CONTROLLER_STATE = CONTROLLER_TURN;
  MOTOR_STARTUP = true;
}

void ESP_DRIVE::drive_forward(uint8_t velocity, uint16_t distance, int8_t steering)
{
  Velocity_Target = MAX_VELOCITY*((float)velocity/100.00);
  //Distance_Target = distance/(PI*WHEEL_DIAMTER*TICKS_PER_REV); // Diatance in mm in ticks
  Distance_Target = distance; 
  Serial.print("Driving forward with velocity: ");
  Serial.println(Velocity_Target);
  Serial.print("For a distance of: ");
  Serial.println(Distance_Target);
  Serial.print("With steering of: ");
  Serial.println(steering);

  forward(velocity, steering); // start the motors
  CONTROLLER_STATE = CONTROLLER_DRIVE_DISTANCE;
  MOTOR_STARTUP = true;
}

void ESP_DRIVE::drive_backward(uint8_t velocity, uint16_t distance, int8_t steering)
{
  Velocity_Target = MAX_VELOCITY*((float)velocity/100.00);
  Distance_Target = -distance/(PI*WHEEL_DIAMETER*TICKS_PER_REV); // Diatance in mm in ticks
  Serial.print("Driving backward with velocity: ");
  Serial.println(Velocity_Target);
  Serial.print("For a distance of: ");
  Serial.println(Distance_Target);
  Serial.print("With steering of: ");
  Serial.println(steering);

  ENC1.reset_encoder_ticks();
  ENC2.reset_encoder_ticks();
  forward(velocity, steering); // start the motors
  CONTROLLER_STATE = CONTROLLER_DRIVE_DISTANCE;
  MOTOR_STARTUP = true;
}

void ESP_DRIVE::forward_vel(uint8_t velocity, int8_t steering)
{
  Velocity_Target = MAX_VELOCITY*((float)velocity/100.00);
  Serial.print("Setting Velocity target to: ");
  Serial.println(Velocity_Target);

  forward(velocity, steering); // start the motors
  CONTROLLER_STATE = CONTROLLER_SET_VELOCITY;
  MOTOR_STARTUP = true;
}

void ESP_DRIVE::backward_vel(uint8_t velocity, int8_t steering)
{
  Velocity_Target = -MAX_VELOCITY*((float)velocity/100.00);
  Serial.print("Setting Velocity target to: ");
  Serial.println(Velocity_Target);

  backward(velocity, steering); // start the motors
  CONTROLLER_STATE = CONTROLLER_SET_VELOCITY;
  MOTOR_STARTUP = true;
}

void ESP_DRIVE::stop()
{
  CONTROLLER_STATE = CONTROLLER_OFF;
  Serial.println("Stop ");
  M1.stop();
  M2.stop();
  M3.stop();
  M4.stop();
}

void ESP_DRIVE::forward(int16_t velocity, int8_t sterring)
{
  CONTROLLER_STATE = CONTROLLER_OFF;
  int pwm_sig = (int)255*((float)velocity/100.00);
  M1.pwm(-pwm_sig);
  M2.pwm(pwm_sig);
  M3.pwm(-pwm_sig);
  M4.pwm(pwm_sig);
}

void ESP_DRIVE::backward(int16_t velocity, int8_t sterring)
{
  CONTROLLER_STATE = CONTROLLER_OFF;
  int pwm_sig = (int)255*((float)velocity/100.00);
  M1.pwm(pwm_sig);
  M2.pwm(-pwm_sig);
  M3.pwm(pwm_sig);
  M4.pwm(-pwm_sig);
}

void ESP_DRIVE::left(int16_t velocity)
{
  CONTROLLER_STATE = CONTROLLER_OFF;
  int pwm_sig = (int)255*((float)velocity/100.00);
  M1.pwm(pwm_sig);
  M2.pwm(pwm_sig);
  M3.pwm(pwm_sig);
  M4.pwm(pwm_sig);
}

void ESP_DRIVE::right(int16_t velocity)
{
  CONTROLLER_STATE = CONTROLLER_OFF;
  int pwm_sig = (int)255*((float)velocity/100.00);
  M1.pwm(-pwm_sig);
  M2.pwm(-pwm_sig);
  M3.pwm(-pwm_sig);
  M4.pwm(-pwm_sig);
}

//***************************************TEST FUNCTIONS***************************************

void ESP_DRIVE::drive_test()
{
  int delay_time = 2000;
  int speed_set = 75;
  vTaskDelay(delay_time / portTICK_PERIOD_MS);
  stop();
  vTaskDelay(delay_time / portTICK_PERIOD_MS);
  Serial.println("Right Turn");
  right(speed_set);
  vTaskDelay(delay_time / portTICK_PERIOD_MS);
  stop();
  vTaskDelay(delay_time / portTICK_PERIOD_MS);
  Serial.println("Left Turn");
  left(speed_set);
  vTaskDelay(delay_time / portTICK_PERIOD_MS);
  stop();
  vTaskDelay(delay_time / portTICK_PERIOD_MS);
  Serial.println("Drive Forward");
  forward(speed_set, 0);
  vTaskDelay(delay_time / portTICK_PERIOD_MS);
  stop();
  vTaskDelay(delay_time / portTICK_PERIOD_MS);
  Serial.println("Drive Backward");
  backward(speed_set, 0);
  vTaskDelay(delay_time / portTICK_PERIOD_MS);
  stop();
}


void ESP_DRIVE::motor_test()
{
  int delay_time = 80;
  int test_pwm = 0;

  // Motor 1
  for(test_pwm = 0; test_pwm < 255; test_pwm += 5)
  {
    Serial.print("Motor1: "); 
    Serial.println(test_pwm);
    M1.pwm(test_pwm);
    vTaskDelay(delay_time / portTICK_PERIOD_MS);
  }
  for(test_pwm = 255; test_pwm > -255; test_pwm -= 5)
  {
    Serial.print("Motor1: ");
    Serial.println(test_pwm);
    M1.pwm(test_pwm);
    vTaskDelay(delay_time / portTICK_PERIOD_MS);
  }
  for(test_pwm = -255; test_pwm < 0; test_pwm += 5)
  {
    Serial.print("Motor1: ");
    Serial.println(test_pwm);
    M1.pwm(test_pwm);
    vTaskDelay(delay_time / portTICK_PERIOD_MS);
  }
  M1.stop();
  // Motor 2
  for(test_pwm = 0; test_pwm < 255; test_pwm += 5)
  {
    Serial.print("Motor2: ");
    Serial.println(test_pwm);
    M2.pwm(test_pwm);
    vTaskDelay(delay_time / portTICK_PERIOD_MS);
  }
  for(test_pwm = 255; test_pwm > -255; test_pwm -= 5)
  {
    Serial.print("Motor2: ");
    Serial.println(test_pwm);
    M2.pwm(test_pwm);
    vTaskDelay(delay_time / portTICK_PERIOD_MS);
  }
  for(test_pwm = -255; test_pwm < 0; test_pwm += 5)
  {
    Serial.print("Motor2: ");
    Serial.println(test_pwm);
    M2.pwm(test_pwm);
    vTaskDelay(delay_time / portTICK_PERIOD_MS);
  }
  M2.stop();
  // Motor 3
  for(test_pwm = 0; test_pwm < 255; test_pwm += 5)
  {
    Serial.print("Motor3: ");
    Serial.println(test_pwm);
    M3.pwm(test_pwm);
    vTaskDelay(delay_time / portTICK_PERIOD_MS);
  }
  for(test_pwm = 255; test_pwm > -255; test_pwm -= 5)
  {
    Serial.print("Motor3: ");
    Serial.println(test_pwm);
    M3.pwm(test_pwm);
    vTaskDelay(delay_time / portTICK_PERIOD_MS);
  }
  for(test_pwm = -255; test_pwm < 0; test_pwm += 5)
  {
    Serial.print("Motor3: ");
    Serial.println(test_pwm);
    M3.pwm(test_pwm);
    vTaskDelay(delay_time / portTICK_PERIOD_MS);
  }
  M3.stop();
  // Motor 4
  for(test_pwm = 0; test_pwm < 255; test_pwm += 5)
  {
    Serial.print("Motor4: ");
    Serial.println(test_pwm);
    M4.pwm(abs(test_pwm));
    vTaskDelay(delay_time / portTICK_PERIOD_MS);
  }
  for(test_pwm = 255; test_pwm > -255; test_pwm -= 5)
  {
    Serial.print("Motor4: ");
    Serial.println(test_pwm);
    M4.pwm(test_pwm);
    vTaskDelay(delay_time / portTICK_PERIOD_MS);
  }
  for(test_pwm = -255; test_pwm < 0; test_pwm += 5)
  {
    Serial.print("Motor4: ");
    Serial.println(test_pwm);
    M4.pwm(test_pwm);
    vTaskDelay(delay_time / portTICK_PERIOD_MS);
  }
  M4.stop();
}
