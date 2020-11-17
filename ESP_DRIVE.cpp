#include "ESP_DRIVE.h"

uint32_t prev_time = 0;
uint32_t delta_t = 0;
uint32_t current_time = 0;

uint32_t encoder_ticks1 = 0;
uint32_t encoder_ticks1_prev = 0;
uint32_t encoder_ticks2 = 0;
uint32_t encoder_ticks2_prev = 0;
uint32_t delta_ticks1 = 0;
uint32_t delta_ticks2 = 0;

hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

void M1_Stop()
{
  ledcWrite(M1_CH_N, 0);
  ledcWrite(M1_CH_P, 0);
}

void M2_Stop()
{
  ledcWrite(M2_CH_N, 0);
  ledcWrite(M2_CH_P, 0);
}

void M3_Stop()
{
  ledcWrite(M3_CH_N, 0);
  ledcWrite(M3_CH_P, 0);
}

void M4_Stop()
{
  ledcWrite(M4_CH_N, 0);
  ledcWrite(M4_CH_P, 0);
}

static void zero_time()
{
    current_time = esp_timer_get_time();
    prev_time = current_time;
    delta_t = 0;
}

// Calculate the change in time
static void get_delta_t()
{
    current_time = esp_timer_get_time();
    delta_t = current_time - prev_time;
    prev_time = current_time;
}

void IRAM_ATTR encoder_tick1() 
{
  if(digitalRead(ENCODER1)) return;
  get_delta_t();
  if(delta_t >= DEBOUNCE_DELAY) encoder_ticks1++;
  zero_time();
}

void IRAM_ATTR encoder_tick2() 
{
  if(digitalRead(ENCODER2)) return;
  get_delta_t();
  if(delta_t >= DEBOUNCE_DELAY) encoder_ticks2++;
  zero_time();
}

void IRAM_ATTR onTimer()
{
  portENTER_CRITICAL_ISR(&timerMux);
  delta_ticks1 = encoder_ticks1;
  delta_ticks2 = encoder_ticks2;
  //encoder_ticks1 = 0;
  //encoder_ticks2 = 0;
  portEXIT_CRITICAL_ISR(&timerMux);
}

uint32_t get_delta_ticks1()
{
  return delta_ticks1;
}

uint32_t get_delta_ticks2()
{
  return delta_ticks2;
}

void init_Motors()
{
  pinMode(M1_Pos, OUTPUT);
  pinMode(M2_Pos, OUTPUT);
  pinMode(M3_Pos, OUTPUT);
  pinMode(M4_Pos, OUTPUT);
  pinMode(M1_Neg, OUTPUT);
  pinMode(M2_Neg, OUTPUT);
  pinMode(M3_Neg, OUTPUT);
  pinMode(M4_Neg, OUTPUT);
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);

  // Init Encoder
  pinMode(ENCODER1, INPUT_PULLUP);
  pinMode(ENCODER2, INPUT_PULLUP);

  timer = timerBegin(0, 80, true); // 80 is the prescaler needed to set microsends
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, TIMER_PERIOD, true);
  timerAlarmEnable(timer);
  
  attachInterrupt(ENCODER1, encoder_tick1, FALLING);
  attachInterrupt(ENCODER2, encoder_tick2, FALLING);
  // end of init encoder

  ledcSetup(M1_CH_P, frequency, resolution);
  ledcSetup(M1_CH_N, frequency, resolution);
  ledcSetup(M2_CH_P, frequency, resolution);
  ledcSetup(M2_CH_N, frequency, resolution);
  ledcSetup(M3_CH_P, frequency, resolution);
  ledcSetup(M3_CH_N, frequency, resolution);
  ledcSetup(M4_CH_P, frequency, resolution);
  ledcSetup(M4_CH_N, frequency, resolution);

  ledcAttachPin(M1_Pos, M1_CH_P);
  ledcAttachPin(M1_Neg, M1_CH_N);
  ledcAttachPin(M2_Pos, M2_CH_P);
  ledcAttachPin(M2_Neg, M2_CH_N);
  ledcAttachPin(M3_Pos, M3_CH_P);
  ledcAttachPin(M3_Neg, M3_CH_N);
  ledcAttachPin(M4_Pos, M4_CH_P);
  ledcAttachPin(M4_Neg, M4_CH_N);

  M1_Stop();
  M2_Stop();
  M3_Stop();
  M4_Stop();
}

void M1_PWM(int pwm)
{
  if(pwm>0)
  {
    ledcWrite(M1_CH_N, 0);
    ledcWrite(M1_CH_P, abs(pwm));
    return;
  }
  else
  {
    ledcWrite(M1_CH_N, abs(pwm));
    ledcWrite(M1_CH_P, 0);
  }
}

void M2_PWM(int pwm)
{
  if(pwm>0)
  {
    ledcWrite(M2_CH_N, 0);
    ledcWrite(M2_CH_P, abs(pwm));
    return;
  }
  else
  {
    ledcWrite(M2_CH_N, abs(pwm));
    ledcWrite(M2_CH_P, 0);
  }
}

void M3_PWM(int pwm)
{
  if(pwm>0)
  {
    ledcWrite(M3_CH_P, abs(pwm));
    ledcWrite(M3_CH_N, 0);
    return;
  }
  else{
    ledcWrite(M3_CH_P, 0);
    ledcWrite(M3_CH_N, abs(pwm));
  }
}

void M4_PWM(int pwm)
{
  if(pwm>0)
  {
    ledcWrite(M4_CH_P, abs(pwm));
    ledcWrite(M4_CH_N, 0);
    return;
  }
  else
  {
    ledcWrite(M4_CH_P, 0);
    ledcWrite(M4_CH_N, abs(pwm));
  }
}

void backward(int speed)
{
  int pwm = (int)255*((float)speed/100.00);
  Serial.print("Backwards ");
  Serial.println(pwm);
  M1_PWM(-pwm);
  M3_PWM(-pwm);
  M2_PWM(-pwm);
  M4_PWM(-pwm);
}

void forward(int speed)
{
  int pwm = (int)255*((float)speed/100.00);
  Serial.print("Forwards ");
  Serial.println(pwm);
  M1_PWM(pwm);
  M3_PWM(pwm);
  M2_PWM(pwm);
  M4_PWM(pwm);
}

void left(int speed)
{
  int pwm = (int)255*((float)speed/100.00);
  Serial.print("Left ");
  Serial.println(pwm);
  M1_PWM(pwm);
  M2_PWM(-pwm);
  M3_PWM(pwm);
  M4_PWM(-pwm);
}

void right(int speed)
{
  int pwm = (int)255*((float)speed/100.00);
  Serial.print("Right ");
  Serial.println(pwm);
  M1_PWM(-pwm);
  M2_PWM(pwm);
  M3_PWM(-pwm);
  M4_PWM(pwm);
}

void stop()
{
  Serial.print("Stop ");
  M1_Stop();
  M2_Stop();
  M3_Stop();
  M4_Stop();
}

int distance_mm(){
  long  duration_us;
  int distance_mm;
  
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  duration_us = pulseIn(ECHO, HIGH);
  distance_mm= duration_us*0.343/2.0;
  Serial.print("Distance in mm: ");
  Serial.println(distance_mm);
  return(distance_mm);
}

int distance_cm(){
  int distance_cm = distance_mm()/10;
  Serial.print("Distance in cm: ");
  Serial.println(distance_cm);
  return distance_cm;
}
