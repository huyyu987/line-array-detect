#include <Wire.h>
#include <PCF8574.h>
#include <math.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include "detectroad.h"

#include <Arduino.h>
#define INTERRUPT_PIN 12 
#define INTERRUPT_PIN1 32
#define vdx PI * 43 / 600
#define cvq PI * 87
#define A_IN1 26
#define A_IN2 25
#define B_IN1 27
#define B_IN2 33

#define buttonPin 16

PCF8574 pcf8574(0x23);
const int pwmFreq = 5000;
const int pwmResolution = 8;
unsigned long last_update = 0;  
const unsigned long update_interval = 10;
int previousError = 0;
int baseSpeed = 220;
float Kp = 5.0;
float Ki = 0.5;
float Kd = 0.5;
bool PID_State = true, finish = false, checkfn = true, checkright1 = false;
float error = 0;
float error_p, error_i, error_d, er2, distancesum = 75;
int set = 0, checkleft = 0, checkright = 0, check = 0, checkqd = 0;
int pid_value1, checkrl = 0, premode = -1, dema = 0, checka = 0, dem_v = 0;
long t = 0;
long long t2;
double tim, tim2;
int countdc1 = 0, countdc2 = 0, check_start = 0;
int sensor1, sensor2, sensor3, sensor4; 
void IRAM_ATTR handleInterrupt() {
  countdc1++;
}
void setup() {
  Wire.begin(); 
  Serial.begin(115200);

  pcf8574.begin(); // Khởi tạo PCF8574
  road_detect_init();
  // Cấu hình chân GPIO cho ngắt
  pinMode(INTERRUPT_PIN, INPUT_PULLUP);
  pinMode(INTERRUPT_PIN1, INPUT_PULLUP);
  pinMode(buttonPin, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), handleInterrupt, FALLING);
  
  ledcAttach(A_IN1, pwmFreq, pwmResolution);
  ledcAttach(A_IN2, pwmFreq, pwmResolution);
  ledcAttach(B_IN1, pwmFreq, pwmResolution);
  ledcAttach(B_IN2, pwmFreq, pwmResolution);



}

void loop() {
  sensor1 = pcf8574.read(0);
  sensor2 = pcf8574.read(1);
  sensor3 = pcf8574.read(2);
  sensor4 = pcf8574.read(3);
  //call();  

  if(sensor1 != 0 || sensor2 != 0 || sensor3 != 0 || sensor4 != 0 ){
    if (sensor1 == 0 && sensor2 == 1 && sensor3 == 1 && sensor4 == 0 ) error = 0;
    else if (sensor1 == 0 && sensor2 == 0  && sensor3 == 0  && sensor4 == 1) error += 50;
    else if (sensor4 == 1 && sensor2 == 0 && sensor3 == 0 && sensor4 == 0) error += -50;  
    else if (sensor1 == 1 && sensor2 == 1 && sensor3 == 0 && sensor4 == 0) error += 24; 
    else if (sensor1 == 0 && sensor2 == 0 && sensor3 == 1 && sensor4 == 1) error += -24; 
  } else if (sensor1 == 0 && sensor2 == 0 && sensor3 == 0 && sensor4 == 0){
    setMotorSpeed(0, 200);
    return;
  }

  tim = (double)(micros() - tim2) / 1000000;
  tim2 = micros();
  pid_value1 = pid(Kp, Ki, Kd, tim, error);

  int r = baseSpeed + pid_value1;
  if (r > 255)
  r = 255;
  if (r < -255)
  r = -255;
  int l = baseSpeed - pid_value1;
  if (l > 255)
  l = 255;
  if (l < -255)
  l = -255;   
  setMotorSpeed(r, l);


}
//============= in het con me ra cho tao=======================
void call(){
  Serial.println(sensor1);
  Serial.println(sensor2);
  Serial.println(sensor3);
  Serial.println(sensor4);
 // Serial.println(error);
}
//====toc do chay cham vcl ======================================

void setMotorSpeed(int leftSpeed, int rightSpeed) {
  if(leftSpeed>=0 && rightSpeed>=0){
    Speed(leftSpeed, 0, rightSpeed, 0);
  }
  else if(leftSpeed>=0 && rightSpeed <0){
    Speed(leftSpeed, 0, 0, -rightSpeed);
  }
  else if(leftSpeed<0 && rightSpeed >= 0){
    Speed(0, -leftSpeed, rightSpeed, 0);
  }
  else{
    Speed(0, -leftSpeed, 0, -rightSpeed);
  }
  // if (leftSpeed >= 0) {
  //   Speed1(leftSpeed, 0);  // Forward direction
  // } else {
  //   Speed1(0, -leftSpeed);  // Reverse direction
  // }

  // if (rightSpeed >= 0) {
  //   Speed2(rightSpeed, 0);
  // } else {
  //   Speed2(0, -rightSpeed);  // Reverse direction
  // }
}

void Speed(int speed1, int speed2, int speed3, int speed4) {
  ledcWrite(A_IN1, speed1);
  ledcWrite(A_IN2, speed2);
  ledcWrite(B_IN1, speed3);
  ledcWrite(B_IN2, speed4);
}
// =================PID=========================
int pid(float kp, float ki, float kd, double tim, int value) {
  int pid_value;
  error_p = (set - value);
  if (pid_value < 255 && pid_value > -255) {
    error_i += (float)error_p * tim;
  }
  if (error_p == 0) {
    error_i = 0;
  }
  error_d = (float)(error_p - er2) / tim;
  er2 = error_p;
  pid_value = error_p * kp + error_i * ki + error_d * kd;
  if (pid_value > 255) {
    pid_value = 255;
  }
  if (pid_value < -255) {
    pid_value = -255;
  }
  return pid_value;
}
