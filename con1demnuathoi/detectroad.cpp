#include <stdbool.h>
#include <stdint.h>
#include "Arduino.h"
#include "detectroad.h"
#include "PCF8574.h"
#define HIGH 0
#define LOW 1

int sensorA = 0;
int sensorB = 0;
int sensorC = 0;
int sensorD = 0;

static uint32_t time_out_sensor_left = 0;
static uint32_t time_out_sensor_right = 0;

static bool check_timeout_ms(uint32_t time, uint32_t timeout) {
  static uint32_t elapsedtime;
  if (millis() >= time) {
    elapsedtime = millis() - time;
  } else {
    elapsedtime = millis();
  }
  if (elapsedtime > timeout) {
    return 1;
  }
  return 0;
}


void road_detect_init() {  //pcf8574.begin();
                           // pinMode(SENSOR_FRONT, INPUT);

  // pinMode(SENSOR_LEFT_45, INPUT);
  // pinMode(SENSOR_LEFT, INPUT);

  // pinMode(SENSOR_RIGHT_45, INPUT);
  // pinMode(SENSOR_RIGHT, INPUT);
}
void read_ir_sensor_maintask() {
  // sensor1 = pcf8574.read(0);
  // sensor2 = pcf8574.read(1);
  // sensor3 = pcf8574.read(2);
  // sensor4 = pcf8574.read(3);
}

bool api_road_left() {
  if (sensorA == 1 && sensorB == 1 && sensorC == 1 && sensorD == 0) {
    return true;
  }
  return false;
}

bool api_road_right() {
  if (sensorA == 0 && sensorB == 1 && sensorC == 1 && sensorD == 1) {
    return true;
  }
  return false;
}

bool api_road_cross() {
  if (sensorA == 1 && sensorB == 1 && sensorC == 1 && sensorD == 1) {
    return true;
  }
  return false;
}

bool api_road_straight(){
  if(sensorA == 0 && sensorB == 1 && sensorC == 1 && sensorD == 0){
    return true;
    }
  return false;
}
