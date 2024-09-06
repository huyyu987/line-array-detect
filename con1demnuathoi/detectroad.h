#ifndef DETECTROAD_H_
#define DETECTROAD_H_

void road_detect_init();
void read_ir_sensor_maintask();
bool api_road_cross();
bool api_road_left();
bool api_road_right();

#endif