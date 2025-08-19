#ifndef SENSORS_H
#define SENSORS_H
#include "micromouse.h"
void calibrate_sensors(void);
void update_sensors(void);
void update_walls(void);
void turn_on_emitters(void);
void turn_off_emitters(void);
uint16_t read_adc_channel(uint32_t channel);
bool are_sensors_healthy(void);
#endif
