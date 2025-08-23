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
void adc_system_diagnostics(void);
uint16_t get_calibrated_threshold(int sensor_index);
bool is_sensor_calibration_valid(void);
void send_detailed_sensor_status(void);
void diagnostic_sensor_test();
#endif
