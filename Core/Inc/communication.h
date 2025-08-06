#ifndef COMMUNICATION_H
#define COMMUNICATION_H
#include "micromouse.h"
void send_bluetooth_message(const char* message);
void send_bluetooth_printf(const char* format, ...);
void send_maze_state(void);
void send_sensor_data(void);
void send_position_data(void);
void send_performance_metrics(void);
void send_battery_status(void);
void send_championship_stats(void);
#endif
