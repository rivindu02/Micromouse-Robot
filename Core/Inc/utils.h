#ifndef UTILS_H
#define UTILS_H
#include "micromouse.h"
void delay_ms(uint32_t ms);
int abs_int(int value);
float calculate_distance(int x1, int y1, int x2, int y2);
void led_status(uint8_t left_state, uint8_t right_state);
void led_sequence_startup(void);
void led_sequence_exploring(void);
void led_sequence_returning(void);
void led_sequence_complete(void);
void led_sequence_error(void);
int map_value(int value, int from_low, int from_high, int to_low, int to_high);
int constrain_int(int value, int min_val, int max_val);
const char* get_direction_name(int direction);
int manhattan_distance(int x1, int y1, int x2, int y2);
bool is_valid_coordinate(int x, int y);
float moving_average_filter(float new_value, float previous_average, int samples);
bool system_health_check(void);
void performance_start_timer(void);
void performance_end_timer(const char* operation_name);
#endif
