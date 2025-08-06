#ifndef MOVEMENT_H
#define MOVEMENT_H
#include "micromouse.h"
void start_encoders(void);
void move_forward(void);
void turn_left(void);
void turn_right(void);
void turn_around(void);
void stop_motors(void);
void move_forward_distance(int distance_mm);
void move_forward_adaptive_speed(float speed_multiplier);
#endif
