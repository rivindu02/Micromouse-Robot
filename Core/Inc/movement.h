#ifndef MOVEMENT_H
#define MOVEMENT_H
#include "micromouse.h"
void start_encoders(void);
void move_forward(void);
void turn_left(void);
void turn_right(void);
void turn_around(void);
void stop_motors(void);
void break_motors(void);
void move_forward_distance(int distance_mm);
void move_forward_adaptive_speed(float speed_multiplier);
void motor_set(uint16_t ch_pwm, GPIO_TypeDef *dirPort, uint16_t dirPin, bool forward, uint16_t duty);
void moveStraightPID(void);
void moveStraightGyroPID(void);
void moveStraightGyroPID_Reset(void);

// Enhanced encoder functions
int32_t get_left_encoder_total(void);
int32_t get_right_encoder_total(void);
void reset_encoder_totals(void);
void update_encoder_totals(void);
void debug_encoder_setup(void);
void test_encoder_manual(void);
void test_encoder_rotation(void);
#endif
