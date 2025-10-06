#ifndef MOVEMENT_H
#define MOVEMENT_H
#include "micromouse.h"
void start_encoders(void);
void turn_left(void);
void turn_right(void);
void move_forward_distance(int Left_target_counts,int Right_target_counts);
void turn_around(void);
void stop_motors(void);
void break_motors(void);
void motor_set(uint8_t motor, bool forward, uint16_t duty);
void moveStraightPID(int base_pwm, bool left_forward, bool right_forward);
void moveStraightGyroPID(void);
void moveStraightGyroPID_Reset(void);
float gyro_rate_pid_step(float sp_dps, float meas_dps, float *p_dt);
void turn_in_place_gyro(float angle_deg, int base_pwm, uint32_t timeout_ms) ;
// Enhanced encoder functions
int32_t get_left_encoder_total(void);
int32_t get_right_encoder_total(void);
void reset_encoder_totals(void);
void update_encoder_totals(void);
void debug_encoder_setup(void);
void test_encoder_manual(void);
void test_encoder_rotation(void);
void gyro_turn_reset(void);



#endif
