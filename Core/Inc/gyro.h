#ifndef GYRO_H
#define GYRO_H
#include "micromouse.h"
bool mpu9250_init(void);
uint8_t mpu9250_read_register(uint8_t reg);
void mpu9250_write_register(uint8_t reg, uint8_t data);
void mpu9250_read_gyro(void);
void mpu9250_read_accel(void);
void mpu9250_read_all(void);
float mpu9250_get_gyro_z_dps(void);
bool mpu9250_detect_turn(void);
void mpu9250_calibrate_bias(void);
float mpu9250_get_gyro_z_compensated(void);

#endif
