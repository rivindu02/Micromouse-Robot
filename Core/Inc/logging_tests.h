/*
 * logging_tests.h
 *
 *  Created on: Aug 24, 2025
 *      Author: ASUS
 */

#ifndef INC_LOGGING_TESTS_H_
#define INC_LOGGING_TESTS_H_
#include "micromouse.h"
void run_gyro_step_test(int base_pwm, int delta_pwm, uint32_t step_delay_ms, uint32_t step_duration_ms, uint32_t sample_ms, uint32_t total_ms);
void run_encoder_step_test(int base_pwm, int delta_pwm, uint32_t step_delay_ms, uint32_t step_duration_ms, uint32_t sample_ms, uint32_t total_ms);

#endif /* INC_LOGGING_TESTS_H_ */
