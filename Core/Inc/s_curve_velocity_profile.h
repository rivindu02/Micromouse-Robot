#ifndef INC_S_CURVE_VELOCITY_PROFILE_H_
#define INC_S_CURVE_VELOCITY_PROFILE_H_

#include "micromouse.h"
#include <stdint.h>
#include <stdbool.h>

/* REMOVED: SCurveProfile struct definition - now using the one from micromouse.h */

/* Default S-Curve Parameters for Micromouse */
#define SCURVE_DEFAULT_MAX_VELOCITY 400.0f // mm/s
#define SCURVE_DEFAULT_MAX_ACCELERATION 2000.0f // mm/s²
#define SCURVE_DEFAULT_MAX_JERK 10000.0f // mm/s³
#define SCURVE_CELL_DISTANCE 180.0f // mm - Standard micromouse cell size

/* S-Curve Profile Functions */

/**
 * @brief Initialize S-curve velocity profile
 * @param profile Pointer to S-curve profile structure
 * @param distance Target distance in mm
 * @param max_vel Maximum velocity in mm/s
 * @param max_accel Maximum acceleration in mm/s²
 * @param max_jerk_limit Maximum jerk in mm/s³
 */
void scurve_profile_init(SCurveProfile* profile, float distance, float max_vel,
                        float max_accel, float max_jerk_limit);

/**
 * @brief Calculate optimal time segments for S-curve profile
 * @param profile Pointer to S-curve profile structure
 */
void scurve_calculate_time_segments(SCurveProfile* profile);

/**
 * @brief Update S-curve profile state (call every 1ms)
 * @param profile Pointer to S-curve profile structure
 */
void scurve_profile_update(SCurveProfile* profile);

/**
 * @brief Get current target velocity from S-curve profile
 * @param profile Pointer to S-curve profile structure
 * @return Current velocity in mm/s
 */
float scurve_profile_get_velocity(SCurveProfile* profile);

/**
 * @brief Get current target acceleration from S-curve profile
 * @param profile Pointer to S-curve profile structure
 * @return Current acceleration in mm/s²
 */
float scurve_profile_get_acceleration(SCurveProfile* profile);

/**
 * @brief Get current position from S-curve profile
 * @param profile Pointer to S-curve profile structure
 * @return Current position in mm
 */
float scurve_profile_get_position(SCurveProfile* profile);

/**
 * @brief Check if S-curve profile is complete
 * @param profile Pointer to S-curve profile structure
 * @return true if profile is complete, false otherwise
 */
bool scurve_profile_is_complete(SCurveProfile* profile);

/**
 * @brief Get S-curve profile progress percentage
 * @param profile Pointer to S-curve profile structure
 * @return Progress percentage (0-100%)
 */
float scurve_profile_get_progress(SCurveProfile* profile);

/**
 * @brief Reset S-curve profile to initial state
 * @param profile Pointer to S-curve profile structure
 */
void scurve_profile_reset(SCurveProfile* profile);

/**
 * @brief Send S-curve profile status via Bluetooth for debugging
 * @param profile Pointer to S-curve profile structure
 */
void scurve_profile_send_status(SCurveProfile* profile);

/* Advanced S-Curve Functions */

/**
 * @brief Create S-curve profile for one micromouse cell movement
 * @param profile Pointer to S-curve profile structure
 * @param speed_multiplier Speed multiplier (0.5-2.0 typical)
 */
void scurve_profile_init_cell_movement(SCurveProfile* profile, float speed_multiplier);

/**
 * @brief Create S-curve profile for turn movements
 * @param profile Pointer to S-curve profile structure
 * @param angle_degrees Turn angle in degrees (90, 180, etc.)
 */
void scurve_profile_init_turn_movement(SCurveProfile* profile, float angle_degrees);

/**
 * @brief Convert S-curve velocity to motor PWM value
 * @param velocity_mm_s Velocity in mm/s
 * @return PWM duty cycle (0-1000)
 */
uint16_t scurve_velocity_to_pwm(float velocity_mm_s);

/**
 * @brief Get optimal S-curve parameters based on distance
 * @param distance Distance in mm
 * @param max_vel Output: recommended max velocity
 * @param max_accel Output: recommended max acceleration
 * @param max_jerk Output: recommended max jerk
 */
void scurve_get_optimal_parameters(float distance, float* max_vel, float* max_accel, float* max_jerk);

#endif /* INC_S_CURVE_VELOCITY_PROFILE_H_ */
