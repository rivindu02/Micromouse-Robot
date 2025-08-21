/*
 * velocity_profile.c
 *
 *  Created on: Aug 18, 2025
 *      Author: ASUS
 */


#include "velocity_profile.h"

void velocity_profile_init(VelocityProfile* profile, float distance, float max_vel) {
    profile->max_velocity = max_vel;
    profile->max_acceleration = max_vel / 0.5f;  // Reach max vel in 0.5s
    profile->distance_remaining = distance;
    profile->current_velocity = 0.0f;
    profile->profile_start_time = HAL_GetTick();
    profile->profile_active = true;
    profile->t_accel = profile->max_velocity / profile->max_acceleration;
}

void velocity_profile_update(VelocityProfile* profile) {
    if (!profile->profile_active) return;

    uint32_t current_time = HAL_GetTick();
    float elapsed = (current_time - profile->profile_start_time) / 1000.0f;

    // Simple trapezoidal profile
    if (elapsed < profile->t_accel) {
        // Acceleration phase
        profile->current_velocity = profile->max_acceleration * elapsed;
    } else if (elapsed < 2 * profile->t_accel) {
        // Deceleration phase
        float decel_time = elapsed - profile->t_accel;
        profile->current_velocity = profile->max_velocity - profile->max_acceleration * decel_time;
        if (profile->current_velocity <= 0) {
            profile->current_velocity = 0;
            profile->profile_active = false;
        }
    } else {
        profile->current_velocity = 0;
        profile->profile_active = false;
    }
}

float velocity_profile_get_target_velocity(VelocityProfile* profile) {
    return profile->current_velocity;
}

bool velocity_profile_is_complete(VelocityProfile* profile) {
    return !profile->profile_active;
}
