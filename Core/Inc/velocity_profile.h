/*
 * velocity_profile.h
 *
 *  Created on: Aug 18, 2025
 *      Author: ASUS
 */

#ifndef INC_VELOCITY_PROFILE_H_
#define INC_VELOCITY_PROFILE_H_


#include "micromouse.h"

typedef struct {
    float max_velocity;       // mm/s
    float max_acceleration;   // mm/s²
    float current_velocity;
    float distance_remaining;
    uint32_t profile_start_time;
    bool profile_active;
    float t_accel;           // Time for acceleration phase
} VelocityProfile;

void velocity_profile_init(VelocityProfile* profile, float distance, float max_vel);
void velocity_profile_update(VelocityProfile* profile);
float velocity_profile_get_target_velocity(VelocityProfile* profile);
bool velocity_profile_is_complete(VelocityProfile* profile);


#endif /* INC_VELOCITY_PROFILE_H_ */
