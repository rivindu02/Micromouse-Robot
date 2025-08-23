/*
 * s_curve_velocity_profile.c - Robust 7-segment jerk-limited S-curve
 * Works at arbitrary loop rates (uses real dt from HAL_GetTick()).
 * Handles short segments (no cruise) and ultra-short (no constant-accel).
 */

#include "s_curve_velocity_profile.h"
#include "main.h"     // for HAL_GetTick
#include <math.h>
#include <string.h>

/* ===== Internal helpers ===== */

static inline float clampf(float x, float lo, float hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}




/* Compute segment durations (tj, ta, tv) that fit distance with given limits.
   Adjust amax downward if the move is extremely short. Symmetric accel/decel. */
static void compute_segments(SCurveProfile* p) {
    const float s = fmaxf(0.0f, p->distance_mm);
    float vmax = fmaxf(10.0f, p->vmax);
    float amax = fmaxf(10.0f, p->amax);
    float jmax = fmaxf(100.0f, p->jmax);

    /* Base jerk-up time to reach amax */
    float tj = amax / jmax;   // s

    /* If vmax is so small we wouldn't have constant-accel, allow ta=0 */
    float ta = (vmax / amax) - tj;   // s
    if (ta < 0.0f) ta = 0.0f;

    /* Distance to accelerate from 0 -> vmax (half-profile):
         s_acc_half = amax*(tj^2 + 1.5*ta*tj + 0.5*ta^2)
       Proof done by integrating jerk -> accel -> vel -> pos. */
    float s_acc_half = amax * (tj*tj + 1.5f*ta*tj + 0.5f*ta*ta);
    float s_min_full = 2.0f * s_acc_half;  // distance with zero cruise at given vmax/amax/jmax

    float tv = 0.0f;  // cruise time

    if (s > s_min_full + 1e-6f) {
        /* We have space to cruise at vmax */
        tv = (s - s_min_full) / vmax;
    } else {
        /* No cruise; solve for ta (>=0) that fits the distance exactly with given amax,jmax.
           s/2 = amax*(tj^2 + 1.5*ta*tj + 0.5*ta^2)  =>  quadratic in ta. */
        tv = 0.0f;
        float K = (s * 0.5f) / amax - tj*tj;           // K = 1.5*ta*tj + 0.5*ta^2
        float A = 0.5f;
        float B = 1.5f * tj;
        float C = -K;
        float D = B*B - 4*A*C;

        if (D >= 0.0f) {
            float root = (-B + sqrtf(D)) / (2*A);
            if (root < 0.0f) root = (-B - sqrtf(D)) / (2*A);
            ta = fmaxf(0.0f, root);
        } else {
            /* Even ta=0 is too long => reduce amax so move is purely jerk-limited (no const-accel) */
            ta = 0.0f;
            /* s = 2 * a_eff * (a_eff/jmax)^2  => a_eff = (0.5*s*jmax^2)^(1/3) */
            amax = cbrtf(0.5f * s * jmax*jmax);
            tj   = amax / jmax;
        }

        /* Also cap vmax to the reachable peak for this short move */
        float v_half = amax * (ta + tj);
        vmax = 2.0f * v_half - vmax; // not used further; keep p->vmax as requested
        (void)vmax;
    }

    p->tj = tj;
    p->ta = ta;
    p->tv = tv;

    /* Total time: 4*tj + 2*ta + tv */
    p->t_total = 4.0f*tj + 2.0f*ta + tv;

    /* Persist possibly-reduced amax back into the profile */
    p->amax = amax;
}

/* Compute commanded jerk J(t) for the current elapsed time */
static float jerk_at(const SCurveProfile* p, float t) {
    const float tj = p->tj, ta = p->ta, tv = p->tv;

    float t1 = tj;
    float t2 = tj + ta;
    float t3 = 2*tj + ta;
    float t4 = 2*tj + ta + tv;
    float t5 = 3*tj + ta + tv;
    float t6 = 3*tj + 2*ta + tv;
    float t7 = 4*tj + 2*ta + tv;

    if      (t < t1) return +p->jmax;   // 1) jerk up
    else if (t < t2) return  0.0f;      // 2) constant +a
    else if (t < t3) return -p->jmax;   // 3) jerk down to 0
    else if (t < t4) return  0.0f;      // 4) cruise
    else if (t < t5) return -p->jmax;   // 5) jerk negative
    else if (t < t6) return  0.0f;      // 6) constant -a
    else if (t < t7) return +p->jmax;   // 7) jerk up back to 0
    else             return  0.0f;
}

/* Clamp acceleration in constant-acc phases to exactly {+amax,0,-amax} to limit drift */
static void clamp_accel_in_phase(SCurveProfile* p) {
    const float tj = p->tj, ta = p->ta, tv = p->tv;
    float t = p->t_elapsed;

    float t1 = tj;
    float t2 = tj + ta;
    float t3 = 2*tj + ta;
    float t4 = 2*tj + ta + tv;
    float t5 = 3*tj + ta + tv;
    float t6 = 3*tj + 2*ta + tv;
    float t7 = 4*tj + 2*ta + tv;

    if      (t < t1) { /* accelerating jerk-up */ }
    else if (t < t2) { p->a =  p->amax; }
    else if (t < t3) { /* jerk down */ }
    else if (t < t4) { p->a =  0.0f;    }
    else if (t < t5) { /* jerk negative */ }
    else if (t < t6) { p->a = -p->amax; }
    else if (t < t7) { /* jerk up to zero */ }
    else              { p->a = 0.0f; }
}

/* ===== Public API ===== */

void scurve_profile_init(SCurveProfile* p, float distance_mm, float vmax, float amax, float jmax) {
    memset(p, 0, sizeof(*p));
    p->distance_mm = fmaxf(0.0f, distance_mm);
    p->vmax = fmaxf(10.0f, vmax);
    p->amax = fmaxf(10.0f, amax);
    p->jmax = fmaxf(100.0f, jmax);
    p->complete = false;
    p->t_elapsed = 0.0f;
    p->s = p->v = p->a = p->j = 0.0f;
    p->phase = 0;
    p->last_update_ms = HAL_GetTick();
    compute_segments(p);
}

void scurve_profile_reset_time(SCurveProfile* p) {
    p->t_elapsed = 0.0f;
    p->s = p->v = p->a = p->j = 0.0f;
    p->phase = 0;
    p->complete = false;
    p->last_update_ms = HAL_GetTick();
}

void scurve_profile_update(SCurveProfile* p) {
    if (p->complete) return;

    uint32_t now = HAL_GetTick();
    float dt = (now - p->last_update_ms) / 1000.0f;
    if (dt <= 0.0f) return;
    p->last_update_ms = now;

    /* Command jerk according to segment */
    p->j = jerk_at(p, p->t_elapsed);

    /* Integrate jerk -> accel -> vel -> pos (semi-implicit Euler) */
    p->a += p->j * dt;
    clamp_accel_in_phase(p);                // snap accel on flat phases to limit drift
    p->v += p->a * dt;
    if (p->v < 0.0f) p->v = 0.0f;           // no negative forward speed
    p->s += p->v * dt;

    p->t_elapsed += dt;

    /* Done? */
    bool time_done = (p->t_elapsed >= p->t_total - 1e-4f);
    bool dist_done = (p->s >= p->distance_mm - 0.5f);  // 0.5mm tolerance
    if (time_done || dist_done) {
        p->complete = true;
        p->j = 0.0f; p->a = 0.0f; p->v = 0.0f;
    }
}

float scurve_profile_get_velocity(SCurveProfile* p) {
    return p->complete ? 0.0f : p->v;
}

bool scurve_profile_is_complete(SCurveProfile* p) {
    return p->complete;
}

/* Heuristic constraints by distance; tune to your bot safely */
void scurve_get_optimal_parameters(float distance, float* vmax, float* amax, float* jmax) {
    /* Baselines (keep modest until you finish calibration) */
    float V = 600.0f;    // mm/s
    float A = 2000.0f;   // mm/s^2
    float J = 8000.0f;   // mm/s^3

    if (distance < 100.0f) {      // very short (half-cell-ish)
        V *= 0.55f; A *= 0.65f; J *= 0.80f;
    } else if (distance < 180.0f) { // one cell
        V *= 0.75f; A *= 0.85f; J *= 1.00f;
    } else {                      // long
        /* keep defaults */
    }
    *vmax = V; *amax = A; *jmax = J;
}

/* Keep this consistent with your calibrated speed map!
   Start with 500 mm/s -> 1000 PWM and refine after straight-line tests. */
uint16_t scurve_velocity_to_pwm(float velocity_mm_s) {
    float pwm = (velocity_mm_s / 500.0f) * 1000.0f;
    if (velocity_mm_s > 1.0f && pwm < 50.0f) pwm = 50.0f; /* min to overcome stiction */
    if (pwm > 1000.0f) pwm = 1000.0f;
    if (pwm < 0.0f) pwm = 0.0f;
    return (uint16_t)(pwm + 0.5f);
}


/* REMOVED: Duplicate SCurveProfile struct definition - using micromouse.h version */

/* S-Curve Profile Segments:
 * 1: Jerk up (acceleration increasing)
 * 2: Constant acceleration
 * 3: Jerk down (acceleration decreasing to zero)
 * 4: Constant velocity (cruise)
 * 5: Jerk down (deceleration increasing)
 * 6: Constant deceleration
 * 7: Jerk up (deceleration decreasing to zero)
 */

/**
 * @brief Initialize S-curve velocity profile
 * @param profile Pointer to S-curve profile structure
 * @param distance Target distance in mm
 * @param max_vel Maximum velocity in mm/s
 * @param max_accel Maximum acceleration in mm/s²
 * @param max_jerk_limit Maximum jerk in mm/s³
 */
//void scurve_profile_init(SCurveProfile* profile, float distance, float max_vel,
//                        float max_accel, float max_jerk_limit) {
//    // Initialize parameters
//    profile->target_distance = fabsf(distance);
//    profile->max_velocity = max_vel;
//    profile->max_acceleration = max_accel;
//    profile->max_jerk = max_jerk_limit;
//
//    // Reset state
//    profile->current_position = 0.0f;
//    profile->current_velocity = 0.0f;
//    profile->current_acceleration = 0.0f;
//    profile->current_jerk = 0.0f;
//    profile->current_segment = 1;
//    profile->profile_active = true;
//    profile->profile_complete = false;
//    profile->start_time = HAL_GetTick();
//
//    // Calculate time segments for optimal S-curve
//    scurve_calculate_time_segments(profile);
//
//    send_bluetooth_printf("S-Curve initialized: d=%.1fmm, v_max=%.1fmm/s, a_max=%.1fmm/s², j_max=%.1fmm/s³\r\n",
//                         distance, max_vel, max_accel, max_jerk_limit);
//}
//
///**
// * @brief Calculate optimal time segments for S-curve profile
// */
//void scurve_calculate_time_segments(SCurveProfile* profile) {
//    float T_j = profile->max_acceleration / profile->max_jerk; // Jerk time
//    float T_a = profile->max_velocity / profile->max_acceleration; // Acceleration time
//
//    // Check if we can reach maximum velocity
//    float s_min = profile->max_velocity * T_a; // Minimum distance to reach max velocity
//
//    if (profile->target_distance >= 2 * s_min) {
//        // Case 1: Can reach maximum velocity (7-segment profile)
//        profile->t1 = T_j; // Jerk up
//        profile->t2 = T_a - 2 * T_j; // Constant acceleration
//        profile->t3 = T_j; // Jerk down
//
//        // Calculate cruise distance and time
//        float accel_distance = profile->max_velocity * T_a / 2;
//        float cruise_distance = profile->target_distance - 2 * accel_distance;
//        profile->t4 = cruise_distance / profile->max_velocity; // Cruise time
//
//        profile->t5 = T_j; // Jerk down (decel)
//        profile->t6 = T_a - 2 * T_j; // Constant deceleration
//        profile->t7 = T_j; // Jerk up (decel end)
//    } else {
//        // Case 2: Cannot reach maximum velocity (triangular-like profile)
//        // Solve for reduced acceleration and velocity
//        float discriminant = 4 * profile->max_jerk * profile->target_distance;
//        float T_total = sqrtf(discriminant / profile->max_jerk);
//
//        profile->t1 = T_total / 4;
//        profile->t2 = T_total / 2 - profile->t1;
//        profile->t3 = profile->t1;
//        profile->t4 = 0; // No cruise phase
//        profile->t5 = profile->t1;
//        profile->t6 = profile->t2;
//        profile->t7 = profile->t1;
//
//        // Recalculate max velocity for this profile
//        profile->max_velocity = profile->max_jerk * profile->t1 * (profile->t1 + profile->t2);
//    }
//
//    profile->total_time = profile->t1 + profile->t2 + profile->t3 + profile->t4 +
//                         profile->t5 + profile->t6 + profile->t7;
//
//    send_bluetooth_printf("S-Curve segments: t1=%.3f t2=%.3f t3=%.3f t4=%.3f t5=%.3f t6=%.3f t7=%.3f (total=%.3fs)\r\n",
//                         profile->t1, profile->t2, profile->t3, profile->t4,
//                         profile->t5, profile->t6, profile->t7, profile->total_time);
//}
//
///**
// * @brief Update S-curve profile state
// * @param profile Pointer to S-curve profile structure
// */
//void scurve_profile_update(SCurveProfile* profile) {
//    if (!profile->profile_active || profile->profile_complete) {
//        return;
//    }
//
//    float elapsed = (HAL_GetTick() - profile->start_time) / 1000.0f;
//    float dt = 0.001f; // 1ms update period
//
//    // Determine current segment based on elapsed time
//    float t_cumulative = 0;
//    uint8_t segment = 1;
//    if (elapsed <= (t_cumulative += profile->t1)) segment = 1;
//    else if (elapsed <= (t_cumulative += profile->t2)) segment = 2;
//    else if (elapsed <= (t_cumulative += profile->t3)) segment = 3;
//    else if (elapsed <= (t_cumulative += profile->t4)) segment = 4;
//    else if (elapsed <= (t_cumulative += profile->t5)) segment = 5;
//    else if (elapsed <= (t_cumulative += profile->t6)) segment = 6;
//    else if (elapsed <= (t_cumulative += profile->t7)) segment = 7;
//    else {
//        // Profile complete
//        profile->profile_complete = true;
//        profile->profile_active = false;
//        profile->current_velocity = 0.0f;
//        profile->current_acceleration = 0.0f;
//        profile->current_jerk = 0.0f;
//        profile->current_position = profile->target_distance;
//        return;
//    }
//
//    profile->current_segment = segment;
//
//    // Calculate jerk, acceleration, and velocity for current segment
//    switch (segment) {
//        case 1: // Jerk up (acceleration increasing)
//            profile->current_jerk = profile->max_jerk;
//            break;
//        case 2: // Constant acceleration
//            profile->current_jerk = 0.0f;
//            break;
//        case 3: // Jerk down (acceleration decreasing)
//            profile->current_jerk = -profile->max_jerk;
//            break;
//        case 4: // Constant velocity (cruise)
//            profile->current_jerk = 0.0f;
//            break;
//        case 5: // Jerk down (deceleration increasing)
//            profile->current_jerk = -profile->max_jerk;
//            break;
//        case 6: // Constant deceleration
//            profile->current_jerk = 0.0f;
//            break;
//        case 7: // Jerk up (deceleration decreasing)
//            profile->current_jerk = profile->max_jerk;
//            break;
//    }
//
//    // Integrate to get acceleration, velocity, and position
//    profile->current_acceleration += profile->current_jerk * dt;
//    profile->current_velocity += profile->current_acceleration * dt;
//    profile->current_position += profile->current_velocity * dt;
//
//    // Clamp values to reasonable limits
//    if (profile->current_velocity < 0) profile->current_velocity = 0;
//    if (profile->current_velocity > profile->max_velocity * 1.1f) {
//        profile->current_velocity = profile->max_velocity * 1.1f;
//    }
//}
//
///**
// * @brief Get current target velocity from S-curve profile
// */
//float scurve_profile_get_velocity(SCurveProfile* profile) {
//    return profile->current_velocity;
//}
//
///**
// * @brief Get current target acceleration from S-curve profile
// */
//float scurve_profile_get_acceleration(SCurveProfile* profile) {
//    return profile->current_acceleration;
//}
//
///**
// * @brief Get current position from S-curve profile
// */
//float scurve_profile_get_position(SCurveProfile* profile) {
//    return profile->current_position;
//}
//
///**
// * @brief Check if S-curve profile is complete
// */
//bool scurve_profile_is_complete(SCurveProfile* profile) {
//    return profile->profile_complete;
//}
//
///**
// * @brief Get S-curve profile progress percentage
// */
//float scurve_profile_get_progress(SCurveProfile* profile) {
//    if (profile->target_distance <= 0) return 100.0f;
//    return (profile->current_position / profile->target_distance) * 100.0f;
//}
//
///**
// * @brief Reset S-curve profile
// */
//void scurve_profile_reset(SCurveProfile* profile) {
//    profile->current_position = 0.0f;
//    profile->current_velocity = 0.0f;
//    profile->current_acceleration = 0.0f;
//    profile->current_jerk = 0.0f;
//    profile->current_segment = 1;
//    profile->profile_active = false;
//    profile->profile_complete = false;
//    profile->start_time = HAL_GetTick();
//}
//
///**
// * @brief Send S-curve profile status via Bluetooth
// */
//void scurve_profile_send_status(SCurveProfile* profile) {
//    send_bluetooth_printf("S-Curve Status: seg=%d pos=%.1fmm vel=%.1fmm/s accel=%.1fmm/s² jerk=%.1fmm/s³ progress=%.1f%%\r\n",
//                         profile->current_segment,
//                         profile->current_position,
//                         profile->current_velocity,
//                         profile->current_acceleration,
//                         profile->current_jerk,
//                         scurve_profile_get_progress(profile));
//}
//
///**
// * @brief Create S-curve profile for one micromouse cell movement
// * @param profile Pointer to S-curve profile structure
// * @param speed_multiplier Speed multiplier (0.5-2.0 typical)
// */
//void scurve_profile_init_cell_movement(SCurveProfile* profile, float speed_multiplier) {
//    float max_vel = SCURVE_DEFAULT_MAX_VELOCITY * speed_multiplier;
//    float max_accel = SCURVE_DEFAULT_MAX_ACCELERATION;
//    float max_jerk = SCURVE_DEFAULT_MAX_JERK;
//
//    scurve_profile_init(profile, SCURVE_CELL_DISTANCE, max_vel, max_accel, max_jerk);
//}
//
///**
// * @brief Create S-curve profile for turn movements
// * @param profile Pointer to S-curve profile structure
// * @param angle_degrees Turn angle in degrees (90, 180, etc.)
// */
//void scurve_profile_init_turn_movement(SCurveProfile* profile, float angle_degrees) {
//    // Convert angle to arc length for differential drive
//    float wheel_base = 100.0f; // mm - distance between wheels
//    float arc_length = (angle_degrees / 360.0f) * 3.14159f * wheel_base;
//
//    float max_vel = SCURVE_DEFAULT_MAX_VELOCITY * 0.5f; // Slower for turns
//    float max_accel = SCURVE_DEFAULT_MAX_ACCELERATION * 0.7f;
//    float max_jerk = SCURVE_DEFAULT_MAX_JERK * 0.8f;
//
//    scurve_profile_init(profile, arc_length, max_vel, max_accel, max_jerk);
//}
//
///**
// * @brief Convert S-curve velocity to motor PWM value
// * @param velocity_mm_s Velocity in mm/s
// * @return PWM duty cycle (0-1000)
// */
//uint16_t scurve_velocity_to_pwm(float velocity_mm_s) {
//    // Linear conversion from velocity to PWM
//    // Assume max velocity of 500 mm/s corresponds to PWM 1000
//    float pwm_float = (velocity_mm_s / 500.0f) * 1000.0f;
//
//    // Clamp to valid PWM range
//    if (pwm_float < 50.0f) pwm_float = 50.0f; // Minimum PWM for movement
//    if (pwm_float > 1000.0f) pwm_float = 1000.0f; // Maximum PWM
//
//    return (uint16_t)pwm_float;
//}
//
///**
// * @brief Get optimal S-curve parameters based on distance
// * @param distance Distance in mm
// * @param max_vel Output: recommended max velocity
// * @param max_accel Output: recommended max acceleration
// * @param max_jerk Output: recommended max jerk
// */
//void scurve_get_optimal_parameters(float distance, float* max_vel, float* max_accel, float* max_jerk) {
//    if (distance < 50.0f) {
//        // Short distance - lower speeds
//        *max_vel = SCURVE_DEFAULT_MAX_VELOCITY * 0.3f;
//        *max_accel = SCURVE_DEFAULT_MAX_ACCELERATION * 0.5f;
//        *max_jerk = SCURVE_DEFAULT_MAX_JERK * 0.7f;
//    } else if (distance < 180.0f) {
//        // One cell - normal speeds
//        *max_vel = SCURVE_DEFAULT_MAX_VELOCITY * 0.7f;
//        *max_accel = SCURVE_DEFAULT_MAX_ACCELERATION * 0.8f;
//        *max_jerk = SCURVE_DEFAULT_MAX_JERK;
//    } else {
//        // Long distance - higher speeds
//        *max_vel = SCURVE_DEFAULT_MAX_VELOCITY;
//        *max_accel = SCURVE_DEFAULT_MAX_ACCELERATION;
//        *max_jerk = SCURVE_DEFAULT_MAX_JERK;
//    }
//}
