/*
 * enhanced_movement.c - S-Curve Movement with Gyro Heading PID (FIXED VERSION)
 *
 * FIXED: Uses existing VelocityProfile struct instead of conflicting SCurveProfile
 * Integrates seamlessly with your existing velocity profile system
 */

#include "micromouse.h"
#include "velocity_profile.h"
#include <math.h>

// ===== S-CURVE PROFILE CONSTANTS =====
#define SCURVE_DEFAULT_MAX_VELOCITY     600.0f  // mm/s
#define SCURVE_DEFAULT_MAX_ACCELERATION 2000.0f // mm/s²
#define SCURVE_DEFAULT_MAX_JERK         8000.0f // mm/s³
#define ENCODER_COUNTS_TO_MM            (CELL_SIZE_MM / ENCODER_COUNTS_PER_CELL)
#define MOVEMENT_UPDATE_PERIOD_MS       5       // 200Hz update rate

// ===== HEADING PID PARAMETERS =====
//static float Kp_g = 7.0f;   // rate loop P
//static float Ki_g = 0.0f;   // rate loop I
//static float Kd_g = 0.2f;   // rate loop D


static float Kp_yaw = 1.0f;     // Start conservative: 1.0-3.0
static float Ki_yaw = 0.0f;     // Start 0.0-0.05 (add only if slow drift)
static float Kd_yaw = 0.1f;     // Start 0.05-0.25

// ===== PID STATE VARIABLES =====
static float yaw_deg        = 0.0f;  // Integrated heading (deg, 0 = straight)
static float yaw_err_i      = 0.0f;  // Integral term
static float yaw_err_prev   = 0.0f;  // For derivative
static const float I_CLAMP  = 50.0f; // Anti-windup clamp for integral
static const uint16_t PWM_MIN_MOVE = 50;   // Minimum PWM to keep wheels moving
static const uint16_t PWM_MAX      = 1000; // PWM cap

// ===== USE EXISTING VELOCITY PROFILE STRUCT =====
static VelocityProfile forward_profile = {0};

// Outer loop (angle -> desired rate) simple P:
//static float Kp_angle = 4.0f;     // starts modest; increases turn crispness
//static float OMEGA_MAX = 250.0f;  // deg/s cap during turns (safe)

// ===== UTILITY FUNCTIONS =====
static inline float clampf(float v, float lo, float hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
}

/**
 * @brief Reset heading PID state
 */
static void reset_heading_pid(void) {
    yaw_deg = 0.0f;
    yaw_err_i = 0.0f;
    yaw_err_prev = 0.0f;
    send_bluetooth_message("Heading PID reset\r\n");
}

/**
 * @brief Initialize S-curve profile using existing VelocityProfile struct
 */
static void scurve_profile_init_enhanced(VelocityProfile* profile, float distance,
                                        float max_vel, float max_accel, float max_jerk) {
    // Use existing velocity_profile_init as base
    velocity_profile_init(profile, distance, max_vel);

    // Override acceleration if different
    profile->max_acceleration = max_accel;

    // Note: max_jerk not used in current simple implementation
    // Could be added to VelocityProfile struct if needed for true S-curve

    send_bluetooth_printf("S-curve profile: dist=%.0f, vel=%.0f, accel=%.0f\r\n",
                          distance, max_vel, max_accel);
}

/**
 * @brief Convert velocity (mm/s) to PWM (0-1000)
 */
static uint16_t scurve_velocity_to_pwm(float velocity_mm_s) {
    // Linear mapping: 0 mm/s = 0 PWM, 600 mm/s = 800 PWM
    float pwm_float = velocity_mm_s * (800.0f / 600.0f);
    uint16_t pwm = (uint16_t)clampf(pwm_float, 0, PWM_MAX);
    return pwm;
}

/**
 * @brief Get optimal S-curve parameters based on distance
 */
static void scurve_get_optimal_parameters(float distance, float* max_vel,
                                         float* max_accel, float* max_jerk) {
    // Adjust parameters based on distance for optimal performance
    if (distance < 90.0f) { // Half cell
        *max_vel *= 0.7f;   // Reduce speed for short distances
        *max_accel *= 0.8f; // Gentler acceleration for short moves
    } else if (distance > 360.0f) { // Multi-cell moves
        *max_vel *= 1.2f;   // Can go faster for longer distances
        *max_accel *= 1.1f; // More aggressive acceleration

        // Clamp to reasonable limits
        if (*max_vel > 800.0f) *max_vel = 800.0f;
        if (*max_accel > 2500.0f) *max_accel = 2500.0f;
    }

    // Jerk is not used in current simple trapezoidal profile
    // Could be implemented for true S-curve if needed
    (void)max_jerk; // Suppress unused parameter warning
}

/**
 * @brief Enhanced S-curve forward movement with gyro stabilization
 */
void move_forward_scurve(float distance_mm, float speed_multiplier) {
    send_bluetooth_printf("🚀 S-curve forward: %.1f mm, speed=%.2fx\r\n",
                          distance_mm, speed_multiplier);

    // Check bounds before moving
    int new_x = robot.x + dx[robot.direction];
    int new_y = robot.y + dy[robot.direction];
    if (new_x < 0 || new_x >= MAZE_SIZE || new_y < 0 || new_y >= MAZE_SIZE) {
        send_bluetooth_message("❌ Cannot move - out of bounds!\r\n");
        return;
    }

    // Build S-curve profile for this distance
    float max_vel = SCURVE_DEFAULT_MAX_VELOCITY * speed_multiplier;
    float max_accel = SCURVE_DEFAULT_MAX_ACCELERATION;
    float max_jerk = SCURVE_DEFAULT_MAX_JERK;
    scurve_get_optimal_parameters(distance_mm, &max_vel, &max_accel, &max_jerk);
    scurve_profile_init_enhanced(&forward_profile, distance_mm, max_vel, max_accel, max_jerk);

    // Encoder start positions (distance authority)
    int32_t start_left = get_left_encoder_total();
    int32_t start_right = get_right_encoder_total();

    // Reset heading PID state
    reset_heading_pid();

    const float dt = (float)MOVEMENT_UPDATE_PERIOD_MS / 1000.0f;

    // Low-pass filter for gyro
    float gyro_z_filt = 0.0f;
    const float alpha = 0.3f; // Smoothing factor (0-1, higher = less smoothing)

    uint32_t loop_count = 0;
    uint32_t max_loops = (uint32_t)(5000 / MOVEMENT_UPDATE_PERIOD_MS); // 5 second timeout

    while (!velocity_profile_is_complete(&forward_profile) && loop_count < max_loops) {
        loop_count++;

        // Update velocity profile using existing function
        velocity_profile_update(&forward_profile);

        // Target feedforward speed from S-curve
        float v_target = velocity_profile_get_target_velocity(&forward_profile);
        uint16_t pwm_base = scurve_velocity_to_pwm(v_target);

        // Check distance stop (encoders are truth for distance)
        int32_t cur_left = get_left_encoder_total();
        int32_t cur_right = get_right_encoder_total();
        int32_t left_tr = cur_left - start_left;
        int32_t right_tr = cur_right - start_right;
        int32_t avg_tr = (left_tr + right_tr) / 2;
        float d_mm = avg_tr * ENCODER_COUNTS_TO_MM;

        if (d_mm >= distance_mm) {
            send_bluetooth_printf("✅ Distance reached: %.1f mm\r\n", d_mm);
            break;
        }

        // Gyro-based heading PID (if available)
        float pid_out = 0.0f;

        if (mpu9250_is_initialized()) {
            // Read and filter gyro
            mpu9250_read_gyro();
            float gyro_z_dps = mpu9250_get_gyro_z_compensated();

            // Low-pass filter to reduce noise
            gyro_z_filt = (1.0f - alpha) * gyro_z_filt + alpha * gyro_z_dps;

            // Integrate to heading in degrees
            yaw_deg += gyro_z_filt * dt;

            // PID calculation (target heading = 0°)
            float yaw_err = -yaw_deg; // Negative because we want to counter the drift
            yaw_err_i += yaw_err * dt;
            yaw_err_i = clampf(yaw_err_i, -I_CLAMP, I_CLAMP); // Anti-windup

            float yaw_err_d = (yaw_err - yaw_err_prev) / dt;
            yaw_err_prev = yaw_err;

            pid_out = (Kp_yaw * yaw_err) + (Ki_yaw * yaw_err_i) + (Kd_yaw * yaw_err_d);

            // Debug output every 40 loops (200ms at 200Hz) to avoid spam
            if (loop_count % 40 == 0) {
                send_bluetooth_printf("Yaw: %.1f°, PID: %.1f, PWM: %d\r\n",
                                      yaw_deg, pid_out, pwm_base);
            }
        }

        // Apply correction to motor speeds
        float correction = pid_out;
        int32_t left_pwm = (int32_t)pwm_base - (int32_t)correction;  // Left slower to correct right drift
        int32_t right_pwm = (int32_t)pwm_base + (int32_t)correction; // Right faster to correct right drift

        // Apply constraints - ensure motors keep moving
        if (v_target > 1.0f) { // Only apply minimum when actually moving
            if (left_pwm < PWM_MIN_MOVE) left_pwm = PWM_MIN_MOVE;
            if (right_pwm < PWM_MIN_MOVE) right_pwm = PWM_MIN_MOVE;
        } else {
            left_pwm = 0;   // Stop when profile says stop
            right_pwm = 0;
        }

        // Apply maximum limits
        if (left_pwm > PWM_MAX) left_pwm = PWM_MAX;
        if (right_pwm > PWM_MAX) right_pwm = PWM_MAX;


    	motor_set_fixed(0, true, (uint16_t)left_pwm);//Left

    	motor_set_fixed(1, true, (uint16_t)right_pwm);//Right

        HAL_Delay(MOVEMENT_UPDATE_PERIOD_MS);
    }

    // Stop and settle
    stop_motors();
    HAL_Delay(100); // Settling time

    // DON'T update robot position here - let the caller handle it
    // This maintains separation of concerns

    send_bluetooth_printf("✅ S-curve complete. Final yaw: %.2f°, loops: %lu\r\n",
                          yaw_deg, loop_count);

    if (loop_count >= max_loops) {
        send_bluetooth_message("⚠️ Movement timeout - check encoders/motors\r\n");
    }
}

/**
 * @brief Enhanced championship move forward using S-curve + gyro
 */
bool championship_move_forward_enhanced(void) {
    update_sensors();

    // Check for wall before moving
    if (sensors.wall_front) {
        send_bluetooth_message("Front wall detected, cannot move\r\n");
        return false;
    }

    // Check bounds
    int new_x = robot.x + dx[robot.direction];
    int new_y = robot.y + dy[robot.direction];
    if (new_x < 0 || new_x >= MAZE_SIZE || new_y < 0 || new_y >= MAZE_SIZE) {
        send_bluetooth_message("Cannot move - would go out of bounds!\r\n");
        return false;
    }

    // Use S-curve movement for one cell
    move_forward_scurve(CELL_SIZE_MM, 1.0f);

    // Update position after successful movement
    robot.x = new_x;
    robot.y = new_y;
    robot.exploration_steps++;

    return true;
}

/**
 * @brief Set heading PID gains for tuning
 */
void set_heading_pid_gains(float kp, float ki, float kd) {
    Kp_yaw = kp;
    Ki_yaw = ki;
    Kd_yaw = kd;
    reset_heading_pid(); // Reset state when gains change
    send_bluetooth_printf("Heading PID updated: Kp=%.2f, Ki=%.3f, Kd=%.2f\r\n", kp, ki, kd);
}

/**
 * @brief Get current heading PID status
 */
void get_heading_pid_status(void) {
    send_bluetooth_printf("Heading PID - Gains: Kp=%.2f Ki=%.3f Kd=%.2f\r\n",
                          Kp_yaw, Ki_yaw, Kd_yaw);
    send_bluetooth_printf("State: Yaw=%.2f° ErrI=%.2f ErrPrev=%.2f\r\n",
                          yaw_deg, yaw_err_i, yaw_err_prev);
}

/**
 * @brief Test S-curve movement with different distances
 */
void test_scurve_movement(void) {
    send_bluetooth_message("\r\n🧪 TESTING S-CURVE MOVEMENT\r\n");

    if (!mpu9250_is_initialized()) {
        send_bluetooth_message("⚠️ Gyro not available - testing basic S-curve only\r\n");
    } else {
        send_bluetooth_message("✅ Gyro available - testing with heading stabilization\r\n");
    }

    // Test different distances
    float test_distances[] = {90.0f, 180.0f, 360.0f}; // Half cell, full cell, two cells
    int num_tests = sizeof(test_distances) / sizeof(test_distances[0]);

    for (int i = 0; i < num_tests; i++) {
        send_bluetooth_printf("\r\nTest %d: S-curve %.0f mm\r\n", i + 1, test_distances[i]);
        send_bluetooth_message("Press LEFT button to start test...\r\n");

        // Wait for button press
        while (button_pressed != 1) {
            HAL_Delay(100);
            // Blink LED to show waiting
            if ((HAL_GetTick() / 500) % 2) {
                led_status(1, 0);
            } else {
                led_status(0, 0);
            }
        }
        button_pressed = 0;
        led_status(0, 0);

        // Execute test
        send_bluetooth_message("🚀 Starting test movement...\r\n");
        move_forward_scurve(test_distances[i], 1.0f);

        send_bluetooth_message("✅ Test complete. Press RIGHT button for next test.\r\n");
        while (button_pressed != 2 && i < num_tests - 1) { // Skip wait on last test
            HAL_Delay(100);
            // Blink different pattern
            if ((HAL_GetTick() / 300) % 2) {
                led_status(0, 1);
            } else {
                led_status(0, 0);
            }
        }
        button_pressed = 0;
        led_status(0, 0);

        HAL_Delay(1000);
    }

    send_bluetooth_message("🏁 S-curve movement test complete!\r\n");
    play_success_tone();
}

/**
 * @brief Quick S-curve movement test (single cell)
 */
void test_scurve_single_cell(void) {
    send_bluetooth_message("\r\n🧪 QUICK S-CURVE TEST (180mm)\r\n");

    if (mpu9250_is_initialized()) {
        send_bluetooth_message("✅ Using gyro heading stabilization\r\n");
    } else {
        send_bluetooth_message("⚠️ No gyro - basic movement only\r\n");
    }

    move_forward_scurve(CELL_SIZE_MM, 1.0f);
    send_bluetooth_message("✅ Single cell test complete!\r\n");
}
