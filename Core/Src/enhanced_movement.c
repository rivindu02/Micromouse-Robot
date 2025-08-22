/*
 * enhanced_movement.c - Enhanced Movement with S-Curve Profiles - FIXED
 *
 * Replaces old trapezoidal movement with smooth S-curve velocity profiles
 * Integrates with gyro and encoder systems for precise motion control
 *
 * FIXED: Added missing function declarations and removed unused variables
 * Created: August 22, 2025
 */

#include "micromouse.h"
#include "s_curve_velocity_profile.h"
#include <math.h>

/* Global S-curve profile instances */
static SCurveProfile forward_profile;
static SCurveProfile turn_profile;

/* Movement parameters */
#define MOVEMENT_UPDATE_PERIOD_MS 5 // 5ms update rate (200Hz)
#define ENCODER_COUNTS_TO_MM (CELL_SIZE_MM / ENCODER_COUNTS_PER_CELL)
#define PWM_VELOCITY_SCALE 2.5f // Scale factor for velocity->PWM conversion

/**
 * @brief Enhanced move forward with S-curve velocity profile
 * @param distance_mm Distance to move in mm
 * @param speed_multiplier Speed multiplier (0.5-2.0)
 */
void move_forward_scurve(float distance_mm, float speed_multiplier) {
    send_bluetooth_printf("Starting S-curve forward movement: %.1fmm, speed=%.1fx\r\n",
                         distance_mm, speed_multiplier);

    // Initialize S-curve profile for forward movement
    float max_vel = SCURVE_DEFAULT_MAX_VELOCITY * speed_multiplier;
    float max_accel = SCURVE_DEFAULT_MAX_ACCELERATION;
    float max_jerk = SCURVE_DEFAULT_MAX_JERK;

    // Adjust parameters based on distance
    scurve_get_optimal_parameters(distance_mm, &max_vel, &max_accel, &max_jerk);
    scurve_profile_init(&forward_profile, distance_mm, max_vel, max_accel, max_jerk);

    // Get starting encoder positions
    int32_t start_left = get_left_encoder_total();
    int32_t start_right = get_right_encoder_total();

    // Movement loop with S-curve profile
    while (!scurve_profile_is_complete(&forward_profile)) {
        scurve_profile_update(&forward_profile);
        float target_velocity = scurve_profile_get_velocity(&forward_profile);
        uint16_t pwm_duty = scurve_velocity_to_pwm(target_velocity);

        // Check if we've traveled the required distance using encoders
        int32_t current_left = get_left_encoder_total();
        int32_t current_right = get_right_encoder_total();
        int32_t left_traveled = current_left - start_left;
        int32_t right_traveled = current_right - start_right;
        int32_t avg_traveled = (left_traveled + right_traveled) / 2;
        float traveled_mm = avg_traveled * ENCODER_COUNTS_TO_MM;

        if (traveled_mm >= distance_mm) {
            send_bluetooth_printf("Distance reached by encoder: %.1fmm\r\n", traveled_mm);
            break;
        }

        // Apply PWM to motors with direction control
        motor_set(TIM_CHANNEL_1, MOTOR_IN2_GPIO_Port, MOTOR_IN2_Pin, true, pwm_duty); // Left motor
        motor_set(TIM_CHANNEL_3, MOTOR_IN4_GPIO_Port, MOTOR_IN4_Pin, true, pwm_duty); // Right motor

        // Gyroscope feedback for straight-line correction
        if (mpu9250_is_initialized()) {
            mpu9250_read_gyro();
            float gyro_z = mpu9250_get_gyro_z_compensated();

            if (fabsf(gyro_z) > 5.0f) { // Rotation detected
                // Apply minor correction (simple proportional control)
                int16_t correction = (int16_t)(gyro_z * 2.0f);
                if (correction > 50) correction = 50;
                if (correction < -50) correction = -50;

                // Adjust motor speeds to counteract rotation
                uint16_t left_pwm = pwm_duty - correction;
                uint16_t right_pwm = pwm_duty + correction;
                if (left_pwm > 1000) left_pwm = 1000;
                if (right_pwm > 1000) right_pwm = 1000;

                motor_set(TIM_CHANNEL_1, MOTOR_IN2_GPIO_Port, MOTOR_IN2_Pin, true, left_pwm);
                motor_set(TIM_CHANNEL_3, MOTOR_IN4_GPIO_Port, MOTOR_IN4_Pin, true, right_pwm);
            }
        }

        HAL_Delay(MOVEMENT_UPDATE_PERIOD_MS);
    }

    // Stop motors smoothly
    stop_motors();
    HAL_Delay(100); // Settling time

    // Update robot position
    robot.x += dx[robot.direction];
    robot.y += dy[robot.direction];

    send_bluetooth_printf("S-curve forward movement complete: robot at (%d,%d)\r\n",
                         robot.x, robot.y);
}

/**
 * @brief Enhanced turn with S-curve velocity profile
 * @param turn_direction Direction: 1=right, -1=left, 2=around
 */
void turn_scurve(int turn_direction) {
    float turn_angle = 90.0f;
    const char* turn_name = "unknown";

    switch (turn_direction) {
        case 1: // Right turn
            turn_angle = 90.0f;
            turn_name = "right";
            break;
        case -1: // Left turn
            turn_angle = 90.0f;
            turn_name = "left";
            break;
        case 2: // Turn around
            turn_angle = 180.0f;
            turn_name = "around";
            break;
    }

    send_bluetooth_printf("Starting S-curve %s turn: %.1f°\r\n", turn_name, turn_angle);

    // Initialize S-curve profile for turn movement
    scurve_profile_init_turn_movement(&turn_profile, turn_angle);

    // Get starting encoder and gyro positions
    int32_t start_left = get_left_encoder_total();
    int32_t start_right = get_right_encoder_total();

    // REMOVED: unused variable 'start_heading'
    float current_heading = 0.0f;

    if (mpu9250_is_initialized()) {
        mpu9250_read_gyro();
        // Reset gyro integration for turn measurement
        current_heading = 0.0f;
    }

    // Turn movement loop
    while (!scurve_profile_is_complete(&turn_profile)) {
        scurve_profile_update(&turn_profile);
        float target_velocity = scurve_profile_get_velocity(&turn_profile);
        uint16_t pwm_duty = scurve_velocity_to_pwm(target_velocity);

        // Apply differential motor control for turning
        if (turn_direction == 1) { // Right turn
            motor_set(TIM_CHANNEL_1, MOTOR_IN2_GPIO_Port, MOTOR_IN2_Pin, true, pwm_duty); // Left forward
            motor_set(TIM_CHANNEL_3, MOTOR_IN4_GPIO_Port, MOTOR_IN4_Pin, false, pwm_duty); // Right backward
        } else if (turn_direction == -1) { // Left turn
            motor_set(TIM_CHANNEL_1, MOTOR_IN2_GPIO_Port, MOTOR_IN2_Pin, false, pwm_duty); // Left backward
            motor_set(TIM_CHANNEL_3, MOTOR_IN4_GPIO_Port, MOTOR_IN4_Pin, true, pwm_duty); // Right forward
        } else if (turn_direction == 2) { // Turn around (right)
            motor_set(TIM_CHANNEL_1, MOTOR_IN2_GPIO_Port, MOTOR_IN2_Pin, true, pwm_duty); // Left forward
            motor_set(TIM_CHANNEL_3, MOTOR_IN4_GPIO_Port, MOTOR_IN4_Pin, false, pwm_duty); // Right backward
        }

        // Check turn completion using encoders
        int32_t current_left = get_left_encoder_total();
        int32_t current_right = get_right_encoder_total();
        int32_t left_traveled = abs(current_left - start_left);
        int32_t right_traveled = abs(current_right - start_right);
        float target_encoder_counts = (turn_angle / 90.0f) * ENCODER_COUNTS_PER_TURN;
        float avg_traveled = (left_traveled + right_traveled) / 2.0f;

        if (avg_traveled >= target_encoder_counts) {
            send_bluetooth_printf("Turn completed by encoder: %.1f counts (target: %.1f)\r\n",
                                 avg_traveled, target_encoder_counts);
            break;
        }

        // Gyroscope feedback for turn accuracy
        if (mpu9250_is_initialized()) {
            mpu9250_read_gyro();
            float gyro_z_dps = mpu9250_get_gyro_z_compensated();

            // Integrate gyro reading to get heading change
            current_heading += gyro_z_dps * (MOVEMENT_UPDATE_PERIOD_MS / 1000.0f);

            // Check if we've turned enough according to gyro
            if (fabsf(current_heading) >= turn_angle * 0.95f) {
                send_bluetooth_printf("Turn completed by gyro: %.1f° (target: %.1f°)\r\n",
                                     current_heading, turn_angle);
                break;
            }
        }

        HAL_Delay(MOVEMENT_UPDATE_PERIOD_MS);
    }

    // Stop motors
    stop_motors();
    HAL_Delay(200); // Longer settling time for turns

    // Update robot direction
    if (turn_direction == 1) { // Right
        robot.direction = (robot.direction + 1) % 4;
    } else if (turn_direction == -1) { // Left
        robot.direction = (robot.direction + 3) % 4;
    } else if (turn_direction == 2) { // Around
        robot.direction = (robot.direction + 2) % 4;
    }

    send_bluetooth_printf("S-curve %s turn complete: new direction=%s\r\n",
                         turn_name, get_direction_name(robot.direction));
}

/**
 * @brief Move forward one cell using S-curve profile
 */
void move_forward_cell_scurve(void) {
    move_forward_scurve(SCURVE_CELL_DISTANCE, 1.0f);
}

/**
 * @brief Turn left using S-curve profile
 */
void turn_left_scurve(void) {
    turn_scurve(-1);
}

/**
 * @brief Turn right using S-curve profile
 */
void turn_right_scurve(void) {
    turn_scurve(1);
}

/**
 * @brief Turn around using S-curve profile
 */
void turn_around_scurve(void) {
    turn_scurve(2);
}

/**
 * @brief Enhanced movement with adaptive speed based on maze knowledge
 */
void move_forward_adaptive_scurve(float speed_multiplier) {
    // Adjust speed based on maze conditions
    float adaptive_speed = speed_multiplier;

    // Reduce speed if approaching unknown areas
    if (!maze[robot.x + dx[robot.direction]][robot.y + dy[robot.direction]].visited) {
        adaptive_speed *= 0.7f; // 70% speed for unknown areas
        send_bluetooth_message("Reducing speed for unknown area\r\n");
    }

    // Reduce speed if many walls detected recently
    int wall_count = 0;
    if (sensors.wall_front) wall_count++;
    if (sensors.wall_left) wall_count++;
    if (sensors.wall_right) wall_count++;

    if (wall_count >= 2) {
        adaptive_speed *= 0.8f; // 80% speed in tight spaces
        send_bluetooth_message("Reducing speed for tight space\r\n");
    }

    move_forward_scurve(SCURVE_CELL_DISTANCE, adaptive_speed);
}

/**
 * @brief Get S-curve movement status for debugging
 */
void send_scurve_movement_status(void) {
    send_bluetooth_message("\r\n=== S-CURVE MOVEMENT STATUS ===\r\n");

    if (forward_profile.profile_active) {
        send_bluetooth_message("Forward Profile: ACTIVE\r\n");
        scurve_profile_send_status(&forward_profile);
    } else {
        send_bluetooth_message("Forward Profile: INACTIVE\r\n");
    }

    if (turn_profile.profile_active) {
        send_bluetooth_message("Turn Profile: ACTIVE\r\n");
        scurve_profile_send_status(&turn_profile);
    } else {
        send_bluetooth_message("Turn Profile: INACTIVE\r\n");
    }

    send_bluetooth_message("==============================\r\n");
}
