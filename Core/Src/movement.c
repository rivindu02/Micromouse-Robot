/*
 * movement.c - Motor control and movement functions - FIXED VERSION
 *
 * Implements precise movement using DRV8833 motor driver and encoders
 * FIXED: Proper encoder overflow handling and safe movement functions
 * FIXED: Removed unused variables
 */

#include "micromouse.h"
#include "velocity_profile.h"
#include "movement.h"
#include <stdlib.h> // for abs() function

// FIXED: Add static variables for proper encoder overflow tracking
static uint16_t last_left_count = 32768;
static uint16_t last_right_count = 32768;
static int32_t left_total = 0;
static int32_t right_total = 0;

#define PWM_MIN_MOVE_LEFT   510
#define PWM_MIN_MOVE_RIGHT  490

/**
 * @brief Update encoder totals with proper overflow handling - NEW FUNCTION
 */
void update_encoder_totals(void)
{
    uint16_t current_left_raw = __HAL_TIM_GET_COUNTER(&htim2);
    uint16_t current_right_raw = __HAL_TIM_GET_COUNTER(&htim4);

    // Calculate differences accounting for 16-bit overflow
    int16_t left_diff = current_left_raw - last_left_count;
    int16_t right_diff = current_right_raw - last_right_count;

    // FIXED: Invert left encoder to match right encoder direction
    right_diff = -right_diff;  // Make left encoder positive for forward movement

    // Update totals
    left_total += left_diff;
    right_total += right_diff;

    // Update last counts
    last_left_count = current_left_raw;
    last_right_count = current_right_raw;
}

/**
 * @brief Get safe left encoder total - NEW FUNCTION
 */
int32_t get_left_encoder_total(void) {
    update_encoder_totals();
    return left_total;
}

/**
 * @brief Get safe right encoder total - NEW FUNCTION
 */
int32_t get_right_encoder_total(void) {
    update_encoder_totals();
    return right_total;
}

/**
 * @brief Reset encoder totals - NEW FUNCTION
 */
void reset_encoder_totals(void) {
    left_total = 0;
    right_total = 0;
    last_left_count = __HAL_TIM_GET_COUNTER(&htim2);
    last_right_count = __HAL_TIM_GET_COUNTER(&htim4);
}

/**
 * @brief Start encoder timers - FIXED VERSION
 */
void start_encoders(void) {
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL); // Right encoder
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL); // Left encoder

    // Reset encoder counts
    __HAL_TIM_SET_COUNTER(&htim4, 32768);
    __HAL_TIM_SET_COUNTER(&htim2, 32768);

    HAL_Delay(1);
    // FIXED: Initialize our safe tracking variables
    last_left_count = 32768;
    last_right_count = 32768;
    left_total = 0;
    right_total = 0;
    encoders.left_total = 0;
    encoders.right_total = 0;
}

/**
 * @brief Move forward one cell - FIXED VERSION
 */
void move_forward(void)
{
    // Use safe encoder reading
    int32_t start_left = get_left_encoder_total();
    int32_t start_right = get_right_encoder_total();

    // Check bounds before moving
    int new_x = robot.x + dx[robot.direction];
    int new_y = robot.y + dy[robot.direction];
    if (new_x < 0 || new_x >= MAZE_SIZE || new_y < 0 || new_y >= MAZE_SIZE) {
        send_bluetooth_message("Cannot move - would go out of bounds!\r\n");
        return;
    }

    motor_set_fixed(0, true, 800);  // Left motor forward
    motor_set_fixed(1, true, 800);  // Right motor forward

    // Move until target distance reached
    int32_t target_counts = ENCODER_COUNTS_PER_CELL;
    while (1) {
        int32_t current_left = get_left_encoder_total();
        int32_t current_right = get_right_encoder_total();
        int32_t left_traveled = current_left - start_left;
        int32_t right_traveled = current_right - start_right;
        int32_t avg_traveled = (left_traveled + right_traveled) / 2;

        if (avg_traveled >= target_counts) {
            break;
        }
        HAL_Delay(1);
    }

    // Stop motors
    stop_motors();

    // Update position only after successful movement
    robot.x = new_x;
    robot.y = new_y;
    HAL_Delay(100); // Settling time
}



/**
 * @brief Turn left 90 degrees - FIXED VERSION (removed unused variables)
 */
//void turn_left(void) {
//    // REMOVED: unused variable 'start_left'
//    int32_t start_right = get_right_encoder_total();
//
//
//    // Left motor reverse, right motor forward
//	motor_set_fixed(0, false, 800); // Left reverse
//	motor_set_fixed(1, true, 800);  // Right forward
//
//    int32_t target_counts = ENCODER_COUNTS_PER_TURN;
//    while (1) {
//        int32_t current_right = get_right_encoder_total();
//        int32_t right_traveled = abs(current_right - start_right);
//
//        if (right_traveled >= target_counts) {
//            break;
//        }
//        HAL_Delay(1);
//    }
//
//    stop_motors();
//    robot.direction = (robot.direction + 3) % 4; // Turn left
//    HAL_Delay(200);
//}
//
///**
// * @brief Turn right 90 degrees - FIXED VERSION (removed unused variables)
// */
//void turn_right(void) {
//    int32_t start_left = get_left_encoder_total();
//    // REMOVED: unused variable 'start_right'
//
//    // Left motor forward, right motor backward
//    motor_set_fixed(0, true, 800);  // Left forward
//    motor_set_fixed(1, false, 800); // Right reverse
//
//    int32_t target_counts = ENCODER_COUNTS_PER_TURN;
//    while (1) {
//        int32_t current_left = get_left_encoder_total();
//        int32_t left_traveled = abs(current_left - start_left);
//
//        if (left_traveled >= target_counts) {
//            break;
//        }
//        HAL_Delay(1);
//    }
//
//    stop_motors();
//    robot.direction = (robot.direction + 1) % 4; // Turn right
//    HAL_Delay(200);
//}

void turn_left(void) {
    // turn 90 degrees left using gyro PID, 1200 ms timeout for safety
    turn_in_place_gyro(+90.0f, 520, 1200);
    robot.direction = (robot.direction + 3) % 4;
}

void turn_right(void) {
    turn_in_place_gyro(-90.0f, 520, 1200);
    robot.direction = (robot.direction + 1) % 4;
}

/**
 * @brief Turn around 180 degrees
 */
void turn_around(void) {
    turn_right();
    turn_right();
}

/**
 * @brief Stop both motors
 */
void stop_motors(void)
{
    // Stop all PWM channels
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);  // Left motor PWM = 0
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);  // Left motor direction = 0
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);  // Right motor PWM = 0
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);  // Right motor direction = 0
}
void break_motors(void)
{
    // Stop all PWM channels
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1);  // Left motor PWM = 0
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 1);  // Left motor direction = 0
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 1);  // Right motor PWM = 0
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 1);  // Right motor direction = 0
    HAL_Delay(500);
    stop_motors();
}

/**
 * @brief Move forward a specific distance - FIXED VERSION
 */
void move_forward_distance(int distance_mm) {
    int32_t target_counts = (distance_mm * ENCODER_COUNTS_PER_CELL) / CELL_SIZE_MM;

    // FIXED: Use safe encoder reading
    int32_t start_left = get_left_encoder_total();
    int32_t start_right = get_right_encoder_total();
    reset_encoder_totals();
    // Set motors to move forward
//    HAL_GPIO_WritePin(MOTOR_IN1_GPIO_Port, MOTOR_IN1_Pin, GPIO_PIN_SET);
//    HAL_GPIO_WritePin(MOTOR_IN2_GPIO_Port, MOTOR_IN2_Pin, GPIO_PIN_RESET);
//    HAL_GPIO_WritePin(MOTOR_IN3_GPIO_Port, MOTOR_IN3_Pin, GPIO_PIN_SET);
//    HAL_GPIO_WritePin(MOTOR_IN4_GPIO_Port, MOTOR_IN4_Pin, GPIO_PIN_RESET);

    while (1) {
    	mpu9250_read_gyro();
    	moveStraightGyroPID();
        int32_t current_left = get_left_encoder_total();
        int32_t current_right = get_right_encoder_total();
        int32_t left_traveled = current_left - start_left;
        int32_t right_traveled = current_right - start_right;
        int32_t avg_traveled = (left_traveled + right_traveled) / 2;

        if (avg_traveled >= target_counts) {
            break;
        }
        HAL_Delay(1);
    }

    stop_motors();
}

void move_forward_adaptive_speed(float speed_multiplier) {
    // Simple implementation - modify movement timing
    int original_delay = 1;
    int new_delay = (int)(original_delay / speed_multiplier);
    if (new_delay < 1) new_delay = 1;

    // Use existing move_forward but modify timing
    move_forward();
}

bool is_speed_run_ready(void) {
    return (robot.center_reached && robot.returned_to_start);
}

// helper to set speed (0–1000 = 0–100%)
void motor_set(uint16_t ch_pwm, GPIO_TypeDef *dirPort, uint16_t dirPin, bool forward, uint16_t duty) {
    // Validate inputs
    if (duty > 1000) duty = 1000;

    // Set PWM duty cycle
    __HAL_TIM_SET_COMPARE(&htim3, ch_pwm, duty);

    // FIXED: Proper DRV8833 control
    // For DRV8833: PWM on INx, Direction control on INy
    // Forward: INx=PWM, INy=LOW
    // Backward: INx=PWM, INy=HIGH
    if (forward) {
        HAL_GPIO_WritePin(dirPort, dirPin, GPIO_PIN_RESET);  // Direction LOW for forward
    } else {
        HAL_GPIO_WritePin(dirPort, dirPin, GPIO_PIN_SET);    // Direction HIGH for backward
    }
}
// Fixed motor_set function for DRV8833
void motor_set_fixed(uint8_t motor, bool forward, uint16_t duty) {
    if (motor == 0) { // Left motor
        if (forward) {
			// Left reverse: IN1=LOW, IN2=PWM
        	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, duty); // PA7 = PWM
        	HAL_GPIO_WritePin(MOTOR_IN1_GPIO_Port, MOTOR_IN1_Pin, GPIO_PIN_RESET); // PA6 = LOW

        } else {
        	// Left forward: IN1=PWM, IN2=LOW
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, duty); // PA6 = PWM
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0); // PA7 = LOW

        }
    } else { // Right motor
    	bool actual_forward = !forward;  // invert direction
        if (actual_forward) {
            // Right reverse: IN3=LOW, IN4=PWM
        	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, duty); // PB1 = PWM
        	HAL_GPIO_WritePin(MOTOR_IN3_GPIO_Port, MOTOR_IN3_Pin, GPIO_PIN_RESET); // PB0 = LOW
        } else {
            // Right forward: IN3=PWM, IN4=LOW
        	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0); // PB1 = LOW
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, duty); // PB0 = PWM

        }
    }
}

// Add this function to test motors individually
void test_motors_individual(void) {
    send_bluetooth_message("Testing motors individually...\r\n");

    // Test left motor forward
    send_bluetooth_message("Left motor forward...\r\n");
    motor_set_fixed(0, true, 600);
    HAL_Delay(2000);
    stop_motors();
    HAL_Delay(500);

    // Test left motor reverse
    send_bluetooth_message("Left motor reverse...\r\n");
    motor_set_fixed(0, false, 600);
    HAL_Delay(2000);
    stop_motors();
    HAL_Delay(500);

    // Test right motor forward
    send_bluetooth_message("Right motor forward...\r\n");
    motor_set_fixed(1, true, 600);
    HAL_Delay(2000);
    stop_motors();
    HAL_Delay(500);

    // Test right motor reverse
    send_bluetooth_message("Right motor reverse...\r\n");
    motor_set_fixed(1, false, 600);
    HAL_Delay(2000);
    stop_motors();
    HAL_Delay(1000);

    send_bluetooth_message("Motor test complete!\r\n");
}



/**
 * @brief Get encoder status for debugging - NEW FUNCTION
 */
void send_encoder_status(void) {
    update_encoder_totals();
    send_bluetooth_printf("Encoders - Left:%ld Right:%ld Raw_L:%d Raw_R:%d\r\n",
                         left_total, right_total,
                         __HAL_TIM_GET_COUNTER(&htim2), __HAL_TIM_GET_COUNTER(&htim4));
}

/**
 * @brief Move forward with S-curve velocity profile
 */
void move_forward_with_profile(float distance_mm, float max_speed) {
    VelocityProfile profile;
    velocity_profile_init(&profile, distance_mm, max_speed);

    int32_t start_left = get_left_encoder_total();
    int32_t start_right = get_right_encoder_total();

    // Calculate target encoder counts
    int32_t target_counts = (distance_mm * ENCODER_COUNTS_PER_CELL) / CELL_SIZE_MM;

    while (!velocity_profile_is_complete(&profile)) {
        velocity_profile_update(&profile);
        float target_vel = velocity_profile_get_target_velocity(&profile);

        // Convert mm/s to PWM duty (simple linear conversion)
        uint16_t duty = (uint16_t)(target_vel * 0.8f); // Scale factor to be tuned
        if (duty > 1000) duty = 1000; // Cap at max PWM
        if (duty < 50) duty = 50; // Minimum PWM for movement

        // Check distance traveled
        int32_t current_left = get_left_encoder_total();
        int32_t current_right = get_right_encoder_total();
        int32_t avg_traveled = ((current_left - start_left) + (current_right - start_right)) / 2;

        if (avg_traveled >= target_counts) {
            break; // Reached target distance
        }

        // Apply to motors
        motor_set(TIM_CHANNEL_1, MOTOR_IN2_GPIO_Port, MOTOR_IN2_Pin, true, duty);
        motor_set(TIM_CHANNEL_3, MOTOR_IN4_GPIO_Port, MOTOR_IN4_Pin, true, duty);

        HAL_Delay(5); // 200Hz update rate
    }

    stop_motors();
}

// PID parameters for encoders
//float Kp_e =0.02; // Proportional term
//float Ki_e = 0; // Integral term
//float Kd_e = 0.1; // Derivative term
//float error_e = 0;
//float previousError_e = 0;
//float integral_e = 0;
//float derivative_e = 0;
///*
// * Encoder PID
// * */
//void moveStraightPID(void) {
//	left_total=get_left_encoder_total();
//	left_total=get_left_encoder_total();
//	error_e = left_total - right_total;
//
//	integral_e += error_e;
//	derivative_e = error_e - previousError_e;
//
//	float correction = (Kp_e * error_e) + (Ki_e * integral_e) + (Kd_e * derivative_e);
//
//	int motor1Speed = 500 + correction;
//	int motor2Speed = 510 - correction;
//
//	if (motor1Speed>1000){
//	  motor1Speed= 1000;
//	};
//	if (motor2Speed>1000){
//	  motor2Speed= 1000;
//	};
//	if (motor1Speed<0){
//	  motor1Speed= 0;
//	};
//	if (motor2Speed<0){
//	  motor2Speed= 0;
//	};
//
//	motor_set_fixed(0, true, motor2Speed);//Left
//
//	motor_set_fixed(1, true, motor1Speed);//Right
//
//
//	previousError_e = error_e;
//}

/* Tunable gains (start conservative, tune per-bot) */
static float Kp_e = 0.018f;//0.02f;   // Proportional on rate error (PWM per count/s)
static float Ki_e = 0.040f;    // Integral on rate error
static float Kd_e = 0.000f;    // Derivative on rate error

/* Internal state */
static int32_t prev_left_counts = 0;
static int32_t prev_right_counts = 0;
static float prev_error_rate = 0.0f;
static float integral_e = 0.0f;
static float deriv_filt = 0.0f;
static uint32_t pid_last_ms = 0;

/* Filters / clamps */
static const float DERIV_FILTER_ALPHA_E = 0.85f;   // 0..1, larger -> more smoothing
static const float INTEGRAL_LIMIT_E = 1000.0f;    // clamp integral term (tune)
static const int PWM_MIN = 0;
static const int PWM_MAX = 700;
static const int PWM_MIN_MOVE = 40;                // optional min to overcome stiction

/* Helper clamp */
static inline float clampf_local(float v, float lo, float hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
}

/* Call once immediately before starting a straight movement */
void moveStraightPID_Reset(void) {
    prev_left_counts = get_left_encoder_total();
    prev_right_counts = get_right_encoder_total();
    prev_error_rate = 0.0f;
    integral_e = 0.0f;
    deriv_filt = 0.0f;
    pid_last_ms = HAL_GetTick();
}

/*
 * Call this repeatedly while moving forward.
 * base_pwm: nominal forward PWM (same idea as your previous base 500).
 * left_forward/right_forward: booleans for motor_set_fixed() direction
 *
 * If your robot uses fixed forward direction everywhere, pass true/true.
 */
void moveStraightPID(int base_pwm, bool left_forward, bool right_forward) {
    uint32_t now = HAL_GetTick();
    float dt = (now - pid_last_ms) / 1000.0f;
    if (dt <= 0.0f) dt = 0.001f;   // safety
    pid_last_ms = now;

    /* Read encoder totals */
    int32_t left_total = get_left_encoder_total();
    int32_t right_total = get_right_encoder_total();

    /* Delta counts in this interval */
    int32_t dleft = left_total - prev_left_counts;
    int32_t dright = right_total - prev_right_counts;

    /* Save for next iteration */
    prev_left_counts  = left_total;
    prev_right_counts = right_total;

    /* Convert to rate (counts per second). Using counts/s is robust and avoids
       huge integrator values on long runs. If you prefer physical units, convert
       counts -> mm using your ENCODER_COUNTS_PER_MM constant first. */
    float left_rate  = (float)dleft / dt;   // counts / s
    float right_rate = (float)dright / dt;  // counts / s

    /* Error is rate difference: left - right (positive => left faster) */
    float error_rate = left_rate - right_rate;

    /* Integral on rate error */
    integral_e += error_rate * dt;
    integral_e = clampf_local(integral_e, -INTEGRAL_LIMIT_E, INTEGRAL_LIMIT_E);

    /* Derivative (on error rate) with low-pass filtering */
    float deriv_raw = (error_rate - prev_error_rate) / dt;   // d(error_rate)/dt
    deriv_filt = DERIV_FILTER_ALPHA_E * deriv_filt + (1.0f - DERIV_FILTER_ALPHA_E) * deriv_raw;

    /* PID output: signed PWM correction (positive -> speed up right wheel OR slow left wheel) */
    float correction = (Kp_e * error_rate) + (Ki_e * integral_e) + (Kd_e * deriv_filt);

    /* Map correction to motor PWMs: keep same sign mapping as your original code:
       original: motor1 = base + correction; motor2 = base - correction
       (motor1 = right wheel, motor2 = left wheel). */
    int pwm_right = (int)roundf((float)base_pwm + correction);
    int pwm_left  = (int)roundf((float)base_pwm - correction);

    /* Clamp */
    pwm_left  = (int)clampf_local((float)pwm_left, (float)PWM_MIN, (float)PWM_MAX);
    pwm_right = (int)clampf_local((float)pwm_right, (float)PWM_MIN, (float)PWM_MAX);

    /* Optional: enforce minimum non-zero movement so wheel actually turns */
    if (pwm_left > 0 && pwm_left < PWM_MIN_MOVE) pwm_left = PWM_MIN_MOVE;
    if (pwm_right > 0 && pwm_right < PWM_MIN_MOVE) pwm_right = PWM_MIN_MOVE;

    /* Apply to motors (Left = index 0 in your mapping, Right = index 1) */
    motor_set_fixed(0, left_forward,  (uint16_t)pwm_left);   // Left
    motor_set_fixed(1, right_forward, (uint16_t)pwm_right);  // Right

    /* remember for next loop */
    prev_error_rate = error_rate;
}




// PID parameters for gyro
float Kp_g = 0.8657f;//6.4; // Proportional term
float Ki_g = 5.3769f;//0; // Integral term
float Kd_g = 0.0f;//2; // Derivative term
//float error_g = 0;
//float previousError_g = 0;
//float integral_g = 0;
//float derivative_g = 0;
/* ----- Internal PID state (file-static) ----- */
static float pid_error_prev = 0.0f;      // previous error (for derivative)
static float pid_integral = 0.0f;        // integrated error (with dt)
static float pid_deriv_filt = 0.0f;      // filtered derivative


/* Derivative low-pass filter: alpha in [0..1], bigger => more smoothing */
static const float DERIV_FILTER_ALPHA = 0.85f;

/* Integral clamp (anti-windup) */
static const float INTEGRAL_LIMIT = 2000.0f; // tune as needed (units: deg/s * s)





/* Call this once immediately before starting a straight movement */
void moveStraightGyroPID_Reset(void) {
    pid_error_prev = 0.0f;
    pid_integral = 0.0f;
    pid_deriv_filt = 0.0f;
    pid_last_ms = HAL_GetTick();
}

void moveStraightGyroPID(void) {
    uint32_t now = HAL_GetTick();
    float dt = (now - pid_last_ms) / 1000.0f;
    if (dt <= 0.0f) dt = 0.001f; // safety small dt if HAL tick didn't advance
    pid_last_ms = now;

    /* READ: your gyro rate (deg/s). Keep original sign convention:
       original code used error_g = mpu9250_get_gyro_z_compensated();
       and motor1 = base - correction; motor2 = base + correction;
       so we preserve that mapping for compatibility. */
    float error = mpu9250_get_gyro_z_compensated();

    /* Integral (with dt) + anti-windup clamp */
    pid_integral += error * dt;
    pid_integral = clampf_local(pid_integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);

    /* Derivative (on error) and low-pass filter */
    float deriv_raw = (error - pid_error_prev) / dt;    // d(error)/dt
    pid_deriv_filt = DERIV_FILTER_ALPHA * pid_deriv_filt + (1.0f - DERIV_FILTER_ALPHA) * deriv_raw;

    /* PID output (correction) */
    float correction = (Kp_g * error) + (Ki_g * pid_integral) + (Kd_g * pid_deriv_filt);

    /* Base PWM for forward motion (adjust to your nominal cruising PWM) */
    const int base_pwm = 700;

    int motor1Speed = (int)roundf((float)base_pwm - correction); // right wheel in your mapping
    int motor2Speed = (int)roundf((float)base_pwm + correction); // left wheel

    /* Clamp PWM outputs (and provide a safe top, not full 1000 if you prefer) */
    if (motor1Speed > 800) motor1Speed = 800;
    if (motor2Speed > 800) motor2Speed = 800;
    if (motor1Speed < 0) motor1Speed = 0;
    if (motor2Speed < 0) motor2Speed = 0;



    /* Set motors: adjust direction flags if your wiring uses opposite logic */
    motor_set_fixed(0, true, motor2Speed); // Left
    motor_set_fixed(1, true, motor1Speed); // Right

    /* store previous error for next derivative computation */
    pid_error_prev = error;
}




// Measured plant gain (from your CSV): ~1.3828 deg/s per ΔPWM
#define GYRO_K_DPS_PER_DPWM   1.3828f

// Rate PID gains (start conservative; move up if stable)
float Kp_t = 0.866955;   //(PWM per deg/s)
float Ki_t = 5.384815;   //(PWM per deg/s per s)// PWM per deg  (with integral += err*dt)
float Kd_t =0; // PWM per (deg/s^2)

// Desired-rate shaping
static float OMEGA_MAX_DPS   = 300.0f;   // cap desired |rate|
static float ALPHA_MAX_DPS2  = 3000.0f;  // braking accel (deg/s^2)

// Finish criteria
static float ANGLE_TOL_DEG   = 1.0f;
static float RATE_TOL_DPS    = 20.0f;
static uint32_t SETTLE_MS    = 60;

// Small command deadband near stop (don’t twitch)
static float OMEGA_CMD_DEADBAND = 12.0f;   // deg/s

// Derivative filtering + integral clamp
static const float DERIV_ALPHA  = 0.90f;   // 0..1
static const float INTEGRAL_CLAMP = 1000.0f;

// --- PID state ---
static float pid_int = 0.0f, pid_prev_err = 0.0f, pid_deriv_f = 0.0f;

static inline float signf(float x) { return (x >= 0.0f) ? 1.0f : -1.0f; }

static void gyro_turn_reset(void) {
    pid_int = 0.0f;
    pid_prev_err = 0.0f;
    pid_deriv_f = 0.0f;
    pid_last_ms = HAL_GetTick();
}

static float gyro_rate_pid_step(float sp_dps, float meas_dps, float *p_dt) {
    uint32_t now = HAL_GetTick();
    float dt = (now - pid_last_ms) / 1000.0f;
    if (dt <= 0.0f) dt = 0.002f;
    pid_last_ms = now;
    if (p_dt) *p_dt = dt;

    float err = sp_dps - meas_dps;

    // integral (anti-windup)
    pid_int += err * dt;
    if (pid_int >  INTEGRAL_CLAMP) pid_int =  INTEGRAL_CLAMP;
    if (pid_int < -INTEGRAL_CLAMP) pid_int = -INTEGRAL_CLAMP;

    // derivative (filtered)
    float d_raw = (err - pid_prev_err) / dt;
    pid_deriv_f = DERIV_ALPHA * pid_deriv_f + (1.0f - DERIV_ALPHA) * d_raw;
    pid_prev_err = err;

    // PID → ΔPWM (right - left)
    return Kp_g*err + Ki_g*pid_int + Kd_g*pid_deriv_f;
}

/**
 * In-place turn by angle (deg). +angle = CCW/left, -angle = CW/right.
 * base_pwm = 80..250 is typical. timeout_ms is safety.
 */
void turn_in_place_gyro(float angle_deg, int base_pwm, uint32_t timeout_ms)
{
    if (base_pwm < 60)  base_pwm = 60;
    if (base_pwm > 400) base_pwm = 400;

    gyro_turn_reset();

    float yaw = 0.0f;                  // integrated heading (deg)
    const float target = angle_deg;    // signed target
    uint32_t t0 = HAL_GetTick();
    uint32_t settle_start = 0;

    // last timestamp for yaw integration
    uint32_t last_ms = HAL_GetTick();

    while (1) {
        // --- timing ---
        uint32_t now = HAL_GetTick();
        float dt = (now - last_ms) / 1000.0f;
        if (dt <= 0.0f) dt = 0.001f;
        last_ms = now;
        mpu9250_read_gyro();
        // --- sensors ---
        float gz = mpu9250_get_gyro_z_compensated();  // deg/s

        // --- integrate heading (keep sign!) ---
        yaw += gz * dt;

        // signed angle error (THIS FIXES THE MAIN BUG)
        float ang_err = target - yaw;

        // desired rate with braking law (changes sign if you overshoot)
        float omega_brake = sqrtf(fmaxf(0.0f, 2.0f * ALPHA_MAX_DPS2 * fabsf(ang_err)));
        float omega_des = clampf_local(omega_brake, 0.0f, OMEGA_MAX_DPS) * signf(ang_err);

        // small deadband on command (avoid micro twitch)
        if (fabsf(omega_des) < OMEGA_CMD_DEADBAND) omega_des = 0.0f;

        // --- inner rate loop ---
        float pid_dt = 0.0f;
        float dPWM_pid = gyro_rate_pid_step(omega_des, gz, &pid_dt);  // ΔPWM from PID
        float dPWM_ff  = (fabsf(omega_des) > 0.0f) ? (omega_des / GYRO_K_DPS_PER_DPWM) : 0.0f;
        float dPWM     = dPWM_ff + dPWM_pid;   // total ΔPWM (right - left), signed

        // split ΔPWM around base so both sides get torque
        float right_mag = (float)base_pwm + 0.5f * fabsf(dPWM);
        float left_mag  = (float)base_pwm + 0.5f * fabsf(dPWM);

        // decide directions from CURRENT command sign (not the initial turn dir)
        bool left_forward, right_forward;
        if (dPWM >= 0.0f) {
            // turn left: left backward, right forward
            left_forward  = false;
            right_forward = true;
        } else {
            // turn right: left forward, right backward
            left_forward  = true;
            right_forward = false;
        }

        // if command very small AND rate small, cut power to stop cleanly
        if (fabsf(ang_err) <= ANGLE_TOL_DEG && fabsf(gz) <= RATE_TOL_DPS) {
            motor_set_fixed(0, true, 0);
            motor_set_fixed(1, true, 0);
            if (settle_start == 0) settle_start = now;
            if ((now - settle_start) >= SETTLE_MS) break;
        } else {
            settle_start = 0;

            // Apply PWM (no fake “min move” offsets here—let control truly go to 0 near stop)
            int pwmL = (int)roundf(left_mag);
            int pwmR = (int)roundf(right_mag);
            if (pwmL < 0) pwmL = 0; if (pwmL > 1000) pwmL = 1000;
            if (pwmR < 0) pwmR = 0; if (pwmR > 1000) pwmR = 1000;

            motor_set_fixed(0, left_forward,  (uint16_t)pwmL);
            motor_set_fixed(1, right_forward, (uint16_t)pwmR);
        }

        if ((now - t0) > timeout_ms) break;

        HAL_Delay(2); // ~500 Hz outer loop
    }

    stop_motors();
    HAL_Delay(60);
}


/**
 * @brief Simple smooth movement for one cell
 */
void move_forward_smooth(float distance_mm) {
    move_forward_with_profile(distance_mm, 600.0f); // 600 mm/s max speed
}

// Add to movement.c
void debug_encoder_setup(void) {
    send_bluetooth_message("=== ENCODER DEBUG ===\r\n");

    // Check if timer clocks are enabled
    if (RCC->APB1ENR & RCC_APB1ENR_TIM2EN) {
        send_bluetooth_message("TIM2 clock: ENABLED\r\n");
    } else {
        send_bluetooth_message("TIM2 clock: DISABLED\r\n");
    }

    if (RCC->APB1ENR & RCC_APB1ENR_TIM4EN) {
        send_bluetooth_message("TIM4 clock: ENABLED\r\n");
    } else {
        send_bluetooth_message("TIM4 clock: DISABLED\r\n");
    }

    // Check timer register values
    send_bluetooth_printf("TIM2 registers - CR1:0x%08X SMCR:0x%08X\r\n",
                         TIM2->CR1, TIM2->SMCR);
    send_bluetooth_printf("TIM4 registers - CR1:0x%08X SMCR:0x%08X\r\n",
                         TIM4->CR1, TIM4->SMCR);
}

void test_encoder_manual(void) {
    send_bluetooth_message("Testing manual encoder increment...\r\n");

    // Manual test - set counter values
    TIM2->CNT = 100;
    TIM4->CNT = 200;
    HAL_Delay(10);

    send_bluetooth_printf("TIM2 CNT: %d, TIM4 CNT: %d\r\n",
                         TIM2->CNT, TIM4->CNT);
}

// Add to main.c after your current debug code
void test_encoder_rotation(void) {
    send_bluetooth_message("\r\n🔄 ENCODER ROTATION TEST\r\n");
    send_bluetooth_message("Manually rotate each wheel and watch the counts:\r\n");

    // Reset counters to known values
    TIM2->CNT = 1000; // Left encoder
    TIM4->CNT = 2000; // Right encoder

    send_bluetooth_message("Initial values set - TIM2: 1000, TIM4: 2000\r\n");
    send_bluetooth_message("Now manually rotate the wheels...\r\n");

    // Monitor for 10 seconds
    for(int i = 0; i < 20; i++) {
        uint16_t left_raw = TIM2->CNT;
        uint16_t right_raw = TIM4->CNT;
        int32_t left_total = get_left_encoder_total();
        int32_t right_total = get_right_encoder_total();

        send_bluetooth_printf("T+%ds: Raw L:%d R:%d | Total L:%ld R:%ld\r\n",
                             i/2, left_raw, right_raw, left_total, right_total);
        HAL_Delay(500);
    }

    send_bluetooth_message("Rotation test complete!\r\n");
}
