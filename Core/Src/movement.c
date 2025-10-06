/*
 * movement.c - Motor control and movement functions - FIXED VERSION
 *
 * Implements precise movement using DRV8833 motor driver and encoders
 * FIXED: Proper encoder overflow handling and safe movement functions
 * FIXED: Removed unused variables
 */

#include "micromouse.h"
#include "movement.h"
#include <stdlib.h> // for abs() function

// FIXED: Add static variables for proper encoder overflow tracking
static uint16_t last_left_count = 32768;
static uint16_t last_right_count = 32768;
static int32_t left_total = 0;
static int32_t right_total = 0;


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


extern bool align_front_to_wall(int base_pwm, uint32_t timeout_ms);

#define S_CURVE_LUT_LEN 51
#define S_CURVE_LUT_STEP_COUNTS 10
#define S_CURVE_LUT_SPAN_COUNTS 500
#define S_CURVE_LUT_BASE_PWM 700
#define S_CURVE_LUT_MIN_PWM 350
extern void dwt_delay_us(uint32_t us);

static const int16_t S_CURVE_LUT[S_CURVE_LUT_LEN] = {
    700, 700, 700, 699, 698, 697, 695, 692, 689, 685, 680, 674, 667, 660, 652, 643, 633, 623, 612, 601, 589, 577, 564, 551, 538, 525, 512, 499, 486, 473, 461, 449, 438, 427, 417, 407, 398, 390, 383, 376, 370, 365, 361, 358, 355, 353, 352, 351, 350, 350, 350
};

void turn_left(void) {

    // turn 90 degrees left using gyro PID, 1200 ms timeout for safety
	if (sensors.wall_front){
		align_front_to_wall(600,4000);
	}else{

		move_forward_distance(957,957);

		//dwt_delay_us(100);
	}


    gyro_turn_reset();
    turn_in_place_gyro(+90.0f, 520, 1200);

    //move_forward_distance(1549,1537);//////////////
    robot.direction = (robot.direction + 3) % 4;

}

void turn_right(void) {
	if (sensors.wall_front){
		align_front_to_wall(600,3000);
	}else{
		move_forward_distance(957,957);
	}
	gyro_turn_reset();
    turn_in_place_gyro(-90.0f, 520, 1200);
    //move_forward_distance(1530,1562);
    robot.direction = (robot.direction + 1) % 4;
}

/**
 * @brief Turn around 180 degrees
 */
void turn_around(void) {
	if (sensors.wall_front){
		align_front_to_wall(600,3000);
	}else{
		move_forward_distance(957,957);
	}
	turn_in_place_gyro(-90.0f, 520, 1200);
	turn_in_place_gyro(-90.0f, 520, 1200);

	//move_forward_distance(1330,1352);/////////////////

	robot.direction = (robot.direction + 1) % 4;
	robot.direction = (robot.direction + 1) % 4;


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
    // Apply active braking by setting both inputs HIGH for each motor
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1000);  // Left IN1 = HIGH
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 1000);  // Left IN2 = HIGH
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 1000);  // Right IN3 = HIGH
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 1000);  // Right IN4 = HIGH

    HAL_Delay(200);  // Hold brake briefly
    stop_motors();   // Then coast
}


/**
 * @brief Move forward a specific distance - FIXED VERSION
 */
void move_forward_distance(int Left_target_counts,int Right_target_counts) {

    //  Use safe encoder reading
	reset_encoder_totals();
    int32_t start_left = get_left_encoder_total();
    int32_t start_right = get_right_encoder_total();
    moveStraightGyroPID_Reset();


    while (1) {
    	mpu9250_read_gyro();
    	moveStraightGyroPID();


        int32_t current_left = get_left_encoder_total();
        int32_t current_right = get_right_encoder_total();
        int32_t left_traveled =  start_left-current_left;
        int32_t right_traveled = start_right-current_right;
        //int32_t avg_traveled = (left_traveled + right_traveled) / 2;

        //send_bluetooth_printf("L:%ld R:%ld\r\n",current_left,current_right);

        if (left_traveled>=Left_target_counts || right_traveled>=Right_target_counts) {
            break;
        }
        HAL_Delay(1);
    }

    break_motors();		// use a S-curve to apply break/////////////////////
}

/**
 * @brief Move forward a specific distance - FIXED VERSION
 */
void move_forward_WF_distance(int Left_target_counts,int Right_target_counts) {

    // FIXED: Use safe encoder reading
	reset_encoder_totals();
    int32_t start_left = get_left_encoder_total();
    int32_t start_right = get_right_encoder_total();
    // 0 = auto (both → center; else follow visible side), 1 = left, 2 = right
    int mode = 0;               // WF_AUTO
    int base_pwm = 650;         // UPDATEDDDDDDDDDDDDDDDDDD

    // bootstrap targets & reset integrators
    wall_follow_reset_int(mode, base_pwm);

    fusion_align_entry(600, 3000);


    fusion_reset();
    fusion_set_heading_ref_to_current();  // lock the present heading

    while (1) {
    	fusion_step(/*base_pwm=*/650);  // 0 → uses WF_BASE_PWM; or pass an explicit base

    	//wall_follow_step();     // computes e, PID, sets motor PWMs
		//HAL_Delay(200);           // keep a steady loop
		dwt_delay_us(50);


        int32_t current_left = get_left_encoder_total();
        int32_t current_right = get_right_encoder_total();
        int32_t left_traveled =  start_left-current_left;
        int32_t right_traveled = start_right-current_right;
        //int32_t avg_traveled = (left_traveled + right_traveled) / 2;

        //send_bluetooth_printf("L:%ld R:%ld\r\n",current_left,current_right);

        if (left_traveled>=Left_target_counts || right_traveled>=Right_target_counts) {
            break;
        }
        //HAL_Delay(1);
    }
    break_motors();		// use a S-curve to apply break/////////////////////
    //move_forward_distance(Left_target_counts/2,Right_target_counts/2);


}




// helper to set speed (0–1000 = 0–100%)
// Fixed motor_set function for DRV8833
void motor_set(uint8_t motor, bool forward, uint16_t duty) {
    if (motor == 0) { // Left motor
        if (forward) {
			// Left reverse: IN1=LOW, IN2=PWM
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, duty); // PA6 = PWM
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0); // PA7 = LOW

        } else {
        	// Left forward: IN1=PWM, IN2=LOW

        	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, duty); // PA7 = PWM
        	HAL_GPIO_WritePin(MOTOR_IN1_GPIO_Port, MOTOR_IN1_Pin, GPIO_PIN_RESET); // PA6 = LOW

        }
    } else { // Right motor
    	bool actual_forward = forward;  // invert direction
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
    motor_set(0, true, 600);
    HAL_Delay(2000);
    stop_motors();
    HAL_Delay(500);

    // Test left motor reverse
    send_bluetooth_message("Left motor reverse...\r\n");
    motor_set(0, false, 600);
    HAL_Delay(2000);
    stop_motors();
    HAL_Delay(500);

    // Test right motor forward
    send_bluetooth_message("Right motor forward...\r\n");
    motor_set(1, true, 600);
    HAL_Delay(2000);
    stop_motors();
    HAL_Delay(500);

    // Test right motor reverse
    send_bluetooth_message("Right motor reverse...\r\n");
    motor_set(1, false, 600);
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
static const int PWM_MAX = 1000;
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
 * left_forward/right_forward: booleans for motor_set() direction
 *
 * If your robot uses fixed forward direction everywhere, pass true/true.
 */
void moveStraightPID(int base_pwm, bool left_forward, bool right_forward) {
    uint32_t now = HAL_GetTick();
    float dt = (now - pid_last_ms) / 1000.0f;
    if (dt <= 0.0f) dt = 0.001f;   // safety
    pid_last_ms = now;

    /* Read encoder totals */
    int32_t left_total_e = get_left_encoder_total();
    int32_t right_total_e = get_right_encoder_total();

    /* Delta counts in this interval */
    int32_t dleft = left_total_e - prev_left_counts;
    int32_t dright = right_total_e - prev_right_counts;

    /* Save for next iteration */
    prev_left_counts  = left_total_e;
    prev_right_counts = right_total_e;

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
    motor_set(0, left_forward,  (uint16_t)pwm_left);   // Left
    motor_set(1, right_forward, (uint16_t)pwm_right);  // Right

    /* remember for next loop */
    prev_error_rate = error_rate;
}




// PID parameters for gyro
float Kp_g = 0.437213f;
float Ki_g = 1.51547f;
float Kd_g = 0.0189204f;

/* ----- Internal PID state (file-static) ----- */
static float pid_error_prev = 0.0f;      // previous error (for derivative)
static float pid_integral = 0.0f;        // integrated error (with dt)
static float pid_deriv_filt = 0.0f;      // filtered derivative


/* Derivative low-pass filter: alpha in [0..1], bigger => more smoothing */
static const float DERIV_FILTER_ALPHA = 0.936768f;    // computed from dt_med=0.058s, Td=0.03915s, N=10.0
//static const float DERIV_FILTER_ALPHA =0.85f; //for moveStraightGyroPID2

/* Integral clamp (anti-windup) */
static const float INTEGRAL_LIMIT = 5.0f; // tune as needed (units: deg/s * s)
////////////////////////////////////////
// ---- Tuning knobs (start here) ----
static const float ERR_DEADBAND   = 0.25f;   // deg/s; ignore tiny gyro bias
static const float LEAK_RATE      = 0.002f;  // 1/s; integral will decay slowly
static const float KAW            = 0.5f;    // anti-windup back-calc gain (0.2..1.0)
static const float SLIP_THRESH    = 6.0f;    // deg/s; treat as transient/slip
static const float COOLDOWN_TIME  = 0.8f;    // s; pause integration after slip

static float learn_cooldown = 0.0f;          // pause timer for integrator

// ---- Base PWM and limits ----
static const int   base_pwm  = 600;   // your cruise PWM

////////////////////////////////////////
/* Call this once immediately before starting a straight movement */
static inline int clamp_int(int v, int lo, int hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

void moveStraightGyroPID_Reset(void) {
    pid_error_prev = 0.0f;
    pid_integral = 0.0f;
    pid_deriv_filt = 0.0f;
    learn_cooldown = 0.0f;
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
    const int base_pwm = 570;

    int motor1Speed = (int)roundf((float)base_pwm - correction); // right wheel in your mapping
    int motor2Speed = (int)roundf((float)base_pwm + correction); // left wheel

    /* Clamp PWM outputs (and provide a safe top, not full 1000 if you prefer) */
    if (motor1Speed > 1200) motor1Speed = 1200;
    if (motor2Speed > 1200) motor2Speed = 1200;
    if (motor1Speed < 0) motor1Speed = 0;
    if (motor2Speed < 0) motor2Speed = 0;



    /* Set motors: adjust direction flags if your wiring uses opposite logic */
    motor_set(0, true, motor2Speed); // Left
    motor_set(1, true, motor1Speed); // Right

    /* store previous error for next derivative computation */
    pid_error_prev = error;
}






// Measured plant gain (from your CSV): ~1.3828 deg/s per ΔPWM
#define GYRO_K_DPS_PER_DPWM   1.3828f



// Rate PID gains (start conservative; move up if stable)
float Kp_t =5.338354;// 0.866955;   //(PWM per deg/s)
float Ki_t =56.800842;//5.384815;   //(PWM per deg/s per s)// PWM per deg  (with integral += err*dt)
float Kd_t =0.001880;//0; // PWM per (deg/s^2)

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
static const float INTEGRAL_CLAMP = 10.0f;

// --- PID state ---
static float pid_int = 0.0f, pid_prev_err = 0.0f, pid_deriv_f = 0.0f;

static inline float signf(float x) { return (x >= 0.0f) ? 1.0f : -1.0f; }

void gyro_turn_reset(void) {
    pid_int = 0.0f;
    pid_prev_err = 0.0f;
    pid_deriv_f = 0.0f;
    pid_last_ms = HAL_GetTick();
}

float gyro_rate_pid_step(float sp_dps, float meas_dps, float *p_dt) {
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
            motor_set(0, true, 0);
            motor_set(1, true, 0);
            if (settle_start == 0) settle_start = now;
            if ((now - settle_start) >= SETTLE_MS) break;
        } else {
            settle_start = 0;

            // Apply PWM (no fake “min move” offsets here—let control truly go to 0 near stop)
            int pwmL = (int)roundf(left_mag);
            int pwmR = (int)roundf(right_mag);
            if (pwmL < 0) pwmL = 0;
            if (pwmL > 1000) pwmL = 1000;
            if (pwmR < 0) pwmR = 0;
            if (pwmR > 1000) pwmR = 1000;

            motor_set(0, left_forward,  (uint16_t)pwmL);
            motor_set(1, right_forward, (uint16_t)pwmR);
        }

        if ((now - t0) > timeout_ms) break;

        HAL_Delay(2); // ~500 Hz outer loop
    }

    break_motors();
    HAL_Delay(60);
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
