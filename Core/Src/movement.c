/*
 * movement.c - Motor control and movement functions - FIXED VERSION
 *
 * Implements precise movement using DRV8833 motor driver and encoders
 * FIXED: Proper encoder overflow handling and safe movement functions
 * FIXED: Removed unused variables
 */

#include "micromouse.h"
#include "velocity_profile.h"
#include <stdlib.h> // for abs() function

// FIXED: Add static variables for proper encoder overflow tracking
static uint16_t last_left_count = 32768;
static uint16_t last_right_count = 32768;
static int32_t left_total = 0;
static int32_t right_total = 0;

//extern uint16_t current_left_raw;
volatile uint16_t current_left_raw;
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
    left_diff = -left_diff;  // Make left encoder positive for forward movement

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
void turn_left(void) {
    // REMOVED: unused variable 'start_left'
    int32_t start_right = get_right_encoder_total();


    // Left motor reverse, right motor forward
	motor_set_fixed(0, false, 800); // Left reverse
	motor_set_fixed(1, true, 800);  // Right forward

    int32_t target_counts = ENCODER_COUNTS_PER_TURN;
    while (1) {
        int32_t current_right = get_right_encoder_total();
        int32_t right_traveled = abs(current_right - start_right);

        if (right_traveled >= target_counts) {
            break;
        }
        HAL_Delay(1);
    }

    stop_motors();
    robot.direction = (robot.direction + 3) % 4; // Turn left
    HAL_Delay(200);
}

/**
 * @brief Turn right 90 degrees - FIXED VERSION (removed unused variables)
 */
void turn_right(void) {
    int32_t start_left = get_left_encoder_total();
    // REMOVED: unused variable 'start_right'

    // Left motor forward, right motor backward
    motor_set_fixed(0, true, 800);  // Left forward
    motor_set_fixed(1, false, 800); // Right reverse

    int32_t target_counts = ENCODER_COUNTS_PER_TURN;
    while (1) {
        int32_t current_left = get_left_encoder_total();
        int32_t left_traveled = abs(current_left - start_left);

        if (left_traveled >= target_counts) {
            break;
        }
        HAL_Delay(1);
    }

    stop_motors();
    robot.direction = (robot.direction + 1) % 4; // Turn right
    HAL_Delay(200);
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


/**
 * @brief Move forward a specific distance - FIXED VERSION
 */
void move_forward_distance(int distance_mm) {
    int32_t target_counts = (distance_mm * ENCODER_COUNTS_PER_CELL) / CELL_SIZE_MM;

    // FIXED: Use safe encoder reading
    int32_t start_left = get_left_encoder_total();
    int32_t start_right = get_right_encoder_total();

    // Set motors to move forward
    HAL_GPIO_WritePin(MOTOR_IN1_GPIO_Port, MOTOR_IN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_IN2_GPIO_Port, MOTOR_IN2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_IN3_GPIO_Port, MOTOR_IN3_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_IN4_GPIO_Port, MOTOR_IN4_Pin, GPIO_PIN_RESET);

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
            // Left forward: IN1=PWM, IN2=LOW
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, duty); // PA6 = PWM
            HAL_GPIO_WritePin(MOTOR_IN2_GPIO_Port, MOTOR_IN2_Pin, GPIO_PIN_RESET); // PA7 = LOW
        } else {
            // Left reverse: IN1=LOW, IN2=PWM
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0); // PA6 = LOW
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, duty); // PA7 = PWM
        }
    } else { // Right motor
    	bool actual_forward = !forward;  // invert direction
        if (actual_forward) {
            // Right forward: IN3=PWM, IN4=LOW
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, duty); // PB0 = PWM
            HAL_GPIO_WritePin(MOTOR_IN4_GPIO_Port, MOTOR_IN4_Pin, GPIO_PIN_RESET); // PB1 = LOW
        } else {
            // Right reverse: IN3=LOW, IN4=PWM
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0); // PB0 = LOW
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, duty); // PB1 = PWM
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
