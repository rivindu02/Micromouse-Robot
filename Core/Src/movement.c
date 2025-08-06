/*
 * movement.c - Motor control and movement functions
 *
 * Implements precise movement using DRV8833 motor driver and encoders
 */

#include "micromouse.h"

/**
 * @brief Start encoder timers
 */
void start_encoders(void)
{
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL); // Right encoder
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL); // Left encoder

    // Reset encoder counts
    __HAL_TIM_SET_COUNTER(&htim4, 32768);
    __HAL_TIM_SET_COUNTER(&htim2, 32768);

    encoders.left_total = 0;
    encoders.right_total = 0;
}

/**
 * @brief Move forward one cell
 */
void move_forward(void)
{
    // Reset encoder counts for this movement
    int32_t start_left = (int32_t)__HAL_TIM_GET_COUNTER(&htim2) - 32768;
    int32_t start_right = (int32_t)__HAL_TIM_GET_COUNTER(&htim4) - 32768;

    // Set motors to move forward
    HAL_GPIO_WritePin(MOTOR_IN1_GPIO_Port, MOTOR_IN1_Pin, GPIO_PIN_SET);   // Left forward
    HAL_GPIO_WritePin(MOTOR_IN2_GPIO_Port, MOTOR_IN2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_IN3_GPIO_Port, MOTOR_IN3_Pin, GPIO_PIN_SET);   // Right forward
    HAL_GPIO_WritePin(MOTOR_IN4_GPIO_Port, MOTOR_IN4_Pin, GPIO_PIN_RESET);

    // Move until target distance reached
    int32_t target_counts = ENCODER_COUNTS_PER_CELL;

    while (1) {
        int32_t current_left = (int32_t)__HAL_TIM_GET_COUNTER(&htim2) - 32768;
        int32_t current_right = (int32_t)__HAL_TIM_GET_COUNTER(&htim4) - 32768;

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

    // Update position
    robot.x += dx[robot.direction];
    robot.y += dy[robot.direction];

    // Bounds checking
    if (robot.x < 0) robot.x = 0;
    if (robot.x >= MAZE_SIZE) robot.x = MAZE_SIZE - 1;
    if (robot.y < 0) robot.y = 0;
    if (robot.y >= MAZE_SIZE) robot.y = MAZE_SIZE - 1;

    HAL_Delay(100); // Settling time
}

/**
 * @brief Turn left 90 degrees
 */
void turn_left(void)
{
    int32_t start_left = (int32_t)__HAL_TIM_GET_COUNTER(&htim2) - 32768;
    int32_t start_right = (int32_t)__HAL_TIM_GET_COUNTER(&htim4) - 32768;

    // Left motor backward, right motor forward
    HAL_GPIO_WritePin(MOTOR_IN1_GPIO_Port, MOTOR_IN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_IN2_GPIO_Port, MOTOR_IN2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_IN3_GPIO_Port, MOTOR_IN3_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_IN4_GPIO_Port, MOTOR_IN4_Pin, GPIO_PIN_RESET);

    int32_t target_counts = ENCODER_COUNTS_PER_TURN;

    while (1) {
        int32_t current_right = (int32_t)__HAL_TIM_GET_COUNTER(&htim4) - 32768;
        int32_t right_traveled = abs_int(current_right - start_right);

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
 * @brief Turn right 90 degrees
 */
void turn_right(void)
{
    int32_t start_left = (int32_t)__HAL_TIM_GET_COUNTER(&htim2) - 32768;
    int32_t start_right = (int32_t)__HAL_TIM_GET_COUNTER(&htim4) - 32768;

    // Left motor forward, right motor backward
    HAL_GPIO_WritePin(MOTOR_IN1_GPIO_Port, MOTOR_IN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_IN2_GPIO_Port, MOTOR_IN2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_IN3_GPIO_Port, MOTOR_IN3_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_IN4_GPIO_Port, MOTOR_IN4_Pin, GPIO_PIN_SET);

    int32_t target_counts = ENCODER_COUNTS_PER_TURN;

    while (1) {
        int32_t current_left = (int32_t)__HAL_TIM_GET_COUNTER(&htim2) - 32768;
        int32_t left_traveled = abs_int(current_left - start_left);

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
void turn_around(void)
{
    turn_right();
    turn_right();
}

/**
 * @brief Stop both motors
 */
void stop_motors(void)
{
    // Brake mode - both pins low
    HAL_GPIO_WritePin(MOTOR_IN1_GPIO_Port, MOTOR_IN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_IN2_GPIO_Port, MOTOR_IN2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_IN3_GPIO_Port, MOTOR_IN3_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_IN4_GPIO_Port, MOTOR_IN4_Pin, GPIO_PIN_RESET);
}

/**
 * @brief Move forward a specific distance
 */
void move_forward_distance(int distance_mm)
{
    int32_t target_counts = (distance_mm * ENCODER_COUNTS_PER_CELL) / CELL_SIZE_MM;

    int32_t start_left = (int32_t)__HAL_TIM_GET_COUNTER(&htim2) - 32768;
    int32_t start_right = (int32_t)__HAL_TIM_GET_COUNTER(&htim4) - 32768;

    // Set motors to move forward
    HAL_GPIO_WritePin(MOTOR_IN1_GPIO_Port, MOTOR_IN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_IN2_GPIO_Port, MOTOR_IN2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_IN3_GPIO_Port, MOTOR_IN3_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_IN4_GPIO_Port, MOTOR_IN4_Pin, GPIO_PIN_RESET);

    while (1) {
        int32_t current_left = (int32_t)__HAL_TIM_GET_COUNTER(&htim2) - 32768;
        int32_t current_right = (int32_t)__HAL_TIM_GET_COUNTER(&htim4) - 32768;

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
