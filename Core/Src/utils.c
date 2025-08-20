/*
 * utils.c - Utility functions and helpers
 *
 * Common utility functions for the micromouse system
 */

#include "micromouse.h"
#include <math.h>

/**
 * @brief Delay in milliseconds
 */
void delay_ms(uint32_t ms)
{
    HAL_Delay(ms);
}

/**
 * @brief Absolute value for integers
 */
int abs_int(int value)
{
    return (value < 0) ? -value : value;
}

/**
 * @brief Calculate Euclidean distance between two points
 */
float calculate_distance(int x1, int y1, int x2, int y2)
{
    int dx = x2 - x1;
    int dy = y2 - y1;
    return sqrtf((float)(dx*dx + dy*dy));
}

/**
 * @brief Control LED status indicators
 */
void led_status(uint8_t left_state, uint8_t right_state)
{
    HAL_GPIO_WritePin(LED_LEFT_GPIO_Port, LED_LEFT_Pin, left_state ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_RIGHT_GPIO_Port, LED_RIGHT_Pin, right_state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/**
 * @brief LED sequence for different states
 */
void led_sequence_startup(void)
{
    for (int i = 0; i < 3; i++) {
        led_status(1, 0);
        HAL_Delay(150);
        led_status(0, 1);
        HAL_Delay(150);
    }
    led_status(0, 0);
}

/**
 * @brief LED sequence for exploration
 */
void led_sequence_exploring(void)
{
    led_status(1, 0); // Left LED on during exploration
}

/**
 * @brief LED sequence for returning
 */
void led_sequence_returning(void)
{
    led_status(0, 1); // Right LED on during return
}

/**
 * @brief LED sequence for completion
 */
void led_sequence_complete(void)
{
    for (int i = 0; i < 5; i++) {
        led_status(1, 1);
        HAL_Delay(200);
        led_status(0, 0);
        HAL_Delay(200);
    }
}

/**
 * @brief LED sequence for error
 */
void led_sequence_error(void)
{
    for (int i = 0; i < 10; i++) {
        led_status(1, 1);
        HAL_Delay(50);
        led_status(0, 0);
        HAL_Delay(50);
    }
}

/**
 * @brief Map integer value from one range to another
 */
int map_value(int value, int from_low, int from_high, int to_low, int to_high)
{
    return (value - from_low) * (to_high - to_low) / (from_high - from_low) + to_low;
}

/**
 * @brief Constrain value within bounds
 */
int constrain_int(int value, int min_val, int max_val)
{
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}

/**
 * @brief Get direction name as string
 */
const char* get_direction_name(int direction)
{
    switch (direction) {
        case NORTH: return "NORTH";
        case EAST:  return "EAST";
        case SOUTH: return "SOUTH";
        case WEST:  return "WEST";
        default:    return "UNKNOWN";
    }
}

/**
 * @brief Calculate Manhattan distance
 */
int manhattan_distance(int x1, int y1, int x2, int y2)
{
    return abs_int(x2 - x1) + abs_int(y2 - y1);
}

/**
 * @brief Check if coordinates are within maze bounds
 */
bool is_valid_coordinate(int x, int y)
{
    return (x >= 0 && x < MAZE_SIZE && y >= 0 && y < MAZE_SIZE);
}

/**
 * @brief Simple moving average filter
 */
float moving_average_filter(float new_value, float previous_average, int samples)
{
    return ((previous_average * (samples - 1)) + new_value) / samples;
}

/**
 * @brief System health check
 */
bool system_health_check(void)
{
    bool health_ok = true;

    // Check battery voltage
    if (sensors.battery < BATTERY_LOW_THRESHOLD) {
        send_bluetooth_message("WARNING: Low battery detected!\r\n");
        play_battery_warning();
        health_ok = false;
    }

    // Check sensor readings
    if (sensors.front_left == 0 && sensors.front_right == 0 &&
        sensors.side_left == 0 && sensors.side_right == 0) {
        send_bluetooth_message("WARNING: All sensors reading zero!\r\n");
        health_ok = false;
    }

    // Check gyroscope communication
    uint8_t gyro_id = mpu9250_read_register(0x75);
    if (gyro_id != 0x71) {
        send_bluetooth_message("WARNING: Gyroscope communication issue!\r\n");
        health_ok = false;
    }

    return health_ok;
}

/**
 * @brief Performance profiler start
 */
uint32_t perf_timer_start = 0;

void performance_start_timer(void)
{
    perf_timer_start = HAL_GetTick();
}

/**
 * @brief Performance profiler end and report
 */
void performance_end_timer(const char* operation_name)
{
    uint32_t elapsed = HAL_GetTick() - perf_timer_start;
    send_bluetooth_printf("PERF: %s took %lu ms\r\n", operation_name, elapsed);
}


