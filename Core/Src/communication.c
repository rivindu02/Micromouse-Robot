/*
 * communication.c - Bluetooth communication and telemetry
 *
 * Handles USART6 communication for debugging and data transmission
 */

#include "micromouse.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
/**
 * @brief Send message via Bluetooth
 */
void send_bluetooth_message(const char* message)
{
    HAL_UART_Transmit(&huart6, (uint8_t*)message, strlen(message), 1000);
}

/**
 * @brief Send formatted message via Bluetooth
 */
void send_bluetooth_printf(const char* format, ...)
{
    char buffer[256];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    send_bluetooth_message(buffer);
}

/**
 * @brief Send current maze state
 */
void send_maze_state(void)
{
    send_bluetooth_message("\r\n=== MAZE STATE ===\r\n");

    // Send maze in ASCII format
    for (int y = MAZE_SIZE - 1; y >= 0; y--) {
        char line[64] = "";
        for (int x = 0; x < MAZE_SIZE; x++) {
            char cell[8];
            if (maze[x][y].visited) {
                sprintf(cell, "%3d ", maze[x][y].distance < MAX_DISTANCE ? maze[x][y].distance : 999);
            } else {
                sprintf(cell, " -- ");
            }
            strcat(line, cell);
        }
        strcat(line, "\r\n");
        send_bluetooth_message(line);
    }

    send_bluetooth_printf("Robot Position: (%d,%d) Direction: %d\r\n",
                         robot.x, robot.y, robot.direction);
    send_bluetooth_printf("Center Reached: %s\r\n", robot.center_reached ? "YES" : "NO");
    send_bluetooth_printf("Returned to Start: %s\r\n", robot.returned_to_start ? "YES" : "NO");
    send_bluetooth_message("==================\r\n");
}

/**
 * @brief Send current sensor data
 */
void send_sensor_data(void)
{
    send_bluetooth_printf("SENSORS - Battery:%d FL:%d FR:%d SL:%d SR:%d Walls:F%d L%d R%d\r\n",
                         sensors.battery, sensors.front_left, sensors.front_right,
                         sensors.side_left, sensors.side_right,
                         sensors.wall_front ? 1 : 0, sensors.wall_left ? 1 : 0, sensors.wall_right ? 1 : 0);
}

/**
 * @brief Send current position and encoder data
 */
void send_position_data(void)
{
    int32_t left_count = (int32_t)__HAL_TIM_GET_COUNTER(&htim2) - 32768;
    int32_t right_count = (int32_t)__HAL_TIM_GET_COUNTER(&htim4) - 32768;

    send_bluetooth_printf("POSITION - X:%d Y:%d Dir:%d EncL:%ld EncR:%ld\r\n",
                         robot.x, robot.y, robot.direction, left_count, right_count);
}

/**
 * @brief Send performance metrics
 */
void send_performance_metrics(void)
{
    float efficiency = get_exploration_efficiency();  // make functions
    int optimal_distance = get_optimal_distance();		// make functions

    send_bluetooth_message("\r\n=== PERFORMANCE METRICS ===\r\n");
    send_bluetooth_printf("Exploration Steps: %d\r\n", robot.exploration_steps);
    send_bluetooth_printf("Exploration Efficiency: %.1f%%\r\n", efficiency);
    send_bluetooth_printf("Optimal Path Distance: %d steps\r\n", optimal_distance);

    // Performance rating
    if (efficiency <= 50.0f && optimal_distance > 0) {
        send_bluetooth_message("Rating: ⭐⭐⭐⭐⭐ LEVEL\r\n");
    } else if (efficiency <= 65.0f) {
        send_bluetooth_message("Rating: ⭐⭐⭐⭐ COMPETITION READY\r\n");
    } else if (efficiency <= 80.0f) {
        send_bluetooth_message("Rating: ⭐⭐⭐ GOOD PERFORMANCE\r\n");
    } else {
        send_bluetooth_message("Rating: ⭐⭐ NEEDS OPTIMIZATION\r\n");
    }
    send_bluetooth_message("===========================\r\n");
}

/**
 * @brief Send battery status
 */
void send_battery_status(void)
{
    // Convert ADC reading to voltage (assuming 3.3V reference)
    float voltage = (sensors.battery * 3.3f) / 4096.0f;

    send_bluetooth_printf("Battery: %.2fV (ADC:%d)", voltage, sensors.battery);

    if (sensors.battery < BATTERY_LOW_THRESHOLD) {
        send_bluetooth_message(" - LOW BATTERY WARNING!\r\n");
    } else {
        send_bluetooth_message(" - OK\r\n");
    }
}

/**
 * @brief Send  statistics
 */
void send_stats(void)
{
    send_bluetooth_message("\r\n🏆 STATISTICS 🏆\r\n");
    send_bluetooth_printf("Algorithm: Flood Fill +  Heuristics\r\n");
    send_bluetooth_printf("MCU: STM32F411CEU6 @ 84MHz\r\n");
    send_bluetooth_printf("Sensors: 4x TEFT4300 IR + MPU9250 Gyro\r\n");
    send_bluetooth_printf("Motors: DRV8833 H-Bridge with Encoders\r\n");
    send_bluetooth_printf("International Standard: IEEE Micromouse Compliant\r\n");
    send_bluetooth_message("====================================\r\n");
}
