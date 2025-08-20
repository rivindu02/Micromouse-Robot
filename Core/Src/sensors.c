// Create sensors.c - IR sensors and MPU9250 implementation

/*
 * sensors.c
 *
 * IR sensor (SFH4545 + TEFT4300) and MPU9250 implementation
 * Handles wall detection and robot orientation
 * Handles ADC reading for IR sensors and battery monitoring
 *
 * Author: Micromouse v1.0
 * Date: 2025
 */

#include "micromouse.h"

static uint8_t sensor_error_count = 0;
static bool sensors_healthy = true;
/**
 * @brief Turn on IR emitters
 */
void turn_on_emitters(void)
{
    HAL_GPIO_WritePin(EMIT_FRONT_LEFT_GPIO_Port, EMIT_FRONT_LEFT_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(EMIT_SIDE_LEFT_GPIO_Port, EMIT_SIDE_LEFT_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(EMIT_SIDE_RIGHT_GPIO_Port, EMIT_SIDE_RIGHT_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(EMIT_FRONT_RIGHT_GPIO_Port, EMIT_FRONT_RIGHT_Pin, GPIO_PIN_SET);
    HAL_Delay(2); // Emitter stabilization time
}

/**
 * @brief Turn off IR emitters
 */
void turn_off_emitters(void)
{
    HAL_GPIO_WritePin(EMIT_FRONT_LEFT_GPIO_Port, EMIT_FRONT_LEFT_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(EMIT_SIDE_LEFT_GPIO_Port, EMIT_SIDE_LEFT_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(EMIT_SIDE_RIGHT_GPIO_Port, EMIT_SIDE_RIGHT_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(EMIT_FRONT_RIGHT_GPIO_Port, EMIT_FRONT_RIGHT_Pin, GPIO_PIN_RESET);
}

/**
 * @brief Read specific ADC channel using main.c multi-channel setup
 */
uint16_t read_adc_channel(uint32_t channel)
{
    // Use the 5-channel continuous setup from main.c
    uint32_t adc_values[5];

    // Start continuous conversion of all 5 channels
    if (HAL_ADC_Start(&hadc1) != HAL_OK) {
        return 0; // Hardware error - return safe value
    }

    // Read all 5 channels in sequence (as configured in main.c)
    for (int i = 0; i < 5; i++) {
        if (HAL_ADC_PollForConversion(&hadc1, 50) != HAL_OK) {
            HAL_ADC_Stop(&hadc1);
            return 0; // Timeout error
        }
        adc_values[i] = HAL_ADC_GetValue(&hadc1);
    }

    HAL_ADC_Stop(&hadc1);

    // Return the correct channel value based on main.c rank order
    switch (channel) {
        case ADC_CHANNEL_0: return adc_values[0]; // Rank 1 - Battery
        case ADC_CHANNEL_2: return adc_values[1]; // Rank 2 - Front Right
        case ADC_CHANNEL_3: return adc_values[2]; // Rank 3 - Side Right
        case ADC_CHANNEL_4: return adc_values[3]; // Rank 4 - Side Left
        case ADC_CHANNEL_5: return adc_values[4]; // Rank 5 - Front Left
        default: return 0;
    }
}


/**
 * @brief Update all sensor readings
 */
void update_sensors(void)
{
    // Read ambient light levels (emitters off)
    turn_off_emitters();
    HAL_Delay(1);

    uint16_t ambient_front_right = read_adc_channel(ADC_CHANNEL_2);
    uint16_t ambient_side_right = read_adc_channel(ADC_CHANNEL_3);
    uint16_t ambient_side_left = read_adc_channel(ADC_CHANNEL_4);
    uint16_t ambient_front_left = read_adc_channel(ADC_CHANNEL_5);

    // Read with emitters on
    turn_on_emitters();

    sensors.battery = read_adc_channel(ADC_CHANNEL_0);
    sensors.front_right = read_adc_channel(ADC_CHANNEL_2) - ambient_front_right;
    sensors.side_right = read_adc_channel(ADC_CHANNEL_3) - ambient_side_right;
    sensors.side_left = read_adc_channel(ADC_CHANNEL_4) - ambient_side_left;
    sensors.front_left = read_adc_channel(ADC_CHANNEL_5) - ambient_front_left;

    // Turn off emitters to save power
    turn_off_emitters();

    // Process wall detection
    sensors.wall_front = (sensors.front_left > WALL_THRESHOLD_FRONT) ||
                         (sensors.front_right > WALL_THRESHOLD_FRONT);
    sensors.wall_left = (sensors.side_left > WALL_THRESHOLD_SIDE);
    sensors.wall_right = (sensors.side_right > WALL_THRESHOLD_SIDE);


    // Check for sensor health
    bool current_reading_valid = (sensors.battery > 100) && // Reasonable battery reading
                               (sensors.front_left < 4000) && // Not maxed out
                               (sensors.front_right < 4000) &&
                               (sensors.side_left < 4000) &&
                               (sensors.side_right < 4000);

    if (!current_reading_valid) {
        sensor_error_count++;
        if (sensor_error_count > 5) {
            sensors_healthy = false;
            send_bluetooth_message("⚠️ WARNING: Sensor readings abnormal\r\n");
            // Don't halt - allow robot to continue with degraded sensors
        }
    } else {
        if (sensor_error_count > 0) sensor_error_count--; // Recover slowly
    }
}

/**
 * @brief Update maze walls based on sensor readings
 */
void update_walls(void)
{
    // Update walls based on current direction and sensor readings
    if (sensors.wall_front) {
        maze[robot.x][robot.y].walls[robot.direction] = true;
        // Update opposite wall in neighbor cell
        int nx = robot.x + dx[robot.direction];
        int ny = robot.y + dy[robot.direction];
        if (nx >= 0 && nx < MAZE_SIZE && ny >= 0 && ny < MAZE_SIZE) {
            maze[nx][ny].walls[(robot.direction + 2) % 4] = true;
        }
    }

    if (sensors.wall_left) {
        int left_dir = (robot.direction + 3) % 4;
        maze[robot.x][robot.y].walls[left_dir] = true;
        int nx = robot.x + dx[left_dir];
        int ny = robot.y + dy[left_dir];
        if (nx >= 0 && nx < MAZE_SIZE && ny >= 0 && ny < MAZE_SIZE) {
            maze[nx][ny].walls[(left_dir + 2) % 4] = true;
        }
    }

    if (sensors.wall_right) {
        int right_dir = (robot.direction + 1) % 4;
        maze[robot.x][robot.y].walls[right_dir] = true;
        int nx = robot.x + dx[right_dir];
        int ny = robot.y + dy[right_dir];
        if (nx >= 0 && nx < MAZE_SIZE && ny >= 0 && ny < MAZE_SIZE) {
            maze[nx][ny].walls[(right_dir + 2) % 4] = true;
        }
    }

    // Mark current cell as visited
    maze[robot.x][robot.y].visited = true;
    maze[robot.x][robot.y].visit_count++;
}

/**
 * @brief Check if sensors are healthy
 */
bool are_sensors_healthy(void)
{
    return sensors_healthy;
}



/**
 * @brief Calibrate sensors (placeholder for now)
 */
void calibrate_sensors(void)
{
    send_bluetooth_message("Calibrating sensors...\r\n");

    // Take baseline readings
    for (int i = 0; i < 10; i++) {
        update_sensors();
        HAL_Delay(50);
    }

    send_bluetooth_message("Sensor calibration complete\r\n");
}
