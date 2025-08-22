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
uint16_t read_adc_channel(uint32_t channel) {
    ADC_ChannelConfTypeDef sConfig = {0};
    uint16_t adc_value = 0;

    // Configure the channel
    sConfig.Channel = channel;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES; // Longer sampling time

    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        send_bluetooth_message("❌ ADC channel config failed\r\n");
        return 0;
    }

    // Start ADC conversion
    if (HAL_ADC_Start(&hadc1) != HAL_OK) {
        send_bluetooth_message("❌ ADC start failed\r\n");
        return 0;
    }

    // Wait for conversion with longer timeout
    if (HAL_ADC_PollForConversion(&hadc1, 100) != HAL_OK) {
        send_bluetooth_message("❌ ADC conversion timeout\r\n");
        HAL_ADC_Stop(&hadc1);
        return 0;
    }

    // Get the converted value
    adc_value = HAL_ADC_GetValue(&hadc1);

    // Stop ADC
    HAL_ADC_Stop(&hadc1);

    return adc_value;
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
            //send_bluetooth_message("⚠️ WARNING: Sensor readings abnormal\r\n");///////////////
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


void adc_system_diagnostics(void) {
    send_bluetooth_message("\r\n=== ADC SYSTEM DIAGNOSTICS ===\r\n");

    // Check if ADC clock is enabled
    if (__HAL_RCC_ADC1_IS_CLK_ENABLED()) {
        send_bluetooth_message("✅ ADC1 clock: ENABLED\r\n");
    } else {
        send_bluetooth_message("❌ ADC1 clock: DISABLED\r\n");
    }

    // Check GPIO clock
    if (__HAL_RCC_GPIOA_IS_CLK_ENABLED()) {
        send_bluetooth_message("✅ GPIOA clock: ENABLED\r\n");
    } else {
        send_bluetooth_message("❌ GPIOA clock: DISABLED\r\n");
    }

    // Check ADC status
    if (hadc1.State == HAL_ADC_STATE_READY) {
        send_bluetooth_message("✅ ADC state: READY\r\n");
    } else {
        send_bluetooth_printf("⚠️ ADC state: %d\r\n", hadc1.State);
    }

    // Test individual channel readings
    send_bluetooth_message("Testing individual channels:\r\n");

    uint32_t channels[5] = {ADC_CHANNEL_0, ADC_CHANNEL_2, ADC_CHANNEL_3, ADC_CHANNEL_4, ADC_CHANNEL_5};
    const char* channel_names[5] = {"Battery", "Front_Right", "Side_Right", "Side_Left", "Front_Left"};
    uint16_t test_values[5];  // Fixed: Added array brackets

    for (int i = 0; i < 5; i++) {
        test_values[i] = read_adc_channel(channels[i]);
        send_bluetooth_printf("%s (CH%d): %d\r\n", channel_names[i],
                             channels[i] == ADC_CHANNEL_0 ? 0 :
                             channels[i] == ADC_CHANNEL_2 ? 2 :
                             channels[i] == ADC_CHANNEL_3 ? 3 :
                             channels[i] == ADC_CHANNEL_4 ? 4 : 5, test_values[i]);
    }

    send_bluetooth_message("===============================\r\n");
}
