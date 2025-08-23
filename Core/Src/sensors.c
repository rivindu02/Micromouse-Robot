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




// Calibration data structure
typedef struct {
    uint16_t ambient_baseline[4];        // Baseline ambient light for each sensor
    uint16_t wall_thresholds[4];         // Dynamic wall detection thresholds
    uint16_t battery_baseline;           // Battery voltage baseline
    uint16_t sensor_min[4];             // Minimum readings during calibration
    uint16_t sensor_max[4];             // Maximum readings during calibration
    bool calibration_valid;              // Flag indicating successful calibration
    float noise_levels[4];              // Noise level for each sensor
} SensorCalibration;

static SensorCalibration sensor_cal = {0};
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
 * @brief Enhanced update_sensors with calibrated thresholds
 */
void update_sensors(void)
{
    // Read ambient light levels (emitters off)
    turn_off_emitters();
    HAL_Delay(5);
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

    // Process wall detection using calibrated thresholds
    if (sensor_cal.calibration_valid) {
        // Use dynamic thresholds
        sensors.wall_front = (sensors.front_left > get_calibrated_threshold(0)) ||
                            (sensors.front_right > get_calibrated_threshold(1));
        sensors.wall_left = (sensors.side_left > get_calibrated_threshold(2));
        sensors.wall_right = (sensors.side_right > get_calibrated_threshold(3));
    } else {
        // Fallback to static thresholds
        sensors.wall_front = (sensors.front_left > WALL_THRESHOLD_FRONT) ||
                            (sensors.front_right > WALL_THRESHOLD_FRONT);
        sensors.wall_left = (sensors.side_left > WALL_THRESHOLD_SIDE);
        sensors.wall_right = (sensors.side_right > WALL_THRESHOLD_SIDE);
    }

    // Enhanced sensor health monitoring using calibration data
    static uint8_t sensor_error_count = 0;
    static bool sensors_healthy = true;

    if (sensor_cal.calibration_valid) {
        // Check if readings are within expected ranges based on calibration
        bool current_reading_valid = (sensors.battery > sensor_cal.battery_baseline - 200) &&
                                   (sensors.battery < sensor_cal.battery_baseline + 500) &&
                                   (sensors.front_left < 3500) &&
                                   (sensors.front_right < 3500) &&
                                   (sensors.side_left < 3500) &&
                                   (sensors.side_right < 3500);

        if (!current_reading_valid) {
            sensor_error_count++;
            if (sensor_error_count > 5) {
                sensors_healthy = false;
            }
        } else {
            if (sensor_error_count > 0) sensor_error_count--; // Recover slowly
        }
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
 * @brief Get sensor calibration status
 */
bool is_sensor_calibration_valid(void)
{
    return sensor_cal.calibration_valid;
}

/**
 * @brief Send detailed sensor status including calibration data
 */
void send_detailed_sensor_status(void)
{
    send_bluetooth_message("\r\n=== DETAILED SENSOR STATUS ===\r\n");

    if (sensor_cal.calibration_valid) {
        send_bluetooth_message("Calibration: ✅ VALID\r\n");

        // Update sensors first
        update_sensors();

        send_bluetooth_message("Current readings vs baselines:\r\n");
        send_bluetooth_printf("Front Left:  %d (baseline: %d, threshold: %d)\r\n",
                             sensors.front_left + sensor_cal.ambient_baseline[0],
                             sensor_cal.ambient_baseline[0],
                             sensor_cal.wall_thresholds[0]);
        send_bluetooth_printf("Front Right: %d (baseline: %d, threshold: %d)\r\n",
                             sensors.front_right + sensor_cal.ambient_baseline[1],
                             sensor_cal.ambient_baseline[1],
                             sensor_cal.wall_thresholds[1]);
        send_bluetooth_printf("Side Left:   %d (baseline: %d, threshold: %d)\r\n",
                             sensors.side_left + sensor_cal.ambient_baseline[2],
                             sensor_cal.ambient_baseline[2],
                             sensor_cal.wall_thresholds[2]);
        send_bluetooth_printf("Side Right:  %d (baseline: %d, threshold: %d)\r\n",
                             sensors.side_right + sensor_cal.ambient_baseline[3],
                             sensor_cal.ambient_baseline[3],
                             sensor_cal.wall_thresholds[3]);

        send_bluetooth_printf("Battery: %d (baseline: %d)\r\n",
                             sensors.battery, sensor_cal.battery_baseline);

        send_bluetooth_printf("Wall detection: F:%s L:%s R:%s\r\n",
                             sensors.wall_front ? "YES" : "NO",
                             sensors.wall_left ? "YES" : "NO",
                             sensors.wall_right ? "YES" : "NO");
    } else {
        send_bluetooth_message("Calibration: ❌ INVALID - Using default thresholds\r\n");
    }

    send_bluetooth_message("===============================\r\n");
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

// Add this function to sensors.c for debugging
void diagnostic_sensor_test(void) {
    send_bluetooth_message("\r\n=== IR SENSOR DIAGNOSTIC ===\r\n");

    // Test each emitter-detector pair individually
    const char* sensor_names[] = {"Front Left", "Front Right", "Side Left", "Side Right"};
    uint32_t channels[] = {ADC_CHANNEL_5, ADC_CHANNEL_2, ADC_CHANNEL_4, ADC_CHANNEL_3};
    GPIO_TypeDef* emit_ports[] = {EMIT_FRONT_LEFT_GPIO_Port, EMIT_FRONT_RIGHT_GPIO_Port,
                                  EMIT_SIDE_LEFT_GPIO_Port, EMIT_SIDE_RIGHT_GPIO_Port};
    uint16_t emit_pins[] = {EMIT_FRONT_LEFT_Pin, EMIT_FRONT_RIGHT_Pin,
                           EMIT_SIDE_LEFT_Pin, EMIT_SIDE_RIGHT_Pin};

    for(int i = 0; i < 4; i++) {
        // Test with emitter OFF
        HAL_GPIO_WritePin(emit_ports[i], emit_pins[i], GPIO_PIN_RESET);
        HAL_Delay(10);
        uint16_t off_reading = read_adc_channel(channels[i]);

        // Test with emitter ON
        HAL_GPIO_WritePin(emit_ports[i], emit_pins[i], GPIO_PIN_SET);
        HAL_Delay(10);
        uint16_t on_reading = read_adc_channel(channels[i]);



        // Calculate difference
        int16_t difference = on_reading - off_reading;

        send_bluetooth_printf("%s: OFF=%d, ON=%d, DIFF=%d\r\n",
                             sensor_names[i], off_reading, on_reading, difference);

        // Turn off emitter
        HAL_GPIO_WritePin(emit_ports[i], emit_pins[i], GPIO_PIN_RESET);
        HAL_Delay(50);
    }

    send_bluetooth_message("=== Place hand/object 5cm from sensor and retest ===\r\n");
}


/**
 * @brief Enhanced sensor calibration - replaces main.c ADC test code
 * This function performs comprehensive sensor calibration including:
 * - Baseline ambient light measurement
 * - Dynamic wall threshold calculation
 * - Sensor health validation
 * - Battery voltage baseline
 */
void calibrate_sensors(void)
{
    send_bluetooth_message("\r\n=== ENHANCED SENSOR CALIBRATION ===\r\n");

    // Initialize calibration structure
    memset(&sensor_cal, 0, sizeof(sensor_cal));

    // Phase 1: ADC System Validation
    send_bluetooth_message("Phase 1: ADC System Validation\r\n");

    // Check if ADC is properly initialized
    if (hadc1.State != HAL_ADC_STATE_READY) {
        send_bluetooth_message("❌ ADC not ready - attempting re-initialization\r\n");
        if (HAL_ADC_Init(&hadc1) != HAL_OK) {
            send_bluetooth_message("❌ CRITICAL: ADC initialization failed!\r\n");
            return;
        }
    }

    // Verify clock enables
    if (!__HAL_RCC_ADC1_IS_CLK_ENABLED()) {
        send_bluetooth_message("❌ ADC1 clock disabled\r\n");
        return;
    }

    if (!__HAL_RCC_GPIOA_IS_CLK_ENABLED()) {
        send_bluetooth_message("❌ GPIOA clock disabled\r\n");
        return;
    }

    send_bluetooth_message("✅ ADC system validation passed\r\n");

    // Phase 2: Baseline Ambient Light Measurement
    send_bluetooth_message("Phase 2: Measuring ambient baselines (IR emitters OFF)\r\n");

    // Ensure emitters are OFF
    turn_off_emitters();
    HAL_Delay(100); // Allow sensors to stabilize

    // Take multiple ambient readings for stability
    uint32_t ambient_sum[4] = {0};
    uint32_t ambient_readings = 50;

    for (int i = 0; i < ambient_readings; i++) {
        ambient_sum[0] += read_adc_channel(ADC_CHANNEL_5); // Front Left
        ambient_sum[1] += read_adc_channel(ADC_CHANNEL_2); // Front Right
        ambient_sum[2] += read_adc_channel(ADC_CHANNEL_4); // Side Left
        ambient_sum[3] += read_adc_channel(ADC_CHANNEL_3); // Side Right
        HAL_Delay(10);
    }

    // Calculate ambient baselines
    sensor_cal.ambient_baseline[0] = ambient_sum[0] / ambient_readings; // Front Left
    sensor_cal.ambient_baseline[1] = ambient_sum[1] / ambient_readings; // Front Right
    sensor_cal.ambient_baseline[2] = ambient_sum[2] / ambient_readings; // Side Left
    sensor_cal.ambient_baseline[3] = ambient_sum[3] / ambient_readings; // Side Right

    send_bluetooth_message("Ambient baselines (emitters OFF):\r\n");
    send_bluetooth_printf("  Front Left:  %d\r\n", sensor_cal.ambient_baseline[0]);
    send_bluetooth_printf("  Front Right: %d\r\n", sensor_cal.ambient_baseline[1]);
    send_bluetooth_printf("  Side Left:   %d\r\n", sensor_cal.ambient_baseline[2]);
    send_bluetooth_printf("  Side Right:  %d\r\n", sensor_cal.ambient_baseline[3]);

    // Phase 3: Active Sensor Response Measurement
    send_bluetooth_message("Phase 3: Measuring sensor response (IR emitters ON)\r\n");

    // Turn on emitters and measure response
    turn_on_emitters();
    HAL_Delay(50); // Emitter stabilization

    uint32_t active_sum[4] = {0};
    uint32_t active_readings = 50;

    // Track min/max for noise calculation
    uint16_t temp_min[4] = {4095, 4095, 4095, 4095};
    uint16_t temp_max[4] = {0, 0, 0, 0};

    for (int i = 0; i < active_readings; i++) {
        uint16_t readings[4];
        readings[0] = read_adc_channel(ADC_CHANNEL_5); // Front Left
        readings[1] = read_adc_channel(ADC_CHANNEL_2); // Front Right
        readings[2] = read_adc_channel(ADC_CHANNEL_4); // Side Left
        readings[3] = read_adc_channel(ADC_CHANNEL_3); // Side Right

        for (int j = 0; j < 4; j++) {
            active_sum[j] += readings[j];
            if (readings[j] < temp_min[j]) temp_min[j] = readings[j];
            if (readings[j] > temp_max[j]) temp_max[j] = readings[j];
        }
        HAL_Delay(10);
    }

    // Calculate active response levels and noise
    uint16_t active_avg[4];
    for (int i = 0; i < 4; i++) {
        active_avg[i] = active_sum[i] / active_readings;
        sensor_cal.sensor_min[i] = temp_min[i];
        sensor_cal.sensor_max[i] = temp_max[i];
        sensor_cal.noise_levels[i] = (float)(temp_max[i] - temp_min[i]);
    }

    send_bluetooth_message("Active sensor response (emitters ON):\r\n");
    send_bluetooth_printf("  Front Left:  %d (noise: %.1f)\r\n", active_avg[0], sensor_cal.noise_levels[0]);
    send_bluetooth_printf("  Front Right: %d (noise: %.1f)\r\n", active_avg[1], sensor_cal.noise_levels[1]);
    send_bluetooth_printf("  Side Left:   %d (noise: %.1f)\r\n", active_avg[2], sensor_cal.noise_levels[2]);
    send_bluetooth_printf("  Side Right:  %d (noise: %.1f)\r\n", active_avg[3], sensor_cal.noise_levels[3]);

    // Phase 4: Dynamic Threshold Calculation
    send_bluetooth_message("Phase 4: Calculating dynamic wall detection thresholds\r\n");

    // Calculate differential response (active - ambient)
    uint16_t differential[4];
    for (int i = 0; i < 4; i++) {
        if (active_avg[i] > sensor_cal.ambient_baseline[i]) {
            differential[i] = active_avg[i] - sensor_cal.ambient_baseline[i];
        } else {
            differential[i] = 0; // Sensor may be faulty
        }
    }

    // Set dynamic thresholds based on differential response and noise
    // Front sensors (0,1) typically have higher response near walls
    // Side sensors (2,3) have different response characteristics
    for (int i = 0; i < 4; i++) {
        if (i < 2) { // Front sensors
            // For front sensors, set threshold at 40% of differential + noise margin
            sensor_cal.wall_thresholds[i] = sensor_cal.ambient_baseline[i] +
                                          (differential[i] * 0.4f) +
                                          (sensor_cal.noise_levels[i] * 3.0f);
        } else { // Side sensors
            // For side sensors, set threshold at 30% of differential + noise margin
            sensor_cal.wall_thresholds[i] = sensor_cal.ambient_baseline[i] +
                                          (differential[i] * 0.3f) +
                                          (sensor_cal.noise_levels[i] * 3.0f);
        }

        // Sanity check - ensure threshold is reasonable
        if (sensor_cal.wall_thresholds[i] < sensor_cal.ambient_baseline[i] + 100) {
            sensor_cal.wall_thresholds[i] = sensor_cal.ambient_baseline[i] + 100;
        }
        if (sensor_cal.wall_thresholds[i] > 3500) { // Don't exceed reasonable ADC range
            sensor_cal.wall_thresholds[i] = 3500;
        }
    }

    send_bluetooth_message("Dynamic wall detection thresholds:\r\n");
    send_bluetooth_printf("  Front Left:  %d\r\n", sensor_cal.wall_thresholds[0]);
    send_bluetooth_printf("  Front Right: %d\r\n", sensor_cal.wall_thresholds[1]);
    send_bluetooth_printf("  Side Left:   %d\r\n", sensor_cal.wall_thresholds[2]);
    send_bluetooth_printf("  Side Right:  %d\r\n", sensor_cal.wall_thresholds[3]);

    // Phase 5: Battery Baseline Measurement
    send_bluetooth_message("Phase 5: Battery voltage baseline measurement\r\n");

    uint32_t battery_sum = 0;
    for (int i = 0; i < 20; i++) {
        battery_sum += read_adc_channel(ADC_CHANNEL_0);
        HAL_Delay(10);
    }
    sensor_cal.battery_baseline = battery_sum / 20;

    float battery_voltage = (sensor_cal.battery_baseline * 3.3f) / 4096.0f;
    send_bluetooth_printf("Battery baseline: %d (%.2fV)\r\n",
                         sensor_cal.battery_baseline, battery_voltage);

    // Phase 6: Sensor Health Validation
    send_bluetooth_message("Phase 6: Sensor health validation\r\n");

    bool all_sensors_healthy = true;

    // Check each sensor
    const char* sensor_names[4] = {"Front Left", "Front Right", "Side Left", "Side Right"};

    for (int i = 0; i < 4; i++) {
        bool sensor_healthy = true;

        // Check if sensor shows reasonable differential response
        if (differential[i] < 50) {
            send_bluetooth_printf("⚠️ %s: Low differential response (%d)\r\n",
                                 sensor_names[i], differential[i]);
            sensor_healthy = false;
        }

        // Check noise levels
        if (sensor_cal.noise_levels[i] > 200) {
            send_bluetooth_printf("⚠️ %s: High noise level (%.1f)\r\n",
                                 sensor_names[i], sensor_cal.noise_levels[i]);
            sensor_healthy = false;
        }

        // Check if readings are within reasonable ADC range
        if (sensor_cal.ambient_baseline[i] > 3800 || sensor_cal.ambient_baseline[i] < 10) {
            send_bluetooth_printf("⚠️ %s: Ambient reading out of range (%d)\r\n",
                                 sensor_names[i], sensor_cal.ambient_baseline[i]);
            sensor_healthy = false;
        }

        if (sensor_healthy) {
            send_bluetooth_printf("✅ %s: Healthy\r\n", sensor_names[i]);
        } else {
            all_sensors_healthy = false;
        }
    }

    // Check battery
    if (battery_voltage < 3.0f) {
        send_bluetooth_message("⚠️ Battery: Low voltage detected\r\n");
        all_sensors_healthy = false;
    } else if (battery_voltage > 4.5f) {
        send_bluetooth_message("⚠️ Battery: Voltage too high\r\n");
        all_sensors_healthy = false;
    } else {
        send_bluetooth_message("✅ Battery: Healthy\r\n");
    }

    // Phase 7: Calibration Complete
    sensor_cal.calibration_valid = all_sensors_healthy;

    turn_off_emitters(); // Save power

    if (sensor_cal.calibration_valid) {
        send_bluetooth_message("✅ SENSOR CALIBRATION COMPLETE - All systems nominal\r\n");
        send_bluetooth_message("Dynamic thresholds will be used for wall detection\r\n");
    } else {
        send_bluetooth_message("⚠️ SENSOR CALIBRATION COMPLETE - Some issues detected\r\n");
        send_bluetooth_message("Robot will continue with degraded sensor performance\r\n");
    }

    send_bluetooth_message("=====================================\r\n");
}

/**
 * @brief Get calibrated wall threshold for specific sensor
 * @param sensor_index: 0=Front_Left, 1=Front_Right, 2=Side_Left, 3=Side_Right
 * @return Calibrated threshold value
 */
uint16_t get_calibrated_threshold(int sensor_index)
{
    if (sensor_index < 0 || sensor_index > 3 || !sensor_cal.calibration_valid) {
        // Return default thresholds if calibration failed
        return (sensor_index < 2) ? WALL_THRESHOLD_FRONT : WALL_THRESHOLD_SIDE;
    }

    return sensor_cal.wall_thresholds[sensor_index];
}
