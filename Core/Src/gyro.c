/*
 * gyro.c - MPU9250 Gyroscope/Accelerometer - FIXED VERSION
 *
 * Provides 9-axis IMU data for advanced navigation and orientation
 * FIXED: Proper SPI configuration and error handling
 */

#include "micromouse.h"
#include <stdio.h>
#include <math.h>

/* MPU9250 Register definitions */
#define MPU9250_WHO_AM_I 0x75
#define MPU9250_PWR_MGMT_1 0x6B
#define MPU9250_PWR_MGMT_2 0x6C
#define MPU9250_CONFIG 0x1A
#define MPU9250_GYRO_CONFIG 0x1B
#define MPU9250_ACCEL_CONFIG 0x1C
#define MPU9250_ACCEL_CONFIG_2 0x1D
#define MPU9250_SMPLRT_DIV 0x19
#define MPU9250_INT_PIN_CFG 0x37
#define MPU9250_INT_ENABLE 0x38
#define MPU9250_INT_STATUS 0x3A
#define MPU9250_ACCEL_XOUT_H 0x3B
#define MPU9250_GYRO_XOUT_H 0x43
#define MPU9250_TEMP_OUT_H 0x41
#define MPU9250_USER_CTRL 0x6A  // ADDED: For I2C disable
#define MPU9250_WHO_AM_I_RESPONSE 0x70

// ADDED: Global variable to track initialization status
static bool mpu9250_initialized = false;


// Enhanced gyro state structure
typedef struct {
    float gyro_bias_x, gyro_bias_y, gyro_bias_z;
    bool calibrated;
    uint32_t calibration_samples;
} EnhancedGyroState;

static EnhancedGyroState enhanced_gyro = {0};



/**
 * @brief Read register from MPU9250
 */
uint8_t mpu9250_read_register(uint8_t reg)
{
    uint8_t tx_data = reg | 0x80; // Set read bit
    uint8_t rx_data = 0;

    HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_RESET);

    // FIXED: Check SPI transmission status
    HAL_StatusTypeDef status1 = HAL_SPI_Transmit(&hspi2, &tx_data, 1, 100);
    HAL_StatusTypeDef status2 = HAL_SPI_Receive(&hspi2, &rx_data, 1, 100);

    HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_SET);

    // Check for SPI errors
    if (status1 != HAL_OK || status2 != HAL_OK) {
        send_bluetooth_message("⚠️ SPI error in register read\r\n");
        mpu9250_initialized = false; // Mark gyro as failed
        return 0xFF; // Invalid register value
    }

    return rx_data;
}

/**
 * @brief Write register to MPU9250
 */
void mpu9250_write_register(uint8_t reg, uint8_t data)
{
    uint8_t tx_data[2] = {reg, data};
    HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_RESET);

    // FIXED: Check SPI transmission status
    HAL_StatusTypeDef status = HAL_SPI_Transmit(&hspi2, tx_data, 2, 100);

    HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_SET);

    if (status != HAL_OK) {
        send_bluetooth_message("⚠️ SPI error in register write\r\n");
        mpu9250_initialized = false; // Mark gyro as failed
    }

    HAL_Delay(1); // Small delay for register write
}


/**
 * @brief Initialize MPU9250 - FIXED VERSION with proper error handling and SPI setup
 */
bool mpu9250_init(void)
{
    send_bluetooth_message("Initializing MPU9250...\r\n");

    // FIXED: Check SPI speed - should be ≤ 1MHz for register operations
    // Note: SPI2 is configured with BAUDRATEPRESCALER_64 in main.c
    // This gives us: 84MHz / 64 = 1.3125MHz (slightly over 1MHz but should work)
    // For production, change to BAUDRATEPRESCALER_128 for safer 656kHz
    send_bluetooth_message("SPI speed: ~1.3MHz (register operations)\r\n");

    // Wait for device to be ready
    HAL_Delay(100);

    // Check WHO_AM_I register
    uint8_t who_am_i = mpu9250_read_register(MPU9250_WHO_AM_I);
    if (who_am_i == MPU9250_WHO_AM_I_RESPONSE) {
        send_bluetooth_message("MPU9250 detected successfully\r\n");
    } else {
        send_bluetooth_printf("MPU9250 detection failed! Got 0x%02X, expected 0x%02X\r\n",
                             who_am_i, MPU9250_WHO_AM_I_RESPONSE);
        mpu9250_initialized = false;
        return false; // FIXED: Return boolean status instead of void
    }

    // Reset device
    mpu9250_write_register(MPU9250_PWR_MGMT_1, 0x80);
    HAL_Delay(100);

    // FIXED: Force SPI-only mode - disable I2C interface
    // Set USER_CTRL.I2C_IF_DIS (bit 4) to prevent slipping back to I2C
    uint8_t user_ctrl = mpu9250_read_register(MPU9250_USER_CTRL);
    user_ctrl |= 0x10; // Set bit 4 (I2C_IF_DIS)
    mpu9250_write_register(MPU9250_USER_CTRL, user_ctrl);
    HAL_Delay(10);

    // Verify I2C is disabled
    uint8_t user_ctrl_verify = mpu9250_read_register(MPU9250_USER_CTRL);
    if (user_ctrl_verify & 0x10) {
        send_bluetooth_message("✅ I2C interface disabled - SPI-only mode active\r\n");
    } else {
        send_bluetooth_message("⚠️ Warning: Failed to disable I2C interface\r\n");
    }

    // Configure power management
    mpu9250_write_register(MPU9250_PWR_MGMT_1, 0x01); // Use PLL with X-axis gyro
    HAL_Delay(10);
    mpu9250_write_register(MPU9250_PWR_MGMT_2, 0x00); // Enable all axes
    HAL_Delay(10);

    // Configure sample rate (1kHz / (1 + SMPLRT_DIV))
    mpu9250_write_register(MPU9250_SMPLRT_DIV, 0x07); // 125Hz sample rate
    HAL_Delay(10);

    // Configure low-pass filter
    mpu9250_write_register(MPU9250_CONFIG, 0x03); // 41Hz bandwidth
    HAL_Delay(10);

    // Configure gyroscope (±500 dps)
    mpu9250_write_register(MPU9250_GYRO_CONFIG, 0x08);
    HAL_Delay(10);

    // Configure accelerometer (±4g)
    mpu9250_write_register(MPU9250_ACCEL_CONFIG, 0x08);
    HAL_Delay(10);

    // Configure accelerometer low-pass filter
    mpu9250_write_register(MPU9250_ACCEL_CONFIG_2, 0x03);
    HAL_Delay(10);

    // FIXED: Verify configuration by reading back key registers
    uint8_t pwr_mgmt_1 = mpu9250_read_register(MPU9250_PWR_MGMT_1);
    uint8_t gyro_config = mpu9250_read_register(MPU9250_GYRO_CONFIG);
    uint8_t accel_config = mpu9250_read_register(MPU9250_ACCEL_CONFIG);

    send_bluetooth_printf("Configuration verify - PWR:0x%02X GYRO:0x%02X ACCEL:0x%02X\r\n",
                         pwr_mgmt_1, gyro_config, accel_config);

    // Check if configuration is correct
    if (pwr_mgmt_1 == 0x01 && gyro_config == 0x08 && accel_config == 0x08) {
        send_bluetooth_message("✅ MPU9250 initialized successfully\r\n");
        mpu9250_initialized = true;
        return true; // FIXED: Return success status
    } else {
        send_bluetooth_message("❌ MPU9250 configuration verification failed\r\n");
        mpu9250_initialized = false;
        return false; // FIXED: Return failure status
    }
}


/**
 * @brief Calibrate gyro bias (call during startup when robot is stationary)
 */
void mpu9250_calibrate_bias(void) {
    if (!mpu9250_initialized) {
        send_bluetooth_message("Cannot calibrate - gyro not initialized\r\n");
        return;
    }

    send_bluetooth_message("Calibrating gyro bias... Keep robot stationary!\r\n");

    enhanced_gyro.calibration_samples = 1000;
    float sum_x = 0, sum_y = 0, sum_z = 0;

    for(int i = 0; i < enhanced_gyro.calibration_samples; i++) {
        mpu9250_read_gyro();
        sum_x += gyro.gyro_x;
        sum_y += gyro.gyro_y;
        sum_z += gyro.gyro_z;
        HAL_Delay(3); // 333Hz sampling for stable bias
    }

    enhanced_gyro.gyro_bias_x = sum_x / enhanced_gyro.calibration_samples;
    enhanced_gyro.gyro_bias_y = sum_y / enhanced_gyro.calibration_samples;
    enhanced_gyro.gyro_bias_z = sum_z / enhanced_gyro.calibration_samples;
    enhanced_gyro.calibrated = true;

    send_bluetooth_printf("Gyro bias calibrated: X:%.1f Y:%.1f Z:%.1f\r\n",
                         enhanced_gyro.gyro_bias_x, enhanced_gyro.gyro_bias_y, enhanced_gyro.gyro_bias_z);
}

/**
 * @brief Get bias-compensated gyro Z value in degrees per second
 */
float mpu9250_get_gyro_z_compensated(void) {
    if (!enhanced_gyro.calibrated) {
        return mpu9250_get_gyro_z_dps(); // Fall back to uncompensated
    }

    float raw_z_dps = (float)(gyro.gyro_z - enhanced_gyro.gyro_bias_z) / 65.5f;
    return raw_z_dps;
}




/**
 * @brief Check if MPU9250 is initialized - NEW FUNCTION
 */
bool mpu9250_is_initialized(void)
{
    return mpu9250_initialized;
}

/**
 * @brief Read raw gyroscope data - FIXED with error checking
 */
void mpu9250_read_gyro(void)
{
    if (!mpu9250_initialized) {
        send_bluetooth_message("⚠️ MPU9250 not initialized - cannot read gyro\r\n");
        return;
    }

    uint8_t raw_data[6];
    uint8_t reg = MPU9250_GYRO_XOUT_H | 0x80; // Set read bit

    HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_RESET);

    // FIXED: Check SPI transmission status
    HAL_StatusTypeDef spi_status = HAL_SPI_Transmit(&hspi2, &reg, 1, 100);
    if (spi_status != HAL_OK) {
        HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_SET);
        send_bluetooth_message("SPI transmit error in gyro read\r\n");
        return;
    }

    spi_status = HAL_SPI_Receive(&hspi2, raw_data, 6, 100);
    HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_SET);

    if (spi_status != HAL_OK) {
        send_bluetooth_message("SPI receive error in gyro read\r\n");
        return;
    }

    // Convert to signed 16-bit values
    gyro.gyro_x = (int16_t)((raw_data[0] << 8) | raw_data[1]);
    gyro.gyro_y = (int16_t)((raw_data[2] << 8) | raw_data[3]);
    gyro.gyro_z = (int16_t)((raw_data[4] << 8) | raw_data[5]);
}

/**
 * @brief Read raw accelerometer data - FIXED with error checking
 */
void mpu9250_read_accel(void)
{
    if (!mpu9250_initialized) {
        send_bluetooth_message("⚠️ MPU9250 not initialized - cannot read accel\r\n");
        return;
    }

    uint8_t raw_data[6];
    uint8_t reg = MPU9250_ACCEL_XOUT_H | 0x80; // Set read bit

    HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_RESET);

    // FIXED: Check SPI transmission status
    HAL_StatusTypeDef spi_status = HAL_SPI_Transmit(&hspi2, &reg, 1, 100);
    if (spi_status != HAL_OK) {
        HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_SET);
        send_bluetooth_message("SPI transmit error in accel read\r\n");
        return;
    }

    spi_status = HAL_SPI_Receive(&hspi2, raw_data, 6, 100);
    HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_SET);

    if (spi_status != HAL_OK) {
        send_bluetooth_message("SPI receive error in accel read\r\n");
        return;
    }

    // Convert to signed 16-bit values
    gyro.accel_x = (int16_t)((raw_data[0] << 8) | raw_data[1]);
    gyro.accel_y = (int16_t)((raw_data[2] << 8) | raw_data[3]);
    gyro.accel_z = (int16_t)((raw_data[4] << 8) | raw_data[5]);
}

/**
 * @brief Read all IMU data
 */
void mpu9250_read_all(void)
{
    mpu9250_read_accel();
    mpu9250_read_gyro();
}

/**
 * @brief Get gyro Z-axis in degrees per second
 */
float mpu9250_get_gyro_z_dps(void)
{
    // ±500 dps range, 16-bit resolution
    // Sensitivity: 65.5 LSB/(dps)
    return (float)gyro.gyro_z / 65.5f;
}

/**
 * @brief Detect turns using gyroscope
 */
bool mpu9250_detect_turn(void)
{
    if (!mpu9250_initialized) {
        return false; // Can't detect turns if not initialized
    }

    mpu9250_read_gyro();
    float gyro_z_dps = mpu9250_get_gyro_z_dps();

    // Detect significant rotation (threshold: 50 dps)
    return (fabsf(gyro_z_dps) > 50.0f);
}

/**
 * @brief Get MPU9250 status for debugging - NEW FUNCTION
 */
void mpu9250_send_status(void)
{
    send_bluetooth_printf("MPU9250 Status - Init:%s\r\n",
                         mpu9250_initialized ? "OK" : "FAILED");

    if (mpu9250_initialized) {
        uint8_t who_am_i = mpu9250_read_register(MPU9250_WHO_AM_I);
        uint8_t user_ctrl = mpu9250_read_register(MPU9250_USER_CTRL);
        uint8_t pwr_mgmt = mpu9250_read_register(MPU9250_PWR_MGMT_1);

        send_bluetooth_printf("WHO_AM_I:0x%02X USER_CTRL:0x%02X PWR_MGMT:0x%02X\r\n",
                             who_am_i, user_ctrl, pwr_mgmt);

        if (user_ctrl & 0x10) {
            send_bluetooth_message("I2C disabled: ✅\r\n");
        } else {
            send_bluetooth_message("I2C disabled: ❌\r\n");
        }

        // Read current sensor values
        mpu9250_read_all();
        send_bluetooth_printf("Gyro X:%d Y:%d Z:%.1f°/s\r\n",
                             gyro.gyro_x, gyro.gyro_y, mpu9250_get_gyro_z_dps());
        send_bluetooth_printf("Accel X:%d Y:%d Z:%d\r\n",
                             gyro.accel_x, gyro.accel_y, gyro.accel_z);
    }
}
