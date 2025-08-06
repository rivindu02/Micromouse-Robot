/*
 * gyro.c - MPU9250 Gyroscope/Accelerometer/Magnetometer interface
 *
 * Provides 9-axis IMU data for advanced navigation and orientation
 */

#include "micromouse.h"
#include <stdlib.h>
#include <math.h>

/* MPU9250 Register definitions */
#define MPU9250_WHO_AM_I        0x75
#define MPU9250_PWR_MGMT_1      0x6B
#define MPU9250_PWR_MGMT_2      0x6C
#define MPU9250_CONFIG          0x1A
#define MPU9250_GYRO_CONFIG     0x1B
#define MPU9250_ACCEL_CONFIG    0x1C
#define MPU9250_ACCEL_CONFIG_2  0x1D
#define MPU9250_SMPLRT_DIV      0x19
#define MPU9250_INT_PIN_CFG     0x37
#define MPU9250_INT_ENABLE      0x38
#define MPU9250_INT_STATUS      0x3A

#define MPU9250_ACCEL_XOUT_H    0x3B
#define MPU9250_GYRO_XOUT_H     0x43
#define MPU9250_TEMP_OUT_H      0x41

#define MPU9250_WHO_AM_I_RESPONSE 0x71

/**
 * @brief Read register from MPU9250
 */
uint8_t mpu9250_read_register(uint8_t reg)
{
    uint8_t tx_data = reg | 0x80; // Set read bit
    uint8_t rx_data = 0;

    HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, &tx_data, 1, 100);
    HAL_SPI_Receive(&hspi2, &rx_data, 1, 100);
    HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_SET);

    return rx_data;
}

/**
 * @brief Write register to MPU9250
 */
void mpu9250_write_register(uint8_t reg, uint8_t data)
{
    uint8_t tx_data[2] = {reg, data};

    HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, tx_data, 2, 100);
    HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_SET);

    HAL_Delay(1); // Small delay for register write
}

/**
 * @brief Initialize MPU9250
 */
void mpu9250_init(void)
{
    // Check WHO_AM_I register
    uint8_t who_am_i = mpu9250_read_register(MPU9250_WHO_AM_I);

    if (who_am_i == MPU9250_WHO_AM_I_RESPONSE) {
        send_bluetooth_message("MPU9250 detected successfully\r\n");
    } else {
        send_bluetooth_printf("MPU9250 detection failed! Got 0x%02X, expected 0x%02X\r\n",
                             who_am_i, MPU9250_WHO_AM_I_RESPONSE);
        return;
    }

    // Reset device
    mpu9250_write_register(MPU9250_PWR_MGMT_1, 0x80);
    HAL_Delay(100);

    // Configure power management
    mpu9250_write_register(MPU9250_PWR_MGMT_1, 0x01); // Use PLL with X-axis gyro
    mpu9250_write_register(MPU9250_PWR_MGMT_2, 0x00); // Enable all axes

    // Configure sample rate (1kHz / (1 + SMPLRT_DIV))
    mpu9250_write_register(MPU9250_SMPLRT_DIV, 0x07); // 125Hz sample rate

    // Configure low-pass filter
    mpu9250_write_register(MPU9250_CONFIG, 0x03); // 41Hz bandwidth

    // Configure gyroscope (±500 dps)
    mpu9250_write_register(MPU9250_GYRO_CONFIG, 0x08);

    // Configure accelerometer (±4g)
    mpu9250_write_register(MPU9250_ACCEL_CONFIG, 0x08);

    // Configure accelerometer low-pass filter
    mpu9250_write_register(MPU9250_ACCEL_CONFIG_2, 0x03);

    HAL_Delay(10);
    send_bluetooth_message("MPU9250 initialized successfully\r\n");
}

/**
 * @brief Read raw gyroscope data
 */
void mpu9250_read_gyro(void)
{
    uint8_t raw_data[6];
    uint8_t reg = MPU9250_GYRO_XOUT_H | 0x80; // Set read bit

    HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, &reg, 1, 100);
    HAL_SPI_Receive(&hspi2, raw_data, 6, 100);
    HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_SET);

    // Convert to signed 16-bit values
    gyro.gyro_x = (int16_t)((raw_data[0] << 8) | raw_data[1]);
    gyro.gyro_y = (int16_t)((raw_data[2] << 8) | raw_data[3]);
    gyro.gyro_z = (int16_t)((raw_data[4] << 8) | raw_data[5]);
}

/**
 * @brief Read raw accelerometer data
 */
void mpu9250_read_accel(void)
{
    uint8_t raw_data[6];
    uint8_t reg = MPU9250_ACCEL_XOUT_H | 0x80; // Set read bit

    HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, &reg, 1, 100);
    HAL_SPI_Receive(&hspi2, raw_data, 6, 100);
    HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_SET);

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
    mpu9250_read_gyro();
    float gyro_z_dps = mpu9250_get_gyro_z_dps();

    // Detect significant rotation (threshold: 50 dps)
    return (fabsf(gyro_z_dps) > 50.0f);
}
