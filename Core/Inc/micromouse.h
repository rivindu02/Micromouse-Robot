/*
 * micromouse.h - Championship Micromouse Library Header
 *
 * Contains all data structures, constants, and function prototypes
 * for the championship-level micromouse implementation with
 * INTERNATIONAL COMPETITION STANDARDS
 */

#ifndef MICROMOUSE_H
#define MICROMOUSE_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>  // For abs()
#include <string.h>  // For memset()
#include <math.h>    // For fabsf(), sqrtf()

/* Maze configuration */
#define MAZE_SIZE 16
#define MAX_DISTANCE 9999
#define CELL_SIZE_MM 180.0f

/* Direction constants */
#define NORTH 0
#define EAST 1
#define SOUTH 2
#define WEST 3

/* Movement parameters */
#define BASE_SPEED 150
#define TURN_SPEED 100
#define MAX_SPEED 255
#define ENCODER_COUNTS_PER_CELL 1461		// Updated
#define ENCODER_COUNTS_PER_TURN 500

/* Sensor thresholds */
#define WALL_THRESHOLD_FRONT 10   //2000
#define WALL_THRESHOLD_SIDE 4      //1500
#define BATTERY_LOW_THRESHOLD 3000
#define IR_AMBIENT_THRESHOLD 500

/*To move the encoder while stopped*/
#define PWM_MIN_MOVE_LEFT   500
#define PWM_MIN_MOVE_RIGHT  500


/* Audio frequencies (Hz) */
#define TONE_STARTUP 440
#define TONE_CONFIRMATION 523
#define TONE_SUCCESS 659
#define TONE_ERROR 220



#define GYRO_CS_Pin Chip_Select_Pin
#define GYRO_CS_GPIO_Port Chip_Select_GPIO_Port


bool are_sensors_healthy(void);
/* Data structures */
typedef struct {
    int distance;
    bool visited;
    bool walls[4];  // N, E, S, W
    int visit_count;
} MazeCell;

typedef struct {
    int x, y;
    int direction;
    bool center_reached;
    bool returned_to_start;
    int exploration_steps;
} RobotState;

typedef struct {
    uint16_t battery;
    uint16_t front_right;
    uint16_t side_right;
    uint16_t side_left;
    uint16_t front_left;
    bool wall_front;
    bool wall_left;
    bool wall_right;
} SensorData;

typedef struct {
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t mag_x;
    int16_t mag_y;
    int16_t mag_z;
} GyroData;

typedef struct {
    int32_t left_count;
    int32_t right_count;
    int32_t left_total;
    int32_t right_total;
} EncoderData;
/* Units: distance in mm, velocity in mm/s, acceleration in mm/s^2, jerk in mm/s^3 */


/* Global variables */
extern MazeCell maze[MAZE_SIZE][MAZE_SIZE];
extern RobotState robot;
extern SensorData sensors;
extern GyroData gyro;
extern EncoderData encoders;
extern volatile uint8_t button_pressed;
extern volatile uint8_t start_flag;

/* External peripheral handles */
extern ADC_HandleTypeDef hadc1;
extern SPI_HandleTypeDef hspi2;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;  // Left encoder
extern TIM_HandleTypeDef htim3;   /* TIM3 – motor PWM */
extern TIM_HandleTypeDef htim4;  // Right encoder
extern UART_HandleTypeDef huart6;

/* Direction vectors */
extern const int dx[4];
extern const int dy[4];

/* Goal positions */
extern const int goal_x1, goal_y1, goal_x2, goal_y2;



/* Movement functions */
void start_encoders(void);
void move_forward(void);
void turn_left(void);
void turn_right(void);
void turn_around(void);
void stop_motors(void);
void break_motors(void);
void move_forward_distance(int target_counts);
void test_motors_individual(void);
void motor_set(uint8_t motor, bool forward, uint16_t duty);
void moveStraightPID(int base_pwm, bool left_forward, bool right_forward);
void moveStraightPID_Reset(void);
void moveStraightGyroPID(void);
void moveStraightGyroPID_Reset(void);
void turn_in_place_gyro(float angle_deg, int base_pwm, uint32_t timeout_ms);

/* logging _tests */
void run_gyro_step_test(int base_pwm, int delta_pwm, uint32_t step_delay_ms, uint32_t step_duration_ms, uint32_t sample_ms, uint32_t total_ms);
void run_encoder_step_test(int base_pwm, int delta_pwm, uint32_t step_delay_ms, uint32_t step_duration_ms, uint32_t sample_ms, uint32_t total_ms);
void run_gyro_turn_step_test(int base_pwm, int delta_pwm, uint32_t step_delay_ms, uint32_t step_duration_ms,uint32_t sample_ms, uint32_t total_ms);

/* Sensor functions */
void calibrate_sensors(void);
void update_sensors(void);
void update_walls(void);
uint16_t read_adc_channel(uint32_t channel);
bool are_sensors_healthy(void);
void adc_system_diagnostics(void);

static uint32_t dwt_cycles_per_us;
static inline void dwt_delay_us(uint32_t us);
void dwt_delay_init(uint32_t cpu_hz);

extern int point;

extern uint32_t FL_buff[5];
extern uint32_t FR_buff[5];
extern uint32_t L_buff[5];
extern uint32_t R_buff[5];

#define NOMINAL 1000L




uint16_t get_calibrated_threshold(int sensor_index);
bool is_sensor_calibration_valid(void);
void send_detailed_sensor_status(void);
void diagnostic_sensor_test();

/* Gyroscope functions */
bool mpu9250_init(void);
void mpu9250_read_gyro(void);
void mpu9250_read_accel(void);
void mpu9250_read_all(void);
uint8_t mpu9250_read_register(uint8_t reg);
void mpu9250_write_register(uint8_t reg, uint8_t data);
float mpu9250_get_gyro_z_dps(void);
bool mpu9250_detect_turn(void);
void mpu9250_calibrate_bias(void);
float mpu9250_get_gyro_z_compensated(void);
bool gyro_turn_to_angle(float target_angle);


/* Audio functions */
void play_startup_tone(void);
void play_confirmation_tone(void);
void play_success_tone(void);
void play_error_tone(void);
void play_tone(uint16_t frequency, uint16_t duration_ms);
void play_wall_beep(void);
void play_turn_beep(void);
void play_battery_warning(void);
void speaker_off(void);

/* Communication functions */
void send_bluetooth_message(const char* message);
void send_bluetooth_printf(const char* format, ...);
void send_maze_state(void);
void send_sensor_data(void);
void send_position_data(void);
void send_performance_metrics(void);
void send_battery_status(void);
void send_stats(void);

/* Utility functions */
void delay_ms(uint32_t ms);
int abs_int(int value);
float calculate_distance(int x1, int y1, int x2, int y2);
void led_status(uint8_t left_state, uint8_t right_state);
void led_sequence_startup(void);
void led_sequence_exploring(void);
void led_sequence_returning(void);
void led_sequence_complete(void);
void led_sequence_error(void);
int map_value(int value, int from_low, int from_high, int to_low, int to_high);
int constrain_int(int value, int min_val, int max_val);
const char* get_direction_name(int direction);
int manhattan_distance(int x1, int y1, int x2, int y2);
bool is_valid_coordinate(int x, int y);
float moving_average_filter(float new_value, float previous_average, int samples);
bool system_health_check(void);
void performance_start_timer(void);
void performance_end_timer(const char* operation_name);


/*GPIO External Interrupt Callback*/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
/*Verify adc gpio config and calibration*/
void verify_adc_gpio_configuration(void);
void calibrate_adc(void);


bool mpu9250_is_initialized(void);
void mpu9250_send_status(void);
void send_encoder_status(void);

void debug_encoder_setup(void);
void test_encoder_manual(void);
void test_encoder_rotation(void);


/* Encoder safe API */
void update_encoder_totals(void);
int32_t get_left_encoder_total(void);
int32_t get_right_encoder_total(void);
void reset_encoder_totals(void);

// Wall following mode enumeration
typedef enum {
    WALL_FOLLOW_NONE = 0,    // Gyro only, no wall following
    WALL_FOLLOW_LEFT = 1,    // Follow left wall only
    WALL_FOLLOW_RIGHT = 2,   // Follow right wall only
    WALL_FOLLOW_BOTH = 3     // Follow both walls (stay centered)
} WallFollowMode_t;

// New function declarations
void wallFollowPID_Reset(void);
void moveStraightSensorFusion(int base_pwm, WallFollowMode_t wall_mode);
void move_forward_distance_fusion(int target_counts, WallFollowMode_t wall_mode);



/* Function declarations */

/**
 * @brief Initialize maze for exploration
 * Sets up the maze data structure, boundary walls, center coordinates,
 * and robot starting position
 */
void initialize_maze_exploration(void);

/**
 * @brief Run flood fill algorithm
 * Calculates distance values from goal to all reachable cells
 * Updates the distance field in the maze structure
 */
void flood_fill_algorithm(void);

/**
 * @brief Get best direction to move based on current position
 * Uses flood fill values, visit counts, and direction priorities
 * to determine optimal next move
 *
 * @return Direction (NORTH, EAST, SOUTH, WEST)
 */
int get_best_direction(void);

/**
 * @brief Turn robot to face specified direction
 * Handles all turning logic (left, right, around)
 * Updates robot direction state
 *
 * @param target_direction Target direction to face
 */
void turn_to_direction(int target_direction);

/**
 * @brief Move forward one cell with precise control
 * Uses moveStraightGyroPID for accurate movement
 * Updates robot position and maze visit counts
 *
 * @return true if movement successful, false if blocked
 */
bool move_forward_one_cell(void);

/**
 * @brief Check if robot is at current goal position
 * Goal changes based on exploration phase (center vs start)
 *
 * @return true if at goal, false otherwise
 */
bool is_at_goal(void);

/**
 * @brief Update maze wall information from sensor readings
 * Reads sensors and updates wall information for current cell
 * Also updates opposite walls in adjacent cells
 */
void update_maze_walls(void);

/**
 * @brief Main maze exploration function
 * Implements the complete exploration algorithm:
 * - Sensor reading and wall detection
 * - Flood fill calculation
 * - Direction selection
 * - Movement execution
 * - Goal detection
 */
void explore_maze(void);

/**
 * @brief Run complete maze exploration sequence
 * Handles both phases:
 * 1. Exploration to center
 * 2. Return to start
 *
 * Provides comprehensive status reporting and telemetry
 */
void run_maze_exploration_sequence(void);

/**
 * @brief Check if exploration is complete
 * Returns true when both center_reached and returned_to_start are true
 *
 * @return true if exploration complete, false otherwise
 */
bool is_exploration_complete(void);

/**
 * @brief Calculate exploration efficiency
 * Compares actual steps taken to theoretical minimum
 *
 * @return Efficiency percentage (0-100%)
 */
float get_exploration_efficiency(void);

/**
 * @brief Get optimal distance for current maze knowledge
 * Calculates shortest known path distance
 *
 * @return Optimal distance in steps
 */
int get_optimal_distance(void);




#endif /* MICROMOUSE_H */
