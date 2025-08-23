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
#define ENCODER_COUNTS_PER_CELL 1000
#define ENCODER_COUNTS_PER_TURN 500

/* Sensor thresholds */
#define WALL_THRESHOLD_FRONT 2000
#define WALL_THRESHOLD_SIDE 1500
#define BATTERY_LOW_THRESHOLD 3000
#define IR_AMBIENT_THRESHOLD 500

/* Audio frequencies (Hz) */
#define TONE_STARTUP 440
#define TONE_CONFIRMATION 523
#define TONE_SUCCESS 659
#define TONE_ERROR 220

/* Pin definitions (matching your PCB) */
#define EMIT_FRONT_LEFT_Pin GPIO_PIN_9
#define EMIT_FRONT_LEFT_GPIO_Port GPIOB
#define EMIT_SIDE_LEFT_Pin GPIO_PIN_9
#define EMIT_SIDE_LEFT_GPIO_Port GPIOA
#define EMIT_SIDE_RIGHT_Pin GPIO_PIN_8
#define EMIT_SIDE_RIGHT_GPIO_Port GPIOB
#define EMIT_FRONT_RIGHT_Pin GPIO_PIN_8
#define EMIT_FRONT_RIGHT_GPIO_Port GPIOA

#define LED_LEFT_Pin GPIO_PIN_4
#define LED_LEFT_GPIO_Port GPIOB
#define LED_RIGHT_Pin GPIO_PIN_5
#define LED_RIGHT_GPIO_Port GPIOB

#define MOTOR_IN1_Pin GPIO_PIN_6
#define MOTOR_IN1_GPIO_Port GPIOA
#define MOTOR_IN2_Pin GPIO_PIN_7
#define MOTOR_IN2_GPIO_Port GPIOA
#define MOTOR_IN3_Pin GPIO_PIN_0
#define MOTOR_IN3_GPIO_Port GPIOB
#define MOTOR_IN4_Pin GPIO_PIN_1
#define MOTOR_IN4_GPIO_Port GPIOB

#define BTN_LEFT_Pin GPIO_PIN_1
#define BTN_LEFT_GPIO_Port GPIOA
#define BTN_RIGHT_Pin GPIO_PIN_10
#define BTN_RIGHT_GPIO_Port GPIOB

//#define GYRO_CS_Pin GPIO_PIN_2
//#define GYRO_CS_GPIO_Port GPIOD

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
typedef struct {
    // Profile parameters
    float max_velocity;        // mm/s - Maximum velocity
    float max_acceleration;    // mm/s² - Maximum acceleration
    float max_jerk;           // mm/s³ - Maximum jerk
    float target_distance;    // mm - Target distance to travel

    // Current state
    float current_position;   // mm - Current position
    float current_velocity;   // mm/s - Current velocity
    float current_acceleration; // mm/s² - Current acceleration
    float current_jerk;       // mm/s³ - Current jerk

    // Time parameters
    float t1, t2, t3, t4, t5, t6, t7; // 7-segment time durations
    float total_time;         // Total profile time
    uint32_t start_time;      // Profile start timestamp

    // Status
    bool profile_active;
    bool profile_complete;
    uint8_t current_segment;  // 1-7 for each S-curve segment
} SCurveProfile;



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

/* Core micromouse functions */
void championship_micromouse_init(void);
void initialize_championship_maze(void);
void championship_exploration_with_analysis(void);
void execute_championship_path_analysis(void);
void reset_championship_micromouse(void);
void championship_speed_run(void);

/* Core maze algorithm functions */
void championship_flood_fill(void);
int get_championship_direction(void);
void championship_update_walls(void);
void turn_to_direction(int target_dir);
bool championship_move_forward(void);
bool is_at_goal(void);
float get_exploration_efficiency(void);
int get_optimal_distance(void);

/* Movement functions */
void start_encoders(void);
void move_forward(void);
void turn_left(void);
void turn_right(void);
void turn_around(void);
void stop_motors(void);
void break_motors(void);
void move_forward_distance(int distance_mm);
void move_forward_adaptive_speed(float speed_multiplier);
void motor_set(uint16_t ch_pwm, GPIO_TypeDef *dirPort, uint16_t dirPin, bool forward, uint16_t duty);
void test_motors_individual(void);
void motor_set_fixed(uint8_t motor, bool forward, uint16_t duty);
void moveStraightPID(void);

/* Sensor functions */
void calibrate_sensors(void);
void update_sensors(void);
void update_walls(void);
void turn_on_emitters(void);
void turn_off_emitters(void);
uint16_t read_adc_channel(uint32_t channel);
bool are_sensors_healthy(void);
void adc_system_diagnostics(void);

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
void send_championship_stats(void);

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

/* Championship analysis functions */
void calculate_optimal_path_from_explored_areas(void);
void analyze_championship_maze_performance(void);
void print_championship_distance_map(void);
void visualize_championship_optimal_path(void);
void show_championship_distances(void);

/* Speed run functions */
void speed_run(void);
bool is_speed_run_ready(void);
int get_speed_run_optimal_distance(void);

/*GPIO External Interrupt Callback*/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
/*Verify adc gpio config and calibration*/
void verify_adc_gpio_configuration(void);
void calibrate_adc(void);

void move_forward_adaptive_speed(float speed_multiplier);
bool is_speed_run_ready(void);
int get_speed_run_optimal_distance(void);
bool mpu9250_is_initialized(void);
void mpu9250_send_status(void);
void send_encoder_status(void);


void debug_encoder_setup(void);
void test_encoder_manual(void);
void test_encoder_rotation(void);

/* Enhanced movement functions with Trapezoidal -curve */
void move_forward_with_profile(float distance_mm, float max_speed);
void move_forward_smooth(float distance_mm);

// Enhanced S-curve movement functions
void move_forward_scurve(float distance_mm, float speed_multiplier);
bool championship_move_forward_enhanced(void);
void set_heading_pid_gains(float kp, float ki, float kd);
void get_heading_pid_status(void);
void test_scurve_movement(void);

#endif /* MICROMOUSE_H */
