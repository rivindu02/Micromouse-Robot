

# Championship Micromouse – Function Reference \& Debug Guide

*Last Updated: August 29, 2025*

This document explains the purpose of each module and key functions, and provides a step-by-step debugging workflow.

## 📁 Module \& Function Overview

### 1. main.c

- **`main()`**
- Initializes HAL and system clock
- Calls `championship_micromouse_init()` to set up data structures
- Tests LEDs, speaker, Bluetooth
- Waits for **start** button, then runs exploration
- In infinite loop, debounces buttons to trigger new exploration or speed run
- **`SystemClock_Config()`**, **`MX_<Peripheral>_Init()`**
- Auto-generated HAL code to configure clocks, GPIO, ADC, SPI, TIM, UART


### 2. micromouse.c

- **`championship_micromouse_init()`**
- Clears maze array, sets boundary walls, resets visit counts
- Initializes MPU9250, resets robot state
- **`championship_update_walls()`**
- Reads IR sensors via `update_sensors()`
- Updates `maze[x][y].walls[N/E/S/W]` based on sensor booleans
- Increments visit_count and marks `visited = true`
- **`championship_flood_fill()`**
- Breadth-first fill: from center until start reached, then reverse
- Sets `maze[x][y].distance` = minimum steps to goal
- **`get_championship_direction()`**
- Examines four directions in priority order (straight, right, left, back)
- Prefers unvisited cells with lowest distance, then least-visited
- **`championship_exploration_with_analysis()`**
- Loop: update walls, flood fill, choose direction, turn, move forward
- On center reached, switches to return mode, resets visit counts
- After return, calls `execute_championship_path_analysis()`
- **`championship_speed_run()`**
- Executes high-speed optimal path run using explored maze data
- Uses same flood fill + direction selection for maximum speed


### 3. championship_analysis.c

- **`calculate_optimal_path_from_explored_areas()`**
- Flood fill using **only visited cells** to compute optimal distances
- Records `theoretical_minimum` steps from start to center
- **`analyze_championship_maze_performance()`**
- Counts visited cells vs total for **exploration efficiency**
- Compares `theoretical_minimum` vs `MAX_DISTANCE` to rate performance
- Sends Bluetooth ratings: ⭐ to ⭐⭐⭐⭐⭐
- **`print_championship_distance_map()`**
- Outputs a grid of distances (`-` = unvisited, `∞` = unreachable)
- **`visualize_championship_optimal_path()`**
- Traces and logs the optimal path step by step via Bluetooth
- **`show_championship_distances()`**
- Summarizes distribution of distance values for debugging


### 4. movement.c

- **`move_forward()`**, **`turn_left()`**, **`turn_right()`**, **`turn_around()`**
- Use encoder counts (TIM2/TIM4) for precise cell movement and 90° turns
- Incorporate safety checks for walls


### 5. sensors.c

- **`update_sensors()`**
- Reads ADC channels PA0, PA2–PA5 to populate `sensors.front_left`, etc.
- Sets `sensors.wall_front`, `.wall_left`, `.wall_right` based on thresholds


### 6. audio.c

- **`play_startup_tone()`, `play_confirmation_tone()`, `play_success_tone()`, `play_error_tone()`**
- Generate PWM signals on TIM1_CH3 to drive buzzer


### 7. communication.c

- **`send_bluetooth_message(const char*)`**
- Transmits null-terminated string over USART6
- **`send_bluetooth_printf(const char*, ...)`**
- `sprintf`-style formatted output for telemetry


### 8. gyro.c

- **`mpu9250_init()`**
- Configures MPU9250 registers via SPI2
- **`mpu9250_read_gyro()`, `mpu9250_read_accel()`**
- Read raw IMU data into `gyro` struct
- **`mpu9250_detect_turn()`**
- Uses `fabsf(gyro_z_dps) > 50.0f` to detect rotational motion


### 9. utils.c

- Helper routines: `map_value()`, `constrain_int()`, `performance_start_timer()`, etc.

### 10. logging_tests.c

- **`test_sensor_readings()`**
- Comprehensive sensor testing and calibration functions
- Logs sensor data for analysis and debugging
- **`performance_logging()`**
- Records system performance metrics during operation
- Helps optimize algorithm parameters for competition
- **`debug_maze_state()`**
- Outputs current maze knowledge and robot state
- Useful for understanding exploration progress and decision making

## 🔬 Data Analysis Tools

### Python Analysis Scripts (Core Directory)
- **`gyro_turns.py`**: Analyzes gyroscope data from CSV logs to optimize turn detection
- **`pid.py`**: PID controller analysis and parameter tuning based on performance data

### Performance Data Logs
- **`log_encoder.csv`**: Encoder performance and calibration data
- **`log_encoder2.csv`**: Extended encoder testing results  
- **`log_encoder3.csv`**: Additional encoder validation data
- **`log_gyro.csv`**: Gyroscope readings for drift analysis
- **`log_gyro_turns.csv`**: Turn detection accuracy measurements

These tools enable:
- **Real-time Data Collection**: Automated logging during robot operation
- **Offline Analysis**: Python scripts for detailed performance evaluation
- **Parameter Optimization**: Data-driven tuning of sensor thresholds and movement parameters
- **Competition Preparation**: Statistical analysis for consistent performance


## 🐞 Debugging Workflow

1. **Build \& Flash**
    - Confirm 0 errors/warnings in CubeIDE
    - Flash `.hex` to board via ST-Link
2. **Verify Basic Peripherals**
    - LEDs blink on startup
    - Buttons toggle `button_pressed` (set breakpoints in `HAL_GPIO_EXTI_Callback`)
    - Speaker emits startup tone
    - Bluetooth prints “Championship Micromouse Ready!”
3. **Sensor Tests**
    - Trigger wallFront, wallLeft, wallRight flags
    - Place wall at known distances, verify flags toggle
    - Log ADC raw values with `send_bluetooth_printf()`
4. **Motor \& Encoder**
    - Call `move_forward()` alone—verify one cell movement corresponds to encoder counts
    - Call `turn_left()` / `turn_right()`—verify 90° turns via IMU (`mpu9250_detect_turn()`)
5. **Maze Exploration**
    - Start exploration mode (left button)
    - Monitor Bluetooth logs:
        - “Walls updated at (x,y) F:L:R”
        - “Championship flood fill: N updates”
        - “Championship direction: D”
    - Confirm robot explores and returns
6. **Path Analysis**
    - Upon return, logs from `championship_analysis.c`:
        - Exploration efficiency %
        - Best path steps
        - Distance map
    - Verify printed map matches expected visited cells
7. **Speed Run**
    - After exploration, press left button again
    - System uses optimal path calculated during exploration
    - Monitor acceleration and movement timing
8. **Debug Functions**
    - Use functions in `logging_tests.c` for detailed debugging
    - Test individual sensors with `test_sensor_readings()`
    - Monitor performance with `performance_logging()`
    - Export data to CSV files for offline analysis
9. **Data Analysis**
    - Use `gyro_turns.py` to analyze turn performance from CSV logs
    - Run `pid.py` for PID controller optimization
    - Review CSV files for sensor calibration and performance trends
10. **IMU Verification**
    - Use `mpu9250_detect_turn()` prints to confirm gyro integration during turns

## 🛠 Tips \& Tricks

- **Set Breakpoints** in user-code regions in CubeIDE to step through algorithm decisions.
- **Use Serial Plotter** (e.g. PuTTY, CoolTerm) for live sensor and position graphs.
- **Scope Signals**: verify PWM on speaker and motors with oscilloscope.
- **Comment Out** modules (e.g. remove `speed_run()`) to isolate bugs.
- **Adjust Thresholds** in `micromouse.h` when false positives occur.
- **Data Analysis Workflow**: 
  - Export sensor/performance data to CSV during testing
  - Use Python scripts for offline analysis and parameter optimization
  - Import optimized parameters back into embedded code
- **Systematic Testing**: Use logging functions to validate each subsystem independently

You now have a complete function reference and structured debugging plan to validate every subsystem—from hardware to championship algorithms—and ensure reliability in international competition. Good luck!

## 🔄 Recent Updates (August 29, 2025)

### Current Implementation Status:
- **Core modules implemented**: All basic micromouse functionality is complete
- **`logging_tests.h/.c`**: Comprehensive debugging and testing functions
- **Hardware Integration**: Full STM32F411 peripheral support with optimized drivers

### Architecture Features:
- **Modular Design**: Clean separation between sensors, movement, algorithms, and communication
- **Professional Documentation**: Updated function reference and debugging guides
- **Robust Testing**: Integrated test functions for validation and debugging
- **Competition Ready**: All essential features implemented for micromouse competitions

### File Structure Updates:
- All documented modules are implemented and functional
- Logging and testing capabilities integrated throughout
- Enhanced debugging workflows for development and competition preparation
- Professional code organization following embedded systems best practices

<div style="text-align: center">⁂</div>

[^1]: README-3.md

[^2]: image.jpg

