

# Championship Micromouse – Function Reference \& Debug Guide

*Last Updated: August 19, 2025*

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


### 10. velocity_profile.c

- **`velocity_profile_init()`**
- Initializes velocity profile for smooth acceleration/deceleration
- Sets maximum velocity, acceleration parameters, and distance targets
- **`velocity_profile_update()`**
- Updates current velocity based on trapezoidal motion profile
- Handles acceleration, constant velocity, and deceleration phases
- **`velocity_profile_get_target_velocity()`**
- Returns current target velocity for motor control
- **`velocity_profile_is_complete()`**
- Checks if the velocity profile execution is finished

### 11. state_machine.h

- **State Management System**
- Defines main micromouse states: IDLE, CALIBRATE, SEARCH_RUN, RETURN_TO_START, PREPARE_FAST, FAST_RUN, COMPLETE, ERROR
- Movement execution states for various turn types and diagonal movements
- Search behavior states for exploration strategies
- Provides structured state management for complex micromouse operations


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
    - Monitor velocity profile updates and state transitions
    - Verify smooth acceleration/deceleration curves
    - Check state machine transitions between movement states
8. **IMU Verification**
    - Use `mpu9250_detect_turn()` prints to confirm gyro integration during turns

## 🛠 Tips \& Tricks

- **Set Breakpoints** in user-code regions in CubeIDE to step through algorithm decisions.
- **Use Serial Plotter** (e.g. PuTTY, CoolTerm) for live sensor and position graphs.
- **Scope Signals**: verify PWM on speaker and motors with oscilloscope.
- **Comment Out** modules (e.g. remove `speed_run()`) to isolate bugs.
- **Adjust Thresholds** in `micromouse.h` when false positives occur.

You now have a complete function reference and structured debugging plan to validate every subsystem—from hardware to championship algorithms—and ensure reliability in international competition. Good luck!

## 🔄 Recent Updates (August 19, 2025)

### New Modules Added:
- **`velocity_profile.h/.c`**: Advanced motion control with trapezoidal velocity profiling
- **`state_machine.h`**: Comprehensive state management system for robust operation

### Architecture Improvements:
- **Enhanced Movement Control**: Smooth acceleration/deceleration curves for improved accuracy
- **State-Based Design**: Structured state machine for complex behavior management
- **Advanced Motion Planning**: Velocity profiling system for professional-grade movement
- **Modular Expansion**: New modules support future advanced features like diagonal movement

### Updated File Structure:
- Velocity profiling integrated into movement system
- State machine architecture for better code organization
- Enhanced debugging capabilities with state monitoring
- Improved documentation reflecting current implementation

<div style="text-align: center">⁂</div>

[^1]: README-3.md

[^2]: image.jpg

