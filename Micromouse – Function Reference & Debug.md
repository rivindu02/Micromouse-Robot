# Micromouse – Function Reference & Debug Guide

What each source file does, the functions that matter, and a debug workflow.
For hardware, build steps and tuning values see [README.md](README.md).

Conventions used by the code:

- Maze coordinates: `(0,0)` is the start cell (bottom-left), `x` grows east,
  `y` grows north. `maze[x][y]`.
- Directions: `NORTH 0`, `EAST 1`, `SOUTH 2`, `WEST 3`; `dx[]`, `dy[]` in
  `main.c`. Right turn = `+1`, left turn = `+3` (mod 4).
- Encoder totals count **down** when driving forward, so distance travelled
  is `start - current`.
- Motor index `0` = left, `1` = right. PWM duty range used by code 0…1000
  (TIM3 period is 838, so anything above 838 is full duty).

## main.c

| Function | Notes |
|---|---|
| `main()` | HAL + clock (84 MHz) + peripheral init, `dwt_delay_init()`, `initialize_hardware_systems()`, `run_system_diagnostics()`, start-up tone and LEDs, then the button loop. |
| `initialize_hardware_systems()` | Starts TIM3 PWM channels, enables DRV8833 (STBY), checks ADC pins, inits MPU9250 and calibrates gyro bias (robot must be still), starts encoders, `initialize_maze_exploration()`. |
| `run_system_diagnostics()` | One sensor read, battery, gyro and encoder status, `system_health_check()`. Sets `system_ready = true` even when the check warns. |
| `send_periodic_status()` | Battery report every 10 s while idle, LED heartbeat every 2 s. |
| `HAL_GPIO_EXTI_Callback()` | 200 ms debounce. Left button → `button_pressed = 1`, right → `2`. |
| Main loop | Left: start exploration (blocking) or print metrics after a run. Right: status report, or emergency stop if a run is marked active. |

`USER CODE BEGIN 2` lists the bench tests you can call instead of running
the maze.

## micromouse.c – maze logic

| Function | Notes |
|---|---|
| `initialize_maze_exploration()` | Clears the maze, sets boundary walls, centre = cells (7,7)…(8,8), robot at (0,0) facing north. |
| `flood_fill_algorithm()` | BFS from the goal over known walls (unknown = open). Goal is the four centre cells until `center_reached`, then the start cell. |
| `get_best_direction()` | Open neighbour with the lowest distance; ties → least visited, then forward, right, left, back. Never returns a walled direction if any side is open. |
| `turn_to_direction(dir)` | Calls `turn_right()`, `turn_left()` or `turn_around()` and loads the tuned first-cell counts for the next move. |
| `move_forward_one_cell_truns()` | Drives one cell: post-turn counts if a turn just happened; else wall-follow (`move_forward_WF_distance`) when a side wall is seen, else gyro straight (`move_forward_distance`). First move from start uses 1200 counts. Updates position and `visit_count`. |
| `move_forward_one_cell()` | Plain full-cell gyro move; used only as fallback. |
| `update_maze_walls()` | `update_sensors()`, then writes front/left/right walls into the current and neighbouring cell. |
| `explore_maze()` | One phase: sense → flood fill → choose → turn → move, until goal or `MAZE_SIZE² × 3` moves. Sets `center_reached` / `returned_to_start`. |
| `run_maze_exploration_sequence()` | Phase 1 to centre, 1 s pause, phase 2 back to start, then coverage report. |
| `get_exploration_efficiency()`, `get_optimal_distance()` | Report values for `send_performance_metrics()`. |
| `dwt_delay_init()`, `dwt_delay_us()` | Busy-wait microsecond delay on the DWT cycle counter. |

## movement.c – motors, encoders, gyro motion

| Function | Notes |
|---|---|
| `start_encoders()`, `update_encoder_totals()`, `get_left/right_encoder_total()`, `reset_encoder_totals()` | 16-bit timer counters unwrapped into 32-bit totals. Right side is negated so both wheels share a sign. |
| `motor_set(motor, forward, duty)` | DRV8833 drive. Some branches use a GPIO write on a PWM pin (no effect) — intentional, see README *Hardware workarounds*. |
| `stop_motors()` | All four PWM compares to 0 (coast). |
| `break_motors()` | All inputs high for 20 ms (brake), then `stop_motors()`. |
| `moveStraightGyroPID_Reset()`, `moveStraightGyroPID()` | PID on gyro Z rate to hold heading, base 570. Call `mpu9250_read_gyro()` before each step. |
| `move_forward_distance(L, R)` | Gyro straight until either wheel reaches its count, then brake. |
| `move_forward_WF_distance(L, R)` | Same, with `wall_follow_step()` (base 600). |
| `turn_in_place_gyro(angle, base_pwm, timeout)` | +angle = left (CCW). Yaw integration, braking-law rate target, rate PID + feed-forward. `base_pwm` clamped to 60…400. Ends with `break_motors()` + 60 ms. |
| `turn_left()`, `turn_right()`, `turn_around()` | Front-wall align if a wall is ahead, else 957-count move; then ±90° (two −90° for around). |
| `moveStraightPID*()` | Encoder-rate PID, not used by exploration. |
| `test_motors_individual()`, `debug_encoder_setup()`, `test_encoder_manual()`, `test_encoder_rotation()`, `send_encoder_status()` | Bench helpers. |

## wall_handle.c – wall controllers

| Function | Notes |
|---|---|
| `align_front_to_wall(base_pwm, timeout)` | PI on front sensors: distance (mean − 80) and angle (FL − FR). Min drive 500 to beat dead zone; done after 150 ms within tolerance. |
| `wall_follow_reset_int(mode, base_pwm)`, `wall_follow_step()` | Side sensors → cm via `right_adc/right_dist` and `left_adc/left_dist` tables, PID to 2.5 cm from the visible wall (left preferred when both). |
| `fusion_reset()`, `fusion_set_heading_ref_to_current()`, `fusion_step(base)` | Wall PID blended with gyro heading hold. Not used in exploration. |

## sensors.c – IR and ADC

| Function | Notes |
|---|---|
| `update_sensors()` | Emitters off → read ambient; front emitters on 50 µs → read; side emitters on 80 µs → read; store `on − off`; compare with `WALL_THRESHOLD_*`. |
| `read_adc_channel(ch)` | One dummy + one real conversion, 480-cycle sampling. |
| `update_sensors4()` | Slower, averaged per-sensor variant (not used). |
| `calibrate_sensors()` | Measures ambient and differential response, computes thresholds (currently not applied, see `if (false)` in `update_sensors()`). |
| `adc_system_diagnostics()`, `diagnostic_sensor_test()`, `send_detailed_sensor_status()` | Bench reports. |

## gyro.c – MPU9250 over SPI2

| Function | Notes |
|---|---|
| `mpu9250_init()` | Reset, PLL clock, I2C off, 200 Hz sample rate, DLPF, ±500 °/s, ±4 g; checks WHO_AM_I = 0x70. |
| `mpu9250_calibrate_bias()` | 100 samples, 3 ms apart, average Z bias. |
| `mpu9250_read_gyro()` | Burst-reads gyro registers, stores Z only. |
| `mpu9250_get_gyro_z_compensated()` | `(raw − bias) / 65.5` °/s. |

## communication.c, audio.c, utils.c

- `send_bluetooth_message()` / `send_bluetooth_printf()`: blocking USART6
  transmit (256-byte format buffer). Exploration code uses the `TLM_INFO()` /
  `TLM_VERBOSE()` wrappers from `micromouse.h`.
- `send_maze_state()`: distance grid of visited cells, top row first.
- `play_tone()` and tone sequences block for their duration.
  `play_wall_beep()` / `play_turn_beep()` are silent unless `AUDIO_FEEDBACK=1`.
- `led_sequence_*()`: blocking LED patterns. `system_health_check()`: battery,
  sensors not all zero, gyro WHO_AM_I.

## logging_tests.c – identification tests

All take `(base_pwm, delta_pwm, step_delay_ms, step_duration_ms, sample_ms,
total_ms)` and stream CSV over Bluetooth:
`run_gyro_step_test()`, `run_encoder_step_test()`, `run_gyro_turn_step_test()`,
`run_wall_lateral_step_test()`, `run_wall_single_left_step_test()`.
`run_wf_relay_test()` runs a relay test on the wall loop. Fit the logs with the
Python scripts in `Core/` (see README).

## Debug workflow

1. **Serial link.** Open a terminal at 9600 baud. For full logs build with
   `-DTELEMETRY_LEVEL=2`. Remember that verbose output slows the run.
2. **Start-up check.** Look for `MPU9250 init OK`, the gyro bias line,
   `Encoders initialized` and a non-zero sensor line. A health-check warning
   does not stop the robot.
3. **Sensors.** Call `update_sensors()` + `send_sensor_data()` in a loop
   (`USER CODE BEGIN 2`), move a wall by hand, and check readings cross the
   thresholds in `micromouse.h:37`.
4. **Encoders.** `test_encoder_rotation()`; turn each wheel forward by hand —
   both totals must change in the same direction.
5. **Straight line.** Run `move_forward_distance(2460, 2460)` alone; adjust
   `LEFT/RIGHT_ENCODER_COUNTS_PER_CELL` if the robot does not end on the next
   cell centre.
6. **Turns.** Call `turn_left()` / `turn_right()` on their own and check the
   angle. Retune with `run_gyro_turn_step_test()` + `gyro_turns.py` if needed.
7. **Logic.** Change exploration code? Run the simulator first:
   `cd tools/sim && make && ./build/sim --random 200 1`. All mazes must end
   `ok=` with `crashes=0`.
8. **Breakpoints.** Useful spots: `update_maze_walls()`, `get_best_direction()`,
   `turn_in_place_gyro()` settle check, `HAL_GPIO_EXTI_Callback()`.

Common issues:

- Robot "sees" walls that are not there: raise `WALL_THRESHOLD_*`; check
  ambient IR light (readings are differential but strong sunlight saturates).
- Overshoot or undershoot on turns: check battery first, then the turn
  shaping constants near `movement.c:560`.
- Robot drifts on long straights with no walls: re-run gyro bias calibration
  with the robot completely still.
- CubeMX regeneration: keep code inside `USER CODE BEGIN/END` blocks.
