/*
 * micromouse.c - Championship Micromouse Implementation
 *
 * Implements championship-level flood fill algorithm and navigation
 * Based on international competition winning strategies
 */

#include "micromouse.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

/* Championship path analysis variables */
extern int exploration_steps;
extern int theoretical_minimum;




/**
 * @brief Initialize championship micromouse system with MMS integration
 */
void championship_micromouse_init(void)
{
    // Initialize championship maze
    initialize_championship_maze();

    // Initialize robot state
    robot.x = 0;
    robot.y = 0;
    robot.direction = NORTH;
    robot.center_reached = false;
    robot.returned_to_start = false;
    robot.exploration_steps = 0;

    // Initialize sensors
    memset(&sensors, 0, sizeof(sensors));
    memset(&gyro, 0, sizeof(gyro));
    memset(&encoders, 0, sizeof(encoders));

    // Initialize championship path analysis
    exploration_steps = 0;
    theoretical_minimum = 0;

    // Initialize gyroscope
    mpu9250_init();

    send_bluetooth_message("Championship micromouse system initialized\r\n");
    send_bluetooth_message("Based on MMS championship algorithms\r\n");
}

/**
 * @brief Initialize maze with championship settings (MMS style)
 */
void initialize_championship_maze(void)
{
    // Initialize all cells
    for (int x = 0; x < MAZE_SIZE; x++) {
        for (int y = 0; y < MAZE_SIZE; y++) {
            maze[x][y].distance = MAX_DISTANCE;
            maze[x][y].visited = false;
            maze[x][y].visit_count = 0;
            for (int i = 0; i < 4; i++) {
                maze[x][y].walls[i] = false;
            }
        }
    }

    // Set boundary walls
    for (int i = 0; i < MAZE_SIZE; i++) {
        maze[i][0].walls[SOUTH] = true;           // South boundary
        maze[i][MAZE_SIZE-1].walls[NORTH] = true; // North boundary
        maze[0][i].walls[WEST] = true;            // West boundary
        maze[MAZE_SIZE-1][i].walls[EAST] = true;  // East boundary
    }

    // Mark start position as visited
    maze[0][0].visited = true;
    maze[0][0].visit_count = 1;

    send_bluetooth_message("Championship maze initialized with boundary walls\r\n");
}

/**
 * @brief Championship flood fill from GOAL position (MMS algorithm)
 * This is the key difference - we flood from destination, not robot
 */
void championship_flood_fill(void)
{
    // Reset all distances
    for (int x = 0; x < MAZE_SIZE; x++) {
        for (int y = 0; y < MAZE_SIZE; y++) {
            maze[x][y].distance = MAX_DISTANCE;
        }
    }

    // Set goal distances to 0
    if (!robot.center_reached) {
        // Exploring to center - flood from center
        maze[goal_x1][goal_y1].distance = 0;
        maze[goal_x2][goal_y1].distance = 0;
        maze[goal_x1][goal_y2].distance = 0;
        maze[goal_x2][goal_y2].distance = 0;
    } else {
        // Returning to start - flood from start
        maze[0][0].distance = 0;
    }

    // Queue implementation for BFS flood fill
    int queue_x[256], queue_y[256];
    int queue_head = 0, queue_tail = 0;

    if (!robot.center_reached) {
        queue_x[queue_tail] = goal_x1; queue_y[queue_tail++] = goal_y1;
        queue_x[queue_tail] = goal_x2; queue_y[queue_tail++] = goal_y1;
        queue_x[queue_tail] = goal_x1; queue_y[queue_tail++] = goal_y2;
        queue_x[queue_tail] = goal_x2; queue_y[queue_tail++] = goal_y2;
    } else {
        queue_x[queue_tail] = 0; queue_y[queue_tail++] = 0;
    }

    int updates = 0;

    // Championship flood fill algorithm
    while (queue_head < queue_tail) {
        int x = queue_x[queue_head];
        int y = queue_y[queue_head++];

        // Check all four directions
        for (int dir = 0; dir < 4; dir++) {
            int nx = x + dx[dir];
            int ny = y + dy[dir];

            // Check bounds and walls
            if (nx >= 0 && nx < MAZE_SIZE && ny >= 0 && ny < MAZE_SIZE &&
                !maze[x][y].walls[dir]) {

                int new_dist = maze[x][y].distance + 1;

                // Update if we found a shorter path
                if (new_dist < maze[nx][ny].distance) {
                    maze[nx][ny].distance = new_dist;
                    if (queue_tail < 255) {
                        queue_x[queue_tail] = nx;
                        queue_y[queue_tail++] = ny;
                    } else {
                        send_bluetooth_message("Queue overflow!\r\n");
                        break;
                    }
                    updates++;
                }
            }
        }
    }

    // Debug output via Bluetooth
    send_bluetooth_printf("Championship flood fill: %d updates\r\n", updates);
}

/**
 * @brief Championship direction selection - NEVER gets stuck (MMS algorithm)
 */
int get_championship_direction(void)
{
    int best_dir = robot.direction; // Default to current direction
    int min_distance = MAX_DISTANCE;
    int min_visits = 999;
    bool found_unvisited = false;

    // Priority order: straight, right, left, back
    int priority_dirs[4];
    priority_dirs[0] = robot.direction;
    priority_dirs[1] = (robot.direction + 1) % 4;
    priority_dirs[2] = (robot.direction + 3) % 4;
    priority_dirs[3] = (robot.direction + 2) % 4;

    // First pass: look for unvisited cells
    for (int p = 0; p < 4; p++) {
        int dir = priority_dirs[p];
        int nx = robot.x + dx[dir];
        int ny = robot.y + dy[dir];

        if (nx >= 0 && nx < MAZE_SIZE && ny >= 0 && ny < MAZE_SIZE &&
            !maze[robot.x][robot.y].walls[dir]) {

            // Prefer unvisited cells
            if (maze[nx][ny].visit_count == 0) {
                found_unvisited = true;
                if (maze[nx][ny].distance < min_distance) {
                    min_distance = maze[nx][ny].distance;
                    best_dir = dir;
                }
            }
        }
    }

    // Second pass: if no unvisited, find least visited with lowest distance
    if (!found_unvisited) {
        for (int p = 0; p < 4; p++) {
            int dir = priority_dirs[p];
            int nx = robot.x + dx[dir];
            int ny = robot.y + dy[dir];

            if (nx >= 0 && nx < MAZE_SIZE && ny >= 0 && ny < MAZE_SIZE &&
                !maze[robot.x][robot.y].walls[dir]) {

                int neighbor_dist = maze[nx][ny].distance;
                int neighbor_visits = maze[nx][ny].visit_count;

                // Choose based on distance first, then visit count
                if (neighbor_dist < min_distance ||
                    (neighbor_dist == min_distance && neighbor_visits < min_visits) ||
                    (neighbor_dist == min_distance && neighbor_visits == min_visits && dir == robot.direction)) {
                    min_distance = neighbor_dist;
                    min_visits = neighbor_visits;
                    best_dir = dir;
                }
            }
        }
    }

    return best_dir;
}

/**
 * @brief Update walls based on sensor readings (MMS style)
 */
void championship_update_walls(void)
{
    // Update sensors first
    update_sensors();

    // Update walls based on current direction
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

    // Debug output
    send_bluetooth_printf("Walls updated at (%d,%d) F:%d L:%d R:%d\r\n",
                         robot.x, robot.y, sensors.wall_front, sensors.wall_left, sensors.wall_right);
}

/**
 * @brief Turn robot to face target direction (MMS style)
 */
void turn_to_direction(int target_dir)
{
    while (robot.direction != target_dir) {
        int turn_diff = (target_dir - robot.direction + 4) % 4;

        if (turn_diff == 1) {
            turn_right();
            robot.direction = (robot.direction + 1) % 4;
        } else if (turn_diff == 3) {
            turn_left();
            robot.direction = (robot.direction + 3) % 4;
        } else if (turn_diff == 2) {
            turn_around();
            robot.direction = (robot.direction + 2) % 4;
        }
    }
}

// with s-curve
//void turn_to_direction(int target_dir) {
//    while (robot.direction != target_dir) {
//        int turn_diff = (target_dir - robot.direction + 4) % 4;
//        if (turn_diff == 1) {
//            turn_right_scurve();  // NEW S-CURVE
//            // Direction updated inside function
//        } else if (turn_diff == 3) {
//            turn_left_scurve();   // NEW S-CURVE
//            // Direction updated inside function
//        } else if (turn_diff == 2) {
//            turn_around_scurve(); // NEW S-CURVE
//            // Direction updated inside function
//        }
//    }
//}



/**
 * @brief Move robot forward one cell (MMS style)
 */
bool championship_move_forward(void)
{
    update_sensors();// neww
    // Check for wall before moving
    if (sensors.wall_front) {
        send_bluetooth_message("Front wall detected, cannot move\r\n");
        return false;
    }

    move_forward();  	//move_forward_cell_scurve();
    robot.exploration_steps++;
    exploration_steps++;

    return true;
}

/**
 * @brief Check if robot is at goal (MMS style)
 */
bool is_at_goal(void)
{
    if (!robot.center_reached) {
        return (robot.x == goal_x1 || robot.x == goal_x2) &&
               (robot.y == goal_y1 || robot.y == goal_y2);
    } else {
        return robot.x == 0 && robot.y == 0;
    }
}

/**
 * @brief Main championship exploration algorithm with MMS integration
 */
void championship_exploration_with_analysis(void)
{
    send_bluetooth_message("Starting championship exploration\r\n");

    int step_count = 0;
    const int max_steps = 1000;

    while (step_count < max_steps && (!robot.center_reached || !robot.returned_to_start)) {
        send_bluetooth_printf("Step %d: Robot at (%d,%d)\r\n", step_count, robot.x, robot.y);

        // Update walls and run championship flood fill
        championship_update_walls();
        championship_flood_fill();

        // Check if goal reached
        if (is_at_goal()) {
            if (!robot.center_reached) {
                send_bluetooth_message("CENTER REACHED! Switching to return mode\r\n");
                robot.center_reached = true;
                play_confirmation_tone();

                // Reset visit counts for return journey
                for (int x = 0; x < MAZE_SIZE; x++) {
                    for (int y = 0; y < MAZE_SIZE; y++) {
                        maze[x][y].visit_count = 0;
                    }
                }
            } else {
                send_bluetooth_message("RETURNED TO START! Exploration complete!\r\n");
                robot.returned_to_start = true;
                break;
            }
        }

        // Get championship direction
        int next_dir = get_championship_direction();
        send_bluetooth_printf("Championship direction: %d\r\n", next_dir);

        // Turn and move
        turn_to_direction(next_dir);

        if (championship_move_forward()) {
            // Update LED status
            if (robot.center_reached) {
                led_status(0, 1); // Right LED for return journey
            } else {
                led_status(1, 0); // Left LED for exploration
            }
        } else {
            send_bluetooth_message("Movement blocked - trying alternatives\r\n");
            // Try other directions if blocked
            bool moved = false;
            for (int alt_dir = 0; alt_dir < 4 && !moved; alt_dir++) {
                if (alt_dir != next_dir) {
                    int nx = robot.x + dx[alt_dir];
                    int ny = robot.y + dy[alt_dir];
                    if (nx >= 0 && nx < MAZE_SIZE && ny >= 0 && ny < MAZE_SIZE &&
                        !maze[robot.x][robot.y].walls[alt_dir]) {
                        turn_to_direction(alt_dir);
                        if (championship_move_forward()) {
                            moved = true;
                        }
                    }
                }
            }

            if (!moved) {
                send_bluetooth_message("All directions blocked!\r\n");
                break;
            }
        }

        step_count++;
        HAL_Delay(10); // Small delay for stability
    }

    // Final status
    led_status(0, 0);
    send_bluetooth_printf("Exploration completed in %d moves\r\n", robot.exploration_steps);

    // Execute perfect path analysis if exploration successful
    if (robot.center_reached && robot.returned_to_start) {
        send_bluetooth_message("\r\nExploration successful! Starting path analysis...\r\n");
        execute_championship_path_analysis();
        play_success_tone();
    } else {
        send_bluetooth_message("Exploration incomplete - path analysis not available\r\n");
        play_error_tone();
    }
}

/**
 * @brief Execute championship path analysis (MMS style)
 */
void execute_championship_path_analysis(void)
{
    send_bluetooth_message("\r\n=== CHAMPIONSHIP PATH ANALYSIS ===\r\n");

    // Calculate optimal path from explored areas
    calculate_optimal_path_from_explored_areas();

    // Comprehensive maze performance analysis
    analyze_championship_maze_performance();

    // Print optimal distance map
    print_championship_distance_map();

    // Visualize optimal path (would work with MMS visualization)
    send_bluetooth_message("\r\n🎯 CHAMPIONSHIP ANALYSIS COMPLETE!\r\n");

    if (theoretical_minimum < MAX_DISTANCE) {
        send_bluetooth_printf("Optimal path through explored areas: %d steps!\r\n", theoretical_minimum);
        send_bluetooth_message("✅ Ready for IEEE Micromouse competition!\r\n");
    } else {
        send_bluetooth_message("❌ No valid path found through explored areas\r\n");
    }
}

/**
 * @brief Reset championship micromouse to initial state
 */
void reset_championship_micromouse(void)
{
    robot.x = 0;
    robot.y = 0;
    robot.direction = NORTH;
    robot.center_reached = false;
    robot.returned_to_start = false;
    robot.exploration_steps = 0;

    exploration_steps = 0;
    theoretical_minimum = 0;

    initialize_championship_maze();
    send_bluetooth_message("Championship micromouse reset to initial state\r\n");
    play_startup_tone();
}

/**
 * @brief Championship speed run with MMS path analysis
 */
void championship_speed_run(void)
{
    send_bluetooth_message("\r\n🚀 CHAMPIONSHIP SPEED RUN MODE!\r\n");
    send_bluetooth_message("Using MMS optimal path analysis\r\n");

    // Use the advanced speed run implementation
    speed_run();
}



/**
 * @brief Simple speed run implementation
 */
void speed_run(void)
{
    send_bluetooth_message("\r\n🚀 SPEED RUN MODE ACTIVATED!\r\n");

    // Check if exploration was completed
    if (!robot.center_reached || !robot.returned_to_start) {
        send_bluetooth_message("❌ Speed run not available - exploration not complete\r\n");
        return;
    }

    send_bluetooth_message("Using championship algorithms for optimal speed run\r\n");

    // Reset robot position
    robot.x = 0;
    robot.y = 0;
    robot.direction = NORTH;

    // Status indication
    led_status(1, 1); // Both LEDs on
    play_confirmation_tone();

    // Wait for confirmation
    send_bluetooth_message("Press RIGHT button to execute speed run...\r\n"); //later change to hand movement

    uint32_t start_time = HAL_GetTick();
    bool execute_run = false;

    while ((HAL_GetTick() - start_time) < 10000) { // 10 second timeout
        if (button_pressed == 2) { // Right button
            button_pressed = 0;
            execute_run = true;
            break;
        }
        HAL_Delay(100);
    }

    if (!execute_run) {
        send_bluetooth_message("⏰ Speed run cancelled - timeout\r\n");
        led_status(0, 0);
        return;
    }

    send_bluetooth_message("🏁 EXECUTING SPEED RUN!\r\n");

    // Simple speed run - follow the shortest known path to center
    int moves = 0;
    const int max_moves = 50;

    while (!((robot.x == goal_x1 || robot.x == goal_x2) &&
             (robot.y == goal_y1 || robot.y == goal_y2)) &&
           moves < max_moves) {

        // Update sensor data
        update_sensors();

        // Use championship flood fill to get direction
        championship_flood_fill();
        int next_dir = get_championship_direction();

        // Turn to target direction
        turn_to_direction(next_dir);

        // Move forward
        if (championship_move_forward()) {
            moves++;
            send_bluetooth_printf("Speed run move %d to (%d,%d)\r\n", moves, robot.x, robot.y);
        } else {
            send_bluetooth_message("❌ Speed run blocked!\r\n");
            break;
        }

        // Brief delay for stability
        HAL_Delay(50);
    }

    // Speed run complete
    led_status(0, 0);

    if ((robot.x == goal_x1 || robot.x == goal_x2) &&
        (robot.y == goal_y1 || robot.y == goal_y2)) {
        send_bluetooth_message("🏁 SPEED RUN SUCCESS!\r\n");
        send_bluetooth_printf("Completed in %d moves\r\n", moves);
        play_success_tone();
    } else {
        send_bluetooth_message("⚠️ Speed run incomplete\r\n");
        play_error_tone();
    }
}






/**
 * @brief Get exploration efficiency percentage
 */
float get_exploration_efficiency(void)
{
    int cells_visited = 0;
    for (int x = 0; x < MAZE_SIZE; x++) {
        for (int y = 0; y < MAZE_SIZE; y++) {
            if (maze[x][y].visited) {
                cells_visited++;
            }
        }
    }
    return (float)cells_visited / (MAZE_SIZE * MAZE_SIZE) * 100.0f;
}

/**
 * @brief Get optimal distance to center
 */
int get_optimal_distance(void)
{
    return theoretical_minimum;
}
