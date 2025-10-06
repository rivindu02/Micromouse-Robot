/*
 * micromouse.c - Maze Exploration and Solving Implementation
 *
 * Based on the simulation logic from Main.c, this file implements the complete
 * maze exploration algorithm using flood fill for the embedded micromouse.
 *
 * Features:
 * - Variable maze size support
 * - Flood fill algorithm for optimal path finding
 * - Center-reached and return-to-start logic
 * - Integration with moveStraightGyroPID for movement
 * - Real-time telemetry and debugging
 */

#include "micromouse.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdint.h>  // needed for int8_t



/* Maze exploration state variables */
static int maze_center_x1, maze_center_y1, maze_center_x2, maze_center_y2;
static int exploration_completed = 0;
static int optimal_path_calculated = 0;

/* BFS Queue Implementation for Flood Fill */
#define QUEUE_MAX_SIZE (MAZE_SIZE * MAZE_SIZE + 16)
typedef struct {
    int x, y;
} Position;


// ===== Tri-state wall map =====
typedef enum { WALL_UNKNOWN = -1, WALL_OPEN = 0, WALL_CLOSED = 1 } WallState;
static int8_t wall_state[MAZE_SIZE][MAZE_SIZE][4];

// dx/dy must already exist; if not, you need the usual:
///* static const int dx[4] = {0, 1, 0, -1};   // N,E,S,W
//   static const int dy[4] = {1, 0, -1, 0}; */

static inline void set_edge_state(int x, int y, int dir, WallState s){
    wall_state[x][y][dir] = s;
    int nx = x + dx[dir], ny = y + dy[dir];
    if (nx>=0 && nx<MAZE_SIZE && ny>=0 && ny<MAZE_SIZE)
        wall_state[nx][ny][(dir+2)%4] = s;
}

typedef struct {
    Position queue[QUEUE_MAX_SIZE];
    int head, tail;
} BFSQueue;

static BFSQueue bfs_queue;

/* Queue Operations */
static void queue_init(BFSQueue* q) {
    q->head = q->tail = 0;
}

static int queue_empty(BFSQueue* q) {
    return q->head == q->tail;
}

static void queue_push(BFSQueue* q, Position pos) {
    if (q->tail < QUEUE_MAX_SIZE) {
        q->queue[q->tail++] = pos;
    }
}

static Position queue_pop(BFSQueue* q) {
    return q->queue[q->head++];
}
static uint32_t dwt_cycles_per_us;

void dwt_delay_us(uint32_t us) {
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * dwt_cycles_per_us;
    while ((DWT->CYCCNT - start) < ticks) { __NOP(); }
}
void dwt_delay_init(uint32_t cpu_hz) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    dwt_cycles_per_us = cpu_hz / 1000000U; // e.g., 84 for 84 MHz
}




/**
 * @brief Initialize maze for exploration
 */
void initialize_maze_exploration(void) {
    send_bluetooth_message("\r\n=== INITIALIZING MAZE EXPLORATION ===\r\n");

    // Initialize maze structure
    for (int x = 0; x < MAZE_SIZE; x++) {
        for (int y = 0; y < MAZE_SIZE; y++) {
            maze[x][y].distance = MAX_DISTANCE;
            maze[x][y].visited = false;
            maze[x][y].visit_count = 0;

            // Initialize all walls as unknown (false)
            for (int dir = 0; dir < 4; dir++) {
                //maze[x][y].walls[dir] = WALL_UNKNOWN; // was: false
                maze[x][y].walls[dir]=false; // keep writing for legacy uses, but ignore in BFS
            }
        }
    }

    // Everything starts UNKNOWN
    for (int x = 0; x < MAZE_SIZE; x++) {
        for (int y = 0; y < MAZE_SIZE; y++) {
            for (int d = 0; d < 4; d++) {
                wall_state[x][y][d] = WALL_UNKNOWN;
            }
        }
    }


    // Set boundary walls
//    for (int i = 0; i < MAZE_SIZE; i++) {
//        maze[i][0].walls[SOUTH] = true;                    // Bottom boundary
//        maze[i][MAZE_SIZE-1].walls[NORTH] = true;          // Top boundary
//        maze[0][i].walls[WEST] = true;                     // Left boundary
//        maze[MAZE_SIZE-1][i].walls[EAST] = true;           // Right boundary
//    }

    // Set boundary walls (tri-state version)
    for (int i = 0; i < MAZE_SIZE; i++) {
        wall_state[i][0][SOUTH] = WALL_CLOSED;             // Bottom boundary
        wall_state[i][MAZE_SIZE-1][NORTH] = WALL_CLOSED;   // Top boundary
        wall_state[0][i][WEST] = WALL_CLOSED;              // Left boundary
        wall_state[MAZE_SIZE-1][i][EAST] = WALL_CLOSED;    // Right boundary
    }



    // Set maze center coordinates
    maze_center_x1 = MAZE_SIZE / 2 - 1;
    maze_center_y1 = MAZE_SIZE / 2 - 1;
    maze_center_x2 = MAZE_SIZE / 2;
    maze_center_y2 = MAZE_SIZE / 2;

    // Initialize robot state
    robot.x = 0;
    robot.y = 0;
    robot.direction = NORTH;
    robot.center_reached = false;
    robot.returned_to_start = false;
    robot.exploration_steps = 0;

    // Mark starting position
    maze[0][0].visited = true;
    maze[0][0].visit_count = 1;

    // Reset exploration flags
    exploration_completed = 0;
    optimal_path_calculated = 0;

    send_bluetooth_printf("Maze Size: %dx%d\r\n", MAZE_SIZE, MAZE_SIZE);
    send_bluetooth_printf("Center: (%d,%d) to (%d,%d)\r\n",
                         maze_center_x1, maze_center_y1, maze_center_x2, maze_center_y2);
    send_bluetooth_message("Maze exploration initialized successfully!\r\n");
    send_bluetooth_message("==========================================\r\n");
}

// Call this at the start of flood_fill_algorithm() in EXPLORE mode
//static inline void seed_goal_center(void){
//    int cx0 = (MAZE_SIZE/2)-1;
//    int cy0 = (MAZE_SIZE/2)-1;
//    int gxs[4] = { cx0, cx0+1, cx0,   cx0+1 };
//    int gys[4] = { cy0, cy0,   cy0+1, cy0+1 };
//
//    queue_clear();
//    for (int i=0;i<4;i++){
//        int gx = gxs[i], gy = gys[i];
//        maze[gx][gy].distance = 0;
//        queue_push(gx, gy);
//    }
//}



/**
 * @brief Flood fill algorithm implementation
 */
void flood_fill_algorithm(void) {
    // Initialize all distances to MAX_DISTANCE
    for (int x = 0; x < MAZE_SIZE; x++) {
        for (int y = 0; y < MAZE_SIZE; y++) {
            maze[x][y].distance = MAX_DISTANCE;
        }
    }
    //seed_goal_center();


    // Initialize queue
    queue_init(&bfs_queue);

    // Set goal distances and add to queue
    if (!robot.center_reached) {
        // Heading to center
        maze[maze_center_x1][maze_center_y1].distance = 0;
        maze[maze_center_x2][maze_center_y1].distance = 0;
        maze[maze_center_x1][maze_center_y2].distance = 0;
        maze[maze_center_x2][maze_center_y2].distance = 0;

        queue_push(&bfs_queue, (Position){maze_center_x1, maze_center_y1});
        queue_push(&bfs_queue, (Position){maze_center_x2, maze_center_y1});
        queue_push(&bfs_queue, (Position){maze_center_x1, maze_center_y2});
        queue_push(&bfs_queue, (Position){maze_center_x2, maze_center_y2});
    } else {
        // Returning to start
        maze[0][0].distance = 0;
        queue_push(&bfs_queue, (Position){0, 0});
    }

    // Flood fill propagation
    int updates = 0;
    while (!queue_empty(&bfs_queue)) {
        Position current = queue_pop(&bfs_queue);
        int x = current.x;
        int y = current.y;

        // Check all four directions
        for (int dir = 0; dir < 4; dir++) {// UNKNOWN and CLOSED both block
        	if (wall_state[x][y][dir] == WALL_CLOSED) continue;


            int nx = x + dx[dir];
            int ny = y + dy[dir];

            // Check bounds
            if (nx < 0 || nx >= MAZE_SIZE || ny < 0 || ny >= MAZE_SIZE) continue;

            int new_distance = maze[x][y].distance + 1;
            if (new_distance < maze[nx][ny].distance) {
                maze[nx][ny].distance = new_distance;
                queue_push(&bfs_queue, (Position){nx, ny});
                updates++;
            }
        }
    }

    send_bluetooth_printf("Flood fill complete: %d updates\r\n", updates);
}

/**
 * @brief Get best direction to move based on flood fill values
 */
int get_best_direction(void) {
    int best_dir = robot.direction;
    int min_distance = MAX_DISTANCE;
    int min_visits = 999;
    bool found_unvisited = false;

    // Direction priority: forward, right, left, backward
    int priority[4];
    priority[0] = robot.direction;                    // Forward
    priority[1] = (robot.direction + 1) % 4;         // Right
    priority[2] = (robot.direction + 3) % 4;         // Left
    priority[3] = (robot.direction + 2) % 4;         // Backward

    // First pass: prioritize unvisited cells
    for (int p = 0; p < 4; p++) {
        int dir = priority[p];

        // Check if there's a wall in this direction
        if (wall_state[robot.x][robot.y][dir] == WALL_CLOSED) continue;

        int nx = robot.x + dx[dir];
        int ny = robot.y + dy[dir];

        // Check bounds
        if (nx < 0 || nx >= MAZE_SIZE || ny < 0 || ny >= MAZE_SIZE) continue;

        // Prioritize unvisited cells
        if (maze[nx][ny].visit_count == 0) {
            found_unvisited = true;
            if (maze[nx][ny].distance < min_distance) {
                min_distance = maze[nx][ny].distance;
                best_dir = dir;
            }
        }
    }

    // Second pass: if no unvisited cells, choose based on distance and visit count
    if (!found_unvisited) {
        for (int p = 0; p < 4; p++) {
            int dir = priority[p];

            //if (wall_state[robot.x][robot.y][dir] != WALL_OPEN) continue;
            if (wall_state[robot.x][robot.y][dir] == WALL_CLOSED) continue;


            int nx = robot.x + dx[dir];
            int ny = robot.y + dy[dir];

            if (nx < 0 || nx >= MAZE_SIZE || ny < 0 || ny >= MAZE_SIZE) continue;

            int nd = maze[nx][ny].distance;
            int nv = maze[nx][ny].visit_count;

            // Choose cell with minimum distance, then minimum visits, then prefer forward
            if (nd < min_distance ||
                (nd == min_distance && nv < min_visits) ||
                (nd == min_distance && nv == min_visits && dir == robot.direction)) {
                min_distance = nd;
                min_visits = nv;
                best_dir = dir;
            }
        }
    }

    return best_dir;
}
int l=0;
int r=0;

/**
 * @brief Turn robot to face the specified direction
 */
void turn_to_direction(int target_direction) {
    int current_dir = robot.direction;
    int turn_diff = (target_direction - current_dir + 4) % 4;

    switch (turn_diff) {
        case 0:
            // Already facing correct direction
            break;
        case 1:
            // Turn right (90 degrees clockwise)
            send_bluetooth_message("Turning RIGHT...\r\n");
            turn_right();
            l=1549;
            r=1537;
            play_turn_beep();
            break;
        case 2:
            // Turn around (180 degrees)
            send_bluetooth_message("Turning AROUND...\r\n");
            turn_around();
            l=1530;
            r=1562;
            play_turn_beep();
            break;
        case 3:
            // Turn left (90 degrees counter-clockwise)
            send_bluetooth_message("Turning LEFT...\r\n");
            turn_left();
            l=1330;
            r=1352;
            play_turn_beep();
            break;
    }

    robot.direction = target_direction;
}

/**
 * @brief Move forward one cell with precise control
 */
bool move_forward_one_cell(void) {
    send_bluetooth_printf("Moving forward from (%d,%d) to ", robot.x, robot.y);

    int from_x = robot.x;
    int from_y = robot.y;
    int move_dir = robot.direction;
    // Calculate new position
    int new_x = robot.x + dx[robot.direction];
    int new_y = robot.y + dy[robot.direction];

    // Check bounds
    if (new_x < 0 || new_x >= MAZE_SIZE || new_y < 0 || new_y >= MAZE_SIZE) {
        send_bluetooth_message("BLOCKED - Out of bounds!\r\n");
        return false;
    }

    send_bluetooth_printf("(%d,%d)\r\n", new_x, new_y);


    // Use precise encoder-based movement
    move_forward_distance(LEFT_ENCODER_COUNTS_PER_CELL ,RIGHT_ENCODER_COUNTS_PER_CELL);

    // Update robot position
    robot.x = new_x;
    robot.y = new_y;
    robot.exploration_steps++;
    set_edge_state(from_x, from_y, move_dir, WALL_OPEN);

    // Mark cell as visited
    maze[robot.x][robot.y].visited = true;
    maze[robot.x][robot.y].visit_count++;

    return true;
}
int flag=1;

bool move_forward_one_cell_truns(void){
    send_bluetooth_printf("Moving forward from (%d,%d) to ", robot.x, robot.y);
    int from_x = robot.x;
    int from_y = robot.y;
    int move_dir = robot.direction;

    // Calculate new position
    int new_x = robot.x + dx[robot.direction];
    int new_y = robot.y + dy[robot.direction];

    // Check bounds
    if (new_x < 0 || new_x >= MAZE_SIZE || new_y < 0 || new_y >= MAZE_SIZE) {
        send_bluetooth_message("BLOCKED - Out of bounds!\r\n");
        return false;
    }

    send_bluetooth_printf("(%d,%d)\r\n", new_x, new_y);
    if (l!=0 && r!=0){
    	move_forward_distance(l,r);
    	l=0;
    	r=0;
    }
    else if (sensors.wall_left || sensors.wall_right){
    	if (flag==1){
    		move_forward_WF_distance(1250,1250);
    		flag=0;
    	}else{
    		//move_forward_WF_distance(LEFT_ENCODER_COUNTS_PER_CELL ,RIGHT_ENCODER_COUNTS_PER_CELL);
    		move_forward_WF_distance_Profile(LEFT_ENCODER_COUNTS_PER_CELL ,RIGHT_ENCODER_COUNTS_PER_CELL);
    	}



    }else{
    	if (flag==1){
			move_forward_distance(1250 ,1250);
			flag=0;
    	}else{
    		move_forward_distance_Profile(LEFT_ENCODER_COUNTS_PER_CELL ,RIGHT_ENCODER_COUNTS_PER_CELL);
    		//move_forward_distance(LEFT_ENCODER_COUNTS_PER_CELL ,RIGHT_ENCODER_COUNTS_PER_CELL);
    	}
    }


    // Use precise encoder-based movement


    // Update robot position
    robot.x = new_x;
    robot.y = new_y;
    robot.exploration_steps++;

    // Mark cell as visited
    maze[robot.x][robot.y].visited = true;
    maze[robot.x][robot.y].visit_count++;
    set_edge_state(from_x, from_y, move_dir, WALL_OPEN);

    return true;
}

/**
 * @brief Check if robot is at goal position
 */
bool is_at_goal(void) {
    if (!robot.center_reached) {
        // Check if at center
        return ((robot.x == maze_center_x1 || robot.x == maze_center_x2) &&
                (robot.y == maze_center_y1 || robot.y == maze_center_y2));
    } else {
        // Check if returned to start
        return (robot.x == 0 && robot.y == 0);
    }
}

/**
 * @brief Update walls based on sensor readings
 */
void update_maze_walls(void) {

    // Update sensors first
    update_sensors();

    // Directions relative to heading
    int ld = (robot.direction + 3) % 4;
    int rd = (robot.direction + 1) % 4;

    // FRONT
    if (sensors.wall_front) {
        set_edge_state(robot.x, robot.y, robot.direction, WALL_CLOSED);
    } else {
        set_edge_state(robot.x, robot.y, robot.direction, WALL_OPEN);
    }

    // LEFT
    if (sensors.wall_left) {
        set_edge_state(robot.x, robot.y, ld, WALL_CLOSED);
    } else {
        set_edge_state(robot.x, robot.y, ld, WALL_OPEN);
    }

    // RIGHT
    if (sensors.wall_right) {
        set_edge_state(robot.x, robot.y, rd, WALL_CLOSED);
    } else {
        set_edge_state(robot.x, robot.y, rd, WALL_OPEN);
    }

    // (keep your existing send_bluetooth_printf(...) and beeps here)
    // (keep visited bookkeeping)

    // Send wall detection feedback
    // Send wall detection feedback
    if (sensors.wall_front || sensors.wall_left || sensors.wall_right) {
        send_bluetooth_printf("Walls detected: F:%s L:%s R:%s  [FL:%d FR:%d SL:%d SR:%d]\r\n",
                             sensors.wall_front ? "Y" : "N",
                             sensors.wall_left ? "Y" : "N",
                             sensors.wall_right ? "Y" : "N",
                             sensors.front_left, sensors.front_right,
                             sensors.side_left, sensors.side_right);

        play_wall_beep();
    }

    // Mark current cell as visited
    maze[robot.x][robot.y].visited = true;
    //maze[robot.x][robot.y].visit_count++;
}

/**
 * @brief Main maze exploration function
 */
void explore_maze(void) {
    send_bluetooth_message("\r\n🚀 STARTING MAZE EXPLORATION 🚀\r\n");

    int max_steps = MAZE_SIZE * MAZE_SIZE * 3; // Safety limit
    int steps = 0;

    while (!is_at_goal() && steps < max_steps) {
        // Update wall information
        update_maze_walls();
        HAL_Delay(10);

        // Run flood fill algorithm
        flood_fill_algorithm();

        // Get best direction to move
        int best_direction = get_best_direction();

        // Turn to face best direction
        turn_to_direction(best_direction);

        // Move forward if possible
        if (!move_forward_one_cell_truns()){
            send_bluetooth_message("❌ Movement failed! Trying alternative...\r\n");

            // Try alternative directions
            for (int alt_dir = 0; alt_dir < 4; alt_dir++) {
            	// allow OPEN or UNKNOWN; forbid only CLOSED
            	if (alt_dir != best_direction &&
            	    wall_state[robot.x][robot.y][alt_dir] != WALL_CLOSED) {

                    turn_to_direction(alt_dir);
                    if (move_forward_one_cell()) {
                        break;
                    }
                }
            }
        }

        // Send periodic status updates
        if (steps % 5 == 0) {
            send_bluetooth_printf("Step %d: Position (%d,%d), Direction: %s\r\n",
                                 steps, robot.x, robot.y, get_direction_name(robot.direction));
            send_maze_state();
        }

        steps++;

        // Brief delay for stability
        //HAL_Delay(100);
    }

    if (is_at_goal()) {
        if (!robot.center_reached) {
            robot.center_reached = true;
            send_bluetooth_message("🎯 CENTER REACHED! 🎯\r\n");
            play_success_tone();

            // Brief celebration
            led_sequence_complete();
            HAL_Delay(2000);

            send_bluetooth_message("Now returning to start...\r\n");
        } else {
            robot.returned_to_start = true;
            send_bluetooth_message("🏁 RETURNED TO START! 🏁\r\n");
            play_success_tone();
            led_sequence_complete();
            exploration_completed = 1;
        }
    } else {
        send_bluetooth_printf("❌ Exploration incomplete after %d steps\r\n", max_steps);
        play_error_tone();
        led_sequence_error();
    }

    send_bluetooth_printf("Total exploration steps: %d\r\n", robot.exploration_steps);
}

/**
 * @brief Run complete maze exploration sequence
 */
void run_maze_exploration_sequence(void) {
    //send_bluetooth_message("\r\n" "=" * 50 "\r\n");
    send_bluetooth_message("🐭 MICROMOUSE MAZE EXPLORATION 🐭\r\n");
    //send_bluetooth_message("=" * 50 "\r\n");

    // Phase 1: Exploration to center
    if (!robot.center_reached) {
        send_bluetooth_message("Phase 1: Exploring to center...\r\n");
        led_sequence_exploring();
        explore_maze();
    }

    // Phase 2: Return to start
    if (robot.center_reached && !robot.returned_to_start) {
        send_bluetooth_message("Phase 2: Returning to start...\r\n");
        led_sequence_returning();
        HAL_Delay(1000);
        explore_maze();
    }

    // Phase 3: Report results
    if (robot.returned_to_start) {
        send_bluetooth_message("\r\n" "🏆 EXPLORATION COMPLETE! 🏆" "\r\n");
        send_performance_metrics();

        // Calculate exploration efficiency
        int total_cells = MAZE_SIZE * MAZE_SIZE;
        int visited_cells = 0;
        for (int x = 0; x < MAZE_SIZE; x++) {
            for (int y = 0; y < MAZE_SIZE; y++) {
                if (maze[x][y].visited) visited_cells++;
            }
        }

        float exploration_percentage = (float)visited_cells / total_cells * 100.0f;
        send_bluetooth_printf("Exploration Coverage: %d/%d cells (%.1f%%)\r\n",
                             visited_cells, total_cells, exploration_percentage);

        // Ready for speed run (future implementation)
        send_bluetooth_message("🚀 Ready for speed run optimization! 🚀\r\n");

        exploration_completed = 1;
    }
}

/**
 * @brief Check if exploration is complete
 */
bool is_exploration_complete(void) {
    return exploration_completed;
}

/**
 * @brief Get exploration efficiency
 */
float get_exploration_efficiency(void) {
    if (robot.exploration_steps == 0) return 0.0f;

    // Calculate theoretical minimum (Manhattan distance)
    int min_to_center = abs(maze_center_x1) + abs(maze_center_y1);
    int min_to_start = abs(maze_center_x1 - 0) + abs(maze_center_y1 - 0);
    int theoretical_min = min_to_center + min_to_start;

    if (theoretical_min == 0) return 100.0f;

    return ((float)theoretical_min / robot.exploration_steps) * 100.0f;
}

/**
 * @brief Get optimal distance for current maze knowledge
 */
int get_optimal_distance(void) {
    // This would implement A* or similar for optimal path calculation
    // For now, return the flood fill distance to center
    return maze[0][0].distance;
}



static int optimal_steps_explored = -1;

static void calculate_optimal_path_explored(void) {
    // reset distances
    for (int x=0;x<MAZE_SIZE;x++)
        for (int y=0;y<MAZE_SIZE;y++)
            maze[x][y].distance = MAX_DISTANCE;

    // seed from center cells that were actually visited
    int cx0=(MAZE_SIZE/2)-1, cy0=(MAZE_SIZE/2)-1;
    int gxs[4]={cx0,cx0+1,cx0,cx0+1}, gys[4]={cy0,cy0,cy0+1,cy0+1};
    int seeded=0;
    queue_init(&bfs_queue);
    for (int i=0;i<4;i++){
        int gx=gxs[i], gy=gys[i];
        if (maze[gx][gy].visited) {
            maze[gx][gy].distance = 0;
            queue_push(&bfs_queue, (Position){gx,gy});
            seeded=1;
        }
    }
    if (!seeded) {
        send_bluetooth_message("[ERROR] No goal cells were visited.\r\n");
        optimal_steps_explored = -1;
        return;
    }

    int updates=0;
    while (!queue_empty(&bfs_queue)) {
        Position p = queue_pop(&bfs_queue);
        int x=p.x, y=p.y;
        for (int dir=0; dir<4; dir++) {
            if (wall_state[x][y][dir] == WALL_CLOSED) continue;   // CLOSED blocks (like sim)
            int nx = x + dx[dir], ny = y + dy[dir];
            if (nx<0||nx>=MAZE_SIZE||ny<0||ny>=MAZE_SIZE) continue;
            if (!maze[nx][ny].visited) continue;                  // EXPLORED ONLY

            int nd = maze[x][y].distance + 1;
            if (nd < maze[nx][ny].distance) {
                maze[nx][ny].distance = nd;
                queue_push(&bfs_queue,(Position){nx,ny});
                updates++;
            }
        }
    }

    optimal_steps_explored = maze[0][0].distance;
    send_bluetooth_printf("[PATH] optimal steps (explored): %d | updates: %d\r\n",
                          optimal_steps_explored, updates);
}

