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


/* Maze exploration state variables */
static int maze_center_x1, maze_center_y1, maze_center_x2, maze_center_y2;
static int exploration_completed = 0;
static int optimal_path_calculated = 0;

/* BFS Queue Implementation for Flood Fill */
#define QUEUE_MAX_SIZE (MAZE_SIZE * MAZE_SIZE + 16)
typedef struct {
    int x, y;
} Position;

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
                maze[x][y].walls[dir] = false;
            }
        }
    }

    // Set boundary walls
    for (int i = 0; i < MAZE_SIZE; i++) {
        maze[i][0].walls[SOUTH] = true;                    // Bottom boundary
        maze[i][MAZE_SIZE-1].walls[NORTH] = true;          // Top boundary
        maze[0][i].walls[WEST] = true;                     // Left boundary
        maze[MAZE_SIZE-1][i].walls[EAST] = true;           // Right boundary
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

/* Scratch distance maps for path planning (2 KB of RAM) */
static int dist_from_start[MAZE_SIZE][MAZE_SIZE];
static int dist_from_centre[MAZE_SIZE][MAZE_SIZE];

static bool is_centre_cell(int x, int y) {
    return (x == maze_center_x1 || x == maze_center_x2) &&
           (y == maze_center_y1 || y == maze_center_y2);
}

/**
 * @brief Breadth-first distances over the known walls
 * @param dist         output, MAX_DISTANCE where unreachable
 * @param from_centre  true: seed the four centre cells, false: seed the start cell
 * @param visited_only true: only step into visited cells (their walls are all
 *                     known, so the result is a real path); false: unknown
 *                     walls count as open (optimistic)
 */
static void bfs_distances(int dist[MAZE_SIZE][MAZE_SIZE], bool from_centre, bool visited_only) {
    for (int x = 0; x < MAZE_SIZE; x++)
        for (int y = 0; y < MAZE_SIZE; y++)
            dist[x][y] = MAX_DISTANCE;

    queue_init(&bfs_queue);
    if (from_centre) {
        for (int x = maze_center_x1; x <= maze_center_x2; x++)
            for (int y = maze_center_y1; y <= maze_center_y2; y++) {
                dist[x][y] = 0;
                queue_push(&bfs_queue, (Position){x, y});
            }
    } else {
        dist[0][0] = 0;
        queue_push(&bfs_queue, (Position){0, 0});
    }

    while (!queue_empty(&bfs_queue)) {
        Position c = queue_pop(&bfs_queue);
        for (int dir = 0; dir < 4; dir++) {
            if (maze[c.x][c.y].walls[dir]) continue;
            int nx = c.x + dx[dir];
            int ny = c.y + dy[dir];
            if (nx < 0 || nx >= MAZE_SIZE || ny < 0 || ny >= MAZE_SIZE) continue;
            if (visited_only && !maze[nx][ny].visited) continue;
            if (dist[nx][ny] != MAX_DISTANCE) continue;
            dist[nx][ny] = dist[c.x][c.y] + 1;
            queue_push(&bfs_queue, (Position){nx, ny});
        }
    }
}

/**
 * @brief Seed the return-trip flood fill with unproven shortest-path cells
 *
 * The known shortest path uses visited cells only. The optimistic one
 * treats unknown walls as open, so it can never be longer than the true
 * shortest path. While the known path is longer than the optimistic one,
 * a shorter route may exist: it must pass through an unvisited cell where
 * dist_from_start + dist_from_centre equals the optimistic length. The
 * return trip visits those cells first. When the two lengths match, the
 * known path is proven shortest and the robot heads home.
 *
 * @return number of cells pushed as flood fill goals (0 = proven, go home)
 */
static int seed_unproven_route_cells(void) {
    bfs_distances(dist_from_centre, true, true);
    int known = dist_from_centre[0][0];

    bfs_distances(dist_from_start, false, false);
    bfs_distances(dist_from_centre, true, false);

    int best = MAX_DISTANCE;
    for (int x = maze_center_x1; x <= maze_center_x2; x++)
        for (int y = maze_center_y1; y <= maze_center_y2; y++)
            if (dist_from_start[x][y] < best) best = dist_from_start[x][y];
    if (best == MAX_DISTANCE || known == best) return 0;

    queue_init(&bfs_queue);
    int seeds = 0;
    for (int x = 0; x < MAZE_SIZE; x++)
        for (int y = 0; y < MAZE_SIZE; y++)
            if (!maze[x][y].visited && !is_centre_cell(x, y) &&
                dist_from_start[x][y] + dist_from_centre[x][y] == best) {
                maze[x][y].distance = 0;
                queue_push(&bfs_queue, (Position){x, y});
                seeds++;
            }
    return seeds;
}

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

    // Set goal distances and add to queue
    if (!robot.center_reached) {
        queue_init(&bfs_queue);
        // Heading to center
        maze[maze_center_x1][maze_center_y1].distance = 0;
        maze[maze_center_x2][maze_center_y1].distance = 0;
        maze[maze_center_x1][maze_center_y2].distance = 0;
        maze[maze_center_x2][maze_center_y2].distance = 0;

        queue_push(&bfs_queue, (Position){maze_center_x1, maze_center_y1});
        queue_push(&bfs_queue, (Position){maze_center_x2, maze_center_y1});
        queue_push(&bfs_queue, (Position){maze_center_x1, maze_center_y2});
        queue_push(&bfs_queue, (Position){maze_center_x2, maze_center_y2});
    } else if (seed_unproven_route_cells() == 0) {
        // Returning to start: every possible shortest route is explored
        queue_init(&bfs_queue);
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
        for (int dir = 0; dir < 4; dir++) {
            if (maze[x][y].walls[dir]) continue; // Wall blocks this direction

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

    TLM_VERBOSE("Flood fill complete: %d updates\r\n", updates);
    (void)updates;  // only reported with verbose telemetry
}

/**
 * @brief Get best direction to move based on flood fill values
 */
int get_best_direction(void) {
    int best_dir = robot.direction;
    int min_distance = MAX_DISTANCE;
    int min_visits = 0;
    bool found_open = false;

    // Direction priority: forward, right, left, backward.
    // Earlier entries win ties, so going straight (no turn) is preferred.
    int priority[4];
    priority[0] = robot.direction;                    // Forward
    priority[1] = (robot.direction + 1) % 4;         // Right
    priority[2] = (robot.direction + 3) % 4;         // Left
    priority[3] = (robot.direction + 2) % 4;         // Backward

    // Always step to the open neighbour with the lowest flood fill distance.
    // Among equal distances, take the least visited cell (unvisited first),
    // then the earliest direction in the priority list.
    for (int p = 0; p < 4; p++) {
        int dir = priority[p];

        // Check if there's a wall in this direction
        if (maze[robot.x][robot.y].walls[dir]) continue;

        int nx = robot.x + dx[dir];
        int ny = robot.y + dy[dir];

        // Check bounds
        if (nx < 0 || nx >= MAZE_SIZE || ny < 0 || ny >= MAZE_SIZE) continue;

        int nd = maze[nx][ny].distance;
        int nv = maze[nx][ny].visit_count;

        if (!found_open || nd < min_distance || (nd == min_distance && nv < min_visits)) {
            found_open = true;
            min_distance = nd;
            min_visits = nv;
            best_dir = dir;
        }
    }

    return best_dir;
}
/* Hardware-tuned encoder counts for the first cell after a turn. Each turn
 * starts with a 957-count move (or a front-wall align), so the next cell
 * needs a shorter distance. turn_to_direction() sets them and
 * move_forward_one_cell_truns() uses them once, then clears them. */
static int pending_left_counts = 0;
static int pending_right_counts = 0;

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
            TLM_VERBOSE("Turning RIGHT...\r\n");
            turn_right();
            pending_left_counts=1549;
            pending_right_counts=1537;
            play_turn_beep();
            break;
        case 2:
            // Turn around (180 degrees)
            TLM_VERBOSE("Turning AROUND...\r\n");
            turn_around();
            pending_left_counts=1530;
            pending_right_counts=1562;
            play_turn_beep();
            break;
        case 3:
            // Turn left (90 degrees counter-clockwise)
            TLM_VERBOSE("Turning LEFT...\r\n");
            turn_left();
            pending_left_counts=1330;
            pending_right_counts=1352;
            play_turn_beep();
            break;
    }

    robot.direction = target_direction;
}

/**
 * @brief Move forward one cell with precise control
 */
bool move_forward_one_cell(void) {
    TLM_VERBOSE("Moving forward from (%d,%d) to ", robot.x, robot.y);

    // Calculate new position
    int new_x = robot.x + dx[robot.direction];
    int new_y = robot.y + dy[robot.direction];

    // Check bounds
    if (new_x < 0 || new_x >= MAZE_SIZE || new_y < 0 || new_y >= MAZE_SIZE) {
        TLM_INFO("BLOCKED - Out of bounds!\r\n");
        return false;
    }

    TLM_VERBOSE("(%d,%d)\r\n", new_x, new_y);


    // Use precise encoder-based movement
    move_forward_distance(LEFT_ENCODER_COUNTS_PER_CELL ,RIGHT_ENCODER_COUNTS_PER_CELL);

    // Update robot position
    robot.x = new_x;
    robot.y = new_y;
    robot.exploration_steps++;

    // Mark cell as visited
    maze[robot.x][robot.y].visited = true;
    maze[robot.x][robot.y].visit_count++;

    return true;
}
/* The first move out of the start cell uses a shorter, hardware-tuned
 * distance (1200 counts) instead of a full cell. */
static int first_move = 1;

bool move_forward_one_cell_truns(void){
    TLM_VERBOSE("Moving forward from (%d,%d) to ", robot.x, robot.y);

    // Calculate new position
    int new_x = robot.x + dx[robot.direction];
    int new_y = robot.y + dy[robot.direction];

    // Check bounds
    if (new_x < 0 || new_x >= MAZE_SIZE || new_y < 0 || new_y >= MAZE_SIZE) {
        TLM_INFO("BLOCKED - Out of bounds!\r\n");
        return false;
    }

    TLM_VERBOSE("(%d,%d)\r\n", new_x, new_y);
    if (pending_left_counts!=0 && pending_right_counts!=0){
    	move_forward_distance(pending_left_counts,pending_right_counts);
    	pending_left_counts=0;
    	pending_right_counts=0;
    }
    else if (sensors.wall_left || sensors.wall_right){
    	if (first_move==1){
    		move_forward_WF_distance(1200,1200);
    		first_move=0;
    	}else{
    		move_forward_WF_distance(LEFT_ENCODER_COUNTS_PER_CELL ,RIGHT_ENCODER_COUNTS_PER_CELL);
    	}



    }else{
    	if (first_move==1){
			move_forward_distance(1200 ,1200);
			first_move=0;
    	}else{
    		move_forward_distance(LEFT_ENCODER_COUNTS_PER_CELL ,RIGHT_ENCODER_COUNTS_PER_CELL);
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

    // Update wall information based on current direction and sensor readings
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

    // Send wall detection feedback
    if (sensors.wall_front || sensors.wall_left || sensors.wall_right) {
        TLM_VERBOSE("Walls detected: F:%s L:%s R:%s  [FL:%d FR:%d SL:%d SR:%d]\r\n",
                sensors.wall_front ? "Y" : "N",
                sensors.wall_left ? "Y" : "N",
                sensors.wall_right ? "Y" : "N",
                sensors.front_left, sensors.front_right,
                sensors.side_left, sensors.side_right);

        play_wall_beep();
    }

    // Mark current cell as visited. visit_count is incremented once per
    // arrival in move_forward_one_cell*(), not again here.
    maze[robot.x][robot.y].visited = true;
}

/**
 * @brief Main maze exploration function
 */
void explore_maze(void) {
    TLM_INFO("\r\n🚀 STARTING MAZE EXPLORATION 🚀\r\n");

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
            TLM_INFO("❌ Movement failed! Trying alternative...\r\n");

            // Try alternative directions
            for (int alt_dir = 0; alt_dir < 4; alt_dir++) {
                if (alt_dir != best_direction &&
                    !maze[robot.x][robot.y].walls[alt_dir]) {

                    turn_to_direction(alt_dir);
                    if (move_forward_one_cell()) {
                        break;
                    }
                }
            }
        }

        // Send periodic status updates (verbose telemetry only: a maze dump
        // is ~1.2 kB, which blocks for over a second at 9600 baud)
        if (TELEMETRY_LEVEL >= 2 && steps % 5 == 0) {
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
            TLM_INFO("🎯 CENTER REACHED! 🎯\r\n");
            play_success_tone();

            // Brief celebration
            led_sequence_complete();
            HAL_Delay(2000);

            TLM_INFO("Now returning to start...\r\n");
        } else {
            robot.returned_to_start = true;
            TLM_INFO("🏁 RETURNED TO START! 🏁\r\n");
            play_success_tone();
            led_sequence_complete();
            exploration_completed = 1;
        }
    } else {
        TLM_INFO("❌ Exploration incomplete after %d steps\r\n", max_steps);
        play_error_tone();
        led_sequence_error();
    }

    TLM_INFO("Total exploration steps: %d\r\n", robot.exploration_steps);
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
    // Shortest start-to-centre path through visited cells (known walls only)
    bfs_distances(dist_from_centre, true, true);
    return dist_from_centre[0][0] == MAX_DISTANCE ? -1 : dist_from_centre[0][0];
}

/**
 * @brief Drive the shortest known path from the start cell to the centre
 *
 * Plans over visited cells only, so every wall on the path is known. Uses
 * the same turn and move functions (and speed) as exploration; the gain
 * comes from driving the shortest route with no detours.
 *
 * If the robot is not at the start cell (e.g. after a previous run), it
 * assumes it was carried back by hand and placed in the start cell facing
 * north, like at the beginning of a run.
 *
 * @return true when the centre was reached
 */
bool run_speed_run(void) {
    if (!exploration_completed) {
        TLM_INFO("Speed run needs a completed exploration first\r\n");
        return false;
    }

    if (robot.x != 0 || robot.y != 0) {
        robot.x = 0;
        robot.y = 0;
        robot.direction = NORTH;
        first_move = 1;
    }

    bfs_distances(dist_from_centre, true, true);
    if (dist_from_centre[0][0] == MAX_DISTANCE) {
        TLM_INFO("❌ No known path to the centre\r\n");
        play_error_tone();
        return false;
    }

    TLM_INFO("\r\n⚡ SPEED RUN: %d cells ⚡\r\n", dist_from_centre[0][0]);
    int saved_steps = robot.exploration_steps;
    int moves = 0;

    while (!is_centre_cell(robot.x, robot.y) && moves < MAZE_SIZE * MAZE_SIZE) {
        // Next cell is one step closer to the centre; keep straight if possible
        int here = dist_from_centre[robot.x][robot.y];
        int next_dir = -1;
        for (int t = 0; t < 4; t++) {
            int dir = (robot.direction + (t == 0 ? 0 : t == 1 ? 1 : t == 2 ? 3 : 2)) % 4;
            if (maze[robot.x][robot.y].walls[dir]) continue;
            int nx = robot.x + dx[dir];
            int ny = robot.y + dy[dir];
            if (nx < 0 || nx >= MAZE_SIZE || ny < 0 || ny >= MAZE_SIZE) continue;
            if (dist_from_centre[nx][ny] == here - 1) { next_dir = dir; break; }
        }
        if (next_dir < 0) break;

        update_sensors();               // front wall for turn alignment, side walls for wall following
        turn_to_direction(next_dir);
        if (!move_forward_one_cell_truns()) break;
        moves++;
    }
    robot.exploration_steps = saved_steps;

    if (is_centre_cell(robot.x, robot.y)) {
        TLM_INFO("🏆 SPEED RUN COMPLETE: %d cells\r\n", moves);
        play_success_tone();
        led_sequence_complete();
        return true;
    }
    TLM_INFO("❌ Speed run stopped after %d cells\r\n", moves);
    play_error_tone();
    return false;
}
