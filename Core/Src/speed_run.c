/*
 * speed_run.c
 *
 *  Created on: Oct 7, 2025
 *      Author: ASUS
 */
#include "micromouse.h"
#include "movement.h"   // move_forward_*_Profile prototypes
#include "sensors.h"    // sensors struct + read_sensors_fast()


// ====== SEGMENTED EXECUTOR (straight now; diagonals later) ======

typedef enum {
    SEG_FWD = 0,
    SEG_TURN_L,
    SEG_TURN_R,
    SEG_TURN_U,
    // Diagonals reserved (future):
    SEG_DIAG_NE, SEG_DIAG_NW, SEG_DIAG_SE, SEG_DIAG_SW
} SegmentType;


//typedef enum { WALL_UNKNOWN = -1, WALL_OPEN = 0, WALL_CLOSED = 1 } WallState;

static inline int is_open_edge(int x, int y, int dir) {
    if ((unsigned)x >= MAZE_SIZE || (unsigned)y >= MAZE_SIZE) return 0;
    return (get_edge_state(x, y, dir) != WALL_CLOSED);
}

// Count how many cells forward we can blast while the distance strictly decreases by 1
// and edges are open. This is for SPEED RUN (map known).
static int count_forward_cells_along_gradient(int x0, int y0, int dir, int max_cells)
{
    int run = 0;
    int x = x0, y = y0;
    while (run < max_cells) {
        if (!is_open_edge(x, y, dir)) break;

        int nx = x + dx[dir], ny = y + dy[dir];
        if (nx < 0 || nx >= MAZE_SIZE || ny < 0 || ny >= MAZE_SIZE) break;

        // require gradient to drop by exactly 1
        if (maze[nx][ny].distance != maze[x][y].distance - 1) break;

        run++;
        x = nx; y = ny;

        // Optional: stop before big junctions (keep it simple/robust at high speed)
        // int exits = 0;
        // for (int k = 0; k < 4; k++) if (wall_state[x][y][k] != WALL_CLOSED) exits++;
        // if (exits >= 3) break; // slow down decisions at intersections
    }
    return run;
}

// Profiled straight move for K cells (auto-chooses WF vs Gyro based on side walls).
static void execute_forward_cells_auto(int cells)
{
    if (cells <= 0) return;

    const int L = LEFT_ENCODER_COUNTS_PER_CELL  * cells;
    const int R = RIGHT_ENCODER_COUNTS_PER_CELL * cells;

    // Choose WF if corridor present; else Gyro-only.
    const int use_wf = (sensors.wall_left || sensors.wall_right);

    if (use_wf) {
    	move_forward_WF_distance_Profile(L, R);
    } else {
        move_forward_distance_Profile(L, R);
    }

    // Update pose and bookkeeping K times (we skipped per-cell sensor reads on purpose for speed-run)
    for (int i = 0; i < cells; i++) {
        int from_x = robot.x, from_y = robot.y;
        robot.x += dx[robot.direction];
        robot.y += dy[robot.direction];
        robot.exploration_steps++;
        set_edge_state(from_x, from_y, robot.direction, WALL_OPEN);
        maze[robot.x][robot.y].visited = true;
        maze[robot.x][robot.y].visit_count++;
    }
}

// Generic segment executor (turns now, diagonals later)
static void execute_segment(SegmentType t, int arg)
{
    switch (t) {
    case SEG_FWD:
        execute_forward_cells_auto(arg);
        break;
    case SEG_TURN_L:
        turn_left();
        break;
    case SEG_TURN_R:
        turn_right();
        break;
    case SEG_TURN_U:
        turn_around();
        break;
    // ==== Diagonals (future): keep the API; no-ops for now ====
    case SEG_DIAG_NE: case SEG_DIAG_NW: case SEG_DIAG_SE: case SEG_DIAG_SW:
        // TODO: implement diagonal profiles & junction handling
        break;
    default: break;
    }
}

// ===== Speed run using the distance gradient (center already known) =====
void run_speed_to_center(void)
{
    // Start from wherever the robot is; follow the precomputed distance map down to 0
    // Assumes flood-fill distances reflect the final map (search run complete).

    int safety = MAZE_SIZE * MAZE_SIZE * 4;
    while (maze[robot.x][robot.y].distance > 0 && safety-- > 0)
    {
        // Pick neighbor with distance = current - 1
        const int curd = maze[robot.x][robot.y].distance;

        int next_dir = -1;
        for (int k = 0; k < 4; k++) {
            int nx = robot.x + dx[k], ny = robot.y + dy[k];
            if (nx < 0 || nx >= MAZE_SIZE || ny < 0 || ny >= MAZE_SIZE) continue;
            if (get_edge_state(robot.x, robot.y, k) == WALL_CLOSED) continue;

            if (maze[nx][ny].distance == curd - 1) { next_dir = k; break; }
        }

        if (next_dir < 0) {
            send_bluetooth_message("Speed run aborted: no descending neighbor.\r\n");
            break;
        }

        // Turn towards that direction (uses your gyro turn)
        turn_to_direction(next_dir);

        // Compress straight cells while gradient keeps dropping by 1
        int kmax = count_forward_cells_along_gradient(robot.x, robot.y, robot.direction, /*max_cells=*/64);
        if (kmax <= 0) {
            // Shouldn’t happen; move 1 cell as a fallback
            execute_segment(SEG_FWD, 1);
        } else {
            execute_segment(SEG_FWD, kmax);
        }
    }

    // Optionally front-align at goal (if you use it)
    // if (sensors.wall_front) align_front_to_wall(600, 2000);
}

void turn_right_45(void) {

	gyro_turn_reset();
    turn_in_place_gyro(-45.0f, 520, 1000);

}


void turn_left_45(void) {

	gyro_turn_reset();
    turn_in_place_gyro(+45.0f, 520, 1000);

}


void right_diagonal(int halfcell_count) {

	if (sensors.wall_left){

		fusion_align_entry(570, 3000);

	}

	turn_right_45();
	move_forward_distance_Profile(1811*halfcell_count,1811*halfcell_count);
	if (halfcell_count/2 == 1){
		turn_right_45();

	}
	else{turn_left_45();}

	if (sensors.wall_left){
		fusion_align_entry(570, 3000);
	}


}

void left_diagonal(int halfcell_count) {

	if (sensors.wall_right){
		fusion_align_entry(570, 3000);
	}


	turn_left_45();
	move_forward_distance_Profile(1811*halfcell_count,1811*halfcell_count);
	if (halfcell_count/2 == 1){
		turn_left_45();

	}
	else{turn_right_45();}

	if (sensors.wall_right){
		fusion_align_entry(570, 3000);
	}



}
