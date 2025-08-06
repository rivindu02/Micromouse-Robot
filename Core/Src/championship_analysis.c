/*
 * championship_analysis.c - MMS Championship Path Analysis Functions
 *
 * Implements the perfect path analysis algorithms from MMS championship code
 */

#include "micromouse.h"
#include <string.h>
#include <stdlib.h>

/* External variables */
extern int exploration_steps;
extern int theoretical_minimum;

/**
 * @brief Calculate optimal path using ONLY explored areas (MMS algorithm)
 */
void calculate_optimal_path_from_explored_areas(void)
{
    send_bluetooth_message("\r\n🔍 CALCULATING OPTIMAL PATH FROM EXPLORED AREAS...\r\n");

    // Reset all distances
    for (int x = 0; x < MAZE_SIZE; x++) {
        for (int y = 0; y < MAZE_SIZE; y++) {
            maze[x][y].distance = MAX_DISTANCE;
        }
    }

    // Set goal distances to 0 ONLY if they were visited
    bool goal_found = false;

    if (maze[goal_x1][goal_y1].visited) {
        maze[goal_x1][goal_y1].distance = 0;
        goal_found = true;
        send_bluetooth_printf("Goal cell (%d,%d) visited\r\n", goal_x1, goal_y1);
    }

    if (maze[goal_x2][goal_y1].visited) {
        maze[goal_x2][goal_y1].distance = 0;
        goal_found = true;
        send_bluetooth_printf("Goal cell (%d,%d) visited\r\n", goal_x2, goal_y1);
    }

    if (maze[goal_x1][goal_y2].visited) {
        maze[goal_x1][goal_y2].distance = 0;
        goal_found = true;
        send_bluetooth_printf("Goal cell (%d,%d) visited\r\n", goal_x1, goal_y2);
    }

    if (maze[goal_x2][goal_y2].visited) {
        maze[goal_x2][goal_y2].distance = 0;
        goal_found = true;
        send_bluetooth_printf("Goal cell (%d,%d) visited\r\n", goal_x2, goal_y2);
    }

    if (!goal_found) {
        send_bluetooth_message("❌ ERROR: No goal cells were visited during exploration!\r\n");
        theoretical_minimum = MAX_DISTANCE;
        return;
    }

    // Queue implementation for flood fill
    int queue_x[256], queue_y[256];
    int queue_head = 0, queue_tail = 0;

    // Add visited goal cells to queue
    if (maze[goal_x1][goal_y1].visited && maze[goal_x1][goal_y1].distance == 0) {
        queue_x[queue_tail] = goal_x1; queue_y[queue_tail++] = goal_y1;
    }
    if (maze[goal_x2][goal_y1].visited && maze[goal_x2][goal_y1].distance == 0) {
        queue_x[queue_tail] = goal_x2; queue_y[queue_tail++] = goal_y1;
    }
    if (maze[goal_x1][goal_y2].visited && maze[goal_x1][goal_y2].distance == 0) {
        queue_x[queue_tail] = goal_x1; queue_y[queue_tail++] = goal_y2;
    }
    if (maze[goal_x2][goal_y2].visited && maze[goal_x2][goal_y2].distance == 0) {
        queue_x[queue_tail] = goal_x2; queue_y[queue_tail++] = goal_y2;
    }

    int updates = 0;

    // Run flood fill ONLY through explored areas
    while (queue_head < queue_tail) {
        int x = queue_x[queue_head];
        int y = queue_y[queue_head++];

        for (int dir = 0; dir < 4; dir++) {
            int nx = x + dx[dir];
            int ny = y + dy[dir];

            // CRITICAL: Only process VISITED cells with no walls
            if (nx >= 0 && nx < MAZE_SIZE && ny >= 0 && ny < MAZE_SIZE &&
                maze[nx][ny].visited &&  // Must be visited/explored
                !maze[x][y].walls[dir]) { // No wall between cells

                int new_dist = maze[x][y].distance + 1;
                if (new_dist < maze[nx][ny].distance) {
                    maze[nx][ny].distance = new_dist;
                    queue_x[queue_tail] = nx;
                    queue_y[queue_tail++] = ny;
                    updates++;
                }
            }
        }
    }

    // Get theoretical minimum from explored path
    theoretical_minimum = maze[0][0].distance;

    send_bluetooth_printf("[PATH ANALYSIS] Optimal path through explored areas: %d steps\r\n", theoretical_minimum);
    send_bluetooth_printf("[PATH ANALYSIS] Flood fill updates: %d\r\n", updates);
}

/**
 * @brief Comprehensive championship maze performance analysis (MMS style)
 */
void analyze_championship_maze_performance(void)
{
    send_bluetooth_message("\r\n=== CHAMPIONSHIP PERFORMANCE ANALYSIS ===\r\n");

    // Calculate exploration efficiency
    int cells_visited = 0;
    int total_cells = MAZE_SIZE * MAZE_SIZE;

    for (int x = 0; x < MAZE_SIZE; x++) {
        for (int y = 0; y < MAZE_SIZE; y++) {
            if (maze[x][y].visited) {
                cells_visited++;
            }
        }
    }

    float exploration_efficiency = (float)cells_visited / total_cells * 100.0f;

    send_bluetooth_message("📊 EXPLORATION METRICS:\r\n");
    send_bluetooth_printf(" Exploration Efficiency: %.1f%%\r\n", exploration_efficiency);
    send_bluetooth_printf(" Cells Visited: %d/%d\r\n", cells_visited, total_cells);
    send_bluetooth_printf(" Total Exploration Steps: %d moves\r\n", exploration_steps);

    send_bluetooth_message("\r\n🎯 OPTIMAL PATH ANALYSIS:\r\n");
    if (theoretical_minimum < MAX_DISTANCE) {
        send_bluetooth_printf(" Best Path Through Explored Areas: %d steps\r\n", theoretical_minimum);
        send_bluetooth_message(" Path Knowledge: ✅ COMPLETE for explored regions\r\n");
        send_bluetooth_message(" Algorithm Efficiency: ✅ CHAMPIONSHIP LEVEL\r\n");
    } else {
        send_bluetooth_message(" ❌ No path found through explored areas!\r\n");
        send_bluetooth_message(" Check if center was reached and start is accessible\r\n");
    }

    // Performance rating based on exploration efficiency
    send_bluetooth_message("\r\n🏆 PERFORMANCE RATING:\r\n");
    if (exploration_efficiency <= 50.0f && theoretical_minimum < MAX_DISTANCE) {
        send_bluetooth_message(" ⭐⭐⭐⭐⭐ CHAMPIONSHIP LEVEL\r\n");
        send_bluetooth_message(" 🥇 Efficient exploration with optimal path knowledge!\r\n");
    } else if (exploration_efficiency <= 65.0f) {
        send_bluetooth_message(" ⭐⭐⭐⭐ COMPETITION READY\r\n");
        send_bluetooth_message(" 🥈 Good exploration efficiency with complete maze knowledge\r\n");
    } else if (exploration_efficiency <= 80.0f) {
        send_bluetooth_message(" ⭐⭐⭐ GOOD PERFORMANCE\r\n");
        send_bluetooth_message(" 🥉 Solid exploration, room for optimization\r\n");
    } else {
        send_bluetooth_message(" ⭐⭐ NEEDS OPTIMIZATION\r\n");
        send_bluetooth_message(" 🔄 Over-exploration detected, improve search termination\r\n");
    }

    // Championship recommendations
    send_bluetooth_message("\r\n💡 CHAMPIONSHIP RECOMMENDATIONS:\r\n");
    if (exploration_efficiency <= 50.0f && theoretical_minimum < MAX_DISTANCE) {
        send_bluetooth_message(" ✅ Excellent exploration efficiency! Championship ready!\r\n");
        send_bluetooth_message(" ✅ Optimal path knowledge complete!\r\n");
        send_bluetooth_message(" 🏆 Ready for IEEE Micromouse competition!\r\n");
    } else if (exploration_efficiency > 75.0f) {
        send_bluetooth_message(" 🔄 Consider smarter search termination\r\n");
        send_bluetooth_message(" 🔄 Implement early stopping when center is fully explored\r\n");
    } else {
        send_bluetooth_message(" ✅ Good balance of exploration and efficiency\r\n");
        send_bluetooth_message(" ✅ Path knowledge is complete for explored areas\r\n");
    }

    send_bluetooth_message("========================================\r\n");
}

/**
 * @brief Print detailed distance map for EXPLORED areas only (MMS style)
 */
void print_championship_distance_map(void)
{
    send_bluetooth_message("\r\n📍 OPTIMAL DISTANCE MAP (explored areas only):\r\n");
    send_bluetooth_message("   ");

    // Print column headers
    for (int x = 0; x < MAZE_SIZE; x++) {
        send_bluetooth_printf("%3d", x);
    }
    send_bluetooth_message("\r\n");

    // Print maze from top to bottom (MMS style)
    for (int y = MAZE_SIZE - 1; y >= 0; y--) {
        send_bluetooth_printf("%2d ", y);

        for (int x = 0; x < MAZE_SIZE; x++) {
            if (!maze[x][y].visited) {
                send_bluetooth_message(" - "); // Not explored
            } else if (maze[x][y].distance == MAX_DISTANCE) {
                send_bluetooth_message(" ∞ "); // Explored but unreachable
            } else {
                send_bluetooth_printf("%3d", maze[x][y].distance);
            }
        }
        send_bluetooth_message("\r\n");
    }

    if (theoretical_minimum < MAX_DISTANCE) {
        send_bluetooth_printf("\r\nOptimal path through explored areas: %d steps\r\n", theoretical_minimum);
    } else {
        send_bluetooth_message("\r\n❌ No path found through explored areas\r\n");
    }

    send_bluetooth_message("Legend: - = not explored, ∞ = explored but unreachable\r\n");
}

/**
 * @brief Visualize optimal path through explored areas (MMS visualization style)
 * This would work with LED indicators since we don't have MMS display
 */
void visualize_championship_optimal_path(void)
{
    send_bluetooth_message("\r\n🎨 VISUALIZING OPTIMAL PATH THROUGH EXPLORED AREAS...\r\n");

    if (theoretical_minimum >= MAX_DISTANCE) {
        send_bluetooth_message("❌ Cannot visualize path - no valid path found through explored areas\r\n");
        return;
    }

    // Trace optimal path from start to center using ONLY explored areas
    int x = 0, y = 0;

    if (!maze[x][y].visited) {
        send_bluetooth_message("❌ ERROR: Start position was not marked as visited!\r\n");
        return;
    }

    send_bluetooth_message("Optimal path through explored areas: (0,0)");

    int path_steps = 0;
    int max_path_steps = theoretical_minimum + 5; // Safety limit

    while (!((x == goal_x1 || x == goal_x2) && (y == goal_y1 || y == goal_y2)) &&
           path_steps < max_path_steps) {

        int next_x = x, next_y = y;
        int min_dist = maze[x][y].distance;
        int best_dir = -1;

        // Check all four directions with STRICT validation
        for (int dir = 0; dir < 4; dir++) {
            int nx = x + dx[dir];
            int ny = y + dy[dir];

            // STRICT CHECKS: bounds, visited, no walls, better distance
            if (nx >= 0 && nx < MAZE_SIZE && ny >= 0 && ny < MAZE_SIZE &&
                maze[nx][ny].visited &&  // Must be explored
                !maze[x][y].walls[dir] && // No wall from current cell
                maze[nx][ny].distance < min_dist && // Better distance
                maze[nx][ny].distance < MAX_DISTANCE) { // Valid distance

                min_dist = maze[nx][ny].distance;
                next_x = nx;
                next_y = ny;
                best_dir = dir;
            }
        }

        // Check if we made progress
        if (next_x == x && next_y == y) {
            send_bluetooth_printf(" -> BLOCKED at (%d,%d)!\r\n", x, y);
            send_bluetooth_printf("Current distance: %d\r\n", maze[x][y].distance);

            // Debug: Show available directions in explored areas
            send_bluetooth_message("Explored neighbors: ");
            for (int dir = 0; dir < 4; dir++) {
                int nx = x + dx[dir];
                int ny = y + dy[dir];
                if (nx >= 0 && nx < MAZE_SIZE && ny >= 0 && ny < MAZE_SIZE &&
                    maze[nx][ny].visited && !maze[x][y].walls[dir]) {
                    send_bluetooth_printf("dir%d->(%d,%d,d=%d,v=%d) ",
                                         dir, nx, ny, maze[nx][ny].distance, maze[nx][ny].visited);
                }
            }
            send_bluetooth_message("\r\n");
            break; // No progress possible
        }

        // Move to next cell
        x = next_x;
        y = next_y;
        path_steps++;

        send_bluetooth_printf(" -> (%d,%d)", x, y);

        // Flash LED to show path progress
        led_status(1, 1);
        HAL_Delay(100);
        led_status(0, 0);
        HAL_Delay(50);
    }

    send_bluetooth_message("\r\n");

    if (path_steps >= max_path_steps) {
        send_bluetooth_message("⚠️ Path tracing stopped at safety limit\r\n");
    } else if ((x == goal_x1 || x == goal_x2) && (y == goal_y1 || y == goal_y2)) {
        send_bluetooth_message("✅ Successfully traced optimal path to center!\r\n");
        play_confirmation_tone();
    }

    send_bluetooth_message("✅ Optimal path visualized (MMS style):\r\n");
    send_bluetooth_message(" Path traced through explored cells only\r\n");
    send_bluetooth_printf(" Path length traced: %d steps\r\n", path_steps);

    if (path_steps != theoretical_minimum) {
        send_bluetooth_printf("⚠️ Warning: Traced path (%d) differs from theoretical (%d)\r\n",
                             path_steps, theoretical_minimum);
    }
}

/**
 * @brief Show distance values for debugging (MMS style)
 */
void show_championship_distances(void)
{
    send_bluetooth_message("\r\n📋 DISTANCE VALUES IN EXPLORED CELLS...\r\n");

    int displayed_cells = 0;
    for (int x = 0; x < MAZE_SIZE; x++) {
        for (int y = 0; y < MAZE_SIZE; y++) {
            if (maze[x][y].visited && maze[x][y].distance < MAX_DISTANCE && maze[x][y].distance < 100) {
                displayed_cells++;
            }
        }
    }

    send_bluetooth_printf("✅ %d distance values available in explored cells\r\n", displayed_cells);
    send_bluetooth_message("Numbers show steps to reach center through explored path\r\n");

    // Show summary of distance distribution
    int dist_count[50] = {0}; // Count distances 0-49
    for (int x = 0; x < MAZE_SIZE; x++) {
        for (int y = 0; y < MAZE_SIZE; y++) {
            if (maze[x][y].visited && maze[x][y].distance < 50) {
                dist_count[maze[x][y].distance]++;
            }
        }
    }

    send_bluetooth_message("Distance distribution: ");
    for (int d = 0; d < 20; d++) {
        if (dist_count[d] > 0) {
            send_bluetooth_printf("d%d:%d ", d, dist_count[d]);
        }
    }
    send_bluetooth_message("\r\n");
}
