/*
 * sim_main.c - host simulator for the micromouse maze logic
 *
 * Compiles the real firmware files Core/Src/micromouse.c, communication.c,
 * audio.c and utils.c on a PC. Motion and sensor functions are replaced by
 * stubs that move a virtual robot through a known maze, so the exploration
 * algorithm can be measured without hardware:
 *   - cells driven, 90/180 degree turns, wall crashes
 *   - idle time spent in HAL_Delay (beeps, LED sequences, pauses)
 *   - idle time spent blocked on the 9600 baud Bluetooth UART
 *   - shortest path the robot knows at the end vs the true shortest path
 *
 * Usage:
 *   sim <maze.txt> [...]          run on maze files (mazefiles text format)
 *   sim --random <count> [seed]   run on generated 16x16 mazes
 *   add -v to print the per-maze table, -t to echo UART text,
 *   --trace out.txt to log every sense/move for render_gif.py
 */

#include "micromouse.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ---------------- globals normally defined in main.c ---------------- */
MazeCell maze[MAZE_SIZE][MAZE_SIZE];
RobotState robot;
SensorData sensors;
GyroData gyro;
EncoderData encoders;
volatile uint8_t button_pressed = 0;
volatile uint8_t start_flag = 0;
const int dx[4] = {0, 1, 0, -1};
const int dy[4] = {1, 0, -1, 0};
const int goal_x1 = (MAZE_SIZE/2 - 1), goal_y1 = (MAZE_SIZE/2 - 1);
const int goal_x2 = (MAZE_SIZE/2), goal_y2 = (MAZE_SIZE/2);
ADC_HandleTypeDef hadc1;
SPI_HandleTypeDef hspi2;
TIM_HandleTypeDef htim1, htim2, htim3, htim4;
UART_HandleTypeDef huart6;
GPIO_TypeDef sim_gpioa, sim_gpiob, sim_gpioc;
DWT_Type sim_dwt;
CoreDebug_Type sim_coredebug;

/* ---------------- simulation state ---------------- */
#define UART_BAUD 9600U   /* huart6.Init.BaudRate in main.c */

static bool true_wall[MAZE_SIZE][MAZE_SIZE][4];

typedef struct {
    int cells;
    int turns90;
    int turns180;
    int crashes;
    uint64_t delay_ms;
    uint64_t uart_bytes;
} SimStats;

static SimStats st;
static uint64_t sim_ms = 0;
static int echo_uart = 0;
static FILE *trace = NULL;   /* --trace: event log for render_gif.py */

/* ---------------- HAL stubs ---------------- */
void HAL_Delay(uint32_t ms) { sim_ms += ms; st.delay_ms += ms; }
uint32_t HAL_GetTick(void) { return (uint32_t)sim_ms; }
void HAL_GPIO_WritePin(GPIO_TypeDef *p, uint16_t pin, GPIO_PinState s) { (void)p; (void)pin; (void)s; }
void HAL_GPIO_TogglePin(GPIO_TypeDef *p, uint16_t pin) { (void)p; (void)pin; }
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *h, uint32_t c) { (void)h; (void)c; return HAL_OK; }
HAL_StatusTypeDef HAL_TIM_PWM_Stop(TIM_HandleTypeDef *h, uint32_t c) { (void)h; (void)c; return HAL_OK; }

HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *h, const uint8_t *data, uint16_t size, uint32_t timeout)
{
    (void)h; (void)timeout;
    st.uart_bytes += size;
    if (echo_uart) fwrite(data, 1, size, stdout);
    return HAL_OK;
}

/* ---------------- firmware stubs: gyro / sensors / motion ---------------- */
uint8_t mpu9250_read_register(uint8_t reg) { (void)reg; return 0x70; }
bool mpu9250_is_initialized(void) { return true; }

static bool wall_rel(int turn)
{
    int d = (robot.direction + turn) % 4;
    return true_wall[robot.x][robot.y][d];
}

void update_sensors(void)
{
    sensors.wall_front = wall_rel(0);
    sensors.wall_right = wall_rel(1);
    sensors.wall_left  = wall_rel(3);
    sensors.battery = 4000;
    if (trace) fprintf(trace, "S %d %d %d\n", robot.x, robot.y, robot.direction);
}

static void drive_one_cell(void)
{
    /* micromouse.c updates robot.x/y itself after the move returns */
    st.cells++;
    if (true_wall[robot.x][robot.y][robot.direction]) st.crashes++;
    if (trace) {
        /* pose before the move + the flood fill distances that chose it */
        fprintf(trace, "M %d %d %d %d", robot.x, robot.y, robot.direction, robot.center_reached ? 1 : 0);
        for (int y = 0; y < MAZE_SIZE; y++)
            for (int x = 0; x < MAZE_SIZE; x++) fprintf(trace, " %d", maze[x][y].distance);
        fputc('\n', trace);
    }
}

void move_forward_distance(int l, int r) { (void)l; (void)r; drive_one_cell(); }
void move_forward_WF_distance(int l, int r) { (void)l; (void)r; drive_one_cell(); }
void turn_left(void)   { st.turns90++;  robot.direction = (robot.direction + 3) % 4; }
void turn_right(void)  { st.turns90++;  robot.direction = (robot.direction + 1) % 4; }
void turn_around(void) { st.turns180++; robot.direction = (robot.direction + 2) % 4; }

/* ---------------- maze helpers ---------------- */
static void set_wall(int x, int y, int d, bool v)
{
    true_wall[x][y][d] = v;
    int nx = x + dx[d], ny = y + dy[d];
    if (nx >= 0 && nx < MAZE_SIZE && ny >= 0 && ny < MAZE_SIZE)
        true_wall[nx][ny][(d + 2) % 4] = v;
}

/* BFS from start to any centre cell. use_known: use the robot's map,
 * only through visited cells. Returns -1 when no path. */
static int shortest_path(bool use_known)
{
    int dist[MAZE_SIZE][MAZE_SIZE];
    int qx[MAZE_SIZE * MAZE_SIZE], qy[MAZE_SIZE * MAZE_SIZE], h = 0, t = 0;
    for (int x = 0; x < MAZE_SIZE; x++)
        for (int y = 0; y < MAZE_SIZE; y++) dist[x][y] = -1;
    dist[0][0] = 0; qx[t] = 0; qy[t++] = 0;
    while (h < t) {
        int x = qx[h], y = qy[h++];
        if ((x == goal_x1 || x == goal_x2) && (y == goal_y1 || y == goal_y2)) return dist[x][y];
        for (int d = 0; d < 4; d++) {
            bool w = use_known ? maze[x][y].walls[d] : true_wall[x][y][d];
            if (w) continue;
            int nx = x + dx[d], ny = y + dy[d];
            if (nx < 0 || nx >= MAZE_SIZE || ny < 0 || ny >= MAZE_SIZE) continue;
            if (use_known && !maze[nx][ny].visited) continue;
            if (dist[nx][ny] >= 0) continue;
            dist[nx][ny] = dist[x][y] + 1;
            qx[t] = nx; qy[t++] = ny;
        }
    }
    return -1;
}

/* mazefiles text format: 33 lines of "o---o   o" posts and "|" walls, top row first */
static bool load_maze(const char *path)
{
    FILE *f = fopen(path, "r");
    if (!f) { perror(path); return false; }
    char line[2 * MAZE_SIZE + 1][256];
    int n = 0;
    while (n < 2 * MAZE_SIZE + 1 && fgets(line[n], sizeof line[n], f)) {
        if (line[n][0] == 'o') n++;
        else if (n > 0 && (line[n][0] == '|' || line[n][0] == ' ')) n++;
    }
    fclose(f);
    if (n != 2 * MAZE_SIZE + 1 || strlen(line[0]) < 4 * MAZE_SIZE + 1) {
        fprintf(stderr, "%s: skipped, not a 16x16 maze\n", path);
        return false;
    }
    memset(true_wall, 0, sizeof true_wall);
    for (int r = 0; r < MAZE_SIZE; r++) {
        int y = MAZE_SIZE - 1 - r;
        for (int x = 0; x < MAZE_SIZE; x++) {
            const char *top = line[2 * r], *mid = line[2 * r + 1], *bot = line[2 * r + 2];
            if (top[4 * x + 2] == '-') true_wall[x][y][NORTH] = true;
            if (bot[4 * x + 2] == '-') true_wall[x][y][SOUTH] = true;
            if (mid[4 * x] == '|') true_wall[x][y][WEST] = true;
            if ((int)strlen(mid) > 4 * x + 4 && mid[4 * x + 4] == '|') true_wall[x][y][EAST] = true;
        }
    }
    return true;
}

/* Recursive-backtracker perfect maze plus a few removed walls for loops.
 * Centre 2x2 is open inside with one entrance; start cell opens north only. */
static uint32_t rng;
static uint32_t rnd(void) { rng = rng * 1103515245u + 12345u; return rng >> 16; }

static void generate_maze(uint32_t seed)
{
    rng = seed;
    for (int x = 0; x < MAZE_SIZE; x++)
        for (int y = 0; y < MAZE_SIZE; y++)
            for (int d = 0; d < 4; d++) true_wall[x][y][d] = true;

    bool seen[MAZE_SIZE][MAZE_SIZE] = {{false}};
    int sx[MAZE_SIZE * MAZE_SIZE], sy[MAZE_SIZE * MAZE_SIZE], sp = 0;
    /* centre block counts as one visited node, entered once */
    seen[goal_x1][goal_y1] = seen[goal_x2][goal_y1] = seen[goal_x1][goal_y2] = seen[goal_x2][goal_y2] = true;
    set_wall(goal_x1, goal_y1, EAST, false);
    set_wall(goal_x1, goal_y1, NORTH, false);
    set_wall(goal_x2, goal_y2, WEST, false);
    set_wall(goal_x2, goal_y2, SOUTH, false);
    seen[0][0] = true; sx[sp] = 0; sy[sp++] = 0;
    bool centre_linked = false;
    while (sp > 0) {
        int x = sx[sp - 1], y = sy[sp - 1];
        int opts[4], no = 0;
        for (int d = 0; d < 4; d++) {
            int nx = x + dx[d], ny = y + dy[d];
            if (nx < 0 || nx >= MAZE_SIZE || ny < 0 || ny >= MAZE_SIZE) continue;
            if (x == 0 && y == 0 && d != NORTH) continue;
            bool is_centre = (nx == goal_x1 || nx == goal_x2) && (ny == goal_y1 || ny == goal_y2);
            if (is_centre && !centre_linked && (rnd() % 3 == 0)) { opts[no++] = d | 8; continue; }
            if (!seen[nx][ny]) opts[no++] = d;
        }
        if (no == 0) { sp--; continue; }
        int pick = opts[rnd() % no];
        int d = pick & 3;
        set_wall(x, y, d, false);
        if (pick & 8) { centre_linked = true; continue; }
        int nx = x + dx[d], ny = y + dy[d];
        seen[nx][ny] = true; sx[sp] = nx; sy[sp++] = ny;
    }
    if (!centre_linked) set_wall(goal_x1, goal_y1, WEST, false);
    /* loops: remove ~10% of interior walls outside start and centre */
    for (int k = 0; k < 25; k++) {
        int x = 1 + rnd() % (MAZE_SIZE - 2), y = 1 + rnd() % (MAZE_SIZE - 2), d = rnd() % 4;
        bool c = (x == goal_x1 || x == goal_x2) && (y == goal_y1 || y == goal_y2);
        if (!c) set_wall(x, y, d, false);
    }
}

/* ---------------- run one maze ---------------- */
typedef struct {
    SimStats s;
    bool center, home;
    int known_path, true_path;
} RunResult;

static RunResult run_once(void)
{
    memset(&st, 0, sizeof st);
    sim_ms = 0;
    memset(maze, 0, sizeof maze);
    initialize_maze_exploration();
    run_maze_exploration_sequence();
    RunResult r = { st, robot.center_reached, robot.returned_to_start,
                    shortest_path(true), shortest_path(false) };
    return r;
}

static double uart_ms(uint64_t bytes) { return bytes * 10.0 * 1000.0 / UART_BAUD; }

int main(int argc, char **argv)
{
    int verbose = 0, nrandom = 0;
    uint32_t seed = 1;
    static const char *files[4096]; int nfiles = 0;
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "-v")) verbose = 1;
        else if (!strcmp(argv[i], "-t")) echo_uart = 1;
        else if (!strcmp(argv[i], "--trace") && i + 1 < argc) {
            trace = fopen(argv[++i], "w");
            if (!trace) { perror(argv[i]); return 2; }
        }
        else if (!strcmp(argv[i], "--random") && i + 1 < argc) {
            nrandom = atoi(argv[++i]);
            if (i + 1 < argc && argv[i + 1][0] != '-') seed = (uint32_t)strtoul(argv[++i], NULL, 10);
        } else if (nfiles < 4096) files[nfiles++] = argv[i];
    }
    int total = nfiles + nrandom;
    if (total == 0) { fprintf(stderr, "usage: %s [-v] [-t] [--trace out.txt] <maze.txt>... | --random N [seed]\n", argv[0]); return 2; }

    if (verbose)
        printf("%-28s %6s %5s %5s %5s %8s %9s %6s %6s %s\n",
               "maze", "cells", "t90", "t180", "crash", "delay_s", "uart_s", "known", "best", "result");
    double sum_cells = 0, sum_t90 = 0, sum_t180 = 0, sum_delay = 0, sum_uart = 0, sum_gap = 0;
    int ok = 0, crashes = 0, gap_n = 0, skipped = 0;
    for (int i = 0; i < total; i++) {
        char name[64];
        if (i < nfiles) {
            if (!load_maze(files[i])) { skipped++; continue; }
            const char *b = strrchr(files[i], '/');
            snprintf(name, sizeof name, "%.60s", b ? b + 1 : files[i]);
        } else {
            uint32_t s = seed + (uint32_t)(i - nfiles);
            generate_maze(s);
            snprintf(name, sizeof name, "random-%u", s);
        }
        if (trace) {
            /* true walls, one line per cell: x y N E S W */
            fprintf(trace, "MAZE %s\n", name);
            for (int y = 0; y < MAZE_SIZE; y++)
                for (int x = 0; x < MAZE_SIZE; x++)
                    fprintf(trace, "W %d %d %d %d %d %d\n", x, y, true_wall[x][y][NORTH],
                            true_wall[x][y][EAST], true_wall[x][y][SOUTH], true_wall[x][y][WEST]);
        }
        RunResult r = run_once();
        if (trace) fprintf(trace, "END %d %d %d\n", robot.x, robot.y, robot.direction);
        bool good = r.center && r.home && r.s.crashes == 0;
        ok += good;
        crashes += r.s.crashes;
        sum_cells += r.s.cells; sum_t90 += r.s.turns90; sum_t180 += r.s.turns180;
        sum_delay += r.s.delay_ms / 1000.0; sum_uart += uart_ms(r.s.uart_bytes) / 1000.0;
        if (r.known_path > 0 && r.true_path > 0) { sum_gap += r.known_path - r.true_path; gap_n++; }
        if (verbose)
            printf("%-28s %6d %5d %5d %5d %8.1f %9.1f %6d %6d %s\n", name, r.s.cells, r.s.turns90,
                   r.s.turns180, r.s.crashes, r.s.delay_ms / 1000.0, uart_ms(r.s.uart_bytes) / 1000.0,
                   r.known_path, r.true_path, good ? "ok" : (r.center ? "no-return" : "no-centre"));
    }
    total -= skipped;
    if (total == 0) return 1;
    printf("mazes=%d ok=%d crashes=%d | mean per maze: cells=%.1f turns90=%.1f turns180=%.1f "
           "delay_s=%.1f uart_s=%.1f known_path_excess=%.2f\n",
           total, ok, crashes, sum_cells / total, sum_t90 / total, sum_t180 / total,
           sum_delay / total, sum_uart / total, gap_n ? sum_gap / gap_n : -1.0);
    return ok == total ? 0 : 1;
}
