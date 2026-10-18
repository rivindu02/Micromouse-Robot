# Maze simulator (host)

Runs the real firmware maze logic on a PC, so exploration changes can be
measured before they go on the robot.

`make` compiles these firmware files unchanged:

- `Core/Src/micromouse.c` (exploration, flood fill, direction choice)
- `Core/Src/communication.c` (Bluetooth telemetry)
- `Core/Src/audio.c` (beeps)
- `Core/Src/utils.c` (LED sequences, helpers)

`sim_main.c` replaces the hardware parts with stubs:

- `update_sensors()` reads the walls around the virtual robot from the maze.
- `move_forward_*()` and `turn_*()` move the virtual robot one cell or 90/180°.
- `HAL_Delay()` and `HAL_UART_Transmit()` add to a simulated clock instead of
  waiting. UART time uses the firmware baud rate (9600, 10 bits per byte).

`stub/stm32f4xx_hal.h` holds the few HAL types and macros these files need.

## Usage

```sh
cd tools/sim
make                                # builds build/sim with ASan + UBSan
./build/sim --random 200 1          # 200 generated mazes, seeds 1..200
./build/sim -v --random 20 1        # per-maze table
./build/sim -v path/to/maze.txt ... # mazes in "mazefiles" text format
./build/sim -t --random 1 1         # also print all Bluetooth text
make CORE=/path/to/other/Core       # build against another firmware version
```

Classic competition mazes in the expected text format are available at
<https://github.com/micromouseonline/mazefiles> (`classic/` folder). They are
not copied into this repo.

## Output columns

| column | meaning |
|---|---|
| `cells` | cells driven during exploration (to centre and back) |
| `t90`, `t180` | 90° and 180° turns |
| `crash` | moves into a real wall (must be 0) |
| `delay_s` | time in `HAL_Delay()` (beeps, LED sequences, pauses) |
| `uart_s` | time blocked sending Bluetooth text at 9600 baud |
| `known` | shortest start-to-centre path using only walls the robot saw |
| `best` | true shortest path in the maze |

The simulator does not model motion time, wheel slip or sensor noise. Use it
to compare decision logic and idle time, not to predict lap times.
