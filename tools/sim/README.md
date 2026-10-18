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
- After exploration it calls `run_speed_run()` and checks it drives a true
  shortest path.
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

## Animation

```sh
make gif    # builds mazes/complex.txt, sim --trace, render_gif.py -> docs/exploration.gif
```

`--trace out.txt` logs the true maze, every sensor read (`S x y dir`) and every
move with the firmware's flood fill distances (`M x y dir phase d0..d255`).
`render_gif.py` turns that log into a GIF (needs Pillow).

Demo mazes come from `make_demo_maze.py`:

- `mazes/complex.txt` (`complex 248 20`, used for the GIF): backtracker maze
  with long corridors and dead ends, a centre room with two entrances, and
  20 extra openings for loops.
- `mazes/two-routes.txt` (`two-routes`): two carved routes, a simple case
  where the robot goes out on one and comes back on the other.

`find_demo_maze.py [seeds] [loops ...]` searches complex mazes for a good
story: the return trip mostly on new cells, the speed run using cells from
both trips, and a speed run shorter than the first route. Any maze file or
`--random N seed` also works with `--trace`.

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
| `speed` | cells driven by `run_speed_run()` (must equal `best`) |

The simulator does not model motion time, wheel slip or sensor noise. Use it
to compare decision logic and idle time, not to predict lap times.
