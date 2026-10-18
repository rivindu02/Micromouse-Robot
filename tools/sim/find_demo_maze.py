#!/usr/bin/env python3
"""
find_demo_maze.py - search complex demo mazes for a good GIF story

    python3 find_demo_maze.py [seeds] [loops ...]     (run from tools/sim, after make)

For each seed and loop count it builds `make_demo_maze.py complex SEED LOOPS`,
runs ./build/sim with --trace and scores the run. A good demo has:
  - a return trip that mostly uses cells not driven on the way out
  - a speed run that uses cells only seen on the way out AND cells only seen
    on the way back (it combines both trips)
  - a speed run shorter than the first route the robot knew at the centre
Prints the best candidates, best first.
"""

import os
import subprocess
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from render_gif import parse, Scene, known_shortest, CENTRE, DX, DY  # noqa: E402


def score(trace):
    name, walls, events = parse(trace)
    sc = Scene(walls)
    trips = [[], [], []]
    first_route = None
    for ev in events:
        if ev[0] == "S":
            sc.sense(*ev[1:])
        elif ev[0] == "M":
            _, x, y, d, ph, _ = ev
            c = (x + DX[d], y + DY[d])
            trips[ph].append(c)
            sc.visited.add(c)
            if ph == 0 and c in CENTRE and first_route is None:
                p = known_shortest(sc.seen, sc.visited)
                first_route = len(p) - 1 if p else None
    out, back, speed = (set(t) for t in trips)
    out.add((0, 0))
    if not back or not speed or not first_route:
        return None
    overlap = len(back & out) / len(back)
    from_out = len(speed & (out - back))
    from_back = len(speed & (back - out))
    gain = first_route - len(trips[2])
    moves = sum(len(t) for t in trips)
    if gain <= 0 or from_out < 3 or from_back < 3 or moves > 260:
        return None
    s = (1 - overlap) * 10 + min(from_out, from_back) * 0.6 + gain * 0.8 + len(trips[0]) * 0.03
    return s, dict(overlap=round(overlap, 2), out=len(trips[0]), back=len(trips[1]),
                   speed=len(trips[2]), first_route=first_route,
                   speed_from_out=from_out, speed_from_back=from_back)


def main():
    seeds = int(sys.argv[1]) if len(sys.argv) > 1 else 300
    loops = [int(a) for a in sys.argv[2:]] or [12, 20, 30]
    here = os.path.dirname(os.path.abspath(__file__))
    results = []
    with tempfile.TemporaryDirectory() as tmp:
        maze, trace = os.path.join(tmp, "m.txt"), os.path.join(tmp, "t.txt")
        for seed in range(1, seeds + 1):
            for lp in loops:
                with open(maze, "w") as f:
                    subprocess.run([sys.executable, os.path.join(here, "make_demo_maze.py"),
                                    "complex", str(seed), str(lp)], stdout=f, check=True)
                r = subprocess.run([os.path.join(here, "build", "sim"), maze, "--trace", trace],
                                   capture_output=True, text=True)
                if r.returncode != 0:
                    continue
                s = score(trace)
                if s:
                    results.append((s[0], seed, lp, s[1]))
    results.sort(reverse=True)
    for s, seed, lp, info in results[:15]:
        print(f"{s:6.1f}  complex {seed} {lp}  {info}")


if __name__ == "__main__":
    sys.exit(main())
