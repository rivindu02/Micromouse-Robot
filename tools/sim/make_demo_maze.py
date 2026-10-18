#!/usr/bin/env python3
"""
make_demo_maze.py - build the demo mazes used for the README GIF

    python3 make_demo_maze.py two-routes          > mazes/two-routes.txt
    python3 make_demo_maze.py complex SEED LOOPS  > mazes/complex.txt

two-routes: two carved routes to a single centre entrance (north side).
  A goes up the west edge, along the top and down into the centre; B goes
  along the bottom, up column 9 and joins A above the centre. The robot
  explores A first, returns along B, and the speed run drives B's first part
  plus A's last cells. All other cells are closed-off mini mazes.

complex: a full-size maze. A recursive backtracker gives long corridors and
  many dead ends; the centre room gets two entrances; LOOPS extra walls are
  removed (away from the start and the centre) to create alternative routes.
  tools/sim/find_demo_maze.py searches seeds for one where the return trip
  takes a different route and the speed run mixes both.

Output is the "mazefiles" text format read by tools/sim.
"""

import random

N = 16
DX = {'N': 0, 'E': 1, 'S': 0, 'W': -1}
DY = {'N': 1, 'E': 0, 'S': -1, 'W': 0}
OPP = {'N': 'S', 'E': 'W', 'S': 'N', 'W': 'E'}
CENTRE = {(7, 7), (7, 8), (8, 7), (8, 8)}

wall = {(x, y): {'N': True, 'E': True, 'S': True, 'W': True} for x in range(N) for y in range(N)}


def open_wall(x, y, d):
    wall[(x, y)][d] = False
    wall[(x + DX[d], y + DY[d])][OPP[d]] = False


def carve(path):
    """open walls along a list of adjacent cells"""
    for (x0, y0), (x1, y1) in zip(path, path[1:]):
        d = {(0, 1): 'N', (1, 0): 'E', (0, -1): 'S', (-1, 0): 'W'}[(x1 - x0, y1 - y0)]
        open_wall(x0, y0, d)


def line(a, b):
    (x0, y0), (x1, y1) = a, b
    sx = (x1 > x0) - (x1 < x0)
    sy = (y1 > y0) - (y1 < y0)
    pts = [(x0, y0)]
    while pts[-1] != (x1, y1):
        x, y = pts[-1]
        pts.append((x + sx, y + sy))
    return pts


def polyline(*corners):
    pts = [corners[0]]
    for a, b in zip(corners, corners[1:]):
        pts += line(a, b)[1:]
    return pts


def fill_closed_regions(used, rng):
    """carve a closed perfect maze into every region outside the used cells"""
    free = {c for c in wall if c not in used}
    while free:
        start = min(free)
        stack, seen = [start], {start}
        free.discard(start)
        while stack:
            x, y = stack[-1]
            nbrs = [d for d in 'NESW'
                    if (x + DX[d], y + DY[d]) in free and (x + DX[d], y + DY[d]) not in seen]
            if not nbrs:
                stack.pop()
                continue
            d = rng.choice(nbrs)
            open_wall(x, y, d)
            n = (x + DX[d], y + DY[d])
            seen.add(n)
            free.discard(n)
            stack.append(n)


def two_routes():
    # route A: start -> up the west edge -> along the top -> down column 7 -> centre
    route_a = polyline((0, 0), (0, 14), (7, 14), (7, 8))
    # route B: branches east at (0,1) -> along the bottom -> up column 9 -> west to A at (7,11)
    route_b = polyline((0, 1), (9, 1), (9, 11), (7, 11))
    carve(route_a)
    carve(route_b)
    # centre: 2x2 room, entered only from (7,9)
    open_wall(7, 7, 'E'); open_wall(7, 7, 'N'); open_wall(8, 8, 'W'); open_wall(8, 8, 'S')
    fill_closed_regions(set(route_a) | set(route_b) | CENTRE, random.Random(16))


def inside(x, y):
    return 0 <= x < N and 0 <= y < N


def complex_maze(seed, loops):
    rng = random.Random(seed)
    # centre room
    open_wall(7, 7, 'E'); open_wall(7, 7, 'N'); open_wall(8, 8, 'W'); open_wall(8, 8, 'S')
    # recursive backtracker from the start; the start cell only opens north
    seen = set(CENTRE) | {(0, 0)}
    open_wall(0, 0, 'N')
    stack = [(0, 1)]
    seen.add((0, 1))
    while stack:
        x, y = stack[-1]
        nbrs = [d for d in 'NESW' if inside(x + DX[d], y + DY[d]) and (x + DX[d], y + DY[d]) not in seen]
        if not nbrs:
            stack.pop()
            continue
        d = rng.choice(nbrs)
        open_wall(x, y, d)
        n = (x + DX[d], y + DY[d])
        seen.add(n)
        stack.append(n)
    # two centre entrances on different sides
    doors = [((7, 8), 'N'), ((8, 8), 'N'), ((8, 8), 'E'), ((8, 7), 'E'),
             ((8, 7), 'S'), ((7, 7), 'S'), ((7, 7), 'W'), ((7, 8), 'W')]
    first = rng.randrange(8)
    second = (first + 3 + rng.randrange(3)) % 8
    for i in (first, second):
        (x, y), d = doors[i]
        open_wall(x, y, d)
    # loops: remove interior walls away from the start corner and the centre
    near = lambda x, y: (x <= 1 and y <= 1) or (6 <= x <= 9 and 6 <= y <= 9)
    removed = 0
    while removed < loops:
        x, y, d = rng.randrange(N), rng.randrange(N), rng.choice('NESW')
        nx, ny = x + DX[d], y + DY[d]
        if not inside(nx, ny) or near(x, y) or near(nx, ny) or not wall[(x, y)][d]:
            continue
        open_wall(x, y, d)
        removed += 1


def print_maze():
    out = []
    for r in range(N, -1, -1):
        # horizontal walls above row r-1 / below row r
        s = ''
        for x in range(N):
            if r == N:
                w = True
            elif r == 0:
                w = True
            else:
                w = wall[(x, r)]['S']
            s += 'o' + ('---' if w else '   ')
        out.append(s + 'o')
        if r == 0:
            break
        y = r - 1
        s = ''
        for x in range(N):
            s += ('|' if wall[(x, y)]['W'] else ' ') + '   '
        out.append(s + '|')
    print('\n'.join(out))


if __name__ == "__main__":
    import sys
    mode = sys.argv[1] if len(sys.argv) > 1 else "two-routes"
    if mode == "complex":
        complex_maze(int(sys.argv[2]), int(sys.argv[3]))
    else:
        two_routes()
    print_maze()
