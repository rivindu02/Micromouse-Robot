#!/usr/bin/env python3
"""
make_demo_maze.py - build the hand-designed demo maze used for the README GIF

    python3 make_demo_maze.py > mazes/two-routes.txt

Two routes lead from the start to the single centre entrance (north side):

  A  north along the west edge, east along the top, south into the centre
  B  east along the bottom, north, then west to join A a few cells above the
     centre

From the start the robot sees both openings with equal flood fill distance
and keeps straight, so it explores A first. On the way back, A is the only
known path, so cells along B could still hold a shorter route; the robot
returns along B. B's first part plus A's last part is the shortest path,
and that is what the speed run drives.

Every cell outside the two routes and the centre belongs to small closed-off
mazes, so the picture looks like a real maze but the robot cannot enter
them. Output is the "mazefiles" text format read by tools/sim.
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


# route A: start -> up the west edge -> along the top -> down column 7 -> centre
route_a = polyline((0, 0), (0, 14), (7, 14), (7, 8))
# route B: branches east at (0,1) -> along the bottom -> up column 9 -> west to A at (7,11)
route_b = polyline((0, 1), (9, 1), (9, 11), (7, 11))
carve(route_a)
carve(route_b)

# centre: 2x2 room, entered only from (7,9)
open_wall(7, 7, 'E'); open_wall(7, 7, 'N'); open_wall(8, 8, 'W'); open_wall(8, 8, 'S')

# fill every other region with a closed perfect maze (no openings to routes)
used = set(route_a) | set(route_b) | CENTRE
rng = random.Random(16)
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

# print, top row first
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
    print(s + 'o')
    if r == 0:
        break
    y = r - 1
    s = ''
    for x in range(N):
        s += ('|' if wall[(x, y)]['W'] else ' ') + '   '
    print(s + '|')
