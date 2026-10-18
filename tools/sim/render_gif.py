#!/usr/bin/env python3
"""
render_gif.py - animate a simulator trace as a GIF

    ./build/sim --random 1 24 --trace trace.txt
    python3 render_gif.py trace.txt ../../docs/exploration.gif

Shows the three phases of a run:
  1. exploring to the centre (cyan)
  2. returning to start (amber); orange boxes are unvisited cells that could
     still hold a shorter route, so the robot detours to check them
  3. speed run on the proven shortest path (green)

Also draws the true maze faintly, the walls the robot has sensed in red and
the firmware's flood fill distances as background shading.

Requires Pillow.
"""

import sys
from collections import deque
from PIL import Image, ImageDraw, ImageFont

N = 16
CELL = 26
MARGIN = 18
HEADER = 52
FOOTER = 44
W = MARGIN * 2 + CELL * N
H = HEADER + MARGIN * 2 + CELL * N + FOOTER
DX = [0, 1, 0, -1]
DY = [1, 0, -1, 0]
CENTRE = {(7, 7), (7, 8), (8, 7), (8, 8)}
MAXD = 9999

BG = (17, 24, 39)
POST = (75, 85, 99)
WALL_HIDDEN = (71, 85, 105)
WALL_SEEN = (248, 113, 113)
GOAL = (22, 101, 52)
TRAIL = [(34, 211, 238), (251, 191, 36), (74, 222, 128)]
TRAIL_DIM = [(21, 94, 117), (120, 92, 30)]
CANDIDATE = (249, 115, 22)
TEXT = (229, 231, 235)
SUBTEXT = (156, 163, 175)
ROBOT = (255, 255, 255)

LABELS = ["1. Exploring to the centre",
          "2. Back to start, checking other routes",
          "3. Speed run on the shortest path"]


def font(size, bold=False):
    names = ["DejaVuSans-Bold.ttf" if bold else "DejaVuSans.ttf",
             "LiberationSans-Bold.ttf" if bold else "LiberationSans-Regular.ttf"]
    for n in names:
        try:
            return ImageFont.truetype(n, size)
        except OSError:
            pass
    return ImageFont.load_default()


def parse(path):
    walls, events, name = {}, [], ""
    with open(path) as f:
        for line in f:
            p = line.split()
            if not p:
                continue
            if p[0] == "MAZE":
                name = p[1] if len(p) > 1 else ""
            elif p[0] == "W":
                x, y = int(p[1]), int(p[2])
                walls[(x, y)] = [p[3] == "1", p[4] == "1", p[5] == "1", p[6] == "1"]
            elif p[0] == "S":
                events.append(("S", int(p[1]), int(p[2]), int(p[3])))
            elif p[0] == "M":
                d = list(map(int, p[5:5 + N * N]))
                events.append(("M", int(p[1]), int(p[2]), int(p[3]), int(p[4]), d))
            elif p[0] == "END":
                events.append(("END", int(p[1]), int(p[2]), int(p[3])))
                break  # first maze only
    return name, walls, events


def px(x, y):
    """top-left pixel of cell (x, y); y = 0 is the bottom row"""
    return MARGIN + x * CELL, HEADER + MARGIN + (N - 1 - y) * CELL


def centre_px(x, y):
    X, Y = px(x, y)
    return X + CELL / 2, Y + CELL / 2


def wall_segment(x, y, d):
    X, Y = px(x, y)
    if d == 0:
        return (X, Y, X + CELL, Y)
    if d == 1:
        return (X + CELL, Y, X + CELL, Y + CELL)
    if d == 2:
        return (X, Y + CELL, X + CELL, Y + CELL)
    return (X, Y, X, Y + CELL)


def known_shortest(seen, visited):
    """BFS start -> centre over visited cells and sensed walls"""
    prev = {(0, 0): None}
    q = deque([(0, 0)])
    while q:
        c = q.popleft()
        if c in CENTRE:
            path = []
            while c is not None:
                path.append(c)
                c = prev[c]
            return path[::-1]
        x, y = c
        for d in range(4):
            if seen.get((x, y, d)):
                continue
            n = (x + DX[d], y + DY[d])
            if not (0 <= n[0] < N and 0 <= n[1] < N) or n not in visited or n in prev:
                continue
            prev[n] = c
            q.append(n)
    return []


def shade(dist):
    """flood fill distance -> background colour (near goal = brighter), 6 levels"""
    if dist >= MAXD:
        return BG
    t = round(max(0.0, 1.0 - dist / 40.0) * 5) / 5
    return tuple(int(BG[i] + (c - BG[i]) * 0.35 * t) for i, c in enumerate((56, 189, 248)))


class Scene:
    def __init__(self, walls):
        self.walls = walls
        self.seen = {}
        self.visited = {(0, 0)}
        self.trails = [[(0, 0)], [], []]
        self.dists = None
        self.candidates = set()
        self.robot = (0, 0, 0)
        self.phase = 0
        self.plan = None
        for (x, y) in walls:
            if x == 0: self.seen[(x, y, 3)] = True
            if x == N - 1: self.seen[(x, y, 1)] = True
            if y == 0: self.seen[(x, y, 2)] = True
            if y == N - 1: self.seen[(x, y, 0)] = True

    def sense(self, x, y, d):
        for t in (0, 1, 3):
            dd = (d + t) % 4
            if self.walls[(x, y)][dd]:
                self.seen[(x, y, dd)] = True
                nx, ny = x + DX[dd], y + DY[dd]
                if 0 <= nx < N and 0 <= ny < N:
                    self.seen[(nx, ny, (dd + 2) % 4)] = True

    def draw(self, label, stats):
        img = Image.new("RGB", (W, H), BG)
        g = ImageDraw.Draw(img)
        speed = self.phase == 2

        for y in range(N):
            for x in range(N):
                X, Y = px(x, y)
                fill = shade(self.dists[y * N + x]) if (self.dists and not speed) else BG
                if (x, y) in CENTRE:
                    fill = GOAL
                g.rectangle([X, Y, X + CELL, Y + CELL], fill=fill)
                if (x, y) in self.visited and (x, y) not in CENTRE:
                    cx, cy = centre_px(x, y)
                    g.ellipse([cx - 1.5, cy - 1.5, cx + 1.5, cy + 1.5], fill=SUBTEXT)

        for (x, y) in self.candidates:
            X, Y = px(x, y)
            g.rectangle([X + 4, Y + 4, X + CELL - 4, Y + CELL - 4], outline=CANDIDATE, width=2)

        for (x, y), w in self.walls.items():
            for d in range(4):
                if w[d]:
                    g.line(wall_segment(x, y, d), fill=WALL_HIDDEN, width=2)
        for (x, y, d), v in self.seen.items():
            if v:
                g.line(wall_segment(x, y, d), fill=WALL_SEEN, width=3)

        for ph in (0, 1):
            pts = self.trails[ph]
            if len(pts) > 1:
                colour = TRAIL_DIM[ph] if speed else TRAIL[ph]
                g.line([centre_px(*p) for p in pts], fill=colour, width=2, joint="curve")
        if self.plan:
            g.line([centre_px(*p) for p in self.plan], fill=TRAIL_DIM[0], width=1)
        if len(self.trails[2]) > 1:
            g.line([centre_px(*p) for p in self.trails[2]], fill=TRAIL[2], width=5, joint="curve")

        for y in range(N + 1):
            for x in range(N + 1):
                X, Y = MARGIN + x * CELL, HEADER + MARGIN + (N - y) * CELL
                g.rectangle([X - 1, Y - 1, X + 1, Y + 1], fill=POST)

        x, y, d = self.robot
        cx, cy = centre_px(x, y)
        r = CELL * 0.32
        hx, hy = DX[d], -DY[d]          # screen y grows downward
        qx, qy = -hy, hx
        tip = (cx + hx * r, cy + hy * r)
        a = (cx - hx * r * 0.7 + qx * r * 0.75, cy - hy * r * 0.7 + qy * r * 0.75)
        b = (cx - hx * r * 0.7 - qx * r * 0.75, cy - hy * r * 0.7 - qy * r * 0.75)
        g.polygon([tip, a, b], fill=ROBOT)

        g.text((MARGIN, 8), label, font=font(16, True), fill=TEXT)
        g.text((MARGIN, 30), stats, font=font(12), fill=SUBTEXT)

        f = font(11)
        items = [("line", WALL_SEEN, "walls sensed"), ("line", TRAIL[0], "to centre"),
                 ("line", TRAIL[1], "back to start"), ("box", CANDIDATE, "may hold a shorter route"),
                 ("line", TRAIL[2], "speed run")]
        x0, y0 = MARGIN, H - FOOTER + 4
        xx, yy = x0, y0
        for kind, colour, text in items:
            wtxt = 22 + g.textlength(text, font=f) + 12
            if xx + wtxt > W - MARGIN:
                xx, yy = x0, yy + 18
            if kind == "line":
                g.line([xx, yy + 7, xx + 14, yy + 7], fill=colour, width=3)
            else:
                g.rectangle([xx + 2, yy + 1, xx + 12, yy + 12], outline=colour, width=2)
            g.text((xx + 18, yy), text, font=f, fill=SUBTEXT)
            xx += wtxt
        return img


def main():
    if len(sys.argv) != 3:
        print(__doc__)
        return 2
    name, walls, events = parse(sys.argv[1])
    sc = Scene(walls)
    frames, durations = [], []
    cells = [0, 0, 0]
    last_dir = 0
    turns = 0

    def stats():
        s = f"Maze {name}  |  explored {cells[0] + cells[1]} cells, {turns} turns"
        if sc.phase == 2:
            s += f"  |  speed run {cells[2]} cells"
        return s

    def add(label, ms):
        frames.append(sc.draw(label, stats()))
        durations.append(ms)

    add(LABELS[0], 900)
    for ev in events:
        if ev[0] == "S":
            sc.sense(*ev[1:])
        elif ev[0] == "M":
            _, x, y, d, ph, dists = ev
            if ph != sc.phase:
                if ph == 2:
                    # pause on the proven path before the speed run
                    sc.phase = 2
                    sc.candidates = set()
                    sc.plan = known_shortest(sc.seen, sc.visited)
                    add(f"Shortest path proven: {len(sc.plan) - 1} cells", 1500)
                sc.phase = ph
                sc.trails[ph] = [(x, y)]
            if ph < 2 and d != last_dir:
                turns += 1
            last_dir = d
            sc.dists = dists
            sc.candidates = set()
            if ph == 1 and dists[0] != 0:
                sc.candidates = {(i % N, i // N) for i, v in enumerate(dists)
                                 if v == 0 and (i % N, i // N) not in CENTRE}
            nx, ny = x + DX[d], y + DY[d]
            cells[ph] += 1
            sc.visited.add((nx, ny))
            sc.trails[ph].append((nx, ny))
            sc.robot = (nx, ny, d)
            add(LABELS[ph], 40 if ph == 2 else 65)
            if (nx, ny) in CENTRE and ph == 0:
                durations[-1] = 900
            if (nx, ny) == (0, 0) and ph == 1:
                durations[-1] = 900
        elif ev[0] == "END":
            sc.robot = (ev[1], ev[2], ev[3])
            add(f"Speed run done: {cells[2]} cells, start to centre", 3500)

    # one palette built from a sample of all frames keeps colours stable
    sample = frames[::max(1, len(frames) // 12)] + [frames[-1]]
    sheet = Image.new("RGB", (W, H * len(sample)))
    for i, f in enumerate(sample):
        sheet.paste(f, (0, H * i))
    pal = sheet.quantize(colors=64, method=Image.Quantize.MEDIANCUT)
    out = [f.quantize(palette=pal, dither=Image.Dither.NONE) for f in frames]
    out[0].save(sys.argv[2], save_all=True, append_images=out[1:], duration=durations,
                loop=0, optimize=True, disposal=1)
    print(f"{sys.argv[2]}: {len(out)} frames, maze {name}, cells {cells}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
