#!/usr/bin/env python3
"""
render_gif.py - animate a simulator trace as a GIF

    ./build/sim --random 1 7 --trace trace.txt
    python3 render_gif.py trace.txt ../../docs/exploration.gif

Draws the true maze faintly, the walls the robot has sensed in red, the
firmware's flood fill distances as background shading, the path driven
(cyan to the centre, amber back to start) and, on the last frame, the
shortest start-to-centre path the robot knows.

Requires Pillow.
"""

import sys
from collections import deque
from PIL import Image, ImageDraw, ImageFont

N = 16
CELL = 26
MARGIN = 18
HEADER = 52
FOOTER = 26
W = MARGIN * 2 + CELL * N
H = HEADER + MARGIN * 2 + CELL * N + FOOTER
DX = [0, 1, 0, -1]
DY = [1, 0, -1, 0]
CENTRE = {(7, 7), (7, 8), (8, 7), (8, 8)}

BG = (17, 24, 39)
POST = (75, 85, 99)
WALL_HIDDEN = (71, 85, 105)
WALL_SEEN = (248, 113, 113)
GOAL = (22, 101, 52)
TRAIL = [(34, 211, 238), (251, 191, 36)]
BEST = (74, 222, 128)
TEXT = (229, 231, 235)
SUBTEXT = (156, 163, 175)
ROBOT = (255, 255, 255)


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
    """BFS start -> centre over visited cells and sensed walls (mirrors the sim metric)"""
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
    """flood fill distance -> background colour (near goal = brighter)"""
    if dist >= 9999:
        return BG
    # 6 discrete levels keep the GIF palette small
    t = round(max(0.0, 1.0 - dist / 40.0) * 5) / 5
    return tuple(int(BG[i] + (c - BG[i]) * 0.35 * t) for i, c in enumerate((56, 189, 248)))


def draw_frame(walls, seen, dists, visited, trails, robot, label, stats, best=None):
    img = Image.new("RGB", (W, H), BG)
    g = ImageDraw.Draw(img)

    for y in range(N):
        for x in range(N):
            X, Y = px(x, y)
            fill = shade(dists[y * N + x]) if dists else BG
            if (x, y) in CENTRE:
                fill = GOAL
            g.rectangle([X, Y, X + CELL, Y + CELL], fill=fill)
            if (x, y) in visited and (x, y) not in CENTRE:
                cx, cy = centre_px(x, y)
                g.ellipse([cx - 1.5, cy - 1.5, cx + 1.5, cy + 1.5], fill=SUBTEXT)

    for (x, y), w in walls.items():
        for d in range(4):
            if w[d]:
                g.line(wall_segment(x, y, d), fill=WALL_HIDDEN, width=2)
    for (x, y, d), v in seen.items():
        if v:
            g.line(wall_segment(x, y, d), fill=WALL_SEEN, width=3)

    for phase, pts in enumerate(trails):
        if len(pts) > 1:
            g.line([centre_px(*p) for p in pts], fill=TRAIL[phase], width=2, joint="curve")
    if best:
        g.line([centre_px(*p) for p in best], fill=BEST, width=5, joint="curve")

    for y in range(N + 1):
        for x in range(N + 1):
            X, Y = MARGIN + x * CELL, HEADER + MARGIN + (N - y) * CELL
            g.rectangle([X - 1, Y - 1, X + 1, Y + 1], fill=POST)

    if robot:
        x, y, d = robot
        cx, cy = centre_px(x, y)
        r = CELL * 0.32
        # triangle pointing in heading d (screen y grows downward)
        hx, hy = DX[d], -DY[d]
        px_, py_ = -hy, hx
        tip = (cx + hx * r, cy + hy * r)
        a = (cx - hx * r * 0.7 + px_ * r * 0.75, cy - hy * r * 0.7 + py_ * r * 0.75)
        b = (cx - hx * r * 0.7 - px_ * r * 0.75, cy - hy * r * 0.7 - py_ * r * 0.75)
        g.polygon([tip, a, b], fill=ROBOT)

    g.text((MARGIN, 8), label, font=font(16, True), fill=TEXT)
    g.text((MARGIN, 30), stats, font=font(12), fill=SUBTEXT)

    # legend
    f = font(11)
    x, y = MARGIN, H - FOOTER + 4
    for colour, text in [(WALL_SEEN, "walls sensed"), (TRAIL[0], "to centre"),
                         (TRAIL[1], "back to start"), (BEST, "known shortest path")]:
        g.line([x, y + 7, x + 14, y + 7], fill=colour, width=3)
        g.text((x + 18, y), text, font=f, fill=SUBTEXT)
        x += 22 + g.textlength(text, font=f) + 12
    return img


def main():
    if len(sys.argv) != 3:
        print(__doc__)
        return 2
    name, walls, events = parse(sys.argv[1])
    seen = {}
    for (x, y), w in walls.items():
        if x == 0: seen[(x, y, 3)] = True
        if x == N - 1: seen[(x, y, 1)] = True
        if y == 0: seen[(x, y, 2)] = True
        if y == N - 1: seen[(x, y, 0)] = True

    def mark(x, y, d):
        if walls[(x, y)][d]:
            seen[(x, y, d)] = True
            nx, ny = x + DX[d], y + DY[d]
            if 0 <= nx < N and 0 <= ny < N:
                seen[(nx, ny, (d + 2) % 4)] = True

    frames, durations = [], []
    visited = {(0, 0)}
    trails = [[(0, 0)], []]
    dists, cells, turns, phase = None, 0, 0, 0
    last_dir = 0
    labels = ["Exploring to centre", "Returning to start"]

    def stats():
        return f"Maze {name}  |  cells driven {cells}  |  turns {turns}"

    frames.append(draw_frame(walls, seen, dists, visited, trails, (0, 0, 0), labels[0], stats()))
    durations.append(800)

    for ev in events:
        if ev[0] == "S":
            _, x, y, d = ev
            for t in (0, 1, 3):
                mark(x, y, (d + t) % 4)
        elif ev[0] == "M":
            _, x, y, d, ph, dists = ev
            if ph != phase:
                phase = ph
                trails[1] = [(x, y)]
            if d != last_dir:
                turns += 1
            last_dir = d
            nx, ny = x + DX[d], y + DY[d]
            cells += 1
            visited.add((nx, ny))
            trails[phase].append((nx, ny))
            frames.append(draw_frame(walls, seen, dists, visited, trails, (nx, ny, d),
                                     labels[phase], stats()))
            durations.append(70)
            if (nx, ny) in CENTRE and phase == 0:
                durations[-1] = 900
        elif ev[0] == "END":
            best = known_shortest(seen, visited)
            label = f"Back at start - known shortest path {len(best) - 1} cells" if best else "Back at start"
            frames.append(draw_frame(walls, seen, None, visited, trails, (ev[1], ev[2], ev[3]),
                                     label, stats(), best))
            durations.append(4000)

    # one palette built from a sample of all frames keeps colours stable
    sample = frames[::max(1, len(frames) // 12)] + [frames[-1]]
    sheet = Image.new("RGB", (W, H * len(sample)))
    for i, f in enumerate(sample):
        sheet.paste(f, (0, H * i))
    pal = sheet.quantize(colors=64, method=Image.Quantize.MEDIANCUT)
    out = [f.quantize(palette=pal, dither=Image.Dither.NONE) for f in frames]
    out[0].save(sys.argv[2], save_all=True, append_images=out[1:], duration=durations,
                loop=0, optimize=True, disposal=1)
    print(f"{sys.argv[2]}: {len(out)} frames, maze {name}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
