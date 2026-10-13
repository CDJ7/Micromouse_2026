#!/usr/bin/env python3
"""Generate a small test maze in the micromouseonline/mazefiles text format.

Usage: gen_maze.py SIZE SEED LOOP_FRACTION > maze.txt

The maze follows the usual competition rules: closed border, the start cell
(0,0) is open only to the north, the 2x2 center is hollow with exactly one
entrance, and a fraction of the other walls is removed so there are loops
(several candidate routes, which is what makes route choice interesting).
"""
import random
import sys


def main():
    n, seed, loops = int(sys.argv[1]), int(sys.argv[2]), float(sys.argv[3])
    rnd = random.Random(seed)
    lo, hi = (n - 1) // 2, n // 2
    center = {(x, y) for x in range(lo, hi + 1) for y in range(lo, hi + 1)}
    d = [(0, 1), (1, 0), (0, -1), (-1, 0)]  # n e s w
    wall = [[[True] * 4 for _ in range(n)] for _ in range(n)]

    def carve(x, y, k):
        nx, ny = x + d[k][0], y + d[k][1]
        wall[x][y][k] = False
        wall[nx][ny][(k + 2) % 4] = False

    # Depth-first spanning tree over every cell except the center block.
    # (0,0) may only be entered from the north, so block its east side.
    seen = {(0, 0)}
    stack = [(0, 0)]
    while stack:
        x, y = stack[-1]
        opts = []
        for k, (dx, dy) in enumerate(d):
            nx, ny = x + dx, y + dy
            if not (0 <= nx < n and 0 <= ny < n) or (nx, ny) in seen or (nx, ny) in center:
                continue
            if (x, y) == (0, 0) and k != 0:
                continue
            opts.append(k)
        if not opts:
            stack.pop()
            continue
        k = rnd.choice(opts)
        carve(x, y, k)
        seen.add((x + d[k][0], y + d[k][1]))
        stack.append((x + d[k][0], y + d[k][1]))

    # Hollow center: open its inner walls, then add a single entrance.
    for (x, y) in center:
        for k, (dx, dy) in enumerate(d):
            if (x + dx, y + dy) in center:
                wall[x][y][k] = False
    entrances = []
    for (x, y) in center:
        for k, (dx, dy) in enumerate(d):
            nx, ny = x + dx, y + dy
            if (nx, ny) not in center and 0 <= nx < n and 0 <= ny < n:
                entrances.append((x, y, k))
    x, y, k = rnd.choice(entrances)
    carve(x, y, k)

    # Remove some walls to create loops (never near the start or the center).
    cand = []
    for x in range(n):
        for y in range(n):
            for k in (0, 1):
                nx, ny = x + d[k][0], y + d[k][1]
                if not (0 <= nx < n and 0 <= ny < n) or not wall[x][y][k]:
                    continue
                if (x, y) in center or (nx, ny) in center:
                    continue
                if (x, y) == (0, 0) or (nx, ny) == (0, 0):
                    continue
                cand.append((x, y, k))
    rnd.shuffle(cand)
    for x, y, k in cand[: int(len(cand) * loops)]:
        carve(x, y, k)

    # Draw it.
    out = []
    for row in range(n - 1, -1, -1):
        top = "o"
        mid = ""
        for x in range(n):
            top += ("---" if wall[x][row][0] else "   ") + "o"
            mid += ("|" if wall[x][row][3] else " ") + "   "
        mid += "|" if wall[n - 1][row][1] else " "
        out.append(top)
        out.append(mid)
    out.append("o" + "---o" * n)
    sys.stdout.write("\n".join(out) + "\n")


main()
