#!/usr/bin/env python3
"""Protocol-level test: run the REAL mouse program (built with the exact mms
command) against a mock of the mms stdin/stdout protocol.

This is what checks API.c: every line the program prints on stdout must be a
valid mms command, replies must be what mms would send, and nothing else may
appear on stdout (logging must stay on stderr).

Usage: mock_mms.py PATH/TO/a.out MAZE.txt [-v]   (-v prints the program's stderr log)
"""
import subprocess
import sys

D = [(0, 1), (1, 0), (0, -1), (-1, 0)]
COLORS = set("kbacgorwyBCAGRY")


def load(path):
    lines = [l.rstrip("\r\n") for l in open(path) if l.strip()]
    h = (len(lines) - 1) // 2
    w = (len(lines[0]) - 1) // 4
    wall = {}
    for y in range(h):
        r = 2 * (h - 1 - y)
        for x in range(w):
            wall[x, y] = [lines[r][4 * x + 2] != " ", lines[r + 1][4 * x + 4] != " ",
                          lines[r + 2][4 * x + 2] != " ", lines[r + 1][4 * x] != " "]
    return w, h, wall


def main():
    exe, maze = sys.argv[1], sys.argv[2]
    w, h, wall = load(maze)
    p = subprocess.Popen([exe], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                         stderr=subprocess.PIPE, text=True, bufsize=1)
    x = y = hd = 0
    cmds = crashes = bad = 0
    effective = turns = 0
    reached = False
    first_tokens = set()

    def reply(s):
        p.stdin.write(s + "\n")
        p.stdin.flush()

    def err(msg):
        nonlocal bad
        bad += 1
        print("  BAD:", msg)

    while True:
        line = p.stdout.readline()
        if not line:
            break
        cmds += 1
        t = line.split()
        if not t:
            err("empty line on stdout")
            continue
        c, a = t[0], t[1:]
        first_tokens.add(c)
        if c == "mazeWidth": reply(str(w))
        elif c == "mazeHeight": reply(str(h))
        elif c in ("wallFront", "wallRight", "wallLeft"):
            k = {"wallFront": 0, "wallRight": 1, "wallLeft": 3}[c]
            reply("true" if wall[x, y][(hd + k) & 3] else "false")
        elif c == "moveForward":
            n = int(a[0]) if a else 1
            ok = n >= 1
            for i in range(n if ok else 0):
                if wall[x, y][hd]:
                    ok = False
                    break
                x += D[hd][0]; y += D[hd][1]
            if not ok:
                crashes += 1
            else:
                effective += n if n <= 2 else 2 + 0.5 * (n - 2)
            reply("ack" if ok else "crash")
        elif c == "turnRight": hd = (hd + 1) & 3; turns += 1; reply("ack")
        elif c == "turnLeft": hd = (hd + 3) & 3; turns += 1; reply("ack")
        elif c in ("setWall", "clearWall"):
            if len(a) != 3 or a[2] not in "nesw" or not (0 <= int(a[0]) < w and 0 <= int(a[1]) < h):
                err(line.strip())
        elif c == "setColor":
            if len(a) != 3 or a[2] not in COLORS or not (0 <= int(a[0]) < w and 0 <= int(a[1]) < h):
                err(line.strip())
        elif c == "setText":
            text = line.split(None, 3)[3].rstrip("\n") if len(t) >= 4 else ""
            if not text or len(text) > 10 or not (0 <= int(a[0]) < w and 0 <= int(a[1]) < h):
                err(line.strip())
        elif c in ("clearColor", "clearText"):
            if len(a) != 2: err(line.strip())
        elif c in ("clearAllColor", "clearAllText"): pass
        elif c == "wasReset": reply("false")
        elif c == "ackReset": reply("ack")
        else:
            err("unknown command: " + line.strip())

    p.wait()
    stderr = p.stderr.read()
    if "-v" in sys.argv:
        sys.stderr.write(stderr)
    gx, gy = (w - 1) // 2, (h - 1) // 2
    in_goal = gx <= x <= w // 2 and gy <= y <= h // 2
    summary = [l for l in stderr.splitlines() if l.startswith("[summary]")]
    ok = bad == 0 and crashes == 0 and p.returncode == 0 and in_goal and summary
    print(f"{maze}: {cmds} commands, {crashes} crashes, {bad} invalid commands, "
          f"exit code {p.returncode}, ended in goal: {in_goal}")
    print("  " + (summary[0] if summary else "no [summary] line on stderr"))
    print("  [%s] real a.out speaks valid mms protocol" % ("PASS" if ok else "FAIL"))
    sys.exit(0 if ok else 1)


main()
