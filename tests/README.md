# Offline tests

A small harness that runs the real `MMS_Code/main.c` and `MMS_Code/solver.c` against a fake simulator, so the solver can be checked without the mms GUI. It is **not** part of the mms build (`gcc -o a.exe API.c main.c solver.c`).

```
./tests/run_tests.sh [path/to/micromouseonline/mazefiles]
```

Needs `gcc` and `python3`. The optional argument is a clone of
[micromouseonline/mazefiles](https://github.com/micromouseonline/mazefiles); without it only the mazes in `tests/mazes/` are used.

| File | Purpose |
| --- | --- |
| `run_tests.sh` | Builds everything with `-Wall -Wextra -pedantic` and runs all checks |
| `sim_api.c`, `sim.h` | Fake `API.h` backed by a maze file; records every move and validates display calls |
| `test_main.c` | The checks below, plus an independent shortest-route oracle |
| `mock_mms.py` | Runs the real `a.out` against a mock of the mms stdin/stdout protocol (tests `API.c`) |
| `gen_maze.py` | Generates the 8×8 and 10×10 mazes in `mazes/` |
| `mazes/` | Generated test mazes (loops, hollow center, single center entrance) |

Mazes load from the mms `.num` and `.map` formats and the mazefiles text format.

## What is checked

1. The mouse never drives through a wall.
2. It reaches the center.
3. The final route has the same cost as the cheapest route found by searching the true maze (Bellman-Ford, independent of the solver). Cost means cells plus turns, weighted by `COST_FORWARD` and `COST_TURN`, so the weighted runs prove the planner prices turns correctly.
4. Explore + return stays within the exploration budget.
5. Nothing the solver believes about the walls is wrong.
6. If it claims the whole maze is mapped, every reachable cell really is.
7. Display calls are valid (cell text at most 10 characters, color letters, coordinates).
8. `solver.c` and `main.c` never touch stdout or stdin, the exact mms build command compiles with no warnings, and with `LOG` redirected `solver.c` needs nothing from the C library except the `API_*` functions (a check that it can move to a microcontroller).
9. The reset button (`wasReset`, `ackReset`) restarts a run cleanly.
10. The real program speaks valid mms protocol and logs only to stderr.

## Extra runs

| Run | What it shows |
| --- | --- |
| Tiny budget | Exploration stops in time and the mouse still gets home and runs a valid route. |
| Route-only mode | With `EXPLORE_MAP_WHOLE_MAZE=0` the mouse stops once the best route is proven (for example cost 272 instead of 836 on `AAMC15Maze.txt`). |
| Weighted costs | A turn costing 3 cells: the final route is still the cheapest. |
| Clock budget | Budget measured with a (modeled) millisecond clock, cell 250 ms, turn 400 ms. |
| Noisy sensors | 2% of wall readings flipped. With 5 reads per wall and front checks, every run is crash-free. Without them most runs crash. |
| `--all` | Every maze in the mazefiles collection that fits in 16×16. |

Run `./tests/run_tests.sh <mazefiles folder> --all` for everything. The workflow in `.github/workflows/tests.yml` does this on every push.

## Known limitations

- The budget is only enforced once a fully known route to the center exists (otherwise there would be no score). If the budget is too small to even find the center, the mouse keeps searching and logs a warning. With a 250-action budget, `gen10-b.txt` finds the center after 224 actions and ends 14 over budget; with 300 it passes.
- The clock in the simulator is a model (`SIM_MS_PER_CELL`, `SIM_MS_PER_TURN`), because mms moves the mouse instantly. The real mouse needs a real timer.
- The noise test flips readings at random. Real IR errors are systematic (ambient light, wall reflectivity, a mouse that is not centered), so it needs testing on the real mouse as well.
