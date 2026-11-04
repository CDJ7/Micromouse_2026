/*
 * test_hw.c - PC test of the real-mouse firmware.
 *
 * Runs main_hw.c (compiled as hw_main), the motion controller, the IR sensor
 * code, api_hw.c and the real solver (MMS_Code/solver.c) against the simulated
 * mouse in fake_hal.c, in a maze loaded from a file. The only thing not covered
 * is the register-level code in hal_stm32/, which cannot run on a PC.
 *
 * Usage: test_hw MAZE_FILE [--quiet]
 *        test_hw --timing MAZE  measure how long one cell and one turn take
 *
 * Checks: the body never touches a wall, the mouse ends in the center, the
 * solver's map matches the real walls, the final route is as cheap as the true
 * cheapest route, and exploring stopped within the time budget.
 */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "config.h"
#include "fake_hal.h"
#include "hal.h"
#include "motion.h"
#include "sim.h"
#include "solver.h"

int hw_main(void); /* main() from src/main_hw.c */

#define INF 1000000000
static const int DX[4] = {0, 1, 0, -1};
static const int DY[4] = {1, 0, -1, 0};
static int failures;
static uint32_t solverStartMs;

/* main_hw.c calls this instead of solver_init (it is compiled with
 * -Dsolver_init=hw_test_solver_init) so the test knows when the clock started. */
void hw_test_solver_init(void) {
    solverStartMs = hal_millis();
    solver_init();
}

static void check(int ok, const char *what, const char *detail) {
    printf("    [%s] %s%s%s\n", ok ? "PASS" : "FAIL", what, detail[0] ? " - " : "", detail);
    if (!ok) failures++;
}

static int inGoal(const SimMaze *m, int x, int y) {
    return x >= (m->w - 1) / 2 && x <= m->w / 2 && y >= (m->h - 1) / 2 && y <= m->h / 2;
}

/* Independent cheapest-route search over the true maze (Bellman-Ford). */
static int oracle(const SimMaze *m, int cf, int ct) {
    static int d[SIM_MAX][SIM_MAX][4];
    int x, y, h, changed = 1, best = INF;
    for (x = 0; x < SIM_MAX; x++)
        for (y = 0; y < SIM_MAX; y++)
            for (h = 0; h < 4; h++) d[x][y][h] = INF;
    d[0][0][0] = 0;
    while (changed) {
        changed = 0;
        for (x = 0; x < m->w; x++)
            for (y = 0; y < m->h; y++)
                for (h = 0; h < 4; h++) {
                    int c = d[x][y][h], k;
                    if (c >= INF) continue;
                    if (!m->wall[x][y][h] && d[x + DX[h]][y + DY[h]][h] > c + cf) {
                        d[x + DX[h]][y + DY[h]][h] = c + cf;
                        changed = 1;
                    }
                    for (k = 1; k <= 3; k += 2)
                        if (d[x][y][(h + k) & 3] > c + ct) {
                            d[x][y][(h + k) & 3] = c + ct;
                            changed = 1;
                        }
                }
    }
    for (x = 0; x < m->w; x++)
        for (y = 0; y < m->h; y++)
            if (inGoal(m, x, y))
                for (h = 0; h < 4; h++)
                    if (d[x][y][h] < best) best = d[x][y][h];
    return best;
}

static int timing(const char *mazePath) {
    uint32_t t0;
    FakeMouse *mouse = fake_mouse();
    if (sim_load(mazePath) != 0) return 2;
    hal_init();
    motion_init();
    t0 = hal_millis();
    motion_forward(1);
    printf("one cell, stopping:      %u ms\n", (unsigned)(hal_millis() - t0));
    t0 = hal_millis();
    motion_turn(1);
    printf("one 90-degree turn:      %u ms\n", (unsigned)(hal_millis() - t0));
    motion_turn(-1);
    printf("crashes %d, off-center %.1f mm, heading error %.1f deg\n", mouse->crashes,
           mouse->maxOffCenterMm, mouse->maxHeadingErrDeg);
    return 0;
}

/* hal_halt() jumps back here when the firmware says it is finished. */
static void runFirmware(void) {
    if (setjmp(fake_halt_jump) == 0) hw_main();
}

int main(int argc, char **argv) {
    const char *path = NULL;
    int quiet = 0, i, x, y, d, bad, cf, ct, finalCost, best;
    const SimMaze *m;
    FakeMouse *mouse;
    SolverStats st;
    char detail[240];
    double exploreMs;

    for (i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--timing") && i + 1 < argc) return timing(argv[i + 1]);
        if (!strcmp(argv[i], "--quiet")) quiet = 1;
        else path = argv[i];
    }
    if (!path || sim_load(path) != 0) {
        fprintf(stderr, "usage: %s MAZE_FILE [--quiet] | --timing MAZE_FILE\n", argv[0]);
        return 2;
    }
    m = sim_maze();
    mouse = fake_mouse();
    fake_reset();
    runFirmware();

    solver_getStats(&st);
    cf = st.costForward;
    ct = st.costTurn;
    printf("%s (%dx%d)\n", path, m->w, m->h);

    sprintf(detail, "%d wall hits", mouse->crashes);
    check(mouse->crashes == 0, "the mouse body never touches a wall", detail);

    sprintf(detail, "ended in cell (%d,%d)", (int)floor(mouse->x / 180.0), (int)floor(mouse->y / 180.0));
    check(inGoal(m, (int)floor(mouse->x / 180.0), (int)floor(mouse->y / 180.0)), "ends in the center",
          detail);

    bad = 0;
    for (x = 0; x < m->w; x++)
        for (y = 0; y < m->h; y++)
            for (d = 0; d < 4; d++) {
                int e = solver_getEdge(x, y, d);
                if (e != 0 && e != (m->wall[x][y][d] ? 2 : 1)) bad++;
            }
    sprintf(detail, "%d wrong sides (IR wall detection)", bad);
    check(bad == 0, "the map built from IR readings matches the real walls", detail);

    finalCost = st.finalCells * cf + st.finalTurns * ct;
    best = oracle(m, cf, ct);
    sprintf(detail, "final route cost %d, true cheapest %d", finalCost, best);
    check(finalCost == best, "the final route is the cheapest in the whole maze", detail);

    exploreMs = (double)mouse->lastLeaveStartMs - (double)solverStartMs;
    sprintf(detail, "%.0f ms used, budget %d ms", exploreMs, st.budget);
    check(exploreMs <= st.budget, "exploring and returning finished inside the time budget", detail);

    if (!quiet || failures)
        printf("    info: %.0f s total, explored %d/%d cells, off-center at most %.1f mm, heading error at\n"
               "          most %.1f deg while driving, %d sensor warnings, %d front-sensor vetoes\n",
               mouse->ticks / 1000.0, st.visitedCells, st.totalCells, mouse->maxOffCenterMm,
               mouse->maxHeadingErrDeg, st.contradictions, st.frontVetoes);
    return failures ? 1 : 0;
}
