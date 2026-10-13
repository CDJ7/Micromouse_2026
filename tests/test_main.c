/*
 * test_main.c - offline test runner for the solver.
 *
 * Runs the REAL main.c / solver.c (main.c is compiled with -Dmain=mms_main)
 * against a fake simulator (sim_api.c) loaded with a maze file, then checks:
 *   1. the mouse never drives through a wall
 *   2. it reaches the center
 *   3. the final route is as short as the true shortest route (full maze)
 *   4. explore + return stays within the exploration budget
 *   5. everything the solver believes about the walls is true
 *   6. if it claims the whole maze is mapped, every reachable cell is known
 *   7. all display calls are valid (text length, colors, coordinates)
 *
 * Usage: test_runner MAZE_FILE [--relaxed] [--reset-at N] [--quiet]
 *   --relaxed   do not require an optimal final route (for tiny budgets)
 *   --reset-at  press the reset button on the Nth wasReset() poll
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "sim.h"
#include "solver.h"

int mms_main(void); /* the real main() from MMS_Code/main.c */

#define INF 1000000

static const int DX[4] = {0, 1, 0, -1};
static const int DY[4] = {1, 0, -1, 0};

static int failures;

static void check(int ok, const char *what, const char *detail) {
    printf("    [%s] %s%s%s\n", ok ? "PASS" : "FAIL", what, detail[0] ? " - " : "", detail);
    if (!ok) failures++;
}

static int inGoal(const SimMaze *m, int x, int y) {
    return x >= (m->w - 1) / 2 && x <= m->w / 2 && y >= (m->h - 1) / 2 && y <= m->h / 2;
}

/* Independent shortest-route oracle: plain Bellman-Ford over the TRUE maze.
 * Cost = cells + turns, starting at (0,0) facing `startHeading`. Shares no code with the solver. */
static int oracleCost(const SimMaze *m, int startHeading) {
    static int d[SIM_MAX][SIM_MAX][4];
    int x, y, h, changed = 1, best = INF;
    for (x = 0; x < SIM_MAX; x++)
        for (y = 0; y < SIM_MAX; y++)
            for (h = 0; h < 4; h++) d[x][y][h] = INF;
    d[0][0][startHeading] = 0;
    while (changed) {
        changed = 0;
        for (x = 0; x < m->w; x++)
            for (y = 0; y < m->h; y++)
                for (h = 0; h < 4; h++) {
                    int c = d[x][y][h], nh;
                    if (c >= INF) continue;
                    if (!m->wall[x][y][h] && d[x + DX[h]][y + DY[h]][h] > c + 1) {
                        d[x + DX[h]][y + DY[h]][h] = c + 1;
                        changed = 1;
                    }
                    for (nh = 1; nh <= 3; nh += 2) {
                        int hh = (h + nh) & 3;
                        if (d[x][y][hh] > c + 1) {
                            d[x][y][hh] = c + 1;
                            changed = 1;
                        }
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

/* mms-style score estimate from the action log (best run = fewest turns + effective distance). */
static double estimateScore(const SimMaze *m, const SimState *s, double *bestRunOut) {
    double best = 1e9;
    int running = 0, i;
    double runTurns = 0, runEff = 0;
    int n = s->nActions < SIM_LOG_MAX ? s->nActions : SIM_LOG_MAX;
    for (i = 0; i < n; i++) {
        const SimAction *a = &s->log[i];
        if (a->kind == 'F' && a->x == 0 && a->y == 0) {
            running = 1; /* a new run starts when the mouse leaves the start cell */
            runTurns = runEff = 0;
        }
        if (!running) continue;
        if (a->kind == 'F') {
            runEff += a->callIdx <= 2 ? 1.0 : 0.5;
            if (inGoal(m, a->x + DX[a->heading], a->y + DY[a->heading])) {
                if (runTurns + runEff < best) best = runTurns + runEff;
                running = 0;
            }
        } else {
            runTurns += 1;
        }
    }
    if (bestRunOut) *bestRunOut = best;
    return best + 0.1 * (s->turns + s->effDist);
}

int main(int argc, char **argv) {
    const char *path = NULL;
    int relaxed = 0, resetAt = 0, quiet = 0, i;
    const SimMaze *m;
    SimState *s;
    SolverStats st;
    char detail[200];
    int x, y, d, finalStart, finalCost, oracle, oracleBest, preFinal, bad, startOk, startHeading;
    int reach[SIM_MAX][SIM_MAX];
    int stackX[SIM_MAX * SIM_MAX], stackY[SIM_MAX * SIM_MAX], sp = 0;
    double bestRun = 0, score;

    for (i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--relaxed")) relaxed = 1;
        else if (!strcmp(argv[i], "--quiet")) quiet = 1;
        else if (!strcmp(argv[i], "--reset-at") && i + 1 < argc) resetAt = atoi(argv[++i]);
        else path = argv[i];
    }
    if (!path) {
        fprintf(stderr, "usage: %s MAZE_FILE [--relaxed] [--reset-at N] [--quiet]\n", argv[0]);
        return 2;
    }
    if (sim_load(path) != 0) {
        fprintf(stderr, "could not load maze %s\n", path);
        return 2;
    }
    m = sim_maze();
    s = sim_state();
    sim_start();
    s->resetAtCall = resetAt;

    mms_main(); /* the real program: init, loop, reset handling, solver */

    solver_getStats(&st);
    printf("%s (%dx%d)%s\n", path, m->w, m->h, resetAt ? " with reset button" : "");

    /* 1. never through a wall (replay the log against the true maze) */
    bad = s->crashes;
    for (i = 0; i < s->nActions && i < SIM_LOG_MAX; i++)
        if (s->log[i].kind == 'F' && m->wall[s->log[i].x][s->log[i].y][s->log[i].heading]) bad++;
    sprintf(detail, "%d crashes", bad);
    check(bad == 0, "never drives through a wall", detail);

    /* 2. reaches the center (and ends there) */
    sprintf(detail, "ended at (%d,%d)", s->x, s->y);
    check(st.goalReached && inGoal(m, s->x, s->y), "reaches the center", detail);

    /* The final run is the last stretch the solver reports (cells + turns).
     * Check from the log that it really starts at the start cell. */
    finalStart = s->nActions - (st.finalCells + st.finalTurns);
    finalCost = s->nActions - finalStart;
    preFinal = finalStart;
    startOk = finalStart >= 0 && finalStart < SIM_LOG_MAX && s->log[finalStart].x == 0 &&
              s->log[finalStart].y == 0;
    startHeading = startOk ? s->log[finalStart].heading : 0;
    sprintf(detail, "final run begins at log index %d", finalStart);
    check(startOk, "final run starts at the start cell", detail);
    oracle = oracleCost(m, startHeading);
    oracleBest = INF;
    for (i = 0; i < 4; i++)
        if (oracleCost(m, i) < oracleBest) oracleBest = oracleCost(m, i);

    /* 3. final route matches the true shortest route */
    sprintf(detail, "final run %d actions, true shortest %d from that start heading, %d from the best one",
            finalCost, oracle, oracleBest);
    if (relaxed) {
        check(finalCost >= oracle,
              "final route is valid (relaxed: not required to be optimal)", detail);
    } else {
        check(finalCost == oracleBest,
              "final route equals the true shortest route (full maze)", detail);
    }

    /* 4. within the exploration budget */
    sprintf(detail, "%d explore + return actions, budget %d", preFinal, st.budget);
    check(preFinal <= st.budget, "finishes within the exploration budget", detail);

    /* 5. solver's map never contradicts the real maze */
    bad = 0;
    for (x = 0; x < m->w; x++)
        for (y = 0; y < m->h; y++)
            for (d = 0; d < 4; d++) {
                int e = solver_getEdge(x, y, d);
                if (e != 0 && e != (m->wall[x][y][d] ? 2 : 1)) bad++;
            }
    sprintf(detail, "%d wrong sides", bad);
    check(bad == 0, "map matches the real walls", detail);

    /* 6. "whole maze mapped" really means every reachable cell is known */
    memset(reach, 0, sizeof reach);
    reach[0][0] = 1;
    stackX[sp] = 0; stackY[sp] = 0; sp++;
    while (sp > 0) {
        int cx = stackX[--sp], cy = stackY[sp];
        for (d = 0; d < 4; d++) {
            int nx = cx + DX[d], ny = cy + DY[d];
            if (m->wall[cx][cy][d] || reach[nx][ny]) continue;
            reach[nx][ny] = 1;
            stackX[sp] = nx; stackY[sp] = ny; sp++;
        }
    }
    bad = 0;
    for (x = 0; x < m->w; x++)
        for (y = 0; y < m->h; y++)
            if (reach[x][y])
                for (d = 0; d < 4; d++)
                    if (solver_getEdge(x, y, d) == 0) { bad++; break; }
    sprintf(detail, "claims mapped: %s, %d reachable cells with unknown sides",
            st.wholeMazeMapped ? "yes" : "no", bad);
    check(!st.wholeMazeMapped || bad == 0, "'whole maze mapped' claim is true", detail);

    /* 7. valid display calls */
    sprintf(detail, "%d bad out of %d display calls", s->badCalls, s->displayCalls);
    check(s->badCalls == 0, "display calls are valid", detail);

    if (resetAt) {
        sprintf(detail, "%d reset(s) handled", s->resetCalls);
        check(s->resetCalls == 1, "reset button handled", detail);
    }

    score = estimateScore(m, s, &bestRun);
    if (!quiet || failures)
        printf("    info: visited %d/%d, fully mapped %d/%d, proven optimal %s, whole maze %s,\n"
               "          explore %d + return %d actions, mms score estimate %.1f (best run %.1f)\n",
               st.visitedCells, st.totalCells, st.mappedCells, st.totalCells,
               st.routeProven ? "yes" : "no", st.wholeMazeMapped ? "yes" : "no",
               st.exploreActions, st.returnActions, score, bestRun);
    return failures ? 1 : 0;
}
