/*
 * solver.h - public interface of the maze solver.
 *
 *
 * main.c only needs solver_init / solver_step / solver_reset. The rest is for
 * the left-wall-follower example and for the offline test harness in tests/.
 */
#ifndef SOLVER_H
#define SOLVER_H

#include <stdbool.h>

/* Read the maze size, clear the map, and draw the initial debug display.
 * Call once at startup (and again after a reset has been acknowledged). */
void solver_init(void);

/* Do the next piece of work (one turn, or one forward move of one or more
 * cells). Returns false when the run is over (finished or failed). */
bool solver_step(void);

/* Throw away all state and clear the display. Call this when the simulator's
 * reset button was pressed, then API_ackReset(), then solver_init(). */
void solver_reset(void);

/* Reference example: one step of a simple left-wall follower. Not used by
 * default; call it from main() instead of solver_step() to try it. */
void solver_leftWallFollowerStep(void);

/* ---- read-only views, mainly for tests and debugging ---- */

typedef struct {
    int totalCells;
    int visitedCells;      /* cells the mouse has actually driven into */
    int mappedCells;       /* cells whose four walls are all known */
    int exploreActions;    /* turns + cells driven while exploring */
    int returnActions;     /* turns + cells driven on the way back */
    int budget;            /* exploration budget (explore + return) */
    int finalCells;        /* cells in the final run */
    int finalTurns;        /* turns in the final run */
    bool goalReached;      /* the center was reached at least once */
    bool routeProven;      /* the best route is known to be optimal */
    bool wholeMazeMapped;  /* every reachable cell was explored */
} SolverStats;

void solver_getStats(SolverStats *out);

/* Wall knowledge for one cell side: 0 unknown, 1 open, 2 wall.
 * dir is 0 = north, 1 = east, 2 = south, 3 = west. */
int solver_getEdge(int x, int y, int dir);

#endif /* SOLVER_H */
