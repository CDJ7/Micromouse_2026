/*
 * sim.h - fake simulator used by the offline tests.
 *
 * sim_api.c implements API.h on top of a maze loaded from a file, so the real
 * solver.c and main.c can run on a PC with no mms. The simulated mouse records
 * everything it is asked to do so test_main.c can check it afterwards.
 */
#ifndef SIM_H
#define SIM_H

#include <stdbool.h>

#define SIM_MAX 16
#define SIM_LOG_MAX 20000

typedef struct {
    int w, h;
    unsigned char wall[SIM_MAX][SIM_MAX][4]; /* 1 = wall; 0 n, 1 e, 2 s, 3 w */
} SimMaze;

/* Load .num, or the ASCII .map / mazefiles text format. Returns 0 on success. */
int sim_load(const char *path);
const SimMaze *sim_maze(void);

/* Everything the mouse did, one entry per turn or per cell driven. */
typedef struct {
    char kind;  /* 'F' forward one cell, 'L' / 'R' turn */
    int x, y;   /* position BEFORE the action */
    int heading;
    int callIdx; /* 1 for the first cell of a moveForward call, 2 for the next, ... */
} SimAction;

typedef struct {
    int x, y, heading;
    int nActions;
    SimAction log[SIM_LOG_MAX];
    int crashes;          /* moves into a wall */
    int badCalls;         /* invalid display calls (bad text, color, range) */
    int moveCalls;        /* number of moveForward calls */
    int displayCalls;     /* setWall/setColor/setText/clear* calls */
    int resetCalls;       /* ackReset calls */
    double effDist;       /* mms "effective distance", whole session */
    int turns;
    int resetAtCall;      /* make wasReset() true on this call number (0 = never) */
    int wasResetCalls;
} SimState;

SimState *sim_state(void);
void sim_start(void); /* mouse back to (0,0) facing north, log cleared */

#endif /* SIM_H */
