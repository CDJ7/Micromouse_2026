/*
 * api_hw.c - the real mouse's implementation of API.h.
 *
 * This is the hardware twin of MMS_Code/API.c: the same functions, but
 * backed by the IR sensors and the motors instead of the simulator. The solver
 * (MMS_Code/solver.c) calls only these functions, so it runs unchanged.
 */
#include "API.h"

#include "config.h"
#include "hal.h"
#include "ir_sensors.h"
#include "motion.h"

/* Set when a move fails. The motors stay off and every later move fails too,
 * which ends the solver's run instead of driving blind. */
static bool faulted;

int API_mazeWidth(void) { return MAZE_SIZE; }
int API_mazeHeight(void) { return MAZE_SIZE; }

bool API_wallFront(void) { return ir_wall(IR_FRONT); }
bool API_wallRight(void) { return ir_wall(IR_RIGHT); }
bool API_wallLeft(void) { return ir_wall(IR_LEFT); }

uint32_t API_timeMs(void) { return hal_millis(); }

bool API_moveForward(int distance) {
    if (faulted) return false;
    if (!motion_forward(distance)) {
        faulted = true;
        motion_stop();
        return false;
    }
    return true;
}

void API_turnRight(void) {
    if (faulted) return;
    if (!motion_turn(-1)) {
        faulted = true;
        motion_stop();
    }
}

void API_turnLeft(void) {
    if (faulted) return;
    if (!motion_turn(1)) {
        faulted = true;
        motion_stop();
    }
}

/* The simulator's debug display does not exist on the real mouse. */
void API_setWall(int x, int y, char direction) { (void)x; (void)y; (void)direction; }
void API_clearWall(int x, int y, char direction) { (void)x; (void)y; (void)direction; }
void API_setColor(int x, int y, char color) { (void)x; (void)y; (void)color; }
void API_clearColor(int x, int y) { (void)x; (void)y; }
void API_clearAllColor(void) {}
void API_setText(int x, int y, const char *text) { (void)x; (void)y; (void)text; }
void API_clearText(int x, int y) { (void)x; (void)y; }
void API_clearAllText(void) {}

/* TODO: the real mouse has no reset button yet. A long press of the start
 * button (PB12) could be used for this later. */
bool API_wasReset(void) { return false; }
void API_ackReset(void) {}
