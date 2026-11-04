/*
 * main_hw.c - start-up sequence and main loop for the real mouse.
 *
 *   1. set up the hardware
 *   2. wait for the start button, then give the user time to take their hand away
 *   3. calibrate the wall sensors in the start cell
 *   4. run the solver (MMS_Code/solver.c) until it is finished
 *   5. stop and wait
 */
#include "API.h"
#include "config.h"
#include "hal.h"
#include "ir_sensors.h"
#include "motion.h"
#include "solver.h"

#define HAND_AWAY_MS 1500

static void waitMs(uint32_t ms) {
    uint32_t i;
    for (i = 0; i < ms; i++) hal_wait_tick();
}

/* Teach the sensors what a wall looks like, using the start cell.
 *
 * The mouse is placed in the start cell facing the exit. In a competition maze
 * the start cell has walls on its left, right and back, so:
 *   - the left and right sensors see walls right now
 *   - the front sensor sees open space now, and the back wall after a U-turn
 * Returns false if any reading is not believable. */
static bool calibrateAtStart(void) {
    int left = ir_measure(IR_LEFT);
    int right = ir_measure(IR_RIGHT);
    int frontOpen = ir_measure(IR_FRONT);
    int frontWall;

    if (!motion_turn(1) || !motion_turn(1)) return false; /* face the back wall */
    frontWall = ir_measure(IR_FRONT);
    if (!motion_turn(-1) || !motion_turn(-1)) return false; /* and face out again */

    hal_log("[calibrate] wall levels left %d right %d front %d (front open reads %d)\n", left,
            right, frontWall, frontOpen);
    if (left < IR_MIN_LEVEL || right < IR_MIN_LEVEL || frontWall < IR_MIN_LEVEL) {
        hal_log("[calibrate] a sensor sees no wall in the start cell: dead sensor or the mouse is not in the start cell\n");
        return false;
    }
    if ((float)frontOpen >= IR_WALL_FRACTION * (float)frontWall) {
        hal_log("[calibrate] the front sensor sees a wall where the exit should be\n");
        return false;
    }
    ir_set_wall_level(IR_LEFT, left);
    ir_set_wall_level(IR_RIGHT, right);
    ir_set_wall_level(IR_FRONT, frontWall);
    return true;
}

int main(void) {
    hal_init();
    ir_init();

    /* Blink until the start button is pressed. */
    while (!hal_button_pressed()) {
        hal_led((hal_millis() / 250u) & 1u);
        hal_wait_tick();
    }
    hal_led(true);
    waitMs(HAND_AWAY_MS);
    motion_init(); /* "straight ahead" is wherever the mouse points now */

    if (!calibrateAtStart()) {
        hal_log("[main] calibration failed, not starting\n");
        hal_halt();
        return 1;
    }

    solver_init();
    while (solver_step()) {
        /* the solver does one turn or one straight per step */
    }
    motion_stop();
    hal_log("[main] run finished\n");
    hal_halt();
    return 0;
}
