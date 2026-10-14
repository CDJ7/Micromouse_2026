/*
 * main.c - entry point and main loop.
 *
 *
 * Deliberately tiny: it only knows about the API (for the reset button) and the
 * solver. All the thinking happens in solver.c.
 *
 * Build (Windows + MinGW, from this folder):
 *     gcc -o a.exe API.c main.c solver.c
 */
#include "API.h"
#include "solver.h"

int main(void) {
    solver_init();

    for (;;) {
        /* Reset button: the simulator pretends the mouse crashed. Forget
         * everything we know, tell the simulator we are ready to be put back
         * at the start, then start over from scratch. We can only notice the
         * button between moves, not in the middle of a multi-cell straight. */
        if (API_wasReset()) {
            solver_reset();
            API_ackReset();
            solver_init();
            continue;
        }

        /* One turn or one forward move per step. false = run is over; ending
         * the program is what makes mms compute the final score. */
        if (!solver_step()) {
            break;
        }
    }
    return 0;
}
