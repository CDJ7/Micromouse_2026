/*
 * motion.h - driving: straights measured in cells, and 90-degree turns.
 *
 * Both calls are blocking. They run their own control loop on the HAL's 1 ms
 * tick and return when the move is finished, or false if something went wrong
 * (stalled against a wall, timed out).
 */
#ifndef MOTION_H
#define MOTION_H

#include <stdbool.h>

/* Zero the heading reference: "straight ahead" is wherever the mouse points now. */
void motion_init(void);

/* Drive forward `cells` whole cells. A single cell uses the slower search speed
 * and several cells use the faster run speed. If there is a wall ahead at the
 * end, the stopping position is corrected from the front sensor. */
bool motion_forward(int cells);

/* Turn 90 degrees in place. +1 = left, -1 = right. */
bool motion_turn(int direction);

/* Drive an exact distance in mm (negative = backwards), no wall correction. */
bool motion_move_mm(float mm, float max_speed);

void motion_stop(void);

/* Estimated seconds a move of this many cells takes (for COST_FORWARD). */
float motion_cell_time_s(int cells);

#endif /* MOTION_H */
