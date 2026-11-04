/*
 * fake_hal.h - what the simulated mouse reports to the test.
 */
#ifndef FAKE_HAL_H
#define FAKE_HAL_H

#include <setjmp.h>

typedef struct {
    double x, y;          /* body center in mm, (0,0) = bottom-left corner of the maze */
    double thetaDeg;      /* heading, 90 = north (counter-clockwise from east) */
    int crashes;          /* times the body hit a wall */
    double maxOffCenterMm;/* largest distance from a cell's centerline while driving */
    double maxHeadingErrDeg; /* largest heading error vs. the nearest compass direction while driving */
    unsigned long ticks;
    int halted;
    unsigned lastLeaveStartMs; /* when the mouse last left cell (0,0) */
} FakeMouse;

FakeMouse *fake_mouse(void);

/* Place the mouse in the center of cell (0,0), facing north, motors off. */
void fake_reset(void);

/* hal_halt() jumps here instead of stopping forever. */
extern jmp_buf fake_halt_jump;

#endif /* FAKE_HAL_H */
