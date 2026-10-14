/*
 * API.h - the ONLY boundary between the solver and the outside world.
 *
 *
 * The solver (solver.c) and the main loop (main.c) talk to the mouse
 * exclusively through these functions. In the simulator, API.c turns each call
 * into an mms command on stdout and reads the reply from stdin. On the real
 * mouse, API.c will be replaced by a hardware version (read the IR sensors,
 * drive the motors, ignore the display calls) and nothing else needs to change.
 *
 * Coordinates: (0,0) is the bottom-left cell, x grows east, y grows north.
 * Directions for the display calls are the characters 'n', 'e', 's', 'w'.
 */
#ifndef API_H
#define API_H

#include <stdbool.h>

/* ---- maze ---- */
int API_mazeWidth(void);
int API_mazeHeight(void);

/* ---- sensing (relative to where the mouse is facing) ---- */
bool API_wallFront(void);
bool API_wallRight(void);
bool API_wallLeft(void);

/* ---- movement ---- */
/* Moves forward `distance` cells (>= 1). Returns false if the mouse crashed. */
bool API_moveForward(int distance);
void API_turnRight(void);
void API_turnLeft(void);

/* ---- debug display (no-ops on hardware) ---- */
void API_setWall(int x, int y, char direction);
void API_clearWall(int x, int y, char direction);
void API_setColor(int x, int y, char color);
void API_clearColor(int x, int y);
void API_clearAllColor(void);
void API_setText(int x, int y, const char *text); /* at most 10 characters */
void API_clearText(int x, int y);
void API_clearAllText(void);

/* ---- reset button ---- */
bool API_wasReset(void);
void API_ackReset(void);

#endif /* API_H */
