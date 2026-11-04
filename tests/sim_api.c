/*
 * sim_api.c - an offline stand-in for API.c.
 *
 * Instead of talking to mms over stdin/stdout, it answers every API call from
 * a maze loaded from a file. It also records all movement and validates the
 * display calls (cell text length, color letters, coordinates) the way mms
 * would reject them.
 */
#include "sim.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "API.h"

#define maze sim_mazeData
static SimState st;

SimState *sim_state(void) { return &st; }

static const int DX[4] = {0, 1, 0, -1};
static const int DY[4] = {1, 0, -1, 0};

void sim_start(void) {
    int keepResets = st.resetCalls, keepResetAt = st.resetAtCall;
    int keepCalls = st.wasResetCalls;
    int keepNoise = st.noisePermille;
    unsigned keepRng = st.rng;
    memset(&st, 0, sizeof st);
    st.resetCalls = keepResets;
    st.resetAtCall = keepResetAt;
    st.wasResetCalls = keepCalls;
    st.noisePermille = keepNoise;
    st.rng = keepRng ? keepRng : 12345u;
    st.x = st.y = 0;
    st.heading = 0;
}

/* ---- sensing ---- */

/* A wall reading, flipped with probability noisePermille/1000 to imitate a
 * bad IR reading. */
static bool wallToward(int d) {
    bool truth = maze.wall[st.x][st.y][d & 3] != 0;
    if (st.noisePermille > 0) {
        st.rng = st.rng * 1664525u + 1013904223u;
        if ((int)((st.rng >> 8) % 1000u) < st.noisePermille) {
            st.sensorFlips++;
            return !truth;
        }
    }
    return truth;
}

#ifndef SIM_MS_PER_CELL
#define SIM_MS_PER_CELL 250
#endif
#ifndef SIM_MS_PER_TURN
#define SIM_MS_PER_TURN 400
#endif

uint32_t API_timeMs(void) { return st.timeMs; }

bool API_wallFront(void) { return wallToward(st.heading); }
bool API_wallRight(void) { return wallToward(st.heading + 1); }
bool API_wallLeft(void) { return wallToward(st.heading + 3); }
int API_mazeWidth(void) { return maze.w; }
int API_mazeHeight(void) { return maze.h; }

/* ---- movement ---- */

static void record(char kind, int callIdx) {
    if (st.nActions < SIM_LOG_MAX) {
        SimAction *a = &st.log[st.nActions];
        a->kind = kind;
        a->x = st.x;
        a->y = st.y;
        a->heading = st.heading;
        a->callIdx = callIdx;
    }
    st.nActions++;
}

bool API_moveForward(int distance) {
    int i;
    st.moveCalls++;
    if (distance < 1) {
        st.crashes++;
        return false;
    }
    for (i = 0; i < distance; i++) {
        if (maze.wall[st.x][st.y][st.heading]) {
            st.crashes++; /* mms would say "crash" */
            return false;
        }
        record('F', i + 1);
        st.x += DX[st.heading];
        st.y += DY[st.heading];
        st.timeMs += SIM_MS_PER_CELL;
    }
    /* mms: each cell after the second adds only half a point */
    st.effDist += distance <= 2 ? distance : 2 + 0.5 * (distance - 2);
    return true;
}

void API_turnRight(void) {
    record('R', 0);
    st.timeMs += SIM_MS_PER_TURN;
    st.heading = (st.heading + 1) & 3;
    st.turns++;
}

void API_turnLeft(void) {
    record('L', 0);
    st.timeMs += SIM_MS_PER_TURN;
    st.heading = (st.heading + 3) & 3;
    st.turns++;
}

/* ---- display: validate like mms would, otherwise ignore ---- */

static void checkCell(int x, int y) {
    st.displayCalls++;
    if (x < 0 || y < 0 || x >= maze.w || y >= maze.h) st.badCalls++;
}

void API_setWall(int x, int y, char d) {
    checkCell(x, y);
    if (!strchr("nesw", d)) st.badCalls++;
}
void API_clearWall(int x, int y, char d) { API_setWall(x, y, d); }

void API_setColor(int x, int y, char c) {
    checkCell(x, y);
    if (!c || !strchr("kbacgorwyBCAGRY", c)) st.badCalls++;
}
void API_clearColor(int x, int y) { checkCell(x, y); }
void API_clearAllColor(void) { st.displayCalls++; }

void API_setText(int x, int y, const char *text) {
    checkCell(x, y);
    if (strlen(text) > 10) st.badCalls++;
}
void API_clearText(int x, int y) { checkCell(x, y); }
void API_clearAllText(void) { st.displayCalls++; }

/* ---- reset button ---- */

bool API_wasReset(void) {
    st.wasResetCalls++;
    return st.resetAtCall > 0 && st.wasResetCalls == st.resetAtCall;
}

void API_ackReset(void) {
    st.resetCalls++;
    {
        int calls = st.resetCalls;
        sim_start();
        st.resetCalls = calls;
    }
}
