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

static SimMaze maze;
static SimState st;

const SimMaze *sim_maze(void) { return &maze; }
SimState *sim_state(void) { return &st; }

static const int DX[4] = {0, 1, 0, -1};
static const int DY[4] = {1, 0, -1, 0};

void sim_start(void) {
    int keepResets = st.resetCalls, keepResetAt = st.resetAtCall;
    int keepCalls = st.wasResetCalls;
    memset(&st, 0, sizeof st);
    st.resetCalls = keepResets;
    st.resetAtCall = keepResetAt;
    st.wasResetCalls = keepCalls;
    st.x = st.y = 0;
    st.heading = 0;
}

/* ---- maze loading ---- */

static int loadNum(FILE *f) {
    int x, y, n, e, s, w, maxx = -1, maxy = -1;
    static int cells[SIM_MAX * SIM_MAX][6];
    int count = 0;
    while (fscanf(f, "%d %d %d %d %d %d", &x, &y, &n, &e, &s, &w) == 6) {
        if (x < 0 || y < 0 || x >= SIM_MAX || y >= SIM_MAX) return -1;
        cells[count][0] = x; cells[count][1] = y; cells[count][2] = n;
        cells[count][3] = e; cells[count][4] = s; cells[count][5] = w;
        count++;
        if (x > maxx) maxx = x;
        if (y > maxy) maxy = y;
    }
    if (count == 0) return -1;
    memset(&maze, 0, sizeof maze);
    maze.w = maxx + 1;
    maze.h = maxy + 1;
    while (count-- > 0) {
        unsigned char *wl = maze.wall[cells[count][0]][cells[count][1]];
        wl[0] = (unsigned char)cells[count][2];
        wl[1] = (unsigned char)cells[count][3];
        wl[2] = (unsigned char)cells[count][4];
        wl[3] = (unsigned char)cells[count][5];
    }
    return 0;
}

/* ASCII art: 4 columns and 2 rows per cell. Any non-space character at a wall
 * position counts as a wall (that covers the mms .map and mazefiles formats). */
static int loadAscii(FILE *f) {
    static char lines[2 * SIM_MAX + 4][256];
    int n = 0, x, y, rows, cols;
    char buf[256];
    while (fgets(buf, sizeof buf, f) && n < 2 * SIM_MAX + 2) {
        size_t len = strlen(buf);
        while (len > 0 && (buf[len - 1] == '\n' || buf[len - 1] == '\r')) buf[--len] = 0;
        if (len == 0) continue;
        strcpy(lines[n++], buf);
    }
    if (n < 3 || n % 2 == 0) return -1;
    rows = (n - 1) / 2;
    cols = ((int)strlen(lines[0]) - 1) / 4;
    if (rows > SIM_MAX || cols > SIM_MAX || rows < 1 || cols < 1) return -1;
    memset(&maze, 0, sizeof maze);
    maze.w = cols;
    maze.h = rows;
    for (y = 0; y < rows; y++) {
        int r = 2 * (rows - 1 - y); /* line holding this cell's north wall */
        for (x = 0; x < cols; x++) {
            maze.wall[x][y][0] = lines[r][4 * x + 2] != ' ';
            maze.wall[x][y][2] = lines[r + 2][4 * x + 2] != ' ';
            maze.wall[x][y][3] = lines[r + 1][4 * x] != ' ';
            maze.wall[x][y][1] = lines[r + 1][4 * x + 4] != ' ';
        }
    }
    return 0;
}

int sim_load(const char *path) {
    FILE *f = fopen(path, "r");
    size_t len = strlen(path);
    int rc;
    if (!f) return -1;
    if (len > 4 && strcmp(path + len - 4, ".num") == 0) rc = loadNum(f);
    else rc = loadAscii(f);
    fclose(f);
    return rc;
}

/* ---- sensing ---- */

static bool wallToward(int d) { return maze.wall[st.x][st.y][d & 3] != 0; }

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
    }
    /* mms: each cell after the second adds only half a point */
    st.effDist += distance <= 2 ? distance : 2 + 0.5 * (distance - 2);
    return true;
}

void API_turnRight(void) {
    record('R', 0);
    st.heading = (st.heading + 1) & 3;
    st.turns++;
}

void API_turnLeft(void) {
    record('L', 0);
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
