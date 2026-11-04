/*
 * sim_maze.c - loads a maze file (.num, or the ASCII .map / mazefiles text
 * format) into a SimMaze. Shared by the simulation tests and the real-mouse
 * tests (Real_Mouse_Code/test), neither of which needs the other's fake API.
 */
#include "sim.h"

#include <stdio.h>
#include <string.h>

SimMaze sim_mazeData;
#define maze sim_mazeData

const SimMaze *sim_maze(void) { return &sim_mazeData; }

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

