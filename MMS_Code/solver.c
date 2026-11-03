/*
 * solver.c - mapping, exploration, path planning and debug display.
 *
 *
 * GOAL
 *   Best mms score:  bestRunTurns + bestRunEffectiveDistance
 *                    + 0.1 * (totalTurns + totalEffectiveDistance)
 *   A 90-degree turn costs the same as driving one cell, so the "best" route is
 *   the one with the fewest ACTIONS (cells + turns), not the fewest cells.
 *
 * STRATEGY
 *   1. EXPLORE  Walls are tracked per cell side as unknown / open / wall.
 *               a) Plan the cheapest start->center route assuming unknown
 *                  walls are open, and go check the unknown walls on it. When
 *                  that route has no unknown walls left it is PROVEN optimal
 *                  (the plan was a lower bound on every possible route).
 *               b) With the time left, keep visiting the nearest unexplored
 *                  reachable cell until the whole maze is mapped or the
 *                  exploration budget runs out. This is what lets the final
 *                  route be the best in the WHOLE maze, not just the first one.
 *               Every step reserves enough budget to still get home.
 *   2. RETURN   Drive home over known-open sides only, choosing the arrival
 *               heading that gives the cheapest final run.
 *   3. FINAL    Run the best route over known-open sides only, merging
 *               straights into one moveForward(N) (mms scores them cheaper).
 *
 *   Planning is Dijkstra over (cell, heading) states, so turns are priced in.
 *
 * DISPLAY COLOR KEY (mms setColor characters)
 *   G dark green   start cell
 *   y yellow       goal cells (the 2x2 center)
 *   B dark blue    visited cell (the mouse has been inside it)
 *   c cyan         route currently planned (best route if unknown walls were open)
 *   o orange       current exploration target cell
 *   g green        final optimal route, shown just before the final run
 *   (no color)     unvisited cell
 *   Cell text      distance to the goal (see DIST_DISPLAY_FLOOD)
 *   Walls          drawn with setWall the moment they are discovered
 *
 * ROBUSTNESS (matters on a real mouse, harmless in the simulator)
 *   - Each wall can be read several times and decided by majority (WALL_READS).
 *   - Before driving into an unexplored-looking side the front sensor is read
 *     once more (VERIFY_FRONT_BEFORE_MOVE), so one bad reading cannot cause a crash.
 *   - A side we have driven through is open for good; a later "wall" reading
 *     of it is ignored. A wall that is later read as open is kept (the safe
 *     choice: a phantom wall costs time, a missed wall costs a crash).
 *
 * PORTING NOTE
 *   All hardware/simulator access goes through API.h. The only other output is
 *   LOG(), which defaults to stderr (stdout belongs to the simulator protocol).
 *   On a microcontroller define LOG before this file is compiled (e.g. -DLOG=...)
 *   to route it to a serial port, and nothing here needs <stdio.h>.
 */
#include "solver.h"

#include <stdbool.h>
#include <stdint.h>

#include "API.h"

/* ====================================================================== */
/* Tunable settings                                                        */
/* ====================================================================== */

/* Largest maze we have room for. The real size is read from the API at startup
 * (16x16 in the simulator, 10x10 for the competition maze). All arrays are
 * static and sized from this. */
#ifndef MAX_MAZE_SIZE
#define MAX_MAZE_SIZE 16
#endif

/* What one action costs. Only the ratio changes which route is "best", and the
 * exploration budget below is counted in the same unit.
 *   Simulator: 1 and 1. A turn costs the same as a cell, which is how mms
 *              scores a run.
 *   Real mouse: the measured time of one cell and one 90-degree turn in
 *              milliseconds, so the planner minimizes real run time. */
#ifndef COST_FORWARD
#define COST_FORWARD 1
#endif
#ifndef COST_TURN
#define COST_TURN 1
#endif

/* Exploration budget: the most cost (see above) the mouse may spend exploring
 * AND driving home. It stands in for the 10-minute limit. The mouse always
 * keeps enough in reserve to get back to the start.
 * With the 1/1 costs above it is a number of actions. Override on the command
 * line with -DEXPLORE_BUDGET=n. */
#ifndef EXPLORE_BUDGET
#define EXPLORE_BUDGET 1500
#endif

/* How the budget is measured:
 *   0 = add up the cost of every action taken (works anywhere)
 *   1 = read the clock, API_timeMs(). Costs and budget must then be in
 *       milliseconds. TIME_SAFETY_MARGIN (ms) is held back for things the
 *       model does not know about (sensor reads, slip, battery sag). */
#ifndef EXPLORE_BUDGET_USES_CLOCK
#define EXPLORE_BUDGET_USES_CLOCK 0
#endif
#ifndef TIME_SAFETY_MARGIN
#define TIME_SAFETY_MARGIN 0
#endif

/* 1 = after the best route is proven, keep mapping the rest of the maze until
 *     the budget runs out (useful insurance on a real mouse with noisy sensors).
 * 0 = stop exploring as soon as the best route is proven. In mms this scores
 *     better, because exploration still counts at 1/10 of a run. */
#ifndef EXPLORE_MAP_WHOLE_MAZE
#define EXPLORE_MAP_WHOLE_MAZE 1
#endif

/* Reads per wall decision (use an odd number). A wall is declared when at least
 * half of the reads say so. Use 1 in the simulator, 3 or more on noisy sensors. */
#ifndef WALL_READS
#define WALL_READS 1
#endif

/* 1 = read the front sensor once more before each exploration step into a side
 * we have not driven through yet, and refuse the move if it now sees a wall. */
#ifndef VERIFY_FRONT_BEFORE_MOVE
#define VERIFY_FRONT_BEFORE_MOVE 1
#endif

/* 1 = also check, during the return trip and the final run, each side that
 * the mouse has not driven through before, with the front sensor, and
 * re-plan if a wall turns up. Straights are then split at such sides, so this
 * costs a little score in mms. Turn it on for the real mouse, where a single
 * bad reading would otherwise be a crash. 0 = trust the map completely. */
#ifndef VERIFY_UNDRIVEN_ON_RUNS
#define VERIFY_UNDRIVEN_ON_RUNS 0
#endif

/* Distance numbers drawn in each cell:
 *   1 = live flood fill through the known maze (unknown walls count as open),
 *       updated as walls are discovered
 *   0 = plain Manhattan distance to the nearest goal cell, never updated */
#ifndef DIST_DISPLAY_FLOOD
#define DIST_DISPLAY_FLOOD 1
#endif

/* Debug display switches (1 = on, 0 = off). Turn them off on the real mouse. */
#define DEBUG_SHOW_DISTANCES     1 /* distance text in every cell           */
#define DEBUG_SHOW_WALLS         1 /* setWall for every discovered wall     */
#define DEBUG_SHOW_START_GOAL    1 /* start and goal cell colors            */
#define DEBUG_SHOW_VISITED       1 /* color cells the mouse has visited     */
#define DEBUG_SHOW_PLANNED_ROUTE 1 /* highlight the currently planned route */
#define DEBUG_SHOW_TARGET        1 /* highlight the exploration target      */
#define DEBUG_SHOW_FINAL_ROUTE   1 /* highlight the final route             */

/* Log to stderr (the mms "Run Output" tab). */
#ifndef DEBUG_LOG
#define DEBUG_LOG 1
#endif
/* 1 = log every re-plan, 0 = only when the target cell changes. */
#define DEBUG_LOG_EVERY_REPLAN 0

/* ====================================================================== */
/* Constants and types                                                     */
/* ====================================================================== */

#define NUM_CELLS  (MAX_MAZE_SIZE * MAX_MAZE_SIZE)
#define NUM_STATES (NUM_CELLS * 4) /* (cell, heading) pairs */
#define INF_COST   0x3fffffff

/* Path costs are integers. One cost unit is COST_SCALE internally. Entering a
 * cell across a side we have never seen costs a tiny UNKNOWN_PENALTY extra so
 * that, between equally good routes, we prefer the one made of known sides.
 * The penalty total must stay below COST_SCALE (at most NUM_STATES sides). */
#define COST_SCALE      2048
#define UNKNOWN_PENALTY 1
#define FORWARD_STEP    (COST_FORWARD * COST_SCALE)
#define TURN_STEP       (COST_TURN * COST_SCALE)

/* Compile-time check that the largest possible path cost cannot overflow. If
 * this line fails to compile, COST_FORWARD / COST_TURN are too large. */
typedef char costs_must_fit_in_int[(COST_FORWARD > 0 && COST_TURN > 0 &&
    (long)COST_SCALE * (COST_FORWARD > COST_TURN ? COST_FORWARD : COST_TURN) * NUM_STATES <
        INF_COST) ? 1 : -1];

/* The final run is worth ten times the exploration in the score formula. */
#define RUN_WEIGHT 10

/* Safety net against an endless loop (only reachable if something is wrong). */
#define HARD_ACTION_CAP (NUM_STATES * 4)

enum { DIR_NORTH = 0, DIR_EAST = 1, DIR_SOUTH = 2, DIR_WEST = 3 };
enum { EDGE_UNKNOWN = 0, EDGE_OPEN = 1, EDGE_WALL = 2 };
enum { ACT_NONE = 0, ACT_FORWARD, ACT_LEFT, ACT_RIGHT };
enum { PHASE_EXPLORE, PHASE_RETURN, PHASE_FINAL, PHASE_DONE, PHASE_FAILED };

/* A queued move: a turn, or a straight of `count` cells. */
typedef struct {
    unsigned char kind; /* ACT_FORWARD, ACT_LEFT or ACT_RIGHT */
    unsigned char count;
} Move;

static const int DX[4] = {0, 1, 0, -1};
static const int DY[4] = {1, 0, -1, 0};
static const char DIR_CHAR[4] = {'n', 'e', 's', 'w'};

#ifndef LOG
#if DEBUG_LOG
#include <stdio.h>
#define LOG(...) fprintf(stderr, __VA_ARGS__)
#else
#define LOG(...) ((void)0)
#endif
#endif

/* ====================================================================== */
/* State                                                                   */
/* ====================================================================== */

static int mazeW, mazeH;
static int goalLoX, goalHiX, goalLoY, goalHiY; /* the center block */

/* What we know about each cell side. Kept consistent on both sides of a wall. */
static unsigned char edgeKnow[MAX_MAZE_SIZE][MAX_MAZE_SIZE][4];
static unsigned char drivenEdge[MAX_MAZE_SIZE][MAX_MAZE_SIZE][4]; /* crossed by the mouse */
static unsigned char visited[MAX_MAZE_SIZE][MAX_MAZE_SIZE];
static bool mapChanged; /* a wall was learned since the distances were drawn */

static int posX, posY, heading;
static int phase;
static int exploreActions, returnActions, finalActions;
static int exploreCost, returnCost; /* the same, weighted by COST_FORWARD/COST_TURN */
static uint32_t startTimeMs;
static int contradictions, frontVetoes;
static bool goalReached;
static bool routeProven;
static bool wholeMazeMapped;
static bool budgetWarned;
static bool provenLogged;
static int finalCells, finalTurns;
static int finalExecCells, finalExecTurns; /* final-run actions already driven */

/* Exploration decision memory */
static int targetX, targetY; /* -1 = no target */
static int lastReason;       /* why the current target was chosen */

/* Queued moves for the return and final run */
static Move moveQ[NUM_STATES + 8];
static int moveQLen, moveQPos;

/* What the display currently shows, so we only send what changed */
static int shownDist[MAX_MAZE_SIZE][MAX_MAZE_SIZE];     /* -2 = nothing */
static char shownColor[MAX_MAZE_SIZE][MAX_MAZE_SIZE];   /* 0 = no color */
static unsigned char routeMark[MAX_MAZE_SIZE][MAX_MAZE_SIZE]; /* 0/1 planned/2 final */

/* Planner scratch space */
static int dist[NUM_STATES];
static short prevState[NUM_STATES];
static unsigned char prevAct[NUM_STATES];
static short pathBuf[NUM_STATES];
typedef struct {
    int key;
    int state;
} HeapItem;
static HeapItem heap[NUM_STATES * 4 + 8];
static int heapSize;
static unsigned char scratchMark[MAX_MAZE_SIZE][MAX_MAZE_SIZE];
static unsigned char candidate[MAX_MAZE_SIZE][MAX_MAZE_SIZE];
#if DEBUG_SHOW_DISTANCES && DIST_DISPLAY_FLOOD
static int floodDist[MAX_MAZE_SIZE][MAX_MAZE_SIZE];
#endif

enum { REASON_NONE, REASON_CHECK_ROUTE, REASON_MAP_NEAREST };
#if DEBUG_LOG
static const char *reasonText(int r) {
    if (r == REASON_CHECK_ROUTE) return "unknown wall on the best route";
    if (r == REASON_MAP_NEAREST) return "nearest unexplored cell";
    return "none";
}
#endif

/* ====================================================================== */
/* Small helpers                                                           */
/* ====================================================================== */

static bool inMaze(int x, int y) {
    return x >= 0 && y >= 0 && x < mazeW && y < mazeH;
}

static bool isGoal(int x, int y) {
    return x >= goalLoX && x <= goalHiX && y >= goalLoY && y <= goalHiY;
}

static int stateIndex(int x, int y, int h) {
    return (x * MAX_MAZE_SIZE + y) * 4 + h;
}

static int stateX(int s) { return (s >> 2) / MAX_MAZE_SIZE; }
static int stateY(int s) { return (s >> 2) % MAX_MAZE_SIZE; }
static int stateHeading(int s) { return s & 3; }

/* Anything outside the maze counts as a wall. */
static int edgeAt(int x, int y, int d) {
    if (!inMaze(x, y)) return EDGE_WALL;
    return edgeKnow[x][y][d];
}

/* ====================================================================== */
/* Debug display (only sends what changed)                                 */
/* ====================================================================== */

/* Which color a cell should have right now. Higher priority first. */
static char desiredColor(int x, int y) {
    (void)x;
    (void)y;
#if DEBUG_SHOW_TARGET
    if (x == targetX && y == targetY) return 'o';
#endif
#if DEBUG_SHOW_START_GOAL
    if (x == 0 && y == 0) return 'G';
    if (isGoal(x, y)) return 'y';
#endif
#if DEBUG_SHOW_FINAL_ROUTE
    if (routeMark[x][y] == 2) return 'g';
#endif
#if DEBUG_SHOW_PLANNED_ROUTE
    if (routeMark[x][y] == 1) return 'c';
#endif
#if DEBUG_SHOW_VISITED
    if (visited[x][y]) return 'B';
#endif
    return 0;
}

static void refreshCellColor(int x, int y) {
    char want = desiredColor(x, y);
    if (want == shownColor[x][y]) return;
    if (want) {
        API_setColor(x, y, want);
    } else {
        API_clearColor(x, y);
    }
    shownColor[x][y] = want;
}

/* A wall belongs to two cells; draw both sides so it always shows. */
static void drawWall(int x, int y, int d) {
#if DEBUG_SHOW_WALLS
    int nx = x + DX[d], ny = y + DY[d];
    API_setWall(x, y, DIR_CHAR[d]);
    if (inMaze(nx, ny)) API_setWall(nx, ny, DIR_CHAR[(d + 2) & 3]);
#else
    (void)x; (void)y; (void)d;
#endif
}

static void clearWallDisplay(int x, int y, int d) {
    int nx = x + DX[d], ny = y + DY[d];
    API_clearWall(x, y, DIR_CHAR[d]);
    if (inMaze(nx, ny)) API_clearWall(nx, ny, DIR_CHAR[(d + 2) & 3]);
}

#if DEBUG_SHOW_DISTANCES
/* Distance from a cell to the nearest goal cell, ignoring walls. */
static int manhattanToGoal(int x, int y) {
    int dx = x < goalLoX ? goalLoX - x : (x > goalHiX ? x - goalHiX : 0);
    int dy = y < goalLoY ? goalLoY - y : (y > goalHiY ? y - goalHiY : 0);
    return dx + dy;
}

/* Small non-negative number to text, so we do not need printf-style formatting
 * (it is large and slow on a microcontroller). */
static void numberToText(int value, char *out) {
    char digits[8];
    int n = 0, i;
    if (value == 0) {
        out[0] = '0';
        out[1] = 0;
        return;
    }
    while (value > 0 && n < 7) {
        digits[n++] = (char)('0' + value % 10);
        value /= 10;
    }
    for (i = 0; i < n; i++) out[i] = digits[n - 1 - i];
    out[n] = 0;
}

static void showDistance(int x, int y, int value) {
    char text[12];
    if (value == shownDist[x][y]) return;
    if (value < 0) {
        text[0] = text[1] = '-'; /* walled off from the goal */
        text[2] = 0;
    } else {
        numberToText(value, text);
    }
    API_setText(x, y, text);
    shownDist[x][y] = value;
}

#if DIST_DISPLAY_FLOOD
/* Breadth-first flood from the goal through the known maze. Sides we have not
 * seen yet count as open, so the numbers are the mouse's current estimate. */
static void computeFloodDistances(void) {
    static short queue[NUM_CELLS];
    int head = 0, tail = 0, x, y, d;
    for (x = 0; x < mazeW; x++)
        for (y = 0; y < mazeH; y++) floodDist[x][y] = -1;
    for (x = goalLoX; x <= goalHiX; x++)
        for (y = goalLoY; y <= goalHiY; y++) {
            floodDist[x][y] = 0;
            queue[tail++] = (short)(x * MAX_MAZE_SIZE + y);
        }
    while (head < tail) {
        int c = queue[head++];
        int cx = c / MAX_MAZE_SIZE, cy = c % MAX_MAZE_SIZE;
        for (d = 0; d < 4; d++) {
            int nx = cx + DX[d], ny = cy + DY[d];
            if (!inMaze(nx, ny) || edgeAt(cx, cy, d) == EDGE_WALL) continue;
            if (floodDist[nx][ny] >= 0) continue;
            floodDist[nx][ny] = floodDist[cx][cy] + 1;
            queue[tail++] = (short)(nx * MAX_MAZE_SIZE + ny);
        }
    }
}
#endif
#endif

/* Redraw the distance numbers that changed. In flood mode this only does work
 * after a new wall was learned; in Manhattan mode the numbers never change. */
static void refreshDistances(void) {
#if DEBUG_SHOW_DISTANCES && DIST_DISPLAY_FLOOD
    int x, y;
    if (!mapChanged) return;
    computeFloodDistances();
    for (x = 0; x < mazeW; x++)
        for (y = 0; y < mazeH; y++) showDistance(x, y, floodDist[x][y]);
#endif
    mapChanged = false;
}

/* ====================================================================== */
/* Map knowledge                                                           */
/* ====================================================================== */

/* Record one side of a cell (and the matching side of its neighbor). Returns
 * true if this was new information. Walls are drawn as soon as they appear. */
static bool setEdge(int x, int y, int d, int value) {
    int nx = x + DX[d], ny = y + DY[d];
    bool changed = (edgeKnow[x][y][d] != value);
    edgeKnow[x][y][d] = (unsigned char)value;
    if (inMaze(nx, ny)) edgeKnow[nx][ny][(d + 2) & 3] = (unsigned char)value;
    if (changed) {
        mapChanged = true;
        if (value == EDGE_WALL) drawWall(x, y, d);
    }
    return changed;
}

enum { SENSE_FRONT, SENSE_LEFT, SENSE_RIGHT };

/* One wall decision: WALL_READS readings, wall if at least half say wall. A tie
 * goes to "wall" because a phantom wall only costs time. */
static bool readWall(int which) {
    int i, walls = 0;
    for (i = 0; i < WALL_READS; i++) {
        bool w = (which == SENSE_FRONT) ? API_wallFront()
                 : (which == SENSE_LEFT) ? API_wallLeft()
                                         : API_wallRight();
        if (w) walls++;
    }
    return walls * 2 >= WALL_READS;
}

/* Fold one sensor decision into the map, with the safety rules described at
 * the top of this file. */
static void recordSensed(int x, int y, int d, bool isWall) {
    int old = edgeKnow[x][y][d];
    if (drivenEdge[x][y][d]) {
        if (isWall) {
            contradictions++;
            LOG("[warn] ignoring a wall reading at (%d,%d) side %c: we have driven through it\n",
                x, y, DIR_CHAR[d]);
        }
        return;
    }
    if (old == EDGE_WALL && !isWall) {
        contradictions++;
        LOG("[warn] (%d,%d) side %c was a wall and now reads open; keeping the wall\n", x, y,
            DIR_CHAR[d]);
        return;
    }
    if (old == EDGE_OPEN && isWall) {
        contradictions++;
        LOG("[warn] (%d,%d) side %c was open and now reads wall; updating the map\n", x, y,
            DIR_CHAR[d]);
    }
    setEdge(x, y, d, isWall ? EDGE_WALL : EDGE_OPEN);
}

/* Called on arrival in a cell: read the three walls we can see. The side we
 * came through must be open, so we know that one for free. */
static void senseWalls(void) {
    int front = heading, left = (heading + 3) & 3, right = (heading + 1) & 3;
    int back = (heading + 2) & 3;
    recordSensed(posX, posY, front, readWall(SENSE_FRONT));
    recordSensed(posX, posY, left, readWall(SENSE_LEFT));
    recordSensed(posX, posY, right, readWall(SENSE_RIGHT));
    if (edgeKnow[posX][posY][back] == EDGE_UNKNOWN) {
        setEdge(posX, posY, back, EDGE_OPEN);
    }
    visited[posX][posY] = 1;
    if (isGoal(posX, posY)) goalReached = true;
    refreshCellColor(posX, posY);
    refreshDistances();
}

/* ====================================================================== */
/* Path planning: Dijkstra over (cell, heading)                            */
/* ====================================================================== */

static void heapPush(int key, int state) {
    int i = heapSize++;
    while (i > 0) {
        int parent = (i - 1) >> 1;
        if (heap[parent].key <= key) break;
        heap[i] = heap[parent];
        i = parent;
    }
    heap[i].key = key;
    heap[i].state = state;
}

static HeapItem heapPop(void) {
    HeapItem top = heap[0];
    HeapItem last = heap[--heapSize];
    int i = 0;
    for (;;) {
        int child = 2 * i + 1;
        if (child >= heapSize) break;
        if (child + 1 < heapSize && heap[child + 1].key < heap[child].key) child++;
        if (heap[child].key >= last.key) break;
        heap[i] = heap[child];
        i = child;
    }
    heap[i] = last;
    return top;
}

static void relax(int to, int cost, int from, int act) {
    if (cost < dist[to]) {
        dist[to] = cost;
        prevState[to] = (short)from;
        prevAct[to] = (unsigned char)act;
        heapPush(cost, to);
    }
}

/* Find the cheapest way to every (cell, heading) from a start state. A turn
 * and a one-cell move each cost one action. Pass startHeading < 0 to start in
 * all four headings at once (used for the start cell: the heading we leave it
 * in is ours to choose).
 *   knownOnly = false: sides we have not seen count as open (optimistic plan)
 *   knownOnly = true:  only sides we know are open may be used (safe plan)  */
static void runDijkstra(int sx, int sy, int startHeading, bool knownOnly) {
    int i;
    for (i = 0; i < NUM_STATES; i++) {
        dist[i] = INF_COST;
        prevState[i] = -1;
        prevAct[i] = ACT_NONE;
    }
    heapSize = 0;
    if (startHeading < 0) {
        for (i = 0; i < 4; i++) {
            dist[stateIndex(sx, sy, i)] = 0;
            heapPush(0, stateIndex(sx, sy, i));
        }
    } else {
        dist[stateIndex(sx, sy, startHeading)] = 0;
        heapPush(0, stateIndex(sx, sy, startHeading));
    }

    while (heapSize > 0) {
        HeapItem item = heapPop();
        int u = item.state;
        int x, y, h, e;
        if (item.key > dist[u]) continue; /* stale queue entry */
        x = stateX(u);
        y = stateY(u);
        h = stateHeading(u);
        e = edgeAt(x, y, h);
        if (e == EDGE_OPEN || (e == EDGE_UNKNOWN && !knownOnly)) {
            int extra = (e == EDGE_UNKNOWN) ? UNKNOWN_PENALTY : 0;
            relax(stateIndex(x + DX[h], y + DY[h], h),
                  dist[u] + FORWARD_STEP + extra, u, ACT_FORWARD);
        }
        relax(stateIndex(x, y, (h + 1) & 3), dist[u] + TURN_STEP, u, ACT_RIGHT);
        relax(stateIndex(x, y, (h + 3) & 3), dist[u] + TURN_STEP, u, ACT_LEFT);
    }
}

/* Cheapest heading at a cell, or -1 if the cell is unreachable. */
static int bestStateAtCell(int x, int y) {
    int h, best = -1, bestCost = INF_COST;
    for (h = 0; h < 4; h++) {
        int s = stateIndex(x, y, h);
        if (dist[s] < bestCost) {
            bestCost = dist[s];
            best = s;
        }
    }
    return best;
}

static int bestGoalState(void) {
    int x, y, best = -1, bestCost = INF_COST;
    for (x = goalLoX; x <= goalHiX; x++)
        for (y = goalLoY; y <= goalHiY; y++) {
            int s = bestStateAtCell(x, y);
            if (s >= 0 && dist[s] < bestCost) {
                bestCost = dist[s];
                best = s;
            }
        }
    return best;
}

/* Walk back from `goal` to the start state. Fills pathBuf (last step first,
 * start state excluded) and returns the number of steps. The first action to
 * take is prevAct[pathBuf[n - 1]]. */
static int collectPath(int goal) {
    int n = 0, s = goal;
    while (s >= 0 && prevState[s] >= 0) {
        pathBuf[n++] = (short)s;
        s = prevState[s];
    }
    return n;
}

/* Mark every cell on the path to `goal` (start cell included). */
static void markPathCells(int goal, unsigned char grid[MAX_MAZE_SIZE][MAX_MAZE_SIZE]) {
    int s = goal;
    grid[stateX(s)][stateY(s)] = 1;
    while (s >= 0 && prevState[s] >= 0) {
        s = prevState[s];
        grid[stateX(s)][stateY(s)] = 1;
    }
}

static void clearGrid(unsigned char grid[MAX_MAZE_SIZE][MAX_MAZE_SIZE]) {
    int x, y;
    for (x = 0; x < MAX_MAZE_SIZE; x++)
        for (y = 0; y < MAX_MAZE_SIZE; y++) grid[x][y] = 0;
}

/* Show `grid` as the route overlay of the given kind (1 planned, 2 final).
 * Cells that stop/start being on the route are redrawn; the rest are skipped. */
static void showRoute(unsigned char grid[MAX_MAZE_SIZE][MAX_MAZE_SIZE], int kind) {
    int x, y;
    for (x = 0; x < mazeW; x++)
        for (y = 0; y < mazeH; y++) {
            unsigned char want = grid[x][y] ? (unsigned char)kind : 0;
            if (routeMark[x][y] != want) {
                routeMark[x][y] = want;
                refreshCellColor(x, y);
            }
        }
}

/* ====================================================================== */
/* Moving                                                                  */
/* ====================================================================== */

static int actionCost(int act) {
    return act == ACT_FORWARD ? COST_FORWARD : COST_TURN;
}

/* Book `n` actions of the given kind against the current phase. */
static void countActions(int n, int act) {
    if (phase == PHASE_EXPLORE) {
        exploreActions += n;
        exploreCost += n * actionCost(act);
    } else if (phase == PHASE_RETURN) {
        returnActions += n;
        returnCost += n * actionCost(act);
    } else {
        finalActions += n;
        if (act == ACT_FORWARD) finalExecCells += n;
        else finalExecTurns += n;
    }
}

/* How much of the exploration budget is used so far. */
static int budgetUsed(void) {
#if EXPLORE_BUDGET_USES_CLOCK
    return (int)(API_timeMs() - startTimeMs) + TIME_SAFETY_MARGIN;
#else
    return exploreCost;
#endif
}

static void markDriven(int x, int y, int d) {
    int nx = x + DX[d], ny = y + DY[d];
    drivenEdge[x][y][d] = 1;
    if (inMaze(nx, ny)) drivenEdge[nx][ny][(d + 2) & 3] = 1;
}

static void doTurn(int act) {
    if (act == ACT_RIGHT) {
        API_turnRight();
        heading = (heading + 1) & 3;
    } else {
        API_turnLeft();
        heading = (heading + 3) & 3;
    }
    countActions(1, act);
}

/* Drive `n` cells straight ahead. Every side crossed must already be known to
 * be open, so the mouse can never be sent into a wall by the planner. Returns
 * false (and stops the run) if that rule is broken or the simulator reports a
 * crash. */
static bool doForward(int n) {
    int i, x = posX, y = posY;
    for (i = 0; i < n; i++) {
        if (edgeAt(x, y, heading) != EDGE_OPEN) {
            LOG("[error] refusing to drive through (%d,%d) side %c: not known open\n",
                x, y, DIR_CHAR[heading]);
            phase = PHASE_FAILED;
            return false;
        }
        markDriven(x, y, heading); /* open for good, whatever a sensor says later */
        x += DX[heading];
        y += DY[heading];
    }
    if (!API_moveForward(n)) {
        LOG("[error] crash while moving %d cells from (%d,%d)\n", n, posX, posY);
        phase = PHASE_FAILED;
        return false;
    }
    posX = x;
    posY = y;
    if (isGoal(posX, posY)) goalReached = true;
    countActions(n, ACT_FORWARD);
    return true;
}

/* ====================================================================== */
/* Statistics                                                              */
/* ====================================================================== */

static int countVisited(void) {
    int x, y, n = 0;
    for (x = 0; x < mazeW; x++)
        for (y = 0; y < mazeH; y++) n += visited[x][y];
    return n;
}

static int countMapped(void) {
    int x, y, d, n = 0;
    for (x = 0; x < mazeW; x++)
        for (y = 0; y < mazeH; y++) {
            bool all = true;
            for (d = 0; d < 4; d++)
                if (edgeKnow[x][y][d] == EDGE_UNKNOWN) all = false;
            n += all;
        }
    return n;
}

/* ====================================================================== */
/* Phase 1: explore                                                        */
/* ====================================================================== */

/* The start cell's way out. Official mazes have exactly one (north). */
static int startExitHeading(void) {
    int d;
    for (d = 0; d < 4; d++)
        if (edgeKnow[0][0][d] == EDGE_OPEN) return d;
    return DIR_NORTH;
}

/* Known-only cost to get home from a state, arriving at the start FACING its
 * exit (so the final run needs no extra turn), or INF_COST. This is what we
 * keep in reserve out of the exploration budget. */
static int returnCostFrom(int x, int y, int h) {
    int c;
    runDijkstra(x, y, h, true);
    c = dist[stateIndex(0, 0, startExitHeading())];
    return c >= INF_COST ? INF_COST : c / COST_SCALE;
}

static void endExplore(const char *why) {
    SolverStats st;
    (void)why; /* only used by the log */
    solver_getStats(&st);
    LOG("[explore] done (%s)\n", why);
    LOG("[explore] cells visited %d/%d (fully mapped %d/%d), %d actions, cost %d of budget %d\n",
        st.visitedCells, st.totalCells, st.mappedCells, st.totalCells, exploreActions,
        budgetUsed(), EXPLORE_BUDGET);
    LOG("[explore] sensor warnings: %d contradictions, %d front-sensor vetoes\n",
        contradictions, frontVetoes);
    LOG("[explore] best route proven optimal: %s, whole maze mapped: %s\n",
        routeProven ? "yes" : "no", wholeMazeMapped ? "yes" : "no");
    targetX = targetY = -1;
    {
        int x, y;
        for (x = 0; x < mazeW; x++)
            for (y = 0; y < mazeH; y++) refreshCellColor(x, y);
    }
    phase = PHASE_RETURN;
    moveQLen = moveQPos = 0;
    LOG("[phase] EXPLORE -> RETURN\n");
}

/* One explore step: re-plan, pick a target, take ONE turn or ONE cell. */
static void exploreStep(void) {
    int x, y, d, g, n, k, unknownOnRoute = 0, optCost;
    int reason, bestState = -1, bestCost = INF_COST, curCost = INF_COST;
    int bestX = -1, bestY = -1, action, nextX, nextY, nextH;

    if (exploreActions + returnActions >= HARD_ACTION_CAP) {
        endExplore("safety cap on actions");
        return;
    }

    /* (a) Best route from the start if every unknown wall were open. This is
     *     a lower bound on any real route, so if it is fully known it is
     *     optimal. */
    runDijkstra(0, 0, -1, false);
    g = bestGoalState();
    if (g < 0) {
        endExplore("no route to the goal exists");
        return;
    }
    optCost = dist[g] / COST_SCALE;
    (void)optCost; /* only used by the log */
    clearGrid(scratchMark);
    clearGrid(candidate);
    markPathCells(g, scratchMark);
    n = collectPath(g);
    for (k = 0; k < n; k++) {
        int s = pathBuf[k];
        if (prevAct[s] == ACT_FORWARD) {
            int u = prevState[s];
            int ux = stateX(u), uy = stateY(u), ud = stateHeading(u);
            if (edgeAt(ux, uy, ud) == EDGE_UNKNOWN) {
                unknownOnRoute++;
                candidate[ux][uy] = 1;                  /* check it from here   */
                candidate[ux + DX[ud]][uy + DY[ud]] = 1; /* ...or from the far side */
            }
        }
    }
    showRoute(scratchMark, 1);
    routeProven = (unknownOnRoute == 0);
    if (routeProven && !provenLogged) {
        provenLogged = true;
        LOG("[explore] best route proven optimal after %d actions: run cost %d\n",
            exploreActions, optCost);
    }
    if (routeProven && !EXPLORE_MAP_WHOLE_MAZE) {
        endExplore("best route proven; whole-maze mapping is switched off");
        return;
    }

    if (routeProven) {
        /* (b) Route is proven. Use the remaining budget to map the rest:
         *     any cell with a side we have never seen is worth visiting. */
        reason = REASON_MAP_NEAREST;
        clearGrid(candidate);
        for (x = 0; x < mazeW; x++)
            for (y = 0; y < mazeH; y++)
                for (d = 0; d < 4; d++)
                    if (edgeKnow[x][y][d] == EDGE_UNKNOWN) candidate[x][y] = 1;
    } else {
        reason = REASON_CHECK_ROUTE;
    }

    /* Nearest candidate from where the mouse is now. */
    runDijkstra(posX, posY, heading, false);
    for (x = 0; x < mazeW; x++)
        for (y = 0; y < mazeH; y++) {
            int s;
            if (!candidate[x][y]) continue;
            s = bestStateAtCell(x, y);
            if (s < 0) continue; /* walled off, can't be reached */
            if (x == targetX && y == targetY) curCost = dist[s];
            if (dist[s] < bestCost) {
                bestCost = dist[s];
                bestState = s;
                bestX = x;
                bestY = y;
            }
        }
    /* Keep the current target on ties so we never dither between two equally
     * near cells. */
    if (targetX >= 0 && curCost <= bestCost && curCost < INF_COST &&
        (candidate[targetX][targetY])) {
        bestState = bestStateAtCell(targetX, targetY);
        bestX = targetX;
        bestY = targetY;
        bestCost = curCost;
    }

    if (bestState < 0) {
        /* Nothing reachable is left to look at. If the best route is proven,
         * that means every reachable cell has been explored. */
        wholeMazeMapped = routeProven;
        endExplore(routeProven ? "every reachable cell is explored"
                               : "unknown walls on the best route are unreachable");
        return;
    }
    if (bestX == posX && bestY == posY) {
        endExplore("planner picked the cell we are standing on (bug)");
        return;
    }

    n = collectPath(bestState);
    action = prevAct[pathBuf[n - 1]];

    if (bestX != targetX || bestY != targetY || reason != lastReason ||
        DEBUG_LOG_EVERY_REPLAN) {
        LOG("[replan] target (%d,%d) because: %s; route estimate cost %d, "
            "%d unknown walls on it, target is cost %d away\n",
            bestX, bestY, reasonText(reason), optCost, unknownOnRoute,
            bestCost / COST_SCALE);
    }
    if (bestX != targetX || bestY != targetY) {
        int ox = targetX, oy = targetY;
        targetX = bestX;
        targetY = bestY;
        if (ox >= 0) refreshCellColor(ox, oy);
        refreshCellColor(targetX, targetY);
    }
    lastReason = reason;

    /* Work out where this action leaves us, then make sure we could still
     * get home from there inside the budget. Until the goal has been reached
     * once there is nothing worth going home with, so keep searching. */
    nextX = posX;
    nextY = posY;
    nextH = heading;
    if (action == ACT_FORWARD) {
        nextX += DX[heading];
        nextY += DY[heading];
    } else if (action == ACT_RIGHT) {
        nextH = (heading + 1) & 3;
    } else {
        nextH = (heading + 3) & 3;
    }
    if (goalReached || routeProven) { /* a fully known route to the goal exists */
        int back = returnCostFrom(nextX, nextY, nextH);
        if (back >= INF_COST || budgetUsed() + actionCost(action) + back > EXPLORE_BUDGET) {
            endExplore("budget reserve for the trip home reached");
            return;
        }
    } else if (budgetUsed() + actionCost(action) > EXPLORE_BUDGET && !budgetWarned) {
        /* no known route to the goal yet: keep searching, a run with no goal scores nothing */
        budgetWarned = true;
        LOG("[warn] budget used up before a route to the goal was found; continuing until it is\n");
    }

    if (action == ACT_FORWARD) {
#if VERIFY_FRONT_BEFORE_MOVE
        /* One more look with the front sensor before committing. A side we
         * have not driven through could still be a missed wall. */
        if (!drivenEdge[posX][posY][heading] && readWall(SENSE_FRONT)) {
            frontVetoes++;
            LOG("[warn] front sensor sees a wall at (%d,%d) side %c that the map says is open\n",
                posX, posY, DIR_CHAR[heading]);
            recordSensed(posX, posY, heading, true);
            refreshDistances();
            return; /* next step re-plans around it */
        }
#endif
        if (!doForward(1)) return;
        senseWalls();
    } else {
        doTurn(action);
    }
}

/* ====================================================================== */
/* Phases 2 and 3: return home, then the final run                         */
/* ====================================================================== */

static void pushMove(int kind) {
    if (kind == ACT_FORWARD && moveQLen > 0 && moveQ[moveQLen - 1].kind == ACT_FORWARD) {
        moveQ[moveQLen - 1].count++; /* merge consecutive straights */
        return;
    }
    moveQ[moveQLen].kind = (unsigned char)kind;
    moveQ[moveQLen].count = 1;
    moveQLen++;
}

/* Turn the path to `goal` (from the last runDijkstra) into queued moves. */
static void queuePath(int goal) {
    int n = collectPath(goal), k;
    moveQLen = moveQPos = 0;
    for (k = n - 1; k >= 0; k--) pushMove(prevAct[pathBuf[k]]);
}

static void planReturn(void) {
    int ret[4], run[4], h, bestH = -1, bestTotal = INF_COST;
    int remaining = EXPLORE_BUDGET - budgetUsed();

    /* How far is home for each way of arriving at the start? */
    runDijkstra(posX, posY, heading, true);
    for (h = 0; h < 4; h++) {
        int c = dist[stateIndex(0, 0, h)];
        ret[h] = (c >= INF_COST) ? INF_COST : c / COST_SCALE;
    }
    /* And how good is the final run if we start it facing each way? */
    for (h = 0; h < 4; h++) {
        int g;
        run[h] = INF_COST;
        if (ret[h] >= INF_COST) continue;
        runDijkstra(0, 0, h, true);
        g = bestGoalState();
        if (g >= 0) run[h] = dist[g] / COST_SCALE;
    }
    /* The final run is worth RUN_WEIGHT times the return trip in the score.
     * Never pick an arrival that doesn't fit in the remaining budget. */
    for (h = 0; h < 4; h++) {
        int total;
        if (ret[h] >= INF_COST || run[h] >= INF_COST || ret[h] > remaining) continue;
        total = RUN_WEIGHT * run[h] + ret[h];
        if (total < bestTotal) {
            bestTotal = total;
            bestH = h;
        }
    }
    if (bestH < 0) {
        /* No known route to the goal fits (or exists). Just get home. */
        for (h = 0; h < 4; h++)
            if (ret[h] < INF_COST && (bestH < 0 || ret[h] < ret[bestH])) bestH = h;
    }
    if (bestH < 0) {
        LOG("[error] no known way back to the start\n");
        phase = PHASE_FAILED;
        return;
    }
    runDijkstra(posX, posY, heading, true);
    queuePath(stateIndex(0, 0, bestH));
    LOG("[return] heading home: %d actions, arriving facing %c, expected final run %s\n",
        ret[bestH], DIR_CHAR[bestH],
        run[bestH] < INF_COST ? "available" : "NOT available (goal never reached)");
}

static void planFinalRun(void) {
    int g, k, cells = 0, turns = 0;
    runDijkstra(posX, posY, heading, true); /* normally the start; elsewhere after a re-plan */
    g = bestGoalState();
    if (g < 0) {
        LOG("[final] no known route to the goal, nothing to run\n");
        phase = PHASE_DONE;
        return;
    }
    clearGrid(scratchMark);
    markPathCells(g, scratchMark);
    showRoute(scratchMark, 2); /* bright final route, replaces the planned one */
    queuePath(g);
    for (k = 0; k < moveQLen; k++) {
        if (moveQ[k].kind == ACT_FORWARD) cells += moveQ[k].count;
        else turns++;
    }
    finalCells = finalExecCells + cells;
    finalTurns = finalExecTurns + turns;
    LOG("[final] route: %d cells, %d turns, %d straights; cost %d actions\n", cells, turns,
        moveQLen - turns, cells + turns);
}

/* Run one queued move. Returns false if the plan had to be cancelled because
 * a sensor check found a wall (the caller then re-plans from where we are). */
static bool executeMove(const Move *m) {
    if (m->kind != ACT_FORWARD) {
        doTurn(m->kind);
        return true;
    }
#if VERIFY_UNDRIVEN_ON_RUNS
    {
        int left = m->count;
        while (left > 0 && phase != PHASE_FAILED) {
            int run = 0, x = posX, y = posY;
            /* Sides we have driven through before are certain: no check needed. */
            while (run < left && drivenEdge[x][y][heading]) {
                x += DX[heading];
                y += DY[heading];
                run++;
            }
            if (run > 0) {
                if (!doForward(run)) return true;
                left -= run;
                continue;
            }
            if (readWall(SENSE_FRONT)) {
                frontVetoes++;
                LOG("[warn] wall found at (%d,%d) side %c on a planned route; re-planning\n",
                    posX, posY, DIR_CHAR[heading]);
                recordSensed(posX, posY, heading, true);
                refreshDistances();
                return false;
            }
            if (!doForward(1)) return true;
            left--;
        }
        return true;
    }
#else
    doForward(m->count);
    return true;
#endif
}

/* ====================================================================== */
/* Public interface                                                        */
/* ====================================================================== */

void solver_init(void) {
    int x, y, d;

    mazeW = API_mazeWidth();
    mazeH = API_mazeHeight();
    if (mazeW > MAX_MAZE_SIZE || mazeH > MAX_MAZE_SIZE) {
        LOG("[warn] maze %dx%d is larger than MAX_MAZE_SIZE %d; using the corner that fits\n",
            mazeW, mazeH, MAX_MAZE_SIZE);
        if (mazeW > MAX_MAZE_SIZE) mazeW = MAX_MAZE_SIZE;
        if (mazeH > MAX_MAZE_SIZE) mazeH = MAX_MAZE_SIZE;
    }
    /* The goal is the center. For an even size that is the middle 2 cells,
     * for an odd size the single middle cell. */
    goalLoX = (mazeW - 1) / 2;
    goalHiX = mazeW / 2;
    goalLoY = (mazeH - 1) / 2;
    goalHiY = mazeH / 2;

    for (x = 0; x < MAX_MAZE_SIZE; x++)
        for (y = 0; y < MAX_MAZE_SIZE; y++) {
            for (d = 0; d < 4; d++) {
                edgeKnow[x][y][d] = EDGE_UNKNOWN;
                drivenEdge[x][y][d] = 0;
            }
            visited[x][y] = 0;
            routeMark[x][y] = 0;
            shownDist[x][y] = -2;
            shownColor[x][y] = 0;
        }
    mapChanged = true;
    targetX = targetY = -1;
    lastReason = REASON_NONE;
    exploreActions = returnActions = finalActions = 0;
    exploreCost = returnCost = 0;
    contradictions = frontVetoes = 0;
    startTimeMs = API_timeMs();
    finalCells = finalTurns = 0;
    finalExecCells = finalExecTurns = 0;
    goalReached = routeProven = wholeMazeMapped = budgetWarned = provenLogged = false;
    moveQLen = moveQPos = 0;
    posX = posY = 0;
    heading = DIR_NORTH;

    API_clearAllColor();
    API_clearAllText();

    /* The outer border is always wall. */
    for (x = 0; x < mazeW; x++) {
        setEdge(x, 0, DIR_SOUTH, EDGE_WALL);
        setEdge(x, mazeH - 1, DIR_NORTH, EDGE_WALL);
    }
    for (y = 0; y < mazeH; y++) {
        setEdge(0, y, DIR_WEST, EDGE_WALL);
        setEdge(mazeW - 1, y, DIR_EAST, EDGE_WALL);
    }

    /* Initial distances are Manhattan; the flood-fill version takes over as
     * walls are found (it equals Manhattan until the first wall is seen). */
#if DEBUG_SHOW_DISTANCES
    for (x = 0; x < mazeW; x++)
        for (y = 0; y < mazeH; y++) showDistance(x, y, manhattanToGoal(x, y));
#endif
    mapChanged = false;
    for (x = 0; x < mazeW; x++)
        for (y = 0; y < mazeH; y++) refreshCellColor(x, y);

    LOG("[init] maze %dx%d, goal block x %d..%d y %d..%d, budget %d (cell cost %d, turn cost %d)\n",
        mazeW, mazeH, goalLoX, goalHiX, goalLoY, goalHiY, EXPLORE_BUDGET, COST_FORWARD,
        COST_TURN);
    LOG("[phase] EXPLORE\n");
#if DEBUG_LOG
    (void)reasonText; /* unused when LOG is redirected to nothing */
#endif
    phase = PHASE_EXPLORE;
    senseWalls(); /* the start cell: sees its front, left and right walls */
}

void solver_reset(void) {
    int x, y, d;
    LOG("[reset] reset button pressed, clearing all state\n");
    API_clearAllColor();
    API_clearAllText();
    for (x = 0; x < mazeW; x++)
        for (y = 0; y < mazeH; y++)
            for (d = 0; d < 4; d++)
                if (edgeKnow[x][y][d] == EDGE_WALL) clearWallDisplay(x, y, d);
    for (x = 0; x < MAX_MAZE_SIZE; x++)
        for (y = 0; y < MAX_MAZE_SIZE; y++)
            for (d = 0; d < 4; d++) edgeKnow[x][y][d] = EDGE_UNKNOWN;
    phase = PHASE_EXPLORE;
}

bool solver_step(void) {
    switch (phase) {
    case PHASE_EXPLORE:
        exploreStep();
        break;

    case PHASE_RETURN:
        if (moveQPos == 0 && moveQLen == 0) {
            planReturn();
            if (phase == PHASE_FAILED) break;
        }
        if (moveQPos < moveQLen && !executeMove(&moveQ[moveQPos++])) {
            moveQLen = moveQPos = 0; /* next step plans the way home again */
            break;
        }
        if (phase == PHASE_RETURN && moveQPos >= moveQLen) {
            LOG("[return] done: %d actions (explore + return cost %d of budget %d)\n",
                returnActions, exploreCost + returnCost, EXPLORE_BUDGET);
            phase = PHASE_FINAL;
            moveQLen = moveQPos = 0;
            LOG("[phase] RETURN -> FINAL RUN\n");
            planFinalRun();
        }
        break;

    case PHASE_FINAL:
        if (moveQPos < moveQLen && !executeMove(&moveQ[moveQPos++])) {
            planFinalRun(); /* a wall turned up: plan the rest of the run again */
            break;
        }
        if (phase == PHASE_FINAL && moveQPos >= moveQLen) {
            SolverStats st;
            solver_getStats(&st);
            LOG("[final] done: ran %d cells with %d turns (%d actions)\n", finalCells,
                finalTurns, finalActions);
            LOG("[summary] visited %d/%d cells, explore+return cost %d of budget %d, "
                "route proven optimal: %s, whole maze mapped: %s\n",
                st.visitedCells, st.totalCells, exploreCost + returnCost,
                EXPLORE_BUDGET, routeProven ? "yes" : "no", wholeMazeMapped ? "yes" : "no");
            phase = PHASE_DONE;
            LOG("[phase] DONE\n");
        }
        break;

    default:
        break;
    }
    return phase != PHASE_DONE && phase != PHASE_FAILED;
}

/* Reference example, unused by default: follow the wall on the left. It never
 * maps anything and will not find the best route; it is here to show how the
 * API calls fit together. */
void solver_leftWallFollowerStep(void) {
    if (!API_wallLeft()) {
        API_turnLeft();
        API_moveForward(1);
    } else if (!API_wallFront()) {
        API_moveForward(1);
    } else {
        API_turnRight();
    }
}

void solver_getStats(SolverStats *out) {
    out->totalCells = mazeW * mazeH;
    out->visitedCells = countVisited();
    out->mappedCells = countMapped();
    out->exploreActions = exploreActions;
    out->exploreCost = exploreCost;
    out->returnCost = returnCost;
    out->costForward = COST_FORWARD;
    out->costTurn = COST_TURN;
    out->contradictions = contradictions;
    out->frontVetoes = frontVetoes;
    out->returnActions = returnActions;
    out->budget = EXPLORE_BUDGET;
    out->finalCells = finalCells;
    out->finalTurns = finalTurns;
    out->goalReached = goalReached;
    out->routeProven = routeProven;
    out->wholeMazeMapped = wholeMazeMapped;
}

int solver_getEdge(int x, int y, int dir) {
    return edgeAt(x, y, dir);
}
