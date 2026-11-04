/*
 * fake_hal.c - a simulated mouse behind the HAL, so the real firmware
 * (motion, IR sensors, API, start-up) runs on a PC.
 *
 * It is a small physics model, not a measurement of any real mouse:
 *   - two driven wheels with a lag between power and speed, a power dead zone
 *     and a few percent of encoder and gyro scale error
 *   - the body hits a wall if it gets within 35 mm of one
 *   - each IR sensor casts a ray into the maze and reads a signal that falls
 *     with distance (deliberately NOT a perfect inverse square), plus ambient
 *     light that drifts, plus noise
 * The numbers are chosen to be somewhat unfriendly, so the controllers are not
 * tuned to a perfect world.
 */
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "config.h"
#include "fake_hal.h"
#include "hal.h"
#include "sim.h"

#define PI 3.14159265358979
#define SIM_CELL 180.0       /* true cell pitch in the simulated maze */
#define BODY_RADIUS 35.0
#define TRACK_MM 70.0        /* distance between the wheels */
#define MOTOR_TAU_MS 35.0
#ifndef MOTOR_MM_S_PER_PERMILLE
#define MOTOR_MM_S_PER_PERMILLE 1.10 /* the simulated motor is 8% slower than config.h assumes */
#endif
#define DEAD_ZONE_PERMILLE 40
#ifndef ENC_SCALE_ERROR
#define ENC_SCALE_ERROR 1.004        /* encoders over-count by 0.4% */
#endif
#ifndef GYRO_SCALE_ERROR
#define GYRO_SCALE_ERROR 1.002
#endif
#ifndef GYRO_NOISE_DEG
#define GYRO_NOISE_DEG 0.04
#endif
#define IR_GAIN 1200.0               /* signal at 90 mm */
#define IR_OFFSET_MM 10.0
#ifndef IR_NOISE
#define IR_NOISE 12.0
#endif

static FakeMouse m;
static double velL, velR;          /* wheel speeds, mm/s */
static int pwmL, pwmR;
static double encL, encR;          /* wheel travel, mm */
static int emitter[IR_COUNT];
static uint32_t nowMs;
static unsigned rng = 777u;
jmp_buf fake_halt_jump;

FakeMouse *fake_mouse(void) { return &m; }

static double noise(void) { /* roughly Gaussian, mean 0, spread 1 */
    double s = 0;
    int i;
    for (i = 0; i < 4; i++) {
        rng = rng * 1664525u + 1013904223u;
        s += ((rng >> 8) & 0xFFFF) / 65536.0 - 0.5;
    }
    return s * 1.7;
}

void fake_reset(void) {
    memset(&m, 0, sizeof m);
    m.x = SIM_CELL / 2;
    m.y = SIM_CELL / 2;
    m.thetaDeg = 90.0;
    velL = velR = encL = encR = 0;
    pwmL = pwmR = 0;
    nowMs = 0;
    emitter[0] = emitter[1] = emitter[2] = 0;
}

/* ---- walls ---- */

static int wallAt(int cx, int cy, int d) {
    const SimMaze *mz = sim_maze();
    if (cx < 0 || cy < 0 || cx >= mz->w || cy >= mz->h) return 1;
    return mz->wall[cx][cy][d];
}

static int hitsWall(double x, double y) {
    int cx = (int)floor(x / SIM_CELL), cy = (int)floor(y / SIM_CELL);
    double lx = x - cx * SIM_CELL, ly = y - cy * SIM_CELL;
    if (wallAt(cx, cy, 0) && SIM_CELL - ly < BODY_RADIUS) return 1;
    if (wallAt(cx, cy, 1) && SIM_CELL - lx < BODY_RADIUS) return 1;
    if (wallAt(cx, cy, 2) && ly < BODY_RADIUS) return 1;
    if (wallAt(cx, cy, 3) && lx < BODY_RADIUS) return 1;
    return 0;
}

/* Distance along a ray from (x,y) in direction (dx,dy) to the nearest wall. */
static double castRay(double x, double y, double dx, double dy) {
    const SimMaze *mz = sim_maze();
    double best = 1e9;
    int cx, cy, d;
    for (cx = 0; cx < mz->w; cx++)
        for (cy = 0; cy < mz->h; cy++)
            for (d = 0; d < 4; d++) {
                double t = 1e9, x0, y0, x1, y1;
                if (!mz->wall[cx][cy][d]) continue;
                x0 = cx * SIM_CELL; y0 = cy * SIM_CELL;
                x1 = x0 + SIM_CELL; y1 = y0 + SIM_CELL;
                if (d == 0) { /* north wall: horizontal line y = y1 */
                    if (fabs(dy) > 1e-9) t = (y1 - y) / dy;
                    if (t > 0 && (x + t * dx) >= x0 && (x + t * dx) <= x1 && t < best) best = t;
                } else if (d == 2) { /* south wall: y = y0 */
                    if (fabs(dy) > 1e-9) t = (y0 - y) / dy;
                    if (t > 0 && (x + t * dx) >= x0 && (x + t * dx) <= x1 && t < best) best = t;
                } else if (d == 1) { /* east wall: x = x1 */
                    if (fabs(dx) > 1e-9) t = (x1 - x) / dx;
                    if (t > 0 && (y + t * dy) >= y0 && (y + t * dy) <= y1 && t < best) best = t;
                } else { /* west wall: x = x0 */
                    if (fabs(dx) > 1e-9) t = (x0 - x) / dx;
                    if (t > 0 && (y + t * dy) >= y0 && (y + t * dy) <= y1 && t < best) best = t;
                }
            }
    return best;
}

/* ---- physics ---- */

static void step(void) {
    const double dt = 0.001;
    double target, v, omega, th, nx, ny, cellOffX, cellOffY, err;
    int cx, cy;

    target = (abs(pwmL) < DEAD_ZONE_PERMILLE ? 0 : pwmL) * MOTOR_MM_S_PER_PERMILLE;
    velL += (target - velL) * (1.0 / MOTOR_TAU_MS);
    target = (abs(pwmR) < DEAD_ZONE_PERMILLE ? 0 : pwmR) * MOTOR_MM_S_PER_PERMILLE;
    velR += (target - velR) * (1.0 / MOTOR_TAU_MS);

    v = 0.5 * (velL + velR);
    omega = (velR - velL) / TRACK_MM; /* rad/s, positive = counter-clockwise */
    th = m.thetaDeg * PI / 180.0;
    th += omega * dt;
    nx = m.x + v * cos(th) * dt;
    ny = m.y + v * sin(th) * dt;

    if (hitsWall(nx, ny)) {
        m.crashes++; /* stop dead against the wall */
        velL = velR = 0;
    } else {
        m.x = nx;
        m.y = ny;
        m.thetaDeg = th * 180.0 / PI;
        encL += velL * dt;
        encR += velR * dt;
    }

    /* How well is it driving? Only count while moving. */
    if (fabs(v) > 50.0) {
        cx = (int)floor(m.x / SIM_CELL);
        cy = (int)floor(m.y / SIM_CELL);
        cellOffX = fabs(m.x - (cx + 0.5) * SIM_CELL);
        cellOffY = fabs(m.y - (cy + 0.5) * SIM_CELL);
        /* the offset that matters is the one across the direction of travel */
        {
            double c = fabs(cos(th)), s = fabs(sin(th));
            double across = c > s ? cellOffY : cellOffX;
            if (across > m.maxOffCenterMm) m.maxOffCenterMm = across;
        }
        err = fmod(m.thetaDeg - 90.0, 90.0);
        if (err > 45.0) err -= 90.0;
        if (err < -45.0) err += 90.0;
        if (fabs(err) > m.maxHeadingErrDeg) m.maxHeadingErrDeg = fabs(err);
    }
    {
        static int prevCx, prevCy;
        int ncx = (int)floor(m.x / SIM_CELL), ncy = (int)floor(m.y / SIM_CELL);
        if (m.ticks == 0) { prevCx = ncx; prevCy = ncy; }
        if (prevCx == 0 && prevCy == 0 && (ncx != 0 || ncy != 0)) m.lastLeaveStartMs = nowMs;
        prevCx = ncx;
        prevCy = ncy;
    }
    nowMs++;
    m.ticks++;
}

/* ---- HAL ---- */

void hal_init(void) { fake_reset(); }
uint32_t hal_millis(void) { return nowMs; }
void hal_wait_tick(void) { step(); }
void hal_delay_us(uint32_t us) { (void)us; }

void hal_motor_set(int l, int r) {
    pwmL = l > PWM_MAX ? PWM_MAX : (l < -PWM_MAX ? -PWM_MAX : l);
    pwmR = r > PWM_MAX ? PWM_MAX : (r < -PWM_MAX ? -PWM_MAX : r);
}

void hal_encoders(long *l, long *r) {
    *l = (long)floor(encL * ENC_SCALE_ERROR * TICKS_PER_MM);
    *r = (long)floor(encR * ENC_SCALE_ERROR * TICKS_PER_MM);
}

float hal_yaw_deg(void) {
    return (float)((m.thetaDeg - 90.0) * GYRO_SCALE_ERROR + GYRO_NOISE_DEG * noise());
}

void hal_ir_emitter(IrSensor s, bool on) { emitter[s] = on ? 1 : 0; }

int hal_ir_adc(IrSensor s) {
    double th = m.thetaDeg * PI / 180.0, dirAngle, d, signal = 0.0, ambient, adc;
    dirAngle = th + (s == IR_LEFT ? PI / 2 : (s == IR_RIGHT ? -PI / 2 : 0.0));
    if (emitter[s]) {
        d = castRay(m.x, m.y, cos(dirAngle), sin(dirAngle));
        /* falls with distance, but not exactly as 1/d^2 */
        signal = IR_GAIN * (90.0 + IR_OFFSET_MM) * (90.0 + IR_OFFSET_MM) /
                 ((d + IR_OFFSET_MM) * (d + IR_OFFSET_MM));
    }
    ambient = 350.0 + 150.0 * sin(nowMs / 700.0); /* room light that slowly drifts */
    adc = ambient + signal + IR_NOISE * noise();
    return adc < 0 ? 0 : (adc > 4095 ? 4095 : (int)adc);
}

bool hal_button_pressed(void) { return nowMs > 300 && nowMs < 700; }
void hal_led(bool on) { (void)on; }

void hal_log(const char *fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    vfprintf(stderr, fmt, ap);
    va_end(ap);
}

void hal_halt(void) {
    hal_motor_set(0, 0);
    m.halted = 1;
    longjmp(fake_halt_jump, 1);
}
