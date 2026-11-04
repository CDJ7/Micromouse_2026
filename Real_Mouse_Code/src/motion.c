/*
 * motion.c - straights and turns, built only on the HAL.
 *
 * Straights: a trapezoid speed profile (accelerate, cruise, brake so the speed
 * reaches zero exactly at the target distance). Wheel power is feed-forward
 * (power proportional to wanted speed) plus a correction from the measured
 * speed. A second loop holds the heading using the IMU, so the mouse drives
 * straight and does not slowly drift into a side wall.
 *
 * Turns: the IMU heading is held as an absolute target (always a multiple of 90
 * degrees from the start), so turning errors never add up from one turn to the
 * next.
 *
 * Every number tuned for the real mouse lives in config.h.
 */
#include "motion.h"

#include <math.h>

#include "config.h"
#include "hal.h"
#include "ir_sensors.h"

#define HISTORY 32        /* ticks of history kept (power of two) */
#define SPEED_WINDOW 10   /* speed is measured over this many ticks */
#define RATE_WINDOW 25    /* turn rate over this many (the IMU is noisier than the encoders) */

static float targetYaw;                       /* heading we want to hold, degrees */
static float posHist[HISTORY], yawHist[HISTORY];
static unsigned histHead;                     /* next slot to write */
static float curPos, curVel, curYaw, curRate; /* mm, mm/s, deg, deg/s */

static float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

static int clampPwm(float v, int lim) {
    return (int)clampf(v, (float)-lim, (float)lim);
}

/* Average wheel travel in mm since start-up. */
static float travelMm(void) {
    long l, r;
    hal_encoders(&l, &r);
    return 0.5f * ((float)l + (float)r) / TICKS_PER_MM;
}

/* Wait one tick, then refresh position, speed, heading and turn rate. Speed and
 * rate come from windows of several ticks, which is much smoother than one
 * tick (one encoder count or one IMU reading would be mostly noise). */
static void tick(void) {
    unsigned oldSpeed, oldRate;
    hal_wait_tick();
    curPos = travelMm();
    curYaw = hal_yaw_deg();
    oldSpeed = (histHead - SPEED_WINDOW) & (HISTORY - 1);
    oldRate = (histHead - RATE_WINDOW) & (HISTORY - 1);
    curVel = (curPos - posHist[oldSpeed]) / ((float)SPEED_WINDOW * 0.001f);
    curRate = (curYaw - yawHist[oldRate]) / ((float)RATE_WINDOW * 0.001f);
    posHist[histHead & (HISTORY - 1)] = curPos;
    yawHist[histHead & (HISTORY - 1)] = curYaw;
    histHead++;
}

static void resetHistory(void) {
    int i;
    curPos = travelMm();
    curYaw = hal_yaw_deg();
    for (i = 0; i < HISTORY; i++) {
        posHist[i] = curPos;
        yawHist[i] = curYaw;
    }
    histHead = 0;
    curVel = curRate = 0.0f;
}

void motion_init(void) {
    targetYaw = hal_yaw_deg();
    resetHistory();
}

void motion_stop(void) {
    hal_motor_set(0, 0);
}

float motion_cell_time_s(int cells) {
    float v = cells > 1 ? RUN_SPEED_MM_S : SEARCH_SPEED_MM_S;
    float d = (float)cells * CELL_MM;
    float dAccel = v * v / (2.0f * ACCEL_MM_S2); /* distance to reach v (and again to stop) */
    if (2.0f * dAccel >= d) return 2.0f * sqrtf(d / ACCEL_MM_S2);
    return 2.0f * v / ACCEL_MM_S2 + (d - 2.0f * dAccel) / v;
}

/* center = true also nudges the heading to stay in the middle of the corridor. */
static bool drive(float mm, float maxSpeed, bool center) {
    float lateral = 0.0f; /* filtered offset from the corridor center, mm (positive = right) */
    float dir = mm >= 0.0f ? 1.0f : -1.0f;
    float distance = fabsf(mm);
    float startPos, vProfile = 0.0f, integral = 0.0f;
    float feedForward = (float)PWM_MAX / MOTOR_MAX_MM_S;
    uint32_t timeoutMs = (uint32_t)(MOVE_TIMEOUT_FACTOR *
                         (distance / maxSpeed + 2.0f * maxSpeed / ACCEL_MM_S2) * 1000.0f) + 600u;
    uint32_t elapsed = 0, stallMs = 0, doneMs = 0;

    resetHistory();
    startPos = curPos;

    for (;;) {
        float progress, remaining, speed, vStop, speedErr, base, yawErr, steer, aim;
        tick();
        elapsed++;
        progress = (curPos - startPos) * dir; /* mm covered in the wanted direction */
        speed = curVel * dir;
        remaining = distance - progress;

        /* Speed profile: speed up, hold, and brake so we arrive at rest. */
        vStop = sqrtf(2.0f * ACCEL_MM_S2 * (remaining > 0.0f ? remaining : 0.0f));
        vProfile += ACCEL_MM_S2 * 0.001f;
        if (vProfile > maxSpeed) vProfile = maxSpeed;
        if (vProfile > vStop) vProfile = vStop;
        if (remaining < 4.0f) vProfile = clampf(remaining * 10.0f, -30.0f, 30.0f); /* creep in */

        speedErr = vProfile - speed;
        integral = clampf(integral + KI_SPEED * speedErr, -200.0f, 200.0f);
        base = (feedForward * vProfile + KP_SPEED * speedErr + integral) * dir;
        /* Motors do not move below some power; never ask for less while there
         * is still distance to cover, or the last millimeters take forever. */
        if (fabsf(remaining) > POSITION_DONE_MM && fabsf(base) < (float)MOVE_MIN_PWM) {
            base = (remaining > 0.0f ? dir : -dir) * (float)MOVE_MIN_PWM;
        }

        /* Corridor centering: look at the side walls now and then, and aim a
         * few degrees back toward the middle if we have wandered off it. */
        if (center && (elapsed % IR_CONTROL_PERIOD_TICKS) == 0 && remaining > 10.0f) {
            float offset;
            if (ir_side_offset_mm(&offset)) {
                lateral += 0.4f * (offset - lateral);
                /* A steady offset means the IMU's idea of "straight" is a little
                 * off (its error grows with every turn it has made). The walls
                 * know where straight really is, so move the heading reference
                 * to match. This is what stops a small gyro error from adding up
                 * over a whole maze. */
                if (speed > 100.0f && fabsf(lateral) < 25.0f) {
                    targetYaw += HEADING_LEARN_GAIN * lateral;
                }
            } else {
                lateral *= 0.9f; /* no wall to look at: fade out */
            }
        }
        aim = center ? clampf(LATERAL_GAIN_DEG_PER_MM * lateral, -LATERAL_MAX_DEG, LATERAL_MAX_DEG) : 0.0f;

        /* Heading hold: if we have turned left of the target, slow the left wheel. */
        yawErr = targetYaw + aim - curYaw;
        steer = KP_HEADING * yawErr - KD_HEADING * curRate;
        hal_motor_set(clampPwm(base - steer, PWM_MAX), clampPwm(base + steer, PWM_MAX));

        if (fabsf(remaining) < POSITION_DONE_MM && fabsf(speed) < STOP_SPEED_MM_S) {
            if (++doneMs >= 30) break;
        } else {
            doneMs = 0;
        }
        /* Told to move but not moving: we are against something. */
        if (vProfile > 60.0f && fabsf(speed) < 10.0f) {
            if (++stallMs >= STALL_MS) {
                hal_motor_set(0, 0);
                hal_log("[motion] stalled after %d mm of %d\n", (int)progress, (int)distance);
                return false;
            }
        } else {
            stallMs = 0;
        }
        if (elapsed > timeoutMs) {
            hal_motor_set(0, 0);
            hal_log("[motion] move timed out\n");
            return false;
        }
    }
    hal_motor_set(0, 0);
    return true;
}

bool motion_move_mm(float mm, float maxSpeed) {
    return drive(mm, maxSpeed, false);
}

bool motion_forward(int cells) {
    float d, error;
    if (cells < 1) return false;
    if (!drive((float)cells * CELL_MM, cells > 1 ? RUN_SPEED_MM_S : SEARCH_SPEED_MM_S, true)) {
        return false;
    }
    /* Wheel slip makes the distance estimate drift. A wall ahead gives an
     * absolute reference, so use it to put the mouse back where it should be. */
    d = ir_front_distance_mm();
    if (d > 0.0f) {
        error = d - FRONT_REF_DIST_MM; /* positive: still too far from the wall */
        if (fabsf(error) >= FRONT_CORRECT_MIN_MM && fabsf(error) <= FRONT_CORRECT_MAX_MM) {
            return motion_move_mm(error, 100.0f);
        }
    }
    return true;
}

bool motion_turn(int direction) {
    uint32_t elapsed = 0, doneMs = 0;
    targetYaw += 90.0f * (float)(direction >= 0 ? 1 : -1);
    resetHistory();

    for (;;) {
        float err, u;
        tick();
        elapsed++;
        err = targetYaw - curYaw; /* positive: still have to turn left */
        u = KP_TURN * err - KD_TURN * curRate;
        u = clampf(u, -(float)TURN_MAX_PWM, (float)TURN_MAX_PWM);
        /* Motors do not move at all below some power, so never ask for less
         * while there is still turning to do. */
        if (fabsf(err) > TURN_DONE_DEG && fabsf(u) < (float)TURN_MIN_PWM) {
            u = err > 0.0f ? (float)TURN_MIN_PWM : -(float)TURN_MIN_PWM;
        }
        hal_motor_set(clampPwm(-u, PWM_MAX), clampPwm(u, PWM_MAX));

        if (fabsf(err) < TURN_DONE_DEG && fabsf(curRate) < TURN_DONE_RATE) {
            if (++doneMs >= 30) break;
        } else {
            doneMs = 0;
        }
        if (elapsed > 2000u) {
            hal_motor_set(0, 0);
            hal_log("[motion] turn timed out (error %d deg)\n", (int)err);
            return false;
        }
    }
    hal_motor_set(0, 0);
    return true;
}
