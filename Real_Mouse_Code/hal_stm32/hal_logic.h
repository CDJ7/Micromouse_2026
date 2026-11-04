/*
 * hal_logic.h - the small pieces of the STM32 HAL that are pure arithmetic.
 *
 * They are kept apart from the register code so they can be tested on a PC
 * (test/test_hal_logic.c): turning the IMU's compass-style heading into a
 * continuous left-positive yaw, widening 16-bit encoder counters, and scaling
 * motor power to a PWM compare value.
 */
#ifndef HAL_LOGIC_H
#define HAL_LOGIC_H

#include <stdbool.h>
#include <stdint.h>

/* ---- yaw ---- */

/* The BNO055 reports heading in degrees from 0 to 360 that grows clockwise, and
 * only produces a new value about every 10 ms. The tracker turns that into the
 * yaw the firmware wants: continuous (no jump at 360), positive when turning
 * left, zero at the first sample, and extrapolated between samples so the 1 kHz
 * control loop sees a smooth value. */
typedef struct {
    bool started;
    float lastHeadingCw;  /* previous raw heading, 0..360 */
    float yawDeg;         /* continuous yaw at lastMs */
    float prevYawDeg;
    float rateDegPerMs;
    uint32_t lastMs;
    uint32_t prevMs;
} YawTracker;

void yaw_tracker_reset(YawTracker *t);

/* Feed a new heading reading taken at time `nowMs`. */
void yaw_tracker_update(YawTracker *t, float headingCwDeg, uint32_t nowMs);

/* Yaw at time `nowMs` (extrapolated for at most YAW_EXTRAPOLATE_MAX_MS). */
float yaw_tracker_get(const YawTracker *t, uint32_t nowMs);

#define YAW_EXTRAPOLATE_MAX_MS 25u

/* ---- encoders ---- */

/* The timers count in 16 bits and wrap. Add the change since the last reading
 * (correct across a wrap as long as it is read at least once per 32768 counts)
 * to a running 32-bit total. */
int32_t counter_extend(int32_t total, uint16_t previous, uint16_t now);

/* ---- motors ---- */

/* Motor power in permille (0..1000, magnitude only) to a PWM compare value for a
 * timer with `period` steps per cycle. */
uint32_t permille_to_compare(int permille, uint32_t period);

#endif /* HAL_LOGIC_H */
