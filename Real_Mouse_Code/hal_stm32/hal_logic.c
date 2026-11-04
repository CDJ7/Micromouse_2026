/*
 * hal_logic.c - pure arithmetic used by hal_stm32.c. See hal_logic.h.
 */
#include "hal_logic.h"

void yaw_tracker_reset(YawTracker *t) {
    t->started = false;
    t->lastHeadingCw = 0.0f;
    t->yawDeg = 0.0f;
    t->prevYawDeg = 0.0f;
    t->rateDegPerMs = 0.0f;
    t->lastMs = 0;
    t->prevMs = 0;
}

void yaw_tracker_update(YawTracker *t, float headingCwDeg, uint32_t nowMs) {
    float delta;
    if (!t->started) {
        t->started = true;
        t->lastHeadingCw = headingCwDeg;
        t->yawDeg = t->prevYawDeg = 0.0f;
        t->rateDegPerMs = 0.0f;
        t->lastMs = t->prevMs = nowMs;
        return;
    }
    /* Shortest way round: the heading may jump from 359.9 to 0.1. */
    delta = headingCwDeg - t->lastHeadingCw;
    if (delta > 180.0f) delta -= 360.0f;
    if (delta < -180.0f) delta += 360.0f;
    t->lastHeadingCw = headingCwDeg;

    t->prevYawDeg = t->yawDeg;
    t->prevMs = t->lastMs;
    t->yawDeg -= delta; /* clockwise heading grows when turning right; yaw is positive to the left */
    t->lastMs = nowMs;
    if (nowMs > t->prevMs) {
        t->rateDegPerMs = (t->yawDeg - t->prevYawDeg) / (float)(nowMs - t->prevMs);
    }
}

float yaw_tracker_get(const YawTracker *t, uint32_t nowMs) {
    uint32_t dt;
    if (!t->started) return 0.0f;
    dt = nowMs - t->lastMs;
    if (dt > YAW_EXTRAPOLATE_MAX_MS) dt = YAW_EXTRAPOLATE_MAX_MS;
    return t->yawDeg + t->rateDegPerMs * (float)dt;
}

int32_t counter_extend(int32_t total, uint16_t previous, uint16_t now) {
    return total + (int16_t)(uint16_t)(now - previous);
}

uint32_t permille_to_compare(int permille, uint32_t period) {
    if (permille < 0) permille = 0;
    if (permille > 1000) permille = 1000;
    return (uint32_t)permille * period / 1000u;
}
