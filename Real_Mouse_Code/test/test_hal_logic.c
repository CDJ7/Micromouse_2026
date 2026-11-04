/*
 * test_hal_logic.c - PC test of the arithmetic in hal_stm32/hal_logic.c.
 */
#include <math.h>
#include <stdio.h>

#include "../hal_stm32/hal_logic.h"

static int failures;

static void check(int ok, const char *what) {
    printf("    [%s] %s\n", ok ? "PASS" : "FAIL", what);
    if (!ok) failures++;
}

static int near(float a, float b, float tol) { return fabsf(a - b) <= tol; }

int main(void) {
    YawTracker t;
    int i;
    float h;

    printf("hal_logic\n");

    /* Turning left makes the compass heading fall. Yaw should rise, through 0/360. */
    yaw_tracker_reset(&t);
    yaw_tracker_update(&t, 10.0f, 0);
    check(near(yaw_tracker_get(&t, 0), 0.0f, 1e-4f), "yaw starts at zero");
    yaw_tracker_update(&t, 350.0f, 10); /* 20 degrees to the left, across 0 */
    check(near(yaw_tracker_get(&t, 10), 20.0f, 1e-3f), "left turn across 0/360 increases yaw");
    yaw_tracker_update(&t, 340.0f, 20);
    check(near(yaw_tracker_get(&t, 20), 30.0f, 1e-3f), "and keeps counting");

    /* A full left revolution in 15-degree steps, then a right one. */
    yaw_tracker_reset(&t);
    h = 100.0f;
    yaw_tracker_update(&t, h, 0);
    for (i = 1; i <= 24; i++) {
        h -= 15.0f;
        if (h < 0.0f) h += 360.0f;
        yaw_tracker_update(&t, h, (uint32_t)i * 10u);
    }
    check(near(yaw_tracker_get(&t, 240), 360.0f, 1e-2f), "a full left revolution is +360, not 0");
    for (i = 25; i <= 48; i++) {
        h += 15.0f;
        if (h >= 360.0f) h -= 360.0f;
        yaw_tracker_update(&t, h, (uint32_t)i * 10u);
    }
    check(near(yaw_tracker_get(&t, 480), 0.0f, 1e-2f), "and a right revolution brings it back");

    /* Between 10 ms readings the value keeps moving at the measured rate. */
    yaw_tracker_reset(&t);
    yaw_tracker_update(&t, 200.0f, 0);
    yaw_tracker_update(&t, 199.0f, 10); /* 1 degree left in 10 ms = 0.1 deg/ms */
    check(near(yaw_tracker_get(&t, 15), 1.5f, 1e-3f), "extrapolates between readings");
    check(near(yaw_tracker_get(&t, 1000), 1.0f + 0.1f * 25.0f, 1e-3f),
          "but not for long if readings stop");

    /* Encoder counters. */
    check(counter_extend(100, 10, 15) == 105, "counter counts up");
    check(counter_extend(100, 65530, 4) == 110, "counter wraps forward");
    check(counter_extend(100, 3, 65535) == 96, "counter wraps backward");
    check(counter_extend(100, 500, 400) == 0, "counter counts down");

    /* PWM scaling. */
    check(permille_to_compare(0, 5000) == 0, "0 permille is 0");
    check(permille_to_compare(1000, 5000) == 5000, "full power is a full period");
    check(permille_to_compare(250, 5000) == 1250, "250 permille is a quarter");
    check(permille_to_compare(-5, 5000) == 0 && permille_to_compare(5000, 5000) == 5000,
          "out-of-range power is clamped");

    printf("%s\n", failures ? "SOME HAL LOGIC TESTS FAILED" : "ALL HAL LOGIC TESTS PASSED");
    return failures ? 1 : 0;
}
