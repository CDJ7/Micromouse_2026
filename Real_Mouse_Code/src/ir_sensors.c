/*
 * ir_sensors.c - wall detection from the three IR sensors.
 *
 * Each sensor is an IR LED (emitter) and a phototransistor (receiver). Sunlight
 * and room lights also hit the receiver, so every measurement is two readings:
 * one with the LED off (the ambient light) and one with it on. The difference is
 * the light that bounced off a wall.
 */
#include "ir_sensors.h"

#include <math.h>

#include "config.h"

/* Calibrated reading for "wall at the normal distance". 0 = not calibrated. */
static int wallLevel[IR_COUNT];

void ir_init(void) {
    int s;
    for (s = 0; s < IR_COUNT; s++) {
        wallLevel[s] = 0;
        hal_ir_emitter((IrSensor)s, false);
    }
}

int ir_measure_n(IrSensor s, int samples) {
    long sum = 0;
    int i;
    for (i = 0; i < samples; i++) {
        int ambient, lit;
        hal_ir_emitter(s, false);
        hal_delay_us(IR_SETTLE_US);
        ambient = hal_ir_adc(s);
        hal_ir_emitter(s, true);
        hal_delay_us(IR_SETTLE_US);
        lit = hal_ir_adc(s);
        hal_ir_emitter(s, false); /* LEDs are only on while measuring (power, heat) */
        sum += lit - ambient;
    }
    sum /= samples;
    return sum < 0 ? 0 : (int)sum;
}

int ir_measure(IrSensor s) {
    return ir_measure_n(s, IR_SAMPLES);
}

void ir_set_wall_level(IrSensor s, int level) {
    wallLevel[s] = level;
}

bool ir_calibrated(IrSensor s) {
    return wallLevel[s] > 0;
}

/* Fail safe: a sensor that was never calibrated reports a wall, so the mouse
 * does not drive on readings it cannot trust. */
bool ir_wall(IrSensor s) {
    if (!ir_calibrated(s)) return true;
    return ir_measure(s) >= (int)(IR_WALL_FRACTION * (float)wallLevel[s]);
}

float ir_front_distance_mm(void) {
    int m;
    if (!ir_calibrated(IR_FRONT)) return -1.0f;
    m = ir_measure(IR_FRONT);
    if (m < (int)(IR_WALL_FRACTION * (float)wallLevel[IR_FRONT])) return -1.0f; /* no wall */
    /* The reading falls with the square of the distance. */
    return FRONT_REF_DIST_MM * sqrtf((float)wallLevel[IR_FRONT] / (float)m);
}

/* Distance from a side sensor to its wall, from the calibrated reference. */
static float sideDistanceMm(IrSensor s, int m) {
    return SIDE_REF_DIST_MM * sqrtf((float)wallLevel[s] / (float)m);
}

bool ir_side_offset_mm(float *offsetMm) {
    float dl = -1.0f, dr = -1.0f;
    if (ir_calibrated(IR_LEFT)) {
        int m = ir_measure_n(IR_LEFT, IR_CONTROL_SAMPLES);
        if (m >= (int)(IR_WALL_FRACTION * (float)wallLevel[IR_LEFT])) dl = sideDistanceMm(IR_LEFT, m);
    }
    if (ir_calibrated(IR_RIGHT)) {
        int m = ir_measure_n(IR_RIGHT, IR_CONTROL_SAMPLES);
        if (m >= (int)(IR_WALL_FRACTION * (float)wallLevel[IR_RIGHT])) dr = sideDistanceMm(IR_RIGHT, m);
    }
    if (dl > 0.0f && dr > 0.0f) *offsetMm = 0.5f * (dl - dr); /* both walls: the difference */
    else if (dl > 0.0f) *offsetMm = dl - SIDE_REF_DIST_MM;     /* only left: compare with the reference */
    else if (dr > 0.0f) *offsetMm = SIDE_REF_DIST_MM - dr;     /* only right */
    else return false;
    return true;
}
