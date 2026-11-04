/*
 * ir_sensors.h - wall detection from the three infrared sensors
 * (front center, left, right).
 */
#ifndef IR_SENSORS_H
#define IR_SENSORS_H

#include <stdbool.h>

#include "hal.h"

/* Forget any calibration and use defaults (everything reads "no wall"). */
void ir_init(void);

/* One measurement: the sensor reading with its emitter on, minus the reading
 * with it off (that removes ambient light), averaged over IR_SAMPLES. */
int ir_measure(IrSensor s);

/* The same with a chosen number of readings (fewer = faster, noisier). */
int ir_measure_n(IrSensor s, int samples);

/* Teach the sensors what a wall looks like. wallLevel is ir_measure() with a
 * wall at the normal centered distance. */
void ir_set_wall_level(IrSensor s, int wallLevel);

/* True if the sensor is calibrated and sees a wall. */
bool ir_wall(IrSensor s);

/* Estimated distance from the front sensor to the wall ahead, in mm, or a
 * negative number if there is no wall or the sensor is not calibrated. */
float ir_front_distance_mm(void);

bool ir_calibrated(IrSensor s);

/* How far the mouse is from the middle of the corridor, in mm: positive when it
 * is too far right. Uses both side walls, or one if only one is there. Returns
 * false (and leaves *offsetMm alone) when there is no wall to measure against.
 * Quick: IR_CONTROL_SAMPLES readings per sensor. */
bool ir_side_offset_mm(float *offsetMm);

#endif /* IR_SENSORS_H */
