/*
 * config.h - robot geometry and tuning numbers.
 *
 * Every value marked TBD is a PLACEHOLDER that has to be measured on the real
 * mouse once it is built. The values here are what the PC test (test/) uses;
 * they are reasonable for a small mouse but they are not measurements.
 */
#ifndef CONFIG_H
#define CONFIG_H

/* ---- maze ---- */
#ifndef MAZE_SIZE
#define MAZE_SIZE 10      /* cells per side of the real maze (10x10) */
#endif
#define CELL_MM 180.0f    /* TBD: confirm the real maze's cell pitch */

/* ---- drive train ---- */
#define TICKS_PER_MM 10.0f        /* TBD: encoder ticks per mm of wheel travel */
#define MOTOR_MAX_MM_S 1200.0f    /* TBD: wheel speed at full power (sets the feed-forward) */
#define PWM_MAX 1000

/* ---- speeds and accelerations ---- */
#define SEARCH_SPEED_MM_S 300.0f  /* one-cell moves while exploring */
#define RUN_SPEED_MM_S 600.0f     /* multi-cell straights on the final run */
#define ACCEL_MM_S2 2000.0f
#define STOP_SPEED_MM_S 20.0f     /* "stopped" below this */
#define MOVE_MIN_PWM 60           /* TBD: motors do not move at all below this power */
#define POSITION_DONE_MM 1.5f     /* a straight is finished when within this of its target */

/* ---- speed control: power = feed-forward + P on speed error ---- */
#define KP_SPEED 0.6f             /* TBD: permille per (mm/s) */
#define KI_SPEED 0.002f           /* TBD: permille per (mm/s * ms) */

/* ---- heading hold while driving straight (yaw error -> wheel difference) ---- */
#define KP_HEADING 25.0f          /* TBD: permille per degree */
#define KD_HEADING 0.4f           /* TBD: permille per (degree/second) */

/* ---- 90-degree turns, in place ---- */
#define KP_TURN 14.0f             /* TBD: permille per degree */
#define KD_TURN 0.6f              /* TBD: permille per (degree/second) */
#define TURN_MAX_PWM 500
#define TURN_MIN_PWM 70           /* TBD: motors need this much power to move at all */
#define TURN_DONE_DEG 1.0f
#define TURN_DONE_RATE 15.0f      /* degrees per second */

/* ---- fault detection ---- */
#define STALL_MS 400              /* told to move but not moving for this long = fault */
#define MOVE_TIMEOUT_FACTOR 3.0f  /* a move may take this many times its planned time */

/* ---- infrared sensors ---- */
#define IR_SAMPLES 4              /* readings averaged per measurement */
#define IR_SETTLE_US 60           /* TBD: wait after switching an emitter on */
#define IR_WALL_FRACTION 0.5f     /* a wall is declared above this fraction of the calibrated wall level */
#define IR_MIN_LEVEL 40           /* a calibrated wall level below this means a dead sensor */

/* Staying centered between the side walls while driving. The side sensors give
 * the distance to each wall; the difference is how far off center the mouse is,
 * and that nudges the heading target a little back toward the middle. */
#define SIDE_REF_DIST_MM 90.0f    /* TBD: sensor-to-wall distance on each side when centered */
#define LATERAL_GAIN_DEG_PER_MM 0.12f /* TBD: heading nudge per mm off center */
#define LATERAL_MAX_DEG 6.0f
#define HEADING_LEARN_GAIN 0.0008f /* TBD: how fast the walls correct the heading reference (deg per mm of offset, per update) */
#define IR_CONTROL_SAMPLES 1      /* readings per sensor while driving (speed matters more than noise) */
#define IR_CONTROL_PERIOD_TICKS 4 /* look at the side walls every this many ms */

/* Front-wall distance correction at the end of each cell. The front sensor's
 * reading falls with the square of the distance to the wall, so one calibrated
 * reading (taken at the cell center) gives distance from any other reading. */
#define FRONT_REF_DIST_MM 90.0f   /* TBD: sensor-to-wall distance when the mouse is centered */
#define FRONT_CORRECT_MAX_MM 25.0f
#define FRONT_CORRECT_MIN_MM 1.5f

#endif /* CONFIG_H */
