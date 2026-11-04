/*
 * hal.h - hardware abstraction layer (HAL) for the real mouse.
 *
 * Everything the firmware needs from the electronics, and nothing else. The
 * motion, sensor and API code in src/ only ever calls these functions, so:
 *   - hal_stm32/hal_stm32.c implements them for the Black Pill board, and
 *   - test/fake_hal.c implements them with a simulated mouse, so the whole
 *     firmware (everything except the register-level drivers) can be tested on
 *     a PC.
 *
 * Conventions (the HAL must convert from whatever the hardware reports):
 *   - lengths in millimeters, angles in degrees, time in milliseconds
 *   - yaw is positive COUNTER-clockwise (a left turn increases it)
 *   - motor power is -1000..+1000 (permille of full power), positive = forward
 */
#ifndef HAL_H
#define HAL_H

#include <stdbool.h>
#include <stdint.h>

typedef enum { IR_FRONT = 0, IR_LEFT = 1, IR_RIGHT = 2, IR_COUNT = 3 } IrSensor;

/* ---- start-up and timing ---- */
void hal_init(void);
uint32_t hal_millis(void);

/* Block until the next 1 ms control tick. All control loops run on this tick.
 * (The fake HAL advances its physics here.) */
void hal_wait_tick(void);

/* Busy-wait a few microseconds (used to let an IR emitter settle). */
void hal_delay_us(uint32_t us);

/* ---- motors and encoders ---- */
void hal_motor_set(int left_permille, int right_permille);

/* Cumulative encoder counts since start-up, positive when the wheel rolls
 * forward. */
void hal_encoders(long *left_ticks, long *right_ticks);

/* ---- IMU ---- */
/* Heading in degrees, continuous (it does not wrap at 360), positive when
 * turning left. The BNO055 reports compass-style heading that grows clockwise,
 * so the real HAL negates and unwraps it. */
float hal_yaw_deg(void);

/* ---- infrared wall sensors ---- */
void hal_ir_emitter(IrSensor s, bool on);
int hal_ir_adc(IrSensor s); /* 0..4095 */

/* ---- user interface ---- */
bool hal_button_pressed(void);
void hal_led(bool on);

/* Debug text, printf style. On the real mouse this goes to a serial port. */
void hal_log(const char *fmt, ...);

/* Called when the run is over (or something went wrong). On the real mouse it
 * stops the motors and never returns. The fake HAL returns control to the test. */
void hal_halt(void);

#endif /* HAL_H */
