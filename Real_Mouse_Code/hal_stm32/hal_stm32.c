/*
 * hal_stm32.c - the HAL for the real mouse (STM32 Black Pill).
 *
 * NOT IMPLEMENTED YET. This file only fixes the shape: every function in
 * include/hal.h is here with a note on what it has to do and what is still
 * missing. Nothing in it touches hardware, so the mouse cannot run from this yet.
 * It is compile-checked by test/run_hw_tests.sh so it cannot drift away from
 * hal.h.
 *
 * Open decisions before this can be written:
 *   - toolchain (PlatformIO, STM32CubeIDE, or plain arm-none-eabi-gcc + CMSIS)
 *   - which encoders and which motor driver part (see pins.h)
 *   - the exact pin assignments for IR, motors and encoders (see pins.h)
 */
#include <stdarg.h>

#include "../include/hal.h"
#include "pins.h"

/* TODO: clocks to 100 MHz (or whatever the board runs at), SysTick at 1 kHz for
 * millis and the control tick, GPIO, ADC1, PWM timers, encoder timers, I2C1,
 * USART2. Then reset and configure the BNO055 (fusion mode, heading output). */
void hal_init(void) {}

/* TODO: return a counter incremented by the 1 kHz SysTick interrupt. */
uint32_t hal_millis(void) { return 0; }

/* TODO: wait (WFI) until the SysTick flag says a new millisecond has begun. */
void hal_wait_tick(void) {}

/* TODO: a short busy-wait, e.g. from the DWT cycle counter. */
void hal_delay_us(uint32_t us) { (void)us; }

/* TODO: write PWM compare values and direction pins for the dual H-bridge.
 * left_permille and right_permille are -1000..+1000, positive = forward. */
void hal_motor_set(int left_permille, int right_permille) {
    (void)left_permille;
    (void)right_permille;
}

/* TODO: read the encoder timers and extend them to 32 bits, one per wheel, with
 * the sign flipped for whichever side is mounted backwards. */
void hal_encoders(long *left_ticks, long *right_ticks) {
    *left_ticks = 0;
    *right_ticks = 0;
}

/* TODO: read the BNO055 heading over I2C (it grows clockwise, 1 LSB = 1/16 degree
 * in its default units, check the datasheet), negate it so that turning left
 * increases it, and unwrap it so it does not jump at 360 degrees. The BNO055
 * only produces new fusion data at up to 100 Hz, so either interpolate with the
 * gyro rate between updates or read it from an interrupt on PA11 (INT). */
float hal_yaw_deg(void) { return 0.0f; }

/* TODO: set the emitter's GPIO high or low. */
void hal_ir_emitter(IrSensor s, bool on) {
    (void)s;
    (void)on;
}

/* TODO: one ADC1 conversion of the sensor's receiver pin, 12 bit (0..4095). */
int hal_ir_adc(IrSensor s) {
    (void)s;
    return 0;
}

/* TODO: read PB12 (pressed = low, internal pull-up). */
bool hal_button_pressed(void) { return false; }

/* TODO: drive PC13 (active low on most Black Pill boards). */
void hal_led(bool on) { (void)on; }

/* TODO: vsnprintf into a buffer and send it out of USART2 (header J5). */
void hal_log(const char *fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    va_end(ap);
}

/* TODO: motors off, then loop forever (blink the LED so it is obvious we stopped). */
void hal_halt(void) {
    hal_motor_set(0, 0);
    for (;;) {
    }
}
