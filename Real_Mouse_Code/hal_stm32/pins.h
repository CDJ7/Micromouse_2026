/*
 * pins.h - which Black Pill pin does what.
 *
 * Only what can be read from the schematics in docs/schematics is filled in.
 * Everything else is marked TBD: the IR sensor, motor driver and encoder
 * schematics are too low resolution to read their net names, and no encoders
 * are drawn yet. Fill these in from the KiCad project, then check each pin
 * against the STM32 datasheet (ADC channel, timer channel, 5 V tolerance).
 */
#ifndef PINS_H
#define PINS_H

/* ---- known from the schematics ---- */

/* Start button SW2: between PB12 and ground, so use the internal pull-up and
 * read it as pressed when the pin is LOW. */
#define PIN_BUTTON_PORT 'B'
#define PIN_BUTTON_NUM 12

/* Black Pill on-board LED. On most Black Pill boards it is active LOW (the LED
 * lights when the pin is low): verify on the actual board. */
#define PIN_LED_PORT 'C'
#define PIN_LED_NUM 13

/* BNO055 IMU breakout (I2C). ADR is left unconnected on the schematic, which
 * should give the default I2C address 0x28: verify in the breakout's guide. */
#define PIN_IMU_SCL_PORT 'B'
#define PIN_IMU_SCL_NUM 8
#define PIN_IMU_SDA_PORT 'B'
#define PIN_IMU_SDA_NUM 9
#define PIN_IMU_INT_PORT 'A'
#define PIN_IMU_INT_NUM 11 /* WARNING: PA11 and PA12 are the USB D- / D+ pins on the Black Pill. */
#define PIN_IMU_RST_PORT 'A'
#define PIN_IMU_RST_NUM 12 /* Using them for the IMU means USB cannot be used on this board. */

/* 6-pin header J5: pin 2 = 5 V, pin 3 = GND, pin 4 = PA3, pin 5 = PA2 (these two
 * are the USART2 RX / TX pins). What the header is for is not decided yet; a
 * serial port for hal_log() is the obvious use. */
#define PIN_J5_PA3_PORT 'A'
#define PIN_J5_PA3_NUM 3
#define PIN_J5_PA2_PORT 'A'
#define PIN_J5_PA2_NUM 2

/* ---- TBD: read these from the KiCad project ---- */

/* IR emitters: three plain GPIO outputs (front center, left, right).
 *   PIN_IR_EMITTER_FRONT / _LEFT / _RIGHT      TBD
 * IR receivers: three ADC inputs. On the STM32F411, ADC1 inputs 0..7 are PA0..PA7.
 *   PIN_IR_RECEIVER_FRONT / _LEFT / _RIGHT     TBD (the schematic shows PA nets)
 * Motor driver (dual H-bridge): per motor, a PWM pin from a timer channel and a
 * direction or second-input pin.
 *   PIN_MOTOR_LEFT_PWM / _DIR, PIN_MOTOR_RIGHT_PWM / _DIR    TBD
 * Encoders: none are drawn in the schematics yet. Each needs two pins on a
 * timer that supports encoder mode (for example TIM2 / TIM3 / TIM4 channels 1 and
 * 2). Without encoders the mouse cannot measure distance.
 *   PIN_ENC_LEFT_A / _B, PIN_ENC_RIGHT_A / _B                TBD
 */

#endif /* PINS_H */
