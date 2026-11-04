/*
 * pins.h - which Black Pill pin does what.
 *
 * KNOWN pins come straight from the schematics in docs/schematics.
 *
 * PROPOSED pins are NOT from the schematics. The IR, motor and encoder
 * schematics are too low resolution to read their net names, so these are an
 * inference: the microcontroller schematic connects exactly 22 pins, 8 of them
 * are identified above (button, LED, IMU, J5), and the other 14 are exactly
 * what the rest of the design needs (3 IR receivers + 3 IR emitters + 4 motor
 * inputs + 4 encoder inputs). The proposal gives each of those 14 pins a job
 * that its timer or ADC can actually do on the STM32F411.
 *
 * ==> Compare the PROPOSED table below with the KiCad schematic and edit the
 *     lines that differ. Each pin is one line. Then define PINS_CONFIRMED (in the
 *     Makefile or here) to silence the warning at build time.
 */
#ifndef PINS_H
#define PINS_H

#include "stm32f4xx.h"

/* ============================ KNOWN ============================ */

/* Start button SW2 between PB12 and ground: internal pull-up, pressed = LOW. */
#define PIN_BUTTON GPIOB, 12

/* Black Pill on-board LED: active LOW on most boards (verify on yours). */
#define PIN_LED GPIOC, 13

/* BNO055 IMU breakout on I2C1 (alternate pins, AF4). ADR is left unconnected on
 * the schematic, which should give I2C address 0x28 (verify in the breakout's
 * guide). WARNING: PA11 and PA12 are the Black Pill's USB D- / D+ pins. Using
 * them for the IMU means the USB bootloader (DFU) cannot be used: flash with an
 * ST-Link on the SWD pins instead. */
#define PIN_IMU_SCL GPIOB, 8
#define PIN_IMU_SDA GPIOB, 9
#define PIN_IMU_INT GPIOA, 11 /* not used by the firmware yet */
#define PIN_IMU_RST GPIOA, 12 /* active low */
#define IMU_I2C_ADDRESS 0x28

/* Header J5 pin 5 = PA2, pin 4 = PA3: USART2 TX / RX. Used for hal_log(). */
#define PIN_UART_TX GPIOA, 2
#define PIN_UART_RX GPIOA, 3

/* ========================== PROPOSED ========================== */

/* IR receivers (phototransistors): ADC1 inputs. PA1 = channel 1, PA4 = channel
 * 4, PA5 = channel 5 (on the F411, PA0..PA7 are ADC1 channels 0..7). */
#define PIN_IR_RX_FRONT GPIOA, 4
#define IR_RX_FRONT_CHANNEL 4
#define PIN_IR_RX_LEFT GPIOA, 5
#define IR_RX_LEFT_CHANNEL 5
#define PIN_IR_RX_RIGHT GPIOA, 1
#define IR_RX_RIGHT_CHANNEL 1

/* IR emitters: plain GPIO outputs, high = LED on. */
#define PIN_IR_TX_FRONT GPIOA, 6
#define PIN_IR_TX_LEFT GPIOA, 7
#define PIN_IR_TX_RIGHT GPIOA, 10

/* Dual H-bridge, two inputs per motor (the DRV8833 style: forward = PWM on
 * input 1 with input 2 low, reverse = the other way round). If the driver is the
 * PWM + direction style instead, hal_motor_set() has to change.
 *   left motor : TIM1 CH1 on PA8 (AF1), TIM1 CH2 on PA9 (AF1)
 *   right motor: TIM2 CH1 on PA15 (AF1), TIM2 CH2 on PB3 (AF1)
 * (PA15 and PB3 are JTAG pins at reset; SWD on PA13/PA14 is left alone.) */
#define PIN_MOTOR_L_IN1 GPIOA, 8
#define PIN_MOTOR_L_IN2 GPIOA, 9
#define PIN_MOTOR_R_IN1 GPIOA, 15
#define PIN_MOTOR_R_IN2 GPIOB, 3
/* 1 if a wheel's motor is wired so that positive power turns it backwards. */
#define MOTOR_L_REVERSED 0
#define MOTOR_R_REVERSED 0

/* Quadrature encoders on timers in encoder mode (A = channel 1, B = channel 2).
 *   left : TIM3 on PB4 (A), PB5 (B), AF2
 *   right: TIM4 on PB6 (A), PB7 (B), AF2 */
#define PIN_ENC_L_A GPIOB, 4
#define PIN_ENC_L_B GPIOB, 5
#define PIN_ENC_R_A GPIOB, 6
#define PIN_ENC_R_B GPIOB, 7
/* 1 if a wheel's encoder counts down when the wheel rolls forward. */
#define ENC_L_REVERSED 0
#define ENC_R_REVERSED 0

#ifndef PINS_CONFIRMED
#warning "pins.h: the IR, motor and encoder pins are PROPOSED, not read from the schematic. Check them, then define PINS_CONFIRMED."
#endif

#endif /* PINS_H */
