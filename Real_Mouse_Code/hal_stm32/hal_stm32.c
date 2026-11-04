/*
 * hal_stm32.c - the HAL for the real mouse: STM32F411 (Black Pill), register
 * level, using only the CMSIS device header (no vendor HAL library).
 *
 * STATUS: written and compile-checked, but never run on hardware. Treat every
 * part as untested until it has been brought up on the real board in the order
 * given in Real_Mouse_Code/README.md. The pins marked PROPOSED in pins.h have to
 * be checked against the schematic first.
 *
 * What it uses:
 *   clocks   25 MHz crystal -> PLL -> 100 MHz core, APB1 50 MHz, APB2 100 MHz
 *   SysTick  1 kHz: millis, the control tick, and the encoder counters
 *   TIM1/TIM2  PWM for the motor driver (20 kHz)
 *   TIM3/TIM4  encoder mode for the wheel encoders
 *   ADC1     the three IR receivers (polled)
 *   I2C1     the BNO055 IMU (polled, 400 kHz)
 *   USART2   debug text on header J5
 */
#include <stdarg.h>
#include <stdio.h>

#include "../include/hal.h"
#include "hal_logic.h"
#include "pins.h"

/* ---- BNO055 registers (see its datasheet) ---- */
#define BNO_CHIP_ID 0x00
#define BNO_EUL_HEADING_LSB 0x1A /* heading, 16 LSB per degree, clockwise */
#define BNO_UNIT_SEL 0x3B
#define BNO_OPR_MODE 0x3D
#define BNO_PWR_MODE 0x3E
#define BNO_MODE_CONFIG 0x00
#define BNO_MODE_IMUPLUS 0x08 /* gyro + accelerometer fusion, no magnetometer */
#define BNO_CHIP_ID_VALUE 0xA0

#define PWM_PERIOD 5000u        /* 100 MHz / 5000 = 20 kHz */
#define IMU_PERIOD_MS 10u       /* the BNO055 makes a new fusion value about this often */
#define IMU_FAILURES_LIMIT 50   /* this many failed reads in a row: stop */
#define I2C_TIMEOUT_MS 5u

/* ====================================================================== */
/* State                                                                   */
/* ====================================================================== */

static volatile uint32_t msCount;
static volatile uint32_t tickFlag;
static volatile int32_t encLeft, encRight;
static uint16_t encLeftPrev, encRightPrev;
static YawTracker yaw;
static uint32_t lastImuMs;
static int imuFailures;

/* ====================================================================== */
/* Small helpers                                                           */
/* ====================================================================== */

typedef enum { GPIO_IN = 0, GPIO_OUT = 1, GPIO_AF = 2, GPIO_ANALOG = 3 } GpioMode;
typedef enum { PULL_NONE = 0, PULL_UP = 1, PULL_DOWN = 2 } GpioPull;

static void gpioConfig(GPIO_TypeDef *port, int n, GpioMode mode, int af, GpioPull pull, bool openDrain) {
    port->MODER = (port->MODER & ~(3u << (2 * n))) | ((uint32_t)mode << (2 * n));
    port->PUPDR = (port->PUPDR & ~(3u << (2 * n))) | ((uint32_t)pull << (2 * n));
    port->OSPEEDR |= 3u << (2 * n); /* fast edges for PWM */
    if (openDrain) port->OTYPER |= 1u << n;
    else port->OTYPER &= ~(1u << n);
    if (mode == GPIO_AF) {
        if (n < 8) port->AFR[0] = (port->AFR[0] & ~(0xFu << (4 * n))) | ((uint32_t)af << (4 * n));
        else port->AFR[1] = (port->AFR[1] & ~(0xFu << (4 * (n - 8)))) | ((uint32_t)af << (4 * (n - 8)));
    }
}

static void gpioWrite(GPIO_TypeDef *port, int n, bool high) {
    port->BSRR = high ? (1u << n) : (1u << (n + 16));
}

static bool gpioRead(GPIO_TypeDef *port, int n) {
    return (port->IDR & (1u << n)) != 0;
}

/* The macros in pins.h are written "GPIOx, n"; these wrappers let us pass one. */
#define CFG(pin, mode, af, pull, od) gpioConfig(pin, mode, af, pull, od)
#define WRITE(pin, high) gpioWrite(pin, high)
#define READ(pin) gpioRead(pin)

/* Stop everything and blink the LED so it is obvious something is wrong. */
static void fatal(const char *why) {
    volatile uint32_t i;
    hal_motor_set(0, 0);
    hal_log("[hal] FATAL: %s\n", why);
    for (;;) {
        WRITE(PIN_LED, false); /* on */
        for (i = 0; i < 1000000u; i++) {}
        WRITE(PIN_LED, true);
        for (i = 0; i < 1000000u; i++) {}
    }
}

/* ====================================================================== */
/* Clocks, SysTick, delay                                                  */
/* ====================================================================== */

static bool clockInit(void) {
    uint32_t i;
    RCC->CR |= RCC_CR_HSEON;
    for (i = 0; i < 500000u && !(RCC->CR & RCC_CR_HSERDY); i++) {}
    if (!(RCC->CR & RCC_CR_HSERDY)) return false; /* no crystal: stay on the 16 MHz internal clock */

    FLASH->ACR = FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_PRFTEN | FLASH_ACR_LATENCY_3WS;
    /* 25 MHz / 25 = 1 MHz, x 200 = 200 MHz, / 2 = 100 MHz core. */
    RCC->PLLCFGR = (25u << RCC_PLLCFGR_PLLM_Pos) | (200u << RCC_PLLCFGR_PLLN_Pos) |
                   (0u << RCC_PLLCFGR_PLLP_Pos) | RCC_PLLCFGR_PLLSRC_HSE |
                   (4u << RCC_PLLCFGR_PLLQ_Pos);
    RCC->CFGR = RCC_CFGR_PPRE1_DIV2; /* APB1 50 MHz (its limit), APB2 100 MHz */
    RCC->CR |= RCC_CR_PLLON;
    for (i = 0; i < 500000u && !(RCC->CR & RCC_CR_PLLRDY); i++) {}
    if (!(RCC->CR & RCC_CR_PLLRDY)) return false;
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    for (i = 0; i < 500000u && (RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL; i++) {}
    if ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) return false;
    SystemCoreClock = 100000000u;
    return true;
}

/* Runs once per millisecond. Keeping the encoder totals up to date here means
 * the 16-bit timers can never wrap unnoticed, whatever the main loop is doing. */
void SysTick_Handler(void) {
    uint16_t l = (uint16_t)TIM3->CNT, r = (uint16_t)TIM4->CNT;
    int32_t dl = (int32_t)counter_extend(0, encLeftPrev, l);
    int32_t dr = (int32_t)counter_extend(0, encRightPrev, r);
    encLeftPrev = l;
    encRightPrev = r;
    encLeft += ENC_L_REVERSED ? -dl : dl;
    encRight += ENC_R_REVERSED ? -dr : dr;
    msCount++;
    tickFlag = 1;
}

static void delayInit(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

uint32_t hal_millis(void) { return msCount; }

void hal_wait_tick(void) {
    while (!tickFlag) __WFI();
    tickFlag = 0;
}

void hal_delay_us(uint32_t us) {
    uint32_t start = DWT->CYCCNT, ticks = us * (SystemCoreClock / 1000000u);
    while ((DWT->CYCCNT - start) < ticks) {}
}

/* ====================================================================== */
/* Debug text on USART2 (header J5)                                        */
/* ====================================================================== */

static void uartInit(void) {
    CFG(PIN_UART_TX, GPIO_AF, 7, PULL_NONE, false);
    CFG(PIN_UART_RX, GPIO_AF, 7, PULL_UP, false);
    /* 50 MHz / (16 x 115200) = 27.1267: mantissa 27, fraction 2/16. */
    USART2->BRR = (27u << 4) | 2u;
    USART2->CR1 = USART_CR1_TE | USART_CR1_UE;
}

static void uartPut(char c) {
    while (!(USART2->SR & USART_SR_TXE)) {}
    USART2->DR = (uint8_t)c;
}

void hal_log(const char *fmt, ...) {
    char buf[160];
    va_list ap;
    int i;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof buf, fmt, ap);
    va_end(ap);
    for (i = 0; buf[i]; i++) {
        if (buf[i] == '\n') uartPut('\r');
        uartPut(buf[i]);
    }
}

/* ====================================================================== */
/* I2C1 master (polled) and the BNO055                                     */
/* ====================================================================== */

static bool i2cWait(volatile uint32_t *reg, uint32_t mask, bool wantSet) {
    uint32_t t0 = msCount;
    while (((*reg & mask) != 0) != wantSet) {
        if (msCount - t0 > I2C_TIMEOUT_MS) return false;
    }
    return true;
}

static void i2cInit(void) {
    I2C1->CR1 = I2C_CR1_SWRST; /* a stuck bus is cleared by a software reset */
    I2C1->CR1 = 0;
    I2C1->CR2 = 50;            /* APB1 clock in MHz */
    I2C1->CCR = I2C_CCR_FS | 42u; /* 400 kHz fast mode: 50 MHz / (3 x 42) */
    I2C1->TRISE = 16;
    I2C1->CR1 = I2C_CR1_PE;
}

static bool i2cStart(void) {
    I2C1->CR1 |= I2C_CR1_START;
    return i2cWait(&I2C1->SR1, I2C_SR1_SB, true);
}

static bool i2cSendAddress(uint8_t address8) {
    uint32_t t0 = msCount;
    I2C1->DR = address8;
    while (!(I2C1->SR1 & I2C_SR1_ADDR)) {
        if (I2C1->SR1 & I2C_SR1_AF) {
            I2C1->SR1 &= ~I2C_SR1_AF;
            I2C1->CR1 |= I2C_CR1_STOP;
            return false; /* nobody answered */
        }
        if (msCount - t0 > I2C_TIMEOUT_MS) return false;
    }
    return true;
}

static void i2cClearAddr(void) {
    (void)I2C1->SR1;
    (void)I2C1->SR2;
}

static bool i2cWriteReg(uint8_t reg, uint8_t value) {
    if (!i2cStart()) return false;
    if (!i2cSendAddress((uint8_t)(IMU_I2C_ADDRESS << 1))) return false;
    i2cClearAddr();
    if (!i2cWait(&I2C1->SR1, I2C_SR1_TXE, true)) return false;
    I2C1->DR = reg;
    if (!i2cWait(&I2C1->SR1, I2C_SR1_TXE, true)) return false;
    I2C1->DR = value;
    if (!i2cWait(&I2C1->SR1, I2C_SR1_BTF, true)) return false;
    I2C1->CR1 |= I2C_CR1_STOP;
    return true;
}

/* Read 1 or 2 bytes starting at `reg`. The receive sequences are the ones in the
 * STM32F4 reference manual for 1- and 2-byte reads (the chip needs different
 * handling of ACK and STOP for each). */
static bool i2cReadRegs(uint8_t reg, uint8_t *out, int n) {
    if (n < 1 || n > 2) return false;
    /* Write the register address. */
    if (!i2cStart()) return false;
    if (!i2cSendAddress((uint8_t)(IMU_I2C_ADDRESS << 1))) return false;
    i2cClearAddr();
    if (!i2cWait(&I2C1->SR1, I2C_SR1_TXE, true)) return false;
    I2C1->DR = reg;
    if (!i2cWait(&I2C1->SR1, I2C_SR1_BTF, true)) return false;

    /* Repeated start, then read. */
    I2C1->CR1 &= ~I2C_CR1_POS;
    I2C1->CR1 |= I2C_CR1_ACK;
    if (!i2cStart()) return false;
    if (!i2cSendAddress((uint8_t)((IMU_I2C_ADDRESS << 1) | 1u))) return false;
    if (n == 1) {
        I2C1->CR1 &= ~I2C_CR1_ACK;
        i2cClearAddr();
        I2C1->CR1 |= I2C_CR1_STOP;
        if (!i2cWait(&I2C1->SR1, I2C_SR1_RXNE, true)) return false;
        out[0] = (uint8_t)I2C1->DR;
    } else {
        I2C1->CR1 &= ~I2C_CR1_ACK;
        I2C1->CR1 |= I2C_CR1_POS;
        i2cClearAddr();
        if (!i2cWait(&I2C1->SR1, I2C_SR1_BTF, true)) return false;
        I2C1->CR1 |= I2C_CR1_STOP;
        out[0] = (uint8_t)I2C1->DR;
        out[1] = (uint8_t)I2C1->DR;
    }
    I2C1->CR1 &= ~I2C_CR1_POS;
    return true;
}

static void delayMs(uint32_t ms) {
    uint32_t t0 = msCount;
    while (msCount - t0 < ms) __WFI();
}

static bool imuInit(void) {
    uint8_t id = 0;
    /* Reset the chip with its RST pin and give it time to boot (datasheet: 650 ms). */
    WRITE(PIN_IMU_RST, false);
    delayMs(10);
    WRITE(PIN_IMU_RST, true);
    delayMs(700);
    if (!i2cReadRegs(BNO_CHIP_ID, &id, 1) || id != BNO_CHIP_ID_VALUE) {
        hal_log("[hal] IMU: no BNO055 answered (chip id 0x%02X, expected 0xA0)\n", id);
        return false;
    }
    if (!i2cWriteReg(BNO_OPR_MODE, BNO_MODE_CONFIG)) return false;
    delayMs(25);
    if (!i2cWriteReg(BNO_PWR_MODE, 0x00)) return false; /* normal power */
    if (!i2cWriteReg(BNO_UNIT_SEL, 0x00)) return false; /* degrees, deg/s */
    if (!i2cWriteReg(BNO_OPR_MODE, BNO_MODE_IMUPLUS)) return false;
    delayMs(25);
    return true;
}

float hal_yaw_deg(void) {
    uint32_t now = msCount;
    if (now - lastImuMs >= IMU_PERIOD_MS) {
        uint8_t b[2];
        lastImuMs = now;
        if (i2cReadRegs(BNO_EUL_HEADING_LSB, b, 2)) {
            int16_t raw = (int16_t)(((uint16_t)b[1] << 8) | b[0]);
            yaw_tracker_update(&yaw, (float)raw / 16.0f, now);
            imuFailures = 0;
        } else if (++imuFailures >= IMU_FAILURES_LIMIT) {
            fatal("the IMU stopped answering");
        }
    }
    return yaw_tracker_get(&yaw, now);
}

/* ====================================================================== */
/* Motors (TIM1 left, TIM2 right) and encoders (TIM3 left, TIM4 right)     */
/* ====================================================================== */

static void pwmInit(void) {
    CFG(PIN_MOTOR_L_IN1, GPIO_AF, 1, PULL_NONE, false);
    CFG(PIN_MOTOR_L_IN2, GPIO_AF, 1, PULL_NONE, false);
    CFG(PIN_MOTOR_R_IN1, GPIO_AF, 1, PULL_NONE, false);
    CFG(PIN_MOTOR_R_IN2, GPIO_AF, 1, PULL_NONE, false);

    TIM1->PSC = 0;
    TIM1->ARR = PWM_PERIOD - 1u;
    TIM1->CCMR1 = (6u << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE | (6u << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;
    TIM1->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E;
    TIM1->CCR1 = TIM1->CCR2 = 0;
    TIM1->BDTR = TIM_BDTR_MOE; /* advanced timer: outputs stay off until this is set */
    TIM1->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;
    TIM1->EGR = TIM_EGR_UG;

    TIM2->PSC = 0;
    TIM2->ARR = PWM_PERIOD - 1u;
    TIM2->CCMR1 = (6u << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE | (6u << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;
    TIM2->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E;
    TIM2->CCR1 = TIM2->CCR2 = 0;
    TIM2->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;
    TIM2->EGR = TIM_EGR_UG;
}

/* permille is -1000..+1000, positive = forward. */
void hal_motor_set(int left_permille, int right_permille) {
    uint32_t l, r;
    if (MOTOR_L_REVERSED) left_permille = -left_permille;
    if (MOTOR_R_REVERSED) right_permille = -right_permille;
    /* Forward drives input 1, reverse drives input 2; the other input is low. */
    l = permille_to_compare(left_permille < 0 ? -left_permille : left_permille, PWM_PERIOD);
    r = permille_to_compare(right_permille < 0 ? -right_permille : right_permille, PWM_PERIOD);
    TIM1->CCR1 = left_permille > 0 ? l : 0;
    TIM1->CCR2 = left_permille < 0 ? l : 0;
    TIM2->CCR1 = right_permille > 0 ? r : 0;
    TIM2->CCR2 = right_permille < 0 ? r : 0;
}

static void encoderTimerInit(TIM_TypeDef *t) {
    t->PSC = 0;
    t->ARR = 0xFFFFu;
    /* Channel 1 and 2 are inputs (TI1, TI2) with a small noise filter. */
    t->CCMR1 = (1u << TIM_CCMR1_CC1S_Pos) | (1u << TIM_CCMR1_CC2S_Pos) | (3u << TIM_CCMR1_IC1F_Pos) |
               (3u << TIM_CCMR1_IC2F_Pos);
    t->CCER = 0;
    t->SMCR = 3u << TIM_SMCR_SMS_Pos; /* encoder mode 3: count on both edges of both inputs */
    t->CNT = 0;
    t->CR1 = TIM_CR1_CEN;
}

static void encodersInit(void) {
    CFG(PIN_ENC_L_A, GPIO_AF, 2, PULL_NONE, false);
    CFG(PIN_ENC_L_B, GPIO_AF, 2, PULL_NONE, false);
    CFG(PIN_ENC_R_A, GPIO_AF, 2, PULL_NONE, false);
    CFG(PIN_ENC_R_B, GPIO_AF, 2, PULL_NONE, false);
    encoderTimerInit(TIM3);
    encoderTimerInit(TIM4);
    encLeftPrev = (uint16_t)TIM3->CNT;
    encRightPrev = (uint16_t)TIM4->CNT;
}

void hal_encoders(long *left_ticks, long *right_ticks) {
    /* 32-bit loads are atomic on this core, so no need to stop the interrupt. */
    *left_ticks = (long)encLeft;
    *right_ticks = (long)encRight;
}

/* ====================================================================== */
/* IR sensors: emitters are GPIO, receivers are read by ADC1               */
/* ====================================================================== */

static void irInit(void) {
    CFG(PIN_IR_TX_FRONT, GPIO_OUT, 0, PULL_NONE, false);
    CFG(PIN_IR_TX_LEFT, GPIO_OUT, 0, PULL_NONE, false);
    CFG(PIN_IR_TX_RIGHT, GPIO_OUT, 0, PULL_NONE, false);
    WRITE(PIN_IR_TX_FRONT, false);
    WRITE(PIN_IR_TX_LEFT, false);
    WRITE(PIN_IR_TX_RIGHT, false);
    CFG(PIN_IR_RX_FRONT, GPIO_ANALOG, 0, PULL_NONE, false);
    CFG(PIN_IR_RX_LEFT, GPIO_ANALOG, 0, PULL_NONE, false);
    CFG(PIN_IR_RX_RIGHT, GPIO_ANALOG, 0, PULL_NONE, false);

    ADC->CCR = (ADC->CCR & ~ADC_CCR_ADCPRE) | ADC_CCR_ADCPRE_0; /* ADC clock = 100 MHz / 4 */
    /* 144 cycles per sample (about 6 us): the sensors have a high source impedance. */
    ADC1->SMPR2 = (6u << (3 * IR_RX_FRONT_CHANNEL)) | (6u << (3 * IR_RX_LEFT_CHANNEL)) |
                  (6u << (3 * IR_RX_RIGHT_CHANNEL)); /* channels 0..9 live in SMPR2 */
    ADC1->SQR1 = 0; /* one conversion per sequence */
    ADC1->CR2 = ADC_CR2_ADON;
    hal_delay_us(10); /* ADC wake-up time */
}

void hal_ir_emitter(IrSensor s, bool on) {
    switch (s) {
    case IR_FRONT: WRITE(PIN_IR_TX_FRONT, on); break;
    case IR_LEFT: WRITE(PIN_IR_TX_LEFT, on); break;
    default: WRITE(PIN_IR_TX_RIGHT, on); break;
    }
}

int hal_ir_adc(IrSensor s) {
    uint32_t channel = (s == IR_FRONT) ? IR_RX_FRONT_CHANNEL
                       : (s == IR_LEFT) ? IR_RX_LEFT_CHANNEL
                                        : IR_RX_RIGHT_CHANNEL;
    uint32_t t0 = msCount;
    ADC1->SQR3 = channel;
    ADC1->SR = 0;
    ADC1->CR2 |= ADC_CR2_SWSTART;
    while (!(ADC1->SR & ADC_SR_EOC)) {
        if (msCount - t0 > 2u) return 0; /* conversion never finished */
    }
    return (int)(ADC1->DR & 0xFFFu);
}

/* ====================================================================== */
/* User interface, start-up and shut-down                                  */
/* ====================================================================== */

bool hal_button_pressed(void) { return !READ(PIN_BUTTON); }

void hal_led(bool on) { WRITE(PIN_LED, !on); /* active low */ }

void hal_init(void) {
    bool clockOk;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM3EN | RCC_APB1ENR_TIM4EN |
                    RCC_APB1ENR_USART2EN | RCC_APB1ENR_I2C1EN;
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN | RCC_APB2ENR_ADC1EN;

    clockOk = clockInit();
    delayInit();
    CFG(PIN_LED, GPIO_OUT, 0, PULL_NONE, false);
    hal_led(false);
    CFG(PIN_BUTTON, GPIO_IN, 0, PULL_UP, false);

    encodersInit();
    SysTick_Config(SystemCoreClock / 1000u); /* from here on SysTick_Handler runs */
    uartInit();
    hal_log("\n[hal] start, core clock %lu Hz\n", (unsigned long)SystemCoreClock);
    if (!clockOk) fatal("the 25 MHz crystal or the PLL did not start");

    pwmInit();
    hal_motor_set(0, 0);
    irInit();

    CFG(PIN_IMU_RST, GPIO_OUT, 0, PULL_NONE, false);
    WRITE(PIN_IMU_RST, true);
    CFG(PIN_IMU_SCL, GPIO_AF, 4, PULL_UP, true);
    CFG(PIN_IMU_SDA, GPIO_AF, 4, PULL_UP, true);
    i2cInit();
    yaw_tracker_reset(&yaw);
    if (!imuInit()) fatal("the IMU did not start");
    lastImuMs = msCount;
    hal_log("[hal] ready\n");
}

void hal_halt(void) {
    volatile uint32_t i;
    hal_motor_set(0, 0);
    for (;;) { /* slow blink: finished */
        hal_led(true);
        for (i = 0; i < 4000000u; i++) {}
        hal_led(false);
        for (i = 0; i < 4000000u; i++) {}
    }
}
