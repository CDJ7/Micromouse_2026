# Real Mouse Code

Firmware for the physical micromouse (STM32 Black Pill). It reuses the maze solver from [`../MMS_Code`](../MMS_Code) **unchanged** and adds everything the real robot needs: reading the IR wall sensors, driving the motors, using the IMU, and starting up safely.

> [!NOTE]
> **Work in progress.** All of it is written. Everything above the register-level drivers is tested on a PC against a simulated mouse, and the STM32 drivers build and link for the Black Pill but have never run on a board. The numbers in [`include/config.h`](include/config.h) are placeholders until the mouse is built and measured, and the IR, motor and encoder pins are a proposal that has to be checked against the schematic.

---

## Table of Contents

- [How it fits together](#how-it-fits-together)
- [What is done](#what-is-done)
- [Building the firmware](#building-the-firmware)
- [Pins](#pins)
- [How the mouse drives](#how-the-mouse-drives)
- [Testing without hardware](#testing-without-hardware)
- [Solver settings for the real mouse](#solver-settings-for-the-real-mouse)
- [What is still needed](#what-is-still-needed)
- [Bring-up plan](#bring-up-plan)
- [Known risks and limitations](#known-risks-and-limitations)

---

## How it fits together

The solver never touches hardware. It only calls the functions in `MMS_Code/API.h`. In the simulator those calls go to mms; on the real mouse they go to `src/api_hw.c`, which sits on top of two small modules and a hardware abstraction layer (HAL):

```
MMS_Code/solver.c        the maze solver (same file as in the simulator)
        |  API.h
src/api_hw.c             wall sensing, moves, turns, clock
        |
src/ir_sensors.c         wall detection and distances from the IR sensors
src/motion.c             straights and 90-degree turns
        |  include/hal.h
hal_stm32/hal_stm32.c    Black Pill drivers (written, not yet run on a board)
test/fake_hal.c          a simulated mouse, used to test everything above on a PC
```

Because the layers only talk through `hal.h`, the same firmware code runs on the PC test and on the mouse. Only the bottom layer changes.

---

## What is done

| Part | File | Status |
| --- | --- | --- |
| Hardware interface (what the firmware needs from the electronics) | `include/hal.h` | Written |
| Robot numbers (cell size, speeds, gains, thresholds) | `include/config.h` | Written, **placeholders** until measured |
| IR sensing: ambient subtraction, wall decision, distance, corridor offset | `src/ir_sensors.c` | Written, tested on PC |
| Motion: speed profile, heading hold, corridor centering, turns, stall detection | `src/motion.c` | Written, tested on PC |
| API for the solver | `src/api_hw.c` | Written, tested on PC |
| Start-up: wait for button, calibrate in the start cell, run, stop | `src/main_hw.c` | Written, tested on PC |
| Pin list | `hal_stm32/pins.h` | Button, LED, IMU and serial pins are from the schematics. IR, motor and encoder pins are **proposed**, see [Pins](#pins) |
| STM32 drivers: clocks, 1 kHz tick, PWM, encoders, ADC, I2C and the BNO055, serial | `hal_stm32/hal_stm32.c` | Written, builds and links. **Never run on a board** |
| Driver arithmetic (yaw unwrapping, encoder widening, PWM scaling) | `hal_stm32/hal_logic.c` | Written, tested on PC |
| Build for the Black Pill | `Makefile`, `hal_stm32/stm32f411ce.ld`, `solver_flags.mk` | Written, builds `mouse.elf` and `mouse.bin` |
| Simulated mouse and tests | `test/` | Written |

---

## Building the firmware

The build uses plain `arm-none-eabi-gcc` and the CMSIS device headers; there is no vendor HAL library and no IDE project.

```
sudo apt install gcc-arm-none-eabi libnewlib-arm-none-eabi
git clone https://github.com/STMicroelectronics/cmsis_core         ~/cmsis/cmsis_core
git clone https://github.com/STMicroelectronics/cmsis_device_f4    ~/cmsis/cmsis_device_f4
cd Real_Mouse_Code
make          # builds build/mouse.elf and build/mouse.bin
make flash    # writes it with an ST-Link (needs st-flash)
```

The build prints a warning that the IR, motor and encoder pins are proposed. It goes away with `make PINS=confirmed` once they have been checked. The solver is compiled from `../MMS_Code/solver.c` with the settings in `solver_flags.mk`.

Size: about 18.7 KB of the 512 KB flash and 22.5 KB of the 128 KB RAM, including the solver and its map.

> [!WARNING]
> Flash with an **ST-Link on the SWD pins**. The Black Pill's built-in USB bootloader needs the USB pins PA11 and PA12, and the schematic uses those for the IMU.

---

## Pins

Known from the schematics:

| Signal | Pin |
| --- | --- |
| Start button SW2 (to ground, pressed = low) | PB12 |
| On-board LED | PC13 |
| IMU I2C clock / data | PB8 / PB9 |
| IMU interrupt / reset | PA11 / PA12 (the USB pins) |
| Serial debug output (header J5) | PA2 (TX), PA3 (RX) |

The IR, motor and encoder schematics are too low resolution to read, so those pins are **inferred, not read**. The microcontroller schematic connects exactly 22 pins. Eight are identified above, and the other 14 are exactly what the rest of the design needs: 3 IR receivers, 3 IR emitters, 4 motor inputs and 4 encoder inputs. The proposal gives each pin a job that its timer or ADC can really do on the STM32F411:

| Signal | Proposed pin | Peripheral |
| --- | --- | --- |
| IR receiver front / left / right | PA4 / PA5 / PA1 | ADC1 channels 4 / 5 / 1 |
| IR emitter front / left / right | PA6 / PA7 / PA10 | GPIO output |
| Left motor inputs 1 / 2 | PA8 / PA9 | TIM1 channels 1 / 2 (PWM) |
| Right motor inputs 1 / 2 | PA15 / PB3 | TIM2 channels 1 / 2 (PWM) |
| Left encoder A / B | PB4 / PB5 | TIM3 encoder mode |
| Right encoder A / B | PB6 / PB7 | TIM4 encoder mode |

**Compare this table with the KiCad schematic and edit the lines that differ in `hal_stm32/pins.h` (one line per pin).** Anything wired differently from this will not work. Two more assumptions to check: the motor driver takes two inputs per motor (forward = PWM on input 1, reverse = PWM on input 2), and the IR receivers are read by the ADC rather than a comparator.

---

## How the mouse drives

- **Straights.** A speed profile accelerates, cruises and brakes so the mouse stops exactly one cell later. Wheel power is feed-forward plus a correction from the measured speed. One cell uses a slow search speed; several cells in a row (the final run) use a faster run speed.
- **Staying straight.** The IMU holds the heading. The side IR sensors measure how far the mouse is from the middle of the corridor and nudge the heading back toward it.
- **Correcting the IMU with the walls.** The heading reference slowly drifts with every turn the IMU makes. Whenever the mouse sees a wall alongside it, the offset from the middle is used to move the heading reference back to where "straight" really is. Without this, a small gyro error would add up over a whole maze. This was the difference between crashing and not crashing in the PC tests.
- **Stopping accurately.** If there is a wall ahead at the end of a cell, the front sensor gives an absolute distance to it, and the mouse corrects its stopping position. This stops wheel slip from adding up.
- **Turns.** The mouse aims for an absolute heading (a multiple of 90 degrees from the start) rather than "90 degrees more", so one bad turn is not carried into the next.
- **Wall detection.** Each reading is the sensor with its LED on minus the sensor with it off, which cancels room light. A wall is declared above half of the calibrated "wall" level.
- **Calibration.** At start-up the mouse reads its left and right walls in the start cell, turns around to read the back wall with the front sensor, and turns back. This assumes the start cell has walls on its left, right and back, as in a competition maze.
- **Fail-safes.** A sensor that was never calibrated reports a wall (so the mouse refuses to drive on it). A move that makes no progress for 0.4 s, or takes three times too long, stops the motors and ends the run. After any fault the mouse stays stopped.

---

## Testing without hardware

`test/fake_hal.c` is a small physics model of a mouse: wheels with lag and a power dead zone, encoder and gyro errors, a body that crashes if it gets within 35 mm of a wall, and IR sensors that cast rays into the maze and read a signal that falls with distance (not exactly the inverse square the code assumes), plus drifting room light and noise. The real firmware and the real solver run against it in the generated 10×10 mazes.

```
./Real_Mouse_Code/test/run_hw_tests.sh
```

For each maze it checks that the body never touches a wall, the mouse ends in the center, the map built from IR readings matches the real walls, the final route is the cheapest in the whole maze, and exploring finished inside the time budget.

| Simulated mouse | Mazes | Result |
| --- | --- | --- |
| Normal | 6 of 6 (10×10) | All checks pass, mouse stays within 5 mm of the corridor center |
| Worse: noisier sensors, 1.2% encoder error, 0.6% gyro error, different motors | 6 of 6 (10×10) | All checks pass, within 14 mm of the center |

A whole maze takes about 3 minutes of simulated time for exploring, returning and the final run, well inside a 10-minute limit. `./test/run_hw_tests.sh --timing` measures how long one cell and one turn take (832 ms and 319 ms in the model).

This proves the logic, not the electronics. A model is not a mouse: real IR sensors, motors and IMUs will behave differently, and the gains will need tuning.

---

## Solver settings for the real mouse

The solver is configured with compile flags (see the settings at the top of `MMS_Code/solver.c`). The PC test uses these; they are the starting point for the real build:

| Flag | Value in the test | Why |
| --- | --- | --- |
| `MAX_MAZE_SIZE` | `10` | The real maze is 10×10, which also cuts static RAM from about 50 KB to 20 KB |
| `COST_FORWARD`, `COST_TURN` | `830`, `320` | Milliseconds per cell and per turn, measured with `--timing`. Re-measure on the real mouse |
| `EXPLORE_BUDGET` | `600000` | The 10-minute limit, in milliseconds |
| `EXPLORE_BUDGET_USES_CLOCK` | `1` | Measure the budget with the real clock (`API_timeMs`) |
| `TIME_SAFETY_MARGIN` | `5000` | Keep 5 seconds in hand for things the model does not know about |
| `WALL_READS` | `3` | Decide each wall by majority of three readings |
| `VERIFY_FRONT_BEFORE_MOVE`, `VERIFY_UNDRIVEN_ON_RUNS` | `1`, `1` | Re-check the front sensor before stepping into a side that has not been driven, also on the return and final run |
| `DEBUG_SHOW_*`, `DIST_DISPLAY_FLOOD` | `0` | The simulator display does not exist on the mouse |
| `LOG(...)` | `hal_log(...)` | Send the solver's log to a serial port instead of stderr |

---

## What is still needed

**Decisions**
- Toolchain: the build uses plain `arm-none-eabi-gcc`. It can be wrapped in PlatformIO or STM32CubeIDE if the team prefers.
- Whether the mouse has **encoders**. None are drawn in the schematics (the 14-pin count above suggests four encoder pins exist), and without encoders the mouse cannot measure distance.
- Exact STM32 part, motor driver part, motors, encoders and IR emitter/receiver parts.

**Measurements for `include/config.h`** (every value marked TBD): cell pitch of the real maze, encoder ticks per mm, wheel speed at full power, controller gains, motor dead zone, IR settle time, sensor-to-wall distances when centered.

**Checking the drivers on a board.** `hal_stm32.c` was written from the STM32F411 and BNO055 documentation and compile-checked only. Expect to debug it during bring-up: the I2C sequences, the BNO055 heading sign and units, the PWM and encoder pin modes, the ADC sample time.

**Pins.** Confirm the proposed IR, motor and encoder pins against the schematic (see [Pins](#pins)).

---

## Bring-up plan

Do these in order on the real mouse, testing each before the next.

1. Build and flash with an ST-Link. Serial output (`hal_log`) on header J5, the LED and the button.
2. Motors, wheels off the ground: both directions, power against speed, dead zone.
3. Encoders: ticks per mm, direction.
4. BNO055: heading, sign, unwrapping, update rate.
5. IR sensors: raw readings against distance, settle time, ambient light.
6. Measure and fill in `config.h`; tune the speed and turn gains.
7. Straight line along a wall; check the corridor centering.
8. Turns, then calibration in a real start cell.
9. One cell, then the full solver in a real maze with the debug log on.

---

## Known risks and limitations

- **The IMU pins clash with USB.** The schematic puts the BNO055's INT and RST on PA11 and PA12, which are the Black Pill's USB pins, so USB cannot be used on this board. Moving them before the PCB is made would avoid that.
- **No encoders in the schematics.** See above.
- **The IMU update rate.** The BNO055 produces new fusion data at up to 100 Hz, while the control loop runs at 1 kHz. The driver reads it every 10 ms and extrapolates between readings (tested on PC), but how well that works for the turn controller is untested.
- **The drivers have never run.** The BNO055 heading sign and units, the I2C address (0x28 assumed), the LED polarity and the motor driver type are all assumptions from documentation.
- **The control loop has no timing margin check.** It assumes each loop finishes within the 1 ms tick; a slow I2C read or log line would stretch it.
- **The planner prices every cell the same.** On the real mouse, multi-cell straights run faster than one cell, so the planner slightly overestimates their cost. It still finds a good route, but not exactly the fastest in time.
- **The mouse stops in every cell while exploring.** That is simple and safe, but slow. Continuous exploration would be a later improvement.
- **Assumptions about the start.** The mouse is placed in the center of the start cell facing the exit, and the start cell has walls on its left, right and back.
- **Side sensors point straight sideways in the model.** Real mice often angle them forward, which changes the readings.
- **No reset button handling.** `API_wasReset` always returns false for now.
