# Micromouse 2026

A team Micromouse project: a maze-solving robot whose solver is written in C. The solver is developed and tested in the [Micromouse Simulator (mms)](https://github.com/mackorone/mms) and runs on a physical mouse in a 10×10 maze. The simulator uses a 16×16 maze.

> [!NOTE]
> **Work in progress.** The simulation solver is complete and heavily tested. The firmware for the real mouse is written and tested on a PC against a simulated mouse, but the low-level hardware drivers are still to come and nothing has run on a physical mouse yet. The PCB is still being designed.

| Part of the project | Status |
| --- | --- |
| Maze solver for mms | Complete. Tested on 532 mazes, always finds the cheapest route in the maze |
| Offline tests and GitHub Actions | Complete |
| Real-mouse firmware (sensors, motion, start-up) | Written and tested on a PC. Not yet run on hardware |
| STM32 hardware drivers | Not started (skeleton only) |
| Schematics and PCB | Schematics below. PCB layout in progress |

---

## Table of Contents

- [Goal](#goal)
- [How the Solver Works](#how-the-solver-works)
- [Configuration](#configuration)
- [Testing](#testing)
- [Project Structure](#project-structure)
- [Simulator Setup (Windows)](#simulator-setup-windows)
  - [Install the C compiler (MinGW)](#install-the-c-compiler-mingw)
  - [Install the simulator](#install-the-simulator)
  - [Connect this repo to the simulator](#connect-this-repo-to-the-simulator)
  - [Tips](#tips)
- [Real Mouse Firmware](#real-mouse-firmware)
- [Hardware](#hardware)
- [Circuit Diagrams](#circuit-diagrams)
- [KiCad PCB Design](#kicad-pcb-design)
- [Team](#team)
- [Acknowledgements](#acknowledgements)

---

## Goal

The aim is the best possible mms score, not just reaching the center. The official score (lower is better) is:

```
score = best run turns + best run effective distance + 0.1 × (total turns + total effective distance)
```

"Effective distance" equals normal distance when the mouse moves one cell at a time. See the [Scorekeeping section of the mms README](https://github.com/mackorone/mms/blob/main/README.md#scorekeeping) for the full definition.

This has two implications:

1. **Turns cost as much as forward moves.** The best path is the one with the fewest actions, not just the fewest cells.
2. **Exploration counts at a tenth of the final run.** Exploring is only worth it when it shortens the final run.

The real competition adds a time limit (10 minutes of exploring), so the solver also has to decide when to stop and get home.

---

## How the Solver Works

Instead of a classic flood fill, the solver runs a shortest-path search (Dijkstra) over **position and heading**, so the cost of turns is priced in. Walls are tracked as unknown, open or wall for every side of every cell. The run has three phases:

| Phase | What it does |
| --- | --- |
| **Explore** | Plans the cheapest route to the center treating unknown walls as open, then drives out to check the unknown walls on that route and re-plans. When the route has no unknown walls left it is **proven** to be the best one in the whole maze (the plan was a lower bound on every possible route). With the budget that is left it can keep mapping the rest of the maze. Every step keeps enough budget in reserve to get home. |
| **Return** | Drives back to the start over sides it knows are open, arriving facing the way that gives the cheapest final run. |
| **Final run** | Runs the best known route from the start to the center, merging consecutive straights into one `moveForward(N)` call because mms scores long straights more cheaply. |

### Built for real sensors

A simulator never gets a wall wrong; a real mouse does. The solver is written so that one bad reading cannot send the mouse into a wall:

- each wall can be read several times and decided by vote;
- the front sensor is read once more before stepping into a side the mouse has not driven through yet, and optionally also on the return and final run, with re-planning if a wall turns up;
- a side the mouse has driven through is trusted as open from then on;
- a wall that is later read as open is kept (a phantom wall only costs time; a missed wall costs a crash).

### Real time limit, real costs

The planner does not assume a turn costs the same as a cell. `COST_FORWARD` and `COST_TURN` set what each costs: 1 and 1 in the simulator (how mms scores a run), and measured milliseconds on the real mouse. The exploration budget is counted in the same unit, or read from a real clock.

### Debug display in mms

The solver draws what it is thinking in the simulator:

| Display | Meaning |
| --- | --- |
| Number in each cell | Distance to the center: the live flood-fill through the known maze (unknown walls count as open), or plain Manhattan distance with a setting |
| Walls | Drawn the moment they are discovered |
| Dark green / yellow cells | Start cell / goal cells |
| Dark blue cells | Cells the mouse has visited |
| Cyan cells | The route currently planned |
| Orange cell | The current exploration target |
| Green cells | The final route, shown just before the final run |

Only cells whose display changed are updated, which keeps the simulator fast. Every re-plan (target and reason) and every phase change is logged to the **Run Output** tab, with a summary at the end.

---

## Configuration

The solver is tuned with `#define`s at the top of `MMS_Code/solver.c`, all of which can also be set on the compiler command line. The most important ones:

| Setting | Description |
| --- | --- |
| **Maze size** | 16×16 for the simulator, 10×10 for the real maze. Read from the API at startup (arrays are sized for up to `MAX_MAZE_SIZE`, 16 by default). The goal is the center, so it moves with the size. |
| **Exploration budget** (`EXPLORE_BUDGET`) | The most cost the mouse may spend exploring and driving home. It stands in for the 10-minute limit on the real maze, and the mouse always keeps enough in reserve to get back to the start. |
| **Action costs** (`COST_FORWARD`, `COST_TURN`) | What a cell and a 90° turn cost. 1 and 1 in the simulator. On the real mouse, set them to the measured times so the planner minimizes real run time. The budget uses the same unit. |
| **Clock mode** (`EXPLORE_BUDGET_USES_CLOCK`) | Measure the budget with a real clock (`API_timeMs`) instead of adding up action costs. |
| **Map the whole maze** (`EXPLORE_MAP_WHOLE_MAZE`) | On: after the best route is proven, keep mapping with the time left. Off: stop as soon as it is proven, which scores better in mms. |
| **Sensor safety** (`WALL_READS`, `VERIFY_FRONT_BEFORE_MOVE`, `VERIFY_UNDRIVEN_ON_RUNS`) | Vote over several reads, and re-check the front sensor before stepping into a side that has not been driven. Meant for noisy real sensors. |
| **Debug display** (`DEBUG_SHOW_*`, `DIST_DISPLAY_FLOOD`) | Switch the simulator display layers (distances, walls, colors, routes) on or off. |
| **Logging** (`DEBUG_LOG`, `LOG`) | Log to stderr, or redirect `LOG` (for example to a serial port on the mouse). |

The settings used for the real mouse are listed in [`Real_Mouse_Code/README.md`](Real_Mouse_Code/README.md#solver-settings-for-the-real-mouse).

---

## Testing

The solver can be tested without the simulator window, using a fake API that loads a maze file. `tests/run_tests.sh` builds the code and runs:

| Test | What it shows |
| --- | --- |
| Every usable maze in the [mazefiles](https://github.com/micromouseonline/mazefiles) collection (532 mazes up to 16×16) plus generated 8×8 and 10×10 mazes | The mouse never drives through a wall, reaches the center, builds a correct map, and its final route costs exactly the same as the cheapest route found by an independent search of the true maze |
| Weighted costs (a turn costing 3 cells) | The planner prices turns correctly for any cost ratio |
| Clock budget | Exploration stops in time when the budget is measured with a clock |
| Tiny budget | The mouse still gets home and runs a valid route |
| Route-only mode | Exploration can stop as soon as the best route is proven, for example cost 272 instead of 836 on `AAMC15Maze.txt` |
| Noisy sensors (2% of readings flipped) | With voting and front checks every run is crash-free; without them most runs crash |
| Reset button | The solver restarts cleanly |
| Protocol mock | The real `a.out` speaks valid mms commands and logs only to stderr |
| Portability | `solver.c` needs nothing from the C library except the `API_*` functions, so it can move to a microcontroller |

The real-mouse firmware has its own PC tests against a simulated mouse (see [Real Mouse Firmware](#real-mouse-firmware)). GitHub Actions runs all of them on every push and pull request. The test mazes come from [micromouseonline/mazefiles](https://github.com/micromouseonline/mazefiles). See [`tests/README.md`](tests/README.md) for details and known limitations.

---

## Project Structure

The repository is split into simulation code and code for the real mouse.

| Path | Description |
| --- | --- |
| `MMS_Code/` | **Simulation code.** Everything needed to run the solver in mms. |
| `MMS_Code/API.h` | The only boundary between the solver and the outside world: sensing, movement, a millisecond clock and debug display calls. |
| `MMS_Code/API.c` | Simulator version of the API: sends mms commands on stdout and reads replies from stdin. |
| `MMS_Code/main.c` | Entry point and main loop, including reset-button handling. |
| `MMS_Code/solver.h` | Public interface of the solver. |
| `MMS_Code/solver.c` | Mapping, exploration, path planning (Dijkstra over position and heading) and the simulator debug display. Also includes a simple left-wall follower as a reference example (unused by default). |
| `Real_Mouse_Code/` | **Real mouse code.** Reuses `MMS_Code/solver.c` unchanged. See [`Real_Mouse_Code/README.md`](Real_Mouse_Code/README.md). |
| `Real_Mouse_Code/src/` | Motion control, IR wall sensing, the hardware version of the API and the start-up sequence. |
| `Real_Mouse_Code/include/` | The hardware interface (`hal.h`) and the robot numbers to measure and tune (`config.h`). |
| `Real_Mouse_Code/hal_stm32/` | Black Pill pin list and the driver skeleton (to be written). |
| `Real_Mouse_Code/test/` | A simulated mouse and PC tests for the real-mouse firmware. |
| `tests/` | Offline tests for the solver (not part of the mms build): fake simulator, maze loader, generated test mazes, maze generator and `run_tests.sh`. See [`tests/README.md`](tests/README.md). |
| `.github/workflows/` | Runs both sets of tests on every push and pull request. |
| `docs/schematics/` | Schematic images shown in the Circuit Diagrams section. |
| `README.md` | This file. |

<!-- TODO: update this list as files change. -->

---

## Simulator Setup (Windows)

This guide gets the solver building and running in mms on Windows. For everything else about the simulator, see the [mms README](https://github.com/mackorone/mms/blob/main/README.md).

### Install the C compiler (MinGW)

MinGW provides `gcc`, the standard C compiler, on Windows.

<!-- TODO: add the MinGW installer download link. -->

1. Download and run the MinGW installer. *(Download link: coming soon.)*
2. In the MinGW Installation Manager, check `mingw32-gcc-g++-bin`.
3. Open the **Installation** menu, click **Apply Changes**, then click **Apply** in the window that opens. Close the installer when it finishes.
4. Add `gcc` to your `PATH`:
   1. Search Windows for "environment variables" and open **Edit the system environment variables**.
   2. Click **Environment Variables**.
   3. Select the user variable `Path` and click **Edit**.
   4. Click **New** and paste the MinGW `bin` folder. The default is:

      ```
      C:\MinGW\bin
      ```

      Check the folder in File Explorer and use **Copy address as text** to get the exact path.
   5. Click **OK** on every window.
5. Test it. Open Command Prompt and run:

   ```
   gcc
   ```

   If it is set up correctly, you will see `gcc: fatal error: no input files`. That error means it works.

### Install the simulator

1. Download `windows.zip` from the [mms v1.1.0 release](https://github.com/mackorone/mms/releases/tag/v1.1.0).
2. Unzip it anywhere and run `mms.exe`.

> [!TIP]
> If Microsoft Defender SmartScreen blocks it, click **More info**, then **Run anyway**.

### Connect this repo to the simulator

1. Clone or download this repo to a folder **outside** the mms folder, then open its `MMS_Code` folder and copy that folder's path (use **Copy address as text** in File Explorer).
2. In mms, click the **+** button next to the **Mouse** dropdown.
3. Fill in the fields:

   | Field | Value |
   | --- | --- |
   | **Name** | Anything, e.g. `Micromouse_2026` |
   | **Directory** | The path to the `MMS_Code` folder inside this repo, e.g. `C:\path\to\Micromouse_2026\MMS_Code` |
   | **Build Command** | `gcc -o a.exe API.c main.c solver.c` |
   | **Run Command** | The `MMS_Code` folder path followed by `\a.exe`, e.g. `C:\path\to\Micromouse_2026\MMS_Code\a.exe` |

4. Click **Build**. The box next to it should turn green and say `COMPLETE`.
5. Pick a maze from the **Maze** dropdown and click **Run**. The score appears on the **Stats** tab after the run. The solver's log (phases, re-plans, a summary) appears in the **Run Output** tab.

### Tips

- The simulator API (wall checks, movement) is documented in the [mms Mouse API](https://github.com/mackorone/mms#mouse-api). On the real mouse the same API calls are implemented by [`Real_Mouse_Code/src/api_hw.c`](Real_Mouse_Code/src/api_hw.c), which reads the IR sensors and drives the motors.
- The **Reset** button works: the solver forgets its map, acknowledges the reset and starts over. It is only noticed between moves, not in the middle of a multi-cell straight. **Cancel** also restarts a run.
- Custom mazes can be written as text files. See the [Maze Files section of the mms README](https://github.com/mackorone/mms#maze-files). For ready-made mazes, see [micromouseonline/mazefiles](https://github.com/micromouseonline/mazefiles).
- macOS and Linux setup steps: coming soon.

<!-- TODO: add our own example mazes (coming soon) and link them in the maze tip above. -->
<!-- TODO: add macOS and Linux setup steps. -->

---

## Real Mouse Firmware

The real mouse runs the same `solver.c` as the simulator. Only the layer underneath changes: instead of `MMS_Code/API.c` talking to mms, `Real_Mouse_Code/src/api_hw.c` reads the IR wall sensors and drives the motors.

```
MMS_Code/solver.c          the solver, unchanged
        |  API.h
Real_Mouse_Code/src/       wall sensing, straights, turns, start-up
        |  hal.h
hal_stm32/                 Black Pill drivers (still to write)
```

What exists so far:

- **IR wall sensing** that cancels room light, decides walls from a calibrated level, and estimates distances to the walls.
- **Motion control**: speed-profiled straights, heading hold from the IMU, centering between the side walls, correction of the stopping position from the front wall, and absolute-heading 90° turns.
- **Wall-based correction of the IMU heading**, so a small gyro error cannot add up over a whole maze.
- **Start-up and calibration** in the start cell, with fail-safes (an uncalibrated sensor reports a wall, a stalled or slow move stops the motors).
- **A simulated mouse** that all of this is tested against on a PC: 6 of 6 generated 10×10 mazes pass every check, including with noisier sensors and larger encoder and gyro errors.

Still to do: the STM32 drivers, the pin assignments for the IR sensors, motors and encoders, and measuring the real numbers in `config.h`. The full status, bring-up plan and known risks are in [`Real_Mouse_Code/README.md`](Real_Mouse_Code/README.md).

---

## Hardware

| Part | Choice |
| --- | --- |
| Microcontroller | STM32 Black Pill board (schematic part `STM32F11CE_BlackPill`) |
| Motors / encoders | Dual H-bridge motor driver with left and right motor headers (see schematic); motors and encoders TBD |
| IMU | Adafruit BNO055 breakout |
| Wall sensors | Infrared emitters and phototransistor receivers: front center, left and right |
| Power | Battery, fuse and power switch feeding a MIC5219-3.3 voltage regulator (3.3 V) |

**Build and flash steps:** TBD

<!-- TODO: each part will be linked here once a specific part is chosen. Still to confirm: exact STM32 board and part number, motor driver part, motors, encoders and the IR emitter and receiver parts. -->

---

## Circuit Diagrams

<table>
  <tr>
    <td width="50%" align="center">
      <img src="https://github.com/user-attachments/assets/174f34c9-4d9a-4b5e-a684-8d9deb3102d3" alt="Schematic of the voltage regulator circuit" width="100%">
      <br>
      <b>Voltage Regulator</b>
    </td>
    <td width="50%" align="center">
      <img src="https://github.com/user-attachments/assets/33b9558d-0513-4457-ad82-ff45652a9f41" alt="Schematic of the power input and filtering circuit" width="100%">
      <br>
      <b>Power Input and Filtering</b>
    </td>
  </tr>
  <tr>
    <td width="50%" align="center">
      <img src="https://github.com/user-attachments/assets/4074236e-042d-40d0-a153-6fc4828e904b" alt="Schematic of the infrared emitter circuit used for the front, left and right wall sensors" width="100%">
      <br>
      <b>IR Emitter (Front, Left and Right)</b>
    </td>
    <td width="50%" align="center">
      <img src="https://github.com/user-attachments/assets/de23c9d6-72d0-4f77-b1b1-ba1d5da4d40e" alt="Schematic of the infrared receiver circuit used for the front, left and right wall sensors" width="100%">
      <br>
      <b>IR Receiver (Front, Left and Right)</b>
    </td>
  </tr>
  <tr>
    <td width="50%" align="center">
      <img src="docs/schematics/microcontroller.jpg" alt="Schematic of the microcontroller: an STM32 Black Pill board with two 0.1 uF decoupling capacitors on 3.3 V and its GPIO pins broken out to net labels" width="100%">
      <br>
      <b>Microcontroller (Black Pill)</b>
    </td>
    <td width="50%" align="center">
      <img src="docs/schematics/imu-bno055.jpg" alt="Schematic of the Adafruit BNO055 IMU breakout connected to 3.3 V, ground, I2C (PB8 and PB9) and the PA11 and PA12 reset and interrupt pins" width="100%">
      <br>
      <b>IMU (BNO055 Breakout)</b>
    </td>
  </tr>
  <tr>
    <td width="50%" align="center">
      <img src="docs/schematics/connector-j5.jpg" alt="Schematic of a 6-pin header J5 with 5 V on pin 2, ground on pin 3, nets PA3 and PA2 on pins 4 and 5, and pins 1 and 6 unconnected" width="100%">
      <br>
      <b>6-Pin Connector (J5)</b>
    </td>
    <td width="50%" align="center">
      <img src="docs/schematics/push-button-sw2.jpg" alt="Schematic of push button SW2 connected between net PB12 and ground" width="100%">
      <br>
      <b>Push Button (SW2)</b>
    </td>
  </tr>
  <tr>
    <td width="50%" align="center">
      <img src="docs/schematics/power-input-battery.jpg" alt="Schematic of the battery input: two battery connectors, a resettable fuse, a power switch, and a 0.1 uF and 100 uF filter capacitor pair on the switched battery rail" width="100%">
      <br>
      <b>Power Input and Filtering (Battery, Fuse and Switch)</b>
    </td>
    <td width="50%" align="center">
      <img src="docs/schematics/voltage-regulator-mic5219.jpg" alt="Schematic of the 3.3 V voltage regulator: a MIC5219-3.3 regulator fed from the switched battery rail with 10 uF input, 1 nF bypass and 22 uF output capacitors" width="100%">
      <br>
      <b>Voltage Regulator (MIC5219-3.3)</b>
    </td>
  </tr>
  <tr>
    <td width="50%" align="center">
      <img src="docs/schematics/motor-driver.jpg" alt="Schematic of a dual H-bridge motor driver with decoupling capacitors, and header connectors for the left and right motors" width="100%">
      <br>
      <b>Dual H-Bridge Motor Driver and Motor Headers</b>
    </td>
    <td width="50%"></td>
  </tr>
</table>

**Note on the IR sensors:** the IR emitter and IR receiver schematics are each one circuit that is repeated for every wall sensor. The mouse has a copy for the front, the left and the right, and they are identical apart from the microcontroller pin each one connects to.

<!-- TODO: more schematics will be added as they are found for each part. To add one, copy a <td> cell, then swap the image link, the alt text and the bold caption. Start a new <tr> row after every two cells. -->

---

## KiCad PCB Design

**Coming soon.** A picture of the board layout and an explanation of the board will be added once the design is finished.

<!-- TODO: once the design is done, add a picture of the board layout and an explanation of the board. -->

---

## Team

| Name | Role |
| --- | --- |
| Daniel | Team Captain |
| Rafael | Electrical |
| Connor | Programmer |

<!-- TODO: once roles are set, each team member should fill in their role and a short description of what they worked on. -->

---

## Acknowledgements

- This project is developed and tested with [mms](https://github.com/mackorone/mms), the Micromouse Simulator by mackorone.
- The test mazes come from the [micromouseonline/mazefiles](https://github.com/micromouseonline/mazefiles) collection.
