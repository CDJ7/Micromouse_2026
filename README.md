# Micromouse 2026

A team Micromouse project: a maze-solving robot whose solver is written in C. The solver is developed and tested in the [Micromouse Simulator (mms)](https://github.com/mackorone/mms) and will later run on a physical mouse in a 10×10 maze. The simulator uses a 16×16 maze.
A team Micromouse project: a maze-solving robot whose solver is written in C. The solver is developed and tested in [mms](https://github.com/mackorone/mms), the Micromouse Simulator by mackorone, and will later run on a physical mouse in a 10×10 maze. The simulator uses a 16×16 maze.

> [!NOTE]
> **Work in progress.** The code is still being developed, so this README describes what the solver is *designed* to do rather than its exact current state. Details may change as the project evolves.
> **Work in progress.** The code is still being developed, so this README describes the intended design rather than the exact current state. Details may change as the project evolves.

---

## Table of Contents

- [Goal](#goal)
- [Planned Approach](#planned-approach)
- [Configuration](#configuration)
- [Project Structure](#project-structure)
- [Running in the Simulator](#running-in-the-simulator)
- [Simulator Setup (Windows)](#simulator-setup-windows)
  - [Install the C compiler (MinGW)](#install-the-c-compiler-mingw)
  - [Install the simulator](#install-the-simulator)
  - [Connect this repo to the simulator](#connect-this-repo-to-the-simulator)
- [Hardware](#hardware)
- [Circuit Diagrams](#circuit-diagrams)
- [KiCad PCB Design](#kicad-pcb-design)
- [Team](#team)
- [Acknowledgements](#acknowledgements)

---

## Goal

The aim is the best possible mms score, not just reaching the center. The score (lower is better) is:
The aim is the best mms score, not just reaching the center. The official score (lower is better) is:

```
score = best run turns + best run effective distance + 0.1 × (total turns + total effective distance)
```

"Effective distance" equals normal distance when the mouse moves one cell at a time. See the [Scorekeeping section of the mms README](https://github.com/mackorone/mms/blob/main/README.md#scorekeeping) for the full definition.

This has two implications:

1. **Turns cost as much as forward moves.** The best path is the one with the fewest actions, not just the fewest cells.
2. **Exploration counts at a tenth of the final run.** Exploring is only worth it when it shortens the final run.

---

## Planned Approach

Instead of a classic flood fill, the solver is designed to use a shortest-path search over position **and** heading, so the cost of turns is priced in. The run is split into three phases:

| Phase | What it does |
| --- | --- |
| **Explore** | Plans a route to the center treating unknown walls as open, drives out to check the uncertain parts of that route, then re-plans. Stops once the best route is confirmed, rather than mapping the whole maze. |
| **Return** | Heads back to the start on a path that also sets the mouse up well for the final run. |
| **Final run** | Runs the best known route from the start to the center. |

---

## Configuration

The solver is intended to expose two tunable settings:

| Setting | Description |
| --- | --- |
| **Maze size** | 16×16 for the simulator, 10×10 for the real maze. The goal is the center, so it moves with the size. |
| **Exploration margin** | How close to perfect the route must be before exploring stops. Zero means explore until the best route is proven. |

---

## Project Structure

| File | Description |
| --- | --- |
| `MMS_Code` | The core of the project: route planning and the three-phase strategy (explore, return, final run).|
| `README.md` | This file. |

MMS Github includes a simple left-wall follower is also included in the solver source as a reference example.

<!-- TODO: update this list as files are added, renamed or split up. The solver source includes solver.h and API.h, which are not in the repo yet (API.h is presumably supplied by the mms mouse template); list them here once they are committed. -->
<!-- TODO: update this list as files change. The solver source includes solver.h and API.h, and the build command below expects API.c, main.c and solver.c, but none of these are committed yet. List them (with one-line descriptions) once they are added. -->

---

## Running in the Simulator
### Simulator Setup (Windows)

This guide gets the solver building and running in mms on Windows. For everything else about the simulator, see the [mms README](https://github.com/mackorone/mms/blob/main/README.md).

### Install the C compiler (MinGW)

MinGW provides `gcc`, the standard C compiler, on Windows.

<!-- TODO: add the MinGW installer download link. -->

### Connect this repo to the simulator

1. Clone or download this repo to a folder **outside** the mms folder, then copy its folder path (use **Copy address as text** in File Explorer).
2. In mms, click the **+** button next to the **Mouse** dropdown.
3. Fill in the fields:

   | Field | Value |
   | --- | --- |
   | **Name** | Anything, e.g. `Micromouse_2026` |
   | **Directory** | The repo folder path |
   | **Build Command** | `gcc -o a.exe API.c main.c solver.c` |
   | **Run Command** | The repo folder path followed by `\a.exe`, e.g. `C:\path\to\Micromouse_2026\a.exe` |

   > [!WARNING]
   > The build command above lists the files mms expects in a mouse folder (`API.c`, `main.c`, `solver.c`). Only `MMS_Code` is committed to this repo right now, so the build will not succeed until those source files are added.

   <!-- TODO: once the source files are committed, confirm the build command matches the files that actually exist and update it if needed. -->
4. Click **Build**. The box next to it should turn green and say `COMPLETE`.
5. Pick a maze from the **Maze** dropdown and click **Run**. The score appears on the **Stats** tab after the run.

### Tips

- The simulator API (wall checks, movement) is documented in the [mms Mouse API](https://github.com/mackorone/mms#mouse-api). When the code moves to the real mouse, these API calls get replaced with functions that read the IR sensors and drive the motors.
- The **Reset** button isn't supported by this code. Click **Cancel** to restart a run instead.
- Custom mazes can be written as text files. See the [Maze Files section of the mms README](https://github.com/mackorone/mms#maze-files). For ready-made mazes, see [micromouseonline/mazefiles](https://github.com/micromouseonline/mazefiles).
- macOS and Linux setup steps: coming soon.

<!-- TODO: add our own example mazes (coming soon) and link them in the maze tip above. -->
<!-- TODO: add macOS and Linux setup steps. -->

---

## Hardware

| Part | Choice |
| --- | --- |
| Microcontroller | ESP32 |
| Motors / encoders | TBD |
| Wall sensors | TBD |
| Power | TBD |

**Build and flash steps:** TBD

Links Section for Parts when Found.
<!-- TODO: each part will be linked here once a specific part is chosen. -->

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
      <img src="https://github.com/user-attachments/assets/4074236e-042d-40d0-a153-6fc4828e904b" alt="Schematic of the left infrared emitter circuit" width="100%">
      <br>
      <b>IR Emitter (Left)</b>
    </td>
    <td width="50%" align="center">
      <img src="https://github.com/user-attachments/assets/de23c9d6-72d0-4f77-b1b1-ba1d5da4d40e" alt="Schematic of the left infrared receiver circuit" width="100%">
      <br>
      <b>IR Receiver (Left)</b>
    </td>
  </tr>
</table>

More will be added as schematics are found.
<!-- TODO: more schematics will be added as they are found for each part. To add one, copy a <td> cell, then swap the image link, the alt text and the bold caption. Start a new <tr> row after every two cells. -->

---

## KiCad PCB Design

**Once PCB design is finish picture and explination will be added here**

<!-- TODO: once the design is done, add a picture of the board layout and an explanation of the board. -->

---

## Team

| Name | Role |
| --- | --- |
| Daniel | Team Captain |
| Rafael | TBD |
| Marisol | TBD |
| Griselda | TBD |
| Connor | TBD |

<!-- TODO: once roles are set, each team member should fill in their role and a short description of what they worked on. -->

---

## Acknowledgements

This project is developed and tested with [mms](https://github.com/mackorone/mms), the Micromouse Simulator by mackorone.
