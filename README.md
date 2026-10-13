# Micromouse 2026

A team Micromouse project: a maze-solving robot whose solver is written in C. The solver is developed and tested in the [Micromouse Simulator (mms)](https://github.com/mackorone/mms) and will later run on a physical mouse in a 10×10 maze. The simulator uses a 16×16 maze.

> [!NOTE]
> **Work in progress.** The code is still being developed, so this README describes what the solver is *designed* to do rather than its exact current state. Details may change as the project evolves.

---

## Table of Contents

- [Goal](#goal)
- [Planned Approach](#planned-approach)
- [Configuration](#configuration)
- [Project Structure](#project-structure)
- [Running in the Simulator](#running-in-the-simulator)
- [Hardware](#hardware)
- [Circuit Diagrams](#circuit-diagrams)
- [KiCad PCB Design](#kicad-pcb-design)
- [Team](#team)

---

## Goal

The aim is the best possible mms score, not just reaching the center. The score (lower is better) is:

```
score = bestRunDistance + bestRunTurns + (totalDistance + totalTurns) / 10
```

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
| `MMS_Code` | The C solver source used in mms: route planning, the three-phase strategy and the entry point. |
| `README.md` | This file. |

A simple left-wall follower is also included in the solver source as a reference example.

<!-- TODO: update this list as files are added, renamed or split up. The solver source includes solver.h and API.h, which are not in the repo yet (API.h is presumably supplied by the mms mouse template); list them here once they are committed. -->

---

## Running in the Simulator

1. Install [mms](https://github.com/mackorone/mms).
2. Add a new mouse in mms and point its directory at this repo.
3. Set the build and run commands. *(TBD, see below.)*
4. Load a maze and press **Run**. The score is shown on the **Stats** tab.

> Build and run commands: **Coming soon.**

<!-- TODO: add the exact build and run commands for mms once the repo has a main.c, Makefile or similar. None exists yet, so no commands are documented. -->

---

## Hardware

| Part | Choice |
| --- | --- |
| Microcontroller | TBD |
| Motors / encoders | TBD |
| Wall sensors | TBD |
| Power | TBD |

**Build and flash steps:** TBD

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

<!-- TODO: more schematics will be added as they are found for each part. To add one, copy a <td> cell, then swap the image link, the alt text and the bold caption. Start a new <tr> row after every two cells. -->

---

## KiCad PCB Design

**Coming soon.**

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
