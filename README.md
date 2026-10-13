Micromouse_2026

Work in progress. The code is still being developed, so this README describes what the solver is designed to do rather than its exact current state. Details may change as the project evolves.
A maze-solving robot project for Micromouse, written in C. The solver is developed and tested in the Micromouse Simulator (mms) and will be carried over to a physical mouse running a 10×10 maze.

Goal

The aim is to finish with the best possible score, not just to reach the centre. In mms, the score (lower is better) looks like this:

score = bestRunDistance + bestRunTurns + (totalDistance + totalTurns) / 10

That shapes the overall approach:
Turns and forward moves cost about the same, so the best path is the one with the fewest actions, not only the fewest cells.
Time spent exploring counts against the score, but only at a fraction of the final run. Exploring is worth it when it leads to a shorter final run.
Planned approach

Rather than a classic flood fill, the solver is designed to plan routes using a shortest-path search that tracks both position and heading. This lets it account for the cost of turns, not just distance.

The run is split into three phases.

1. Explore

The mouse plans a route to the centre, treating unexplored walls as if they were open. It then drives out to check the parts of that route it isn't sure about and re-plans as it learns more of the maze. The idea is to stop exploring once the best route is confirmed, instead of mapping the whole maze.

2. Return

Once a good route is known, the mouse heads back to the start. It picks a path home that also sets it up well for the final run.

3. Final run

The mouse runs the best known route from the start to the centre as quickly as possible.

Configuration

The solver is intended to have a few simple settings that can be tuned:

Maze size: 16×16 for the simulator, 10×10 for the real maze. The goal is the centre of the maze, so it moves with the size.
Exploration margin: how close to "perfect" the route needs to be before the mouse stops exploring. A margin of zero means it keeps exploring until the best route is proven; a larger margin trades a slightly longer run for less exploring.
Project structure
<!-- TODO: update as files are added or renamed -->
solver.c   Route planning and strategy
solver.h   Solver interface
API.h      Wall sensing and movement interface (simulator or hardware)

A simple left-wall follower is also included as a reference example.

Running in the simulator
Install mms.
Add a new mouse in mms and point its directory at this repo.
Set the build and run commands. <!-- TODO: add the exact build and run commands -->
Load a maze and press Run. The score is shown on the Stats tab.

Hardware:
<!-- Links to exact Hardware chosen for Micromouse-->
Microcontroller: TBD
Motors / encoders: TBD
Wall sensors: TBD
Power: TBD
Build and flash steps: TBD

Circuit diagrams:
<!-- TODO: add schematic image, e.g. ![Circuit diagram](docs/circuit-diagram.png), more will be added when schematic is found for the part -->
![Voltage_Regulator](downloads/<img width="872" height="611" alt="IMG_20261013_114547" src="https://github.com/user-attachments/assets/174f34c9-4d9a-4b5e-a684-8d9deb3102d3" />)
![Power_Input_and_Filtering](downloads/<img width="684" height="552" alt="IMG_20261013_114547 (1)" src="https://github.com/user-attachments/assets/33b9558d-0513-4457-ad82-ff45652a9f41" />)
![IR_Emitter_L](downloads/<img width="572" height="518" alt="IMG_20261013_114547 (3)" src="https://github.com/user-attachments/assets/4074236e-042d-40d0-a153-6fc4828e904b" />)
![IR_Receiver_L](downloads/<img width="315" height="373" alt="IMG_20261013_114547 (2)" src="https://github.com/user-attachments/assets/de23c9d6-72d0-4f77-b1b1-ba1d5da4d40e" />)

KiCAD PCB Design:
<!-- Once design has been made a picture and explanation can be put in this section -->

Team:
<!-- Each person can fill out their role, once it has been set in stone -->
- Daniel (Team Captain)
- Rafael (Role)
- Marisol (Role)
- Griselda (Role)
- Connor (Role)
