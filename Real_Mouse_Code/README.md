# Real Mouse Code

**Coming soon.** This folder will hold the firmware that runs on the physical micromouse.

For now, all solver work happens in [`../MMS_Code`](../MMS_Code), the version that runs in the mms simulator. The two are kept separate on purpose:

| Folder | Purpose |
| --- | --- |
| [`MMS_Code/`](../MMS_Code) | Simulation code: the maze solver plus the mms version of the API. |
| `Real_Mouse_Code/` (this folder) | Firmware for the real mouse: motor, sensor and IMU drivers and the hardware version of the API. |

<!-- TODO: add the hardware firmware here when the team says to start on it. The solver (MMS_Code/solver.c and solver.h) is written to be reused unchanged: it only talks to the outside world through API.h, so the real mouse needs its own API implementation (sensors, motors, a millisecond timer) in place of MMS_Code/API.c. Set COST_FORWARD / COST_TURN to the measured cell and turn times, set EXPLORE_BUDGET for the time limit, and consider WALL_READS=3 or more and VERIFY_UNDRIVEN_ON_RUNS=1 (see the settings at the top of solver.c). -->
