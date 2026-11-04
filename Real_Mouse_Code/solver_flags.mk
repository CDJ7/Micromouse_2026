# Settings for MMS_Code/solver.c on the real mouse (see the table in
# Real_Mouse_Code/README.md). COST_FORWARD and COST_TURN are milliseconds per
# cell and per 90-degree turn: the values here come from the PC model
# (`test/run_hw_tests.sh --timing`), re-measure them on the real mouse.
SOLVER_FLAGS := \
  -DMAX_MAZE_SIZE=10 \
  -DCOST_FORWARD=830 -DCOST_TURN=320 \
  -DEXPLORE_BUDGET=600000 -DEXPLORE_BUDGET_USES_CLOCK=1 -DTIME_SAFETY_MARGIN=5000 \
  -DWALL_READS=3 -DVERIFY_FRONT_BEFORE_MOVE=1 -DVERIFY_UNDRIVEN_ON_RUNS=1 \
  -DDIST_DISPLAY_FLOOD=0 \
  -DDEBUG_SHOW_DISTANCES=0 -DDEBUG_SHOW_WALLS=0 -DDEBUG_SHOW_START_GOAL=0 \
  -DDEBUG_SHOW_VISITED=0 -DDEBUG_SHOW_PLANNED_ROUTE=0 -DDEBUG_SHOW_TARGET=0 \
  -DDEBUG_SHOW_FINAL_ROUTE=0
