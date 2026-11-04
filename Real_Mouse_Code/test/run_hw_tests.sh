#!/bin/sh
# PC tests for the real-mouse firmware (everything except the register-level HAL).
#   ./Real_Mouse_Code/test/run_hw_tests.sh
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
RM="$HERE/.."
SRC="$RM/src"
SOLVER="$RM/../MMS_Code"
TESTS="$RM/../tests"
OUT="${TMPDIR:-/tmp}/micromouse_hw_tests"
mkdir -p "$OUT"

# Solver settings for the real mouse. COST_* are milliseconds per cell and per
# turn (measure with `test_hw --timing`); EXPLORE_BUDGET is the exploration time
# limit in milliseconds.
SOLVER_FLAGS="-DMAX_MAZE_SIZE=10 -DCOST_FORWARD=${COST_FORWARD:-830} -DCOST_TURN=${COST_TURN:-320} \
 -DEXPLORE_BUDGET=${EXPLORE_BUDGET:-600000} -DEXPLORE_BUDGET_USES_CLOCK=1 -DTIME_SAFETY_MARGIN=5000 \
 -DWALL_READS=3 -DVERIFY_FRONT_BEFORE_MOVE=1 -DVERIFY_UNDRIVEN_ON_RUNS=1 -DDIST_DISPLAY_FLOOD=0 \
 -DDEBUG_SHOW_DISTANCES=0 -DDEBUG_SHOW_WALLS=0 -DDEBUG_SHOW_START_GOAL=0 -DDEBUG_SHOW_VISITED=0 \
 -DDEBUG_SHOW_PLANNED_ROUTE=0 -DDEBUG_SHOW_TARGET=0 -DDEBUG_SHOW_FINAL_ROUTE=0"
# LOGGING=1 sends the solver's log through hal_log (to stderr here, a serial port on the mouse)
if [ "${LOGGING:-0}" = 1 ]; then
  SOLVER_FLAGS="$SOLVER_FLAGS -include $RM/include/hal.h -DLOG(...)=hal_log(__VA_ARGS__)"
else
  SOLVER_FLAGS="$SOLVER_FLAGS -DDEBUG_LOG=0"
fi
CFLAGS="-std=c99 -Wall -Wextra -pedantic -O2 -I$RM/include -I$SOLVER -I$TESTS -I$HERE"

cc() { gcc $CFLAGS "$@"; }
cc $SOLVER_FLAGS -c "$SOLVER/solver.c" -o "$OUT/solver.o"
cc -Dsolver_init=hw_test_solver_init -Dmain=hw_main -c "$SRC/main_hw.c" -o "$OUT/main_hw.o"
cc -c "$SRC/api_hw.c" -o "$OUT/api_hw.o"
cc -c "$SRC/motion.c" -o "$OUT/motion.o"
cc -c "$SRC/ir_sensors.c" -o "$OUT/ir_sensors.o"
cc -c "$TESTS/sim_maze.c" -o "$OUT/sim_maze.o"
cc -c "$HERE/test_hw.c" -o "$OUT/test_hw.o"
cc -c "$HERE/fake_hal.c" -o "$OUT/fake_hal.o"
# The pure arithmetic inside the STM32 HAL (yaw unwrapping, encoder counters, PWM scaling).
gcc -std=c99 -Wall -Wextra -pedantic -o "$OUT/test_hal_logic" "$HERE/test_hal_logic.c" "$RM/hal_stm32/hal_logic.c" -lm
"$OUT/test_hal_logic" | grep -E 'FAIL|ALL HAL'
"$OUT/test_hal_logic" >/dev/null
echo "PASS: hal_logic arithmetic"

# The real STM32 firmware must still build (compile and link only; it cannot run on a PC).
CMSIS_CORE="${CMSIS_CORE:-$HOME/cmsis/cmsis_core}"
CMSIS_DEVICE_F4="${CMSIS_DEVICE_F4:-$HOME/cmsis/cmsis_device_f4}"
if command -v arm-none-eabi-gcc >/dev/null 2>&1 && [ -d "$CMSIS_CORE" ] && [ -d "$CMSIS_DEVICE_F4" ]; then
  (cd "$RM" && make clean >/dev/null && make CMSIS_CORE="$CMSIS_CORE" CMSIS_DEVICE_F4="$CMSIS_DEVICE_F4" 2>&1 | tail -3)
  [ -f "$RM/build/mouse.bin" ] && echo "PASS: STM32 firmware builds and links (Cortex-M4)"
else
  echo "SKIP: STM32 firmware build (needs arm-none-eabi-gcc and the two CMSIS folders, see Real_Mouse_Code/Makefile)"
fi
gcc -o "$OUT/test_hw" "$OUT"/solver.o "$OUT"/main_hw.o "$OUT"/api_hw.o "$OUT"/motion.o "$OUT"/ir_sensors.o \
    "$OUT"/fake_hal.o "$OUT"/test_hw.o "$OUT"/sim_maze.o -lm
# The same firmware against a worse mouse: noisier sensors, bigger encoder and gyro errors, slower motors.
STRESS="-DMOTOR_MM_S_PER_PERMILLE=1.0 -DENC_SCALE_ERROR=1.012 -DGYRO_SCALE_ERROR=1.006 -DGYRO_NOISE_DEG=0.08 -DIR_NOISE=30.0"
cc $STRESS -c "$HERE/fake_hal.c" -o "$OUT/fake_hal_stress.o"
gcc -o "$OUT/test_hw_stress" "$OUT"/solver.o "$OUT"/main_hw.o "$OUT"/api_hw.o "$OUT"/motion.o "$OUT"/ir_sensors.o \
    "$OUT"/fake_hal_stress.o "$OUT"/test_hw.o "$OUT"/sim_maze.o -lm
echo "PASS: real-mouse firmware and the real solver build with -Wall -Wextra -pedantic"

if [ "$1" = "--timing" ]; then "$OUT/test_hw" --timing "$TESTS/mazes/gen10-a.txt"; exit 0; fi

status=0
echo "=== simulated mouse (10x10 mazes) ==="
for m in "$TESTS"/mazes/gen10-*.txt; do
  "$OUT/test_hw" "$m" || status=1
done
echo; echo "=== worse mouse: noisier sensors, 1.2% encoder error, 0.6% gyro error, different motors ==="
for m in "$TESTS"/mazes/gen10-*.txt; do
  "$OUT/test_hw_stress" "$m" || status=1
done
[ $status -eq 0 ] && echo "ALL REAL-MOUSE PC TESTS PASSED" || echo "SOME REAL-MOUSE PC TESTS FAILED"
exit $status
