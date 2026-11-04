#!/bin/sh
# Offline tests for the simulation maze solver (MMS_Code/). Not part of the mms build.
#   ./tests/run_tests.sh [path/to/micromouseonline/mazefiles] [--all]
# Needs gcc and python3. With the mazefiles folder
# (git clone https://github.com/micromouseonline/mazefiles) the classic 16x16 mazes are used
# too; --all runs every maze that fits (up to 16x16) in that collection. The generated 8x8 and
# 10x10 mazes in tests/mazes are always used.
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
SRC="$HERE/../MMS_Code"
OUT="${TMPDIR:-/tmp}/micromouse_tests"
MAZEFILES=""
ALL=0
for a in "$@"; do
  case "$a" in
    --all) ALL=1 ;;
    *) MAZEFILES="$a" ;;
  esac
done
mkdir -p "$OUT"
CFLAGS="-std=c99 -Wall -Wextra -pedantic -O1 -I$SRC -I$HERE"

# solver.c must not touch stdout/stdin: only API.c may talk to the simulator.
if grep -nE '\b(printf|puts|putchar|scanf|getchar|fgets|fread)\b|\bstdout\b|\bstdin\b' "$SRC/solver.c" "$SRC/main.c" \
     | grep -v 'fprintf(stderr' | grep -vE '^[^:]+:[0-9]+:\s*(/\*|\*|//)'; then
  echo "FAIL: solver.c / main.c use stdout or stdin directly"; exit 1
fi
echo "PASS: solver.c and main.c never touch stdout/stdin"

# The exact mms build command must compile cleanly with warnings on.
(cd "$SRC" && gcc -Wall -Wextra -o "$OUT/a.out" API.c main.c solver.c)
echo "PASS: gcc -o a.exe API.c main.c solver.c builds with no warnings"

# Portability: with LOG redirected, solver.c must need nothing from the C library, only API_* calls.
gcc -std=c99 -Wall -Wextra -pedantic -Os -ffreestanding -fno-builtin -I"$SRC" \
    '-DLOG(...)=((void)0)' -c "$SRC/solver.c" -o "$OUT/solver_port.o"
if nm -u "$OUT/solver_port.o" | grep -v ' API_'; then
  echo "FAIL: solver.c needs C library functions other than the API"; exit 1
fi
echo "PASS: solver.c depends only on API_* functions when LOG is redirected"
for n in 16 10; do
  gcc -std=c99 -Os -ffreestanding -fno-builtin -I"$SRC" -DMAX_MAZE_SIZE=$n '-DLOG(...)=((void)0)' \
      -c "$SRC/solver.c" -o "$OUT/solver_size$n.o"
  echo "info: MAX_MAZE_SIZE=$n -> $(size "$OUT/solver_size$n.o" | tail -1 | awk '{print "code+const " $1 " bytes, static RAM " $2+$3 " bytes (host build, 64-bit)"}')"
done

build() { # name extra-flags
  gcc $CFLAGS $2 -Dmain=mms_main -c "$SRC/main.c" -o "$OUT/main_$1.o"
  gcc $CFLAGS $2 -c "$SRC/solver.c" -o "$OUT/solver_$1.o"
  gcc $CFLAGS $2 -c "$HERE/sim_api.c" -o "$OUT/sim_api_$1.o"
  gcc $CFLAGS -c "$HERE/sim_maze.c" -o "$OUT/sim_maze_$1.o"
  gcc $CFLAGS -c "$HERE/test_main.c" -o "$OUT/test_main_$1.o"
  gcc -o "$OUT/runner_$1" "$OUT/main_$1.o" "$OUT/solver_$1.o" "$OUT/sim_api_$1.o" "$OUT/sim_maze_$1.o" "$OUT/test_main_$1.o"
}
SMALL_BUDGET="${SMALL_BUDGET:-300}"
build default ""
build small "-DEXPLORE_BUDGET=$SMALL_BUDGET"
build nodebug "-DDEBUG_LOG=0 -DDIST_DISPLAY_FLOOD=0"
build routeonly "-DEXPLORE_MAP_WHOLE_MAZE=0"
build weighted "-DCOST_TURN=3 -DEXPLORE_BUDGET=100000"
build clock "-DEXPLORE_BUDGET_USES_CLOCK=1 -DCOST_FORWARD=250 -DCOST_TURN=400 -DEXPLORE_BUDGET=150000 -DTIME_SAFETY_MARGIN=2000"
build noisy3 "-DWALL_READS=5 -DVERIFY_UNDRIVEN_ON_RUNS=1"
build noisy1 "-DWALL_READS=1 -DVERIFY_FRONT_BEFORE_MOVE=0"

status=0
run() { # runner maze args...
  r="$OUT/runner_$1"; shift
  "$r" "$@" 2>>"$OUT/stderr.log" || status=1
}
: > "$OUT/stderr.log"
M="$HERE/mazes"

CLASSIC=""
if [ -n "$MAZEFILES" ]; then
  for f in classic/AAMC15Maze.txt classic/AAMC17Maze.txt classic/AAMC22Maze.txt \
           classic/Portugal-2025-Final.txt classic/allamerica2013.txt \
           classic/alljapan-010-1989-fin.txt halfsize/UK-techfest-2022-a.txt; do
    [ -f "$MAZEFILES/$f" ] && CLASSIC="$CLASSIC $MAZEFILES/$f"
  done
fi

echo; echo "=== default settings (whole maze mapped, cell = turn = 1) ==="
for m in "$M"/*.txt $CLASSIC; do run default "$m" --quiet; done

echo; echo "=== tiny budget ($SMALL_BUDGET): must still stay in budget and reach the center ==="
for m in "$M/gen10-a.txt" "$M/gen10-b.txt" $(echo $CLASSIC | cut -d' ' -f1); do run small "$m" --relaxed --quiet; done

echo; echo "=== route-only mode: stop as soon as the best route is proven ==="
for m in "$M"/*.txt $CLASSIC; do run routeonly "$m" --route-only --terse; done

echo; echo "=== weighted costs (a turn costs 3 cells): planner must still find the cheapest route ==="
for m in "$M"/*.txt $(echo $CLASSIC | cut -d' ' -f1-2); do run weighted "$m" --terse; done

echo; echo "=== clock budget (cell 250 ms, turn 400 ms, 150 s limit) ==="
for m in "$M/gen10-a.txt" "$M/gen10-b.txt" $(echo $CLASSIC | cut -d' ' -f1); do run clock "$m" --relaxed --terse; done

echo; echo "=== reset button pressed mid-run ==="
run default "$M/gen10-a.txt" --reset-at 40 --quiet

echo; echo "=== debug log and live distances switched off ==="
run nodebug "$M/gen10-b.txt" --quiet

echo; echo "=== real a.out against a mock of the mms stdin/stdout protocol ==="
for m in "$M/gen10-a.txt" "$M/gen8-a.txt" $(echo $CLASSIC | cut -d' ' -f1); do
  python3 "$HERE/mock_mms.py" "$OUT/a.out" "$m" || status=1
done

echo; echo "=== noisy wall sensors: 2% of readings flipped, 10 seeds per maze ==="
echo "    (5 reads per wall + front check before every unproven step)  vs  (1 read, no front check)"
for m in "$M/gen10-a.txt" "$M/gen10-b.txt" "$M/gen10-c.txt" "$M/gen8-a.txt" $(echo $CLASSIC | cut -d' ' -f1-2); do
  ok3=0; ok1=0
  for seed in 1 2 3 4 5 6 7 8 9 10; do
    if "$OUT/runner_noisy3" "$m" --noise 20 --seed $seed --terse >/dev/null 2>&1; then ok3=$((ok3+1)); fi
    if "$OUT/runner_noisy1" "$m" --noise 20 --seed $seed --terse >/dev/null 2>&1; then ok1=$((ok1+1)); fi
  done
  printf '    %-44s protected: %2d/10 runs fine   unprotected: %2d/10\n' "$(basename "$m")" $ok3 $ok1
  [ $ok3 -eq 10 ] || status=1
done

if [ $ALL -eq 1 ] && [ -n "$MAZEFILES" ]; then
  echo; echo "=== every maze in the collection that fits (up to 16x16) ==="
  pass=0; fail=0; skip=0
  for f in "$MAZEFILES"/classic/*.txt "$MAZEFILES"/halfsize/*.txt "$MAZEFILES"/training/*.txt; do
    set +e
    line=$("$OUT/runner_default" "$f" --terse 2>>"$OUT/stderr.log")
    rc=$?
    set -e
    case $rc in
      0) pass=$((pass+1)) ;;
      2) skip=$((skip+1)) ;;
      *) fail=$((fail+1)); echo "$line"; status=1 ;;
    esac
  done
  echo "    passed $pass, failed $fail, skipped $skip (too large or no route to the center)"
fi

echo
[ $status -eq 0 ] && echo "ALL TESTS PASSED" || echo "SOME TESTS FAILED"
exit $status
