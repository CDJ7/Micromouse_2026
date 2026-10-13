#!/bin/sh
# Offline tests for the maze solver. Not part of the mms build.
#   ./tests/run_tests.sh [path/to/micromouseonline/mazefiles]
# Needs gcc. The 16x16 classic mazes are used if the mazefiles folder is given
# (git clone https://github.com/micromouseonline/mazefiles); our own generated
# 8x8 and 10x10 mazes in tests/mazes are always used.
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
SRC="$HERE/../MMS_Code"
OUT="${TMPDIR:-/tmp}/micromouse_tests"
MAZEFILES="$1"
mkdir -p "$OUT"
CFLAGS="-std=c99 -Wall -Wextra -pedantic -O1 -I$SRC -I$HERE"

# solver.c must not touch stdout/stdin: only API.c may talk to the simulator.
if grep -nE '\b(printf|puts|putchar|scanf|getchar|fgets|fread)\b|stdout|stdin' "$SRC/solver.c" "$SRC/main.c" \
     | grep -v 'fprintf(stderr' | grep -vE '^[^:]+:[0-9]+:\s*(/\*|\*|//)'; then
  echo "FAIL: solver.c / main.c use stdout or stdin directly"; exit 1
fi
echo "PASS: solver.c and main.c never touch stdout/stdin"

# The exact mms build command must compile cleanly with warnings on.
(cd "$SRC" && gcc -Wall -Wextra -o "$OUT/a.out" API.c main.c solver.c)
echo "PASS: gcc -o a.exe API.c main.c solver.c builds with no warnings"

build() { # name extra-flags
  gcc $CFLAGS $2 -Dmain=mms_main -c "$SRC/main.c" -o "$OUT/main_$1.o"
  gcc $CFLAGS $2 -c "$SRC/solver.c" -o "$OUT/solver_$1.o"
  gcc $CFLAGS -c "$HERE/sim_api.c" -o "$OUT/sim_api_$1.o"
  gcc $CFLAGS -c "$HERE/test_main.c" -o "$OUT/test_main_$1.o"
  gcc -o "$OUT/runner_$1" "$OUT/main_$1.o" "$OUT/solver_$1.o" "$OUT/sim_api_$1.o" "$OUT/test_main_$1.o"
}
build default ""
SMALL_BUDGET="${SMALL_BUDGET:-300}"
build small "-DEXPLORE_BUDGET=$SMALL_BUDGET"
build nodebug "-DDEBUG_LOG=0 -DDIST_DISPLAY_FLOOD=0"

status=0
run() { # runner maze args...
  r="$OUT/runner_$1"; shift
  "$r" "$@" 2>>"$OUT/stderr.log" || status=1
}
: > "$OUT/stderr.log"

echo; echo "=== default budget ==="
for m in "$HERE"/mazes/*.txt; do run default "$m" --quiet; done
if [ -n "$MAZEFILES" ]; then
  for f in "$MAZEFILES"/classic/AAMC15Maze.txt "$MAZEFILES"/classic/AAMC17Maze.txt \
           "$MAZEFILES"/classic/AAMC22Maze.txt "$MAZEFILES"/classic/Portugal-2025-Final.txt \
           "$MAZEFILES"/classic/allamerica2013.txt "$MAZEFILES"/classic/alljapan-010-1989-fin.txt \
           "$MAZEFILES"/halfsize/UK-techfest-2022-a.txt; do
    [ -f "$f" ] && run default "$f" --quiet
  done
fi

echo; echo "=== tiny budget ($SMALL_BUDGET actions): must still stay in budget and reach the center ==="
for m in "$HERE"/mazes/gen10-a.txt "$HERE"/mazes/gen10-b.txt; do run small "$m" --relaxed --quiet; done
[ -n "$MAZEFILES" ] && run small "$MAZEFILES/classic/AAMC15Maze.txt" --relaxed

echo; echo "=== real a.out against a mock of the mms stdin/stdout protocol ==="
for m in "$HERE/mazes/gen10-a.txt" "$HERE/mazes/gen8-a.txt"; do
  python3 "$HERE/mock_mms.py" "$OUT/a.out" "$m" || status=1
done
[ -n "$MAZEFILES" ] && { python3 "$HERE/mock_mms.py" "$OUT/a.out" "$MAZEFILES/classic/AAMC15Maze.txt" || status=1; }

echo; echo "=== reset button pressed mid-run ==="
run default "$HERE/mazes/gen10-a.txt" --reset-at 40 --quiet

echo; echo "=== debug log and live distances switched off ==="
run nodebug "$HERE/mazes/gen10-b.txt" --quiet

echo
[ $status -eq 0 ] && echo "ALL TESTS PASSED" || echo "SOME TESTS FAILED"
exit $status
