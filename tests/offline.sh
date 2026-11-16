#!/usr/bin/env bash
# Everything that runs without the arm or ROS, in the module's image (rammp_feeding's source and test pieces at
# /module_ws/src/rammp_feeding, FEEDING_SRC): the unit tests and the whole drink against tests/cup_robot.py, on both
# test sites (rammp_feeding's tests/fixture.py: level, and tilt30).
set -uo pipefail
cd "$(dirname "$0")/.."
PY=${PY:-python3}
fail=0
echo "== unit tests"; $PY -W ignore tests/unit_test.py 2>&1 | tail -1 | grep -q OK || { $PY -W ignore tests/unit_test.py; fail=1; }
printf "== drink "; $PY tests/test_drink.py 2>&1 | tail -1 | tee /dev/stderr | grep -q "GOES THROUGH" || { $PY tests/test_drink.py; fail=1; }
printf "== tilt30 drink "                       # rammp_feeding's tilt30 site: the arm leaning 30 deg, the user to its right
RAMMP_TEST_SITE=tilt30 $PY tests/test_drink.py 2>&1 | tail -1 | tee /dev/stderr | grep -q "GOES THROUGH" || fail=1
[ $fail = 0 ] && echo "OFFLINE OK" || { echo "OFFLINE FAILED"; exit 1; }
