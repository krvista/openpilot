#!/usr/bin/env bash
# i6nv3 acceptance triangle — run from repo root on a dev machine:
#   1. phase_tests      : CarController / controlsd behaviour (python)
#   2. safety suite     : opendbc C safety layer (upstream file + i6n CCNC file)
#   3. test_dbc_frames  : DBC packing golden frames (inside phase_tests)
# Every leg must be green before anything is flashed to the car.
set -uo pipefail
ROOT=$(cd "$(dirname "$0")/../.." && pwd); cd "$ROOT"
fail=0
echo "=== [1+3] phase_tests (incl. test_dbc_frames)"
python3 -m pytest phase_tests --noconftest -c phase_tests/pytest.ini -q 2>&1 | tail -1 | tee /tmp/acc1; grep -q " passed" /tmp/acc1 && ! grep -q "failed\|error" /tmp/acc1 || fail=1
echo "=== [2] opendbc safety suite: hyundai canfd (upstream)"
( cd opendbc_repo && PYTHONPATH=$PWD python3 -m pytest opendbc/safety/tests/test_hyundai_canfd.py -q 2>&1 | tail -1 ) | tee /tmp/acc2; grep -q " passed" /tmp/acc2 && ! grep -q "failed\|error" /tmp/acc2 || fail=1
echo "=== [2] opendbc safety suite: i6n CCNC (angle enforcement + model-id falsification)"
( cd opendbc_repo && PYTHONPATH=$PWD python3 -m pytest opendbc/safety/tests/test_hyundai_canfd_i6n.py -q 2>&1 | tail -1 ) | tee /tmp/acc3; grep -q " passed" /tmp/acc3 && ! grep -q "failed\|error" /tmp/acc3 || fail=1
echo "=== generated DBC duplicate-SG_ scan"
python3 - <<'PY' || fail=1
import re, sys
txt = open("opendbc_repo/opendbc/dbc/hyundai_canfd_generated.dbc").read()
bad = [m.group(2) for m in re.finditer(r"^BO_ (\d+) (\w+): \d+.*\n((?: SG_ .*\n)+)", txt, re.M)
       if len(set(re.findall(r" SG_ (\w+) ", m.group(3)))) != len(re.findall(r" SG_ (\w+) ", m.group(3)))]
print("duplicate-SG_ messages:", bad or "none"); sys.exit(1 if bad else 0)
PY
echo; [[ $fail -eq 0 ]] && echo "✔ ACCEPTANCE GREEN" || { echo "✖ ACCEPTANCE FAILED"; exit 1; }
