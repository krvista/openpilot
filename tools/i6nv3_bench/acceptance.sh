#!/usr/bin/env bash
# i6nv3 acceptance triangle — run from repo root on a dev machine:
#   1. phase_tests      : CarController / controlsd behaviour (python)
#   2. safety suite     : opendbc C safety layer (upstream file + i6n CCNC file)
#   3. test_dbc_frames  : DBC packing golden frames (inside phase_tests)
#   4. static review    : sm keys / enums / capnp fields / interp tables / Params keys
#   5. process replay   : real selfdrived+controlsd on a logged segment (DRIVELOG_SEG)
#   6. pre-flight       : regenerated TX through libsafety on that segment, rejection < 0.5%
#   7. end-to-end       : real controlsd + real selfdrived on a lane-dropout segment (DRIVELOG_DROPOUT_SEG) -> latch + alert
# Legs 5/7 replay the daemons on the LOG's carParams (angle steering, CCNC flags); legs 5/6 arm/track the log's panda state.
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
echo "=== [3b] ruff: syntax / undefined names / duplicate defs on the control path (a NameError in publish() slipped past phase_tests once)"
ruff check --select E9,F63,F7,F82,F821,F811 openpilot/selfdrive/controls openpilot/selfdrive/selfdrived openpilot/selfdrive/car openpilot/system/hardware/hardwared.py opendbc_repo/opendbc/car/hyundai opendbc_repo/opendbc/car/lateral.py opendbc_repo/opendbc/car/scalar.py --output-format concise 2>&1 | tee /tmp/acc3b | grep -q "All checks passed" || fail=1
echo "=== [4] static review: sm keys / enum members / capnp fields / interp tables / Params keys (exit 1 on issue)"
PYTHONPATH=$PWD:$PWD/opendbc_repo python3 tools/ccnc_analysis/static_review.py > /tmp/acc4 2>&1 && echo "static review: clean" || { sed -n '/^## ISSUES/,/^## notes/p' /tmp/acc4; fail=1; }
# legs 5-6 need a local drive log: DRIVELOG_SEG=<path to an rlog.zst of a recent i6nv3 drive> (skipped if unset)
if [[ -n "${DRIVELOG_SEG:-}" && -f "${DRIVELOG_SEG}" ]]; then
  echo "=== [5] process replay: real selfdrived + controlsd + card on ${DRIVELOG_SEG##*/}"
  PYTHONPATH=$PWD:$PWD/opendbc_repo python3 tools/i6nv3_bench/process_replay_check.py "$DRIVELOG_SEG" selfdrived,controlsd,card 420 2>&1 | grep -v "^stack\|KjException" | tail -4 | tee /tmp/acc5; grep -q "PROCESS REPLAY GREEN" /tmp/acc5 || fail=1
  echo "=== [6] pre-flight: regenerated LKAS_ALT frames through the panda safety code (closed loop)"
  R=$(basename "$DRIVELOG_SEG" | sed -E 's/^[0-9a-f]+_([0-9a-f]+)--.*/\1/'); SEGN=$(basename "$DRIVELOG_SEG" | sed -E 's/.*--([0-9]+)--rlog.*/\1/')
  PYTHONPATH=$PWD:$PWD/opendbc_repo python3 tools/ccnc_analysis/preflight_replay.py "$R" "$SEGN" 2>&1 | grep -v "^stack\|KjException" | grep -E "^seg|^TOTAL" | tee /tmp/acc6
  python3 - <<'PY' || fail=1
import re,sys
t=open("/tmp/acc6").read(); m=re.search(r"rejected (\d+) \((\d+\.\d+)%\)", t)
ok = bool(m) and float(m.group(2)) < 0.5
print("pre-flight rejection rate", (m.group(2)+"%") if m else "n/a", "->", "OK" if ok else "FAIL (>= 0.5%)"); sys.exit(0 if ok else 1)
PY
else
  echo "=== [5]/[6] process replay + pre-flight: SKIPPED (set DRIVELOG_SEG=<rlog.zst>)"
fi
# leg 7 needs a segment with a logged lane-line dropout (laneLineProbs min < 0.20 while latActive) — route 4 seg 8 of ccnc-drivelog
if [[ -n "${DRIVELOG_DROPOUT_SEG:-}" && -f "${DRIVELOG_DROPOUT_SEG}" ]]; then
  echo "=== [7] end-to-end lane-dropout: real controlsd latch -> real selfdrived alert on ${DRIVELOG_DROPOUT_SEG##*/}"
  PYTHONPATH=$PWD:$PWD/opendbc_repo python3 tools/i6nv3_bench/replay_e2e_dropout.py "$DRIVELOG_DROPOUT_SEG" 50 2>&1 | grep -v "^stack\|KjException" | grep -E "^E2E|Traceback" | tee /tmp/acc7; grep -q "E2E GREEN" /tmp/acc7 || fail=1
else
  echo "=== [7] end-to-end lane-dropout: SKIPPED (set DRIVELOG_DROPOUT_SEG=<rlog.zst with a lane dropout>)"
fi
echo "=== generated DBC duplicate-SG_ scan"
python3 - <<'PY' || fail=1
import re, sys
txt = open("opendbc_repo/opendbc/dbc/hyundai_canfd_generated.dbc").read()
bad = [m.group(2) for m in re.finditer(r"^BO_ (\d+) (\w+): \d+.*\n((?: SG_ .*\n)+)", txt, re.M)
       if len(set(re.findall(r" SG_ (\w+) ", m.group(3)))) != len(re.findall(r" SG_ (\w+) ", m.group(3)))]
print("duplicate-SG_ messages:", bad or "none"); sys.exit(1 if bad else 0)
PY
echo; [[ $fail -eq 0 ]] && echo "✔ ACCEPTANCE GREEN" || { echo "✖ ACCEPTANCE FAILED"; exit 1; }
