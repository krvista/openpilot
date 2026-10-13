"""Phase 52 (report §49): the UI shared core 5 with selfdrived at the same SCHED_FIFO level; a long lane-change redraw
frame stalled selfdrived 120 ms ("System Lagging"). The UI now runs SCHED_OTHER (nice +5) on the pool cores 0-2 only, and
no process other than the control daemons may pin itself to a control core while onroad."""
import os
import re

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
CONTROL_CORES = {3, 4, 5, 6, 7}          # pandad / card+controlsd / selfdrived+plannerd+radard / camerad / modeld
ONROAD_CONTROL = {                        # the only onroad processes allowed on a control core, and where
  "openpilot/selfdrive/selfdrived/selfdrived.py": {5}, "openpilot/selfdrive/controls/plannerd.py": {5},
  "openpilot/selfdrive/controls/radard.py": {5}, "openpilot/selfdrive/car/card.py": {4},
  "openpilot/selfdrive/controls/controlsd.py": {4}, "openpilot/selfdrive/modeld/modeld.py": {7},
  "openpilot/sunnypilot/modeld_v2/modeld.py": {7}, "openpilot/selfdrive/modeld/dmonitoringmodeld.py": {7},
}
OFFROAD_ONLY = {"openpilot/system/ui/mici_setup.py", "openpilot/system/ui/mici_updater.py"}


def _src(rel):
  return open(os.path.join(ROOT, rel)).read()


def _cores(expr):
  return {int(x) for x in re.findall(r"\d+", expr)}


def test_ui_is_not_realtime_and_not_on_a_control_core():
  src = _src("openpilot/selfdrive/ui/ui.py")          # parsed, not imported: pyray cannot start headless
  cores = _cores(re.search(r"^UI_CORES = (\[[^\]]*\])", src, re.M).group(1))
  nice = int(re.search(r"^UI_NICE = (\d+)", src, re.M).group(1))
  assert cores == {0, 1, 2} and nice >= 5
  assert "config_realtime_process" not in src and "sched_setscheduler" not in src
  assert "set_core_affinity(UI_CORES)" in src and "os.nice(UI_NICE)" in src
  assert "Priority." not in src


def test_no_other_process_pins_a_control_core():
  pat = re.compile(r"(?:config_realtime_process|set_core_affinity)\(\s*(\[[^\]]*\]|\d+)")
  for dirpath, _, files in os.walk(os.path.join(ROOT, "openpilot")):
    for fn in files:
      if not fn.endswith(".py") or "/tests" in dirpath or "/openpilot/tools/" in dirpath + "/":
        continue   # dev tools (replay, cabana helpers) never run onroad
      rel = os.path.relpath(os.path.join(dirpath, fn), ROOT)
      if rel in OFFROAD_ONLY or fn in ("realtime.py",):
        continue
      for m in pat.finditer(open(os.path.join(dirpath, fn), errors="ignore").read()):
        cores = _cores(m.group(1))
        if rel in ONROAD_CONTROL:
          assert cores <= ONROAD_CONTROL[rel], (rel, cores)
        else:
          assert not (cores & CONTROL_CORES), (rel, cores)


def test_selfdrived_keeps_ctrl_high_on_core_5():
  from openpilot.common.realtime import Priority
  src = _src("openpilot/selfdrive/selfdrived/selfdrived.py")
  assert re.search(r"config_realtime_process\(\s*5\s*,\s*Priority\.CTRL_HIGH\s*\)", src)
  assert not hasattr(Priority, "UI")
