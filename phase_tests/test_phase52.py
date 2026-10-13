"""Phase 52 (report §49): ui runs on core 5 with selfdrived. At the same SCHED_FIFO level a long UI frame ran to completion
and stalled selfdrived 120 ms ("System Lagging", route 45 863.8 s). ui now sits one level below the control processes."""
import os
import re

from openpilot.common.realtime import Priority

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")


def _src(rel):
  return open(os.path.join(ROOT, rel)).read()


def test_ui_priority_is_below_the_control_processes():
  assert Priority.CTRL_LOW < Priority.UI < Priority.CTRL_HIGH


def test_ui_uses_the_ui_level():
  src = _src("openpilot/selfdrive/ui/ui.py")
  assert re.search(r"config_realtime_process\(\s*0\s*,\s*Priority\.UI\s*\)", src)
  assert "Priority.CTRL_HIGH" not in src


def test_selfdrived_keeps_ctrl_high_on_core_5():
  src = _src("openpilot/selfdrive/selfdrived/selfdrived.py")
  assert re.search(r"config_realtime_process\(\s*5\s*,\s*Priority\.CTRL_HIGH\s*\)", src)
