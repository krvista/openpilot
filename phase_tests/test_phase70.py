"""Phase 70 (report §92): stock-LFA capture mode. One drive with openpilot passive (panda noOutput, the factory camera
stays connected, logging on) so the factory LFA steering this MDPS can be recorded; set from the comma 4 Developer
screen (the toggle right after "quickboot mode"); ends by itself when the ignition goes off or the device boots."""
import importlib
import re
import sys
import types
from unittest.mock import MagicMock

import pytest

from phase_tests.harness import make_cp  # noqa: F401 (path setup side effect)
import openpilot.common.stock_lfa_capture as slc

REPO = __import__("os").path.dirname(__import__("os").path.dirname(__import__("os").path.abspath(__file__)))


class _Params:
  def __init__(self, d=None): self.d = dict(d or {}); self.removed = []
  def get_bool(self, k): return bool(self.d.get(k, False))
  def put_bool(self, k, v, block=False): self.d[k] = bool(v)
  def remove(self, k): self.removed.append(k); self.d.pop(k, None)


@pytest.fixture(autouse=True)
def _restore():
  old = slc.STOCK_LFA_CAPTURE
  yield
  slc.STOCK_LFA_CAPTURE = old


# ---------------------------------------------------------------- the mode itself
def test_capture_request_follows_the_param_and_the_kill_switch():
  assert slc.STOCK_LFA_CAPTURE is True and slc.PARAM == "StockLfaCapture"
  assert slc.capture_requested(_Params({"StockLfaCapture": True})) is True
  assert slc.capture_requested(_Params()) is False
  slc.STOCK_LFA_CAPTURE = False
  assert slc.capture_requested(_Params({"StockLfaCapture": True})) is False


@pytest.mark.parametrize("has_cc, toggle, capture, dashcam, want", [
  (True, True, False, False, True),      # normal sunnypilot drive
  (True, True, True, False, False),      # capture drive: passive
  (True, False, False, False, False),    # 'enable sunnypilot' off: passive as before
  (False, True, False, False, False),
  (True, True, False, True, False),
])
def test_controller_allowed(has_cc, toggle, capture, dashcam, want):
  assert slc.controller_allowed(has_cc, toggle, capture, dashcam) is want


def test_card_uses_the_helper_for_the_passive_decision():
  src = open(f"{REPO}/openpilot/selfdrive/car/card.py").read()
  assert "self.stock_lfa_capture = stock_lfa_capture.capture_requested(self.params)" in src
  assert re.search(r"controller_available = stock_lfa_capture\.controller_allowed\(self\.CI\.CC is not None, openpilot_enabled_toggle,\s+self\.stock_lfa_capture, self\.CP\.dashcamOnly\)", src)
  # passive still means the noOutput safety model (panda: relay passes the factory camera through)
  assert "self.CP.passive = not controller_available or self.CP.dashcamOnly" in src
  assert "safety_config.safetyModel = structs.CarParams.SafetyModel.noOutput" in src


# ---------------------------------------------------------------- one drive only
def test_ignition_off_ends_the_mode():
  p = _Params({"StockLfaCapture": True})
  assert slc.on_ignition(p, True, True) is False and p.get_bool("StockLfaCapture")       # ignition stays on
  assert slc.on_ignition(p, True, False) is False and p.get_bool("StockLfaCapture")      # ignition coming on
  assert slc.on_ignition(p, False, True) is True and not p.get_bool("StockLfaCapture")   # ignition going off
  assert p.removed == ["StockLfaCapture"]
  assert slc.on_ignition(p, False, False) is False and p.removed == ["StockLfaCapture"]  # nothing left to clear


def test_ignition_off_without_the_mode_touches_nothing():
  p = _Params()
  assert slc.on_ignition(p, False, True) is False and p.removed == []


def test_manager_calls_it_every_loop_and_boot_clears_the_param():
  src = open(f"{REPO}/openpilot/system/manager/manager.py").read()
  assert "stock_lfa_capture.on_ignition(params, ignition, ignition_prev)" in src
  assert src.index("stock_lfa_capture.on_ignition") < src.index("ignition_prev = ignition")   # before the edge memory moves on
  keys = open(f"{REPO}/openpilot/common/params_keys.h").read()
  m = re.search(r'\{"StockLfaCapture", \{([^}]*)\}\}', keys)
  assert m and "CLEAR_ON_MANAGER_START" in m.group(1) and "PERSISTENT" not in m.group(1) and "BOOL" in m.group(1)
  # an onroad cycle (the way the toggle applies) must not clear it: no on/offroad-transition flag
  assert "CLEAR_ON_OFFROAD_TRANSITION" not in m.group(1) and "CLEAR_ON_ONROAD_TRANSITION" not in m.group(1)


# ---------------------------------------------------------------- the menu entry (comma 4 Developer screen)
STUB_PREFIXES = ("pyray", "openpilot.system.ui", "openpilot.selfdrive.ui.mici.widgets", "openpilot.selfdrive.ui.widgets",
                 "openpilot.selfdrive.ui.layouts.settings.common", "openpilot.selfdrive.ui.ui_state")


class _StubFinder(importlib.abc.MetaPathFinder, importlib.abc.Loader):
  def find_spec(self, name, path, target=None):
    if name.startswith(STUB_PREFIXES):
      return importlib.machinery.ModuleSpec(name, self, is_package=True)
    return None

  def create_module(self, spec):
    m = MagicMock(); m.__path__ = []; m.__spec__ = spec; return m

  def exec_module(self, module):
    pass


@pytest.fixture
def dev(monkeypatch):
  for k in [k for k in sys.modules if k.startswith(STUB_PREFIXES) or k == "openpilot.selfdrive.ui.mici.layouts.settings.developer"]:
    monkeypatch.delitem(sys.modules, k)
  finder = _StubFinder(); monkeypatch.setattr(sys, "meta_path", [finder] + sys.meta_path)
  import openpilot.system.ui.widgets.scroller as scroller
  scroller.NavScroller = type("NavScroller", (), {"__init__": lambda self: None, "_update_state": lambda self: None,
                                                  "show_event": lambda self: None})
  import openpilot.selfdrive.ui.mici.layouts.settings.developer as dev
  return dev


def test_toggle_sets_the_param_and_restarts_openpilot(dev):
  dev.ui_state.params = _Params()
  dev.restart_needed_callback.reset_mock()
  dev.DeveloperLayoutMici._on_stock_capture_toggled(types.SimpleNamespace(), True)
  assert dev.ui_state.params.get_bool("StockLfaCapture") is True
  dev.restart_needed_callback.assert_called_once()                # onroad cycle: takes effect on this drive
  dev.DeveloperLayoutMici._on_stock_capture_toggled(types.SimpleNamespace(), False)
  assert dev.ui_state.params.get_bool("StockLfaCapture") is False
  assert dev.restart_needed_callback.call_count == 2              # and back to sunnypilot the same way


@pytest.mark.parametrize("offroad, engaged, standstill, v, want", [
  (True, False, False, 0.0, True),       # device on, car off
  (False, False, True, 0.0, True),       # car on, stopped
  (False, False, False, 0.1, True),      # creeping below the threshold counts as stopped
  (False, False, False, 5.0, False),     # moving: the relay must not hand the car over
  (False, True, True, 0.0, False),       # openpilot engaged
])
def test_toggle_only_switches_while_stopped(dev, offroad, engaged, standstill, v, want):
  dev.ui_state.is_offroad = lambda: offroad
  dev.ui_state.engaged = engaged
  dev.ui_state.sm = {"carState": types.SimpleNamespace(standstill=standstill, vEgo=v)}
  assert dev.DeveloperLayoutMici._stock_capture_switchable() is want


def test_toggle_mirrors_a_param_cleared_elsewhere(dev):
  dev.ui_state.params = _Params({"StockLfaCapture": False})
  tog = MagicMock()
  me = types.SimpleNamespace(_stock_capture_toggle=tog, _stock_capture_refresh_n=dev.STOCK_LFA_CAPTURE_REFRESH_FRAMES - 2)
  dev.DeveloperLayoutMici._refresh_stock_capture(me)
  tog.set_checked.assert_not_called()                             # not every frame
  dev.DeveloperLayoutMici._refresh_stock_capture(me)
  tog.set_checked.assert_called_once_with(False)
  assert me._stock_capture_refresh_n == 0
  assert "self._refresh_stock_capture()" in open(dev.__file__).read()


def test_toggle_sits_right_after_quickboot(dev):
  src = open(dev.__file__).read()
  assert re.search(r"self\._quickboot_toggle,\n\s+self\._stock_capture_toggle,\n\s+\]\)", src)
  assert 'BigToggle("stock LFA capture"' in src
  assert "self._stock_capture_toggle.set_enabled(self._stock_capture_switchable)" in src
  assert dev.STOCK_LFA_CAPTURE_PARAM == slc.PARAM
