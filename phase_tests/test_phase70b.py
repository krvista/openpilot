"""Phase 70b (report §93-94): the stock-LFA capture switch without ADAS warnings.
 - the cause, measured on routes 8b-91: on every restart into the capture mode pandad armed the ENDED session's safety
   mode again (harness relay in intercept) off ControlsReady / FirmwareQueryDone / CarParams that outlived it; the passive
   session sent nothing, so the car had no LKAS_ALT / CAM_0x362 at all. manager now drops the two gates when a session ends
 - "stock LFA next start": the arming becomes the drive's mode on the rising edge of the ignition, so that drive never
   restarts and never switches the relay"""
import re
import textwrap
import types

import pytest

from phase_tests.harness import make_cp  # noqa: F401 (path setup side effect)
import openpilot.common.session_gates as sg
import openpilot.common.stock_lfa_capture as slc
from phase_tests import test_phase70 as t70
from phase_tests.test_phase70 import _Params, _screen

dev = t70.dev   # the fixture: the comma 4 Developer screen module with its UI imports stubbed

REPO = __import__("os").path.dirname(__import__("os").path.dirname(__import__("os").path.abspath(__file__)))


@pytest.fixture(autouse=True)
def _restore():
  old = (sg.DROP_SESSION_GATES, slc.STOCK_LFA_CAPTURE)
  yield
  sg.DROP_SESSION_GATES, slc.STOCK_LFA_CAPTURE = old


# ---------------------------------------------------------------- pandad's safety setter, as panda_safety.cc has it
class _PandaSafety:
  """PandaSafety::configureSafetyMode / fetchCarParams (panda_safety.cc), line for line; the panda is a list of the
  safety modes it was given."""

  def __init__(self, params):
    self.params = params; self.initialized = False; self.safety_configured = False; self.modes = ["noOutput"]

  def configure(self, is_onroad):
    if is_onroad and not self.safety_configured:
      if not self.initialized:                       # updateMultiplexingMode(): elm327 for the fingerprinting
        self.modes.append("elm327"); self.initialized = True
      if self.params.get_bool("FirmwareQueryDone") and self.params.get_bool("ControlsReady"):   # fetchCarParams()
        self.modes.append(self.params.d["CarParams"])                                            # setSafetyMode()
        self.safety_configured = True
    elif not is_onroad:
      self.initialized = False; self.safety_configured = False

  def offroad(self):                                 # send_panda_states(): should_close_relay
    self.modes.append("noOutput")

  @property
  def relay_intercepts(self):
    return self.modes[-1] not in ("silent", "noOutput", "elm327")


def test_the_model_is_what_pandad_does():
  src = open(f"{REPO}/openpilot/selfdrive/pandad/panda_safety.cc").read()
  body = src[src.index("void PandaSafety::configureSafetyMode(bool is_onroad) {"):src.index("// TODO-SP: Use structs instead of vector\nvoid PandaSafety::setSafetyMode")]
  assert re.search(r"if \(is_onroad && !safety_configured_\) \{\s+updateMultiplexingMode\(\);\s+auto car_params = fetchCarParams\(\);\s+if \(!car_params\.empty\(\)\) \{", body)
  assert re.search(r"setSafetyMode\(car_params\);\s+safety_configured_ = true;", body)
  assert re.search(r"\} else if \(!is_onroad\) \{\s+initialized_ = false;\s+safety_configured_ = false;", body)
  assert re.search(r"if \(!initialized_\) \{\s+prev_obd_multiplexing_ = false;\s+panda_->set_safety_model\(cereal::CarParams::SafetyModel::ELM327, 1U\);\s+initialized_ = true;", body)
  assert re.search(r'if \(!params_\.getBool\("FirmwareQueryDone"\)\) \{\s+return \{\};', body)
  assert re.search(r'if \(!params_\.getBool\("ControlsReady"\)\) \{\s+return \{\};', body)
  assert 'return {params_.get("CarParams"), params_.get("CarParamsSP")};' in body
  assert set(sg.SESSION_GATES) == {"ControlsReady", "FirmwareQueryDone"}
  # pandad follows deviceState by itself (not a param manager writes after its own clear), at 10 Hz
  pandad = open(f"{REPO}/openpilot/selfdrive/pandad/pandad.cc").read()
  assert 'is_onroad = sm["deviceState"].getDeviceState().getStarted();' in pandad
  assert "bool should_close_relay = !ignition_local || !is_onroad;" in pandad
  # the modes the panda releases the relay in
  main = open(f"{REPO}/panda/board/main.c").read()
  fw = main[main.index("void set_safety_mode("):main.index("bool is_car_safety_mode(")]
  for case in ("SAFETY_SILENT", "SAFETY_NOOUTPUT", "SAFETY_ELM327"):
    assert re.search(rf"case {case}:\s+set_intercept_relay\(false, false\);", fw), case
  assert re.search(r"default:\s+set_intercept_relay\(true, false\);", fw)
  # the keys really live until the next onroad edge, and a passive card never sets ControlsReady again
  keys = open(f"{REPO}/openpilot/common/params_keys.h").read()
  for k in ("ControlsReady", "FirmwareQueryDone", "CarParams"):
    m = re.search(r'\{"%s", \{([^}]*)\}\}' % k, keys)
    assert m and "CLEAR_ON_ONROAD_TRANSITION" in m.group(1) and "CLEAR_ON_OFFROAD_TRANSITION" not in m.group(1), k
  card = open(f"{REPO}/openpilot/selfdrive/car/card.py").read()
  assert "if not self.CP.passive and initialized:\n      self.controls_update(" in card
  assert card.count('put_bool("ControlsReady", True)') == 1
  assert card.index("def controls_update") < card.index('put_bool("ControlsReady", True)') < card.index("  def step(self):")


def _restart(drop_gates, new_session_passive, old_session_passive=False, pandad_first=True):
  """One onroad cycle with the ignition on. Returns the pandad model after the new session has come up."""
  sg.DROP_SESSION_GATES = drop_gates
  p = _Params({"CarParams": "hyundaiCanfd"})
  ps = _PandaSafety(p)
  # the session that is about to end
  ps.configure(True)
  p.put_bool("FirmwareQueryDone", True)
  if not old_session_passive:
    p.put_bool("ControlsReady", True)
  ps.configure(True)
  assert ps.relay_intercepts is (not old_session_passive)
  # deviceState: offroad. manager's offroad transition, pandad's 10 Hz tick
  sg.drop(p)
  ps.offroad(); ps.configure(False)
  assert not ps.relay_intercepts
  # deviceState: onroad again. Who is first: pandad's tick, or manager's CLEAR_ON_ONROAD_TRANSITION?
  def manager_clear():
    for k in ("CarParams", "ControlsReady", "FirmwareQueryDone"):
      p.remove(k)
  if pandad_first:
    ps.configure(True); manager_clear()
  else:
    manager_clear(); ps.configure(True)
  early = ps.relay_intercepts                        # before the new card has sent a single frame
  # the new card: fingerprint, CarParams; a passive one never reaches ControlsReady
  p.d["CarParams"] = "noOutput" if new_session_passive else "hyundaiCanfd"
  p.put_bool("FirmwareQueryDone", True); ps.configure(True)
  if not new_session_passive:
    p.put_bool("ControlsReady", True)
  ps.configure(True)
  return ps, early


def test_the_bug_pandad_rearms_the_ended_session():
  """b62f36a0, routes 8c / 8e / 90: restart into the capture mode, pandad at the onroad edge before manager"""
  ps, early = _restart(drop_gates=False, new_session_passive=True)
  assert early and ps.relay_intercepts               # relay in intercept, and the passive session sends nothing: for good
  assert ps.modes[-2:] == ["elm327", "hyundaiCanfd"]
  # the same hole in a restart into a normal session, until its card is up
  ps, early = _restart(drop_gates=False, new_session_passive=False)
  assert early


def test_the_bug_needs_pandad_to_win_the_race_and_a_session_that_set_controlsready():
  ps, early = _restart(drop_gates=False, new_session_passive=True, pandad_first=False)
  assert not early and not ps.relay_intercepts
  # routes 8f / 91: the restarts OUT of the capture mode did not arm early - the passive session had not set ControlsReady
  ps, early = _restart(drop_gates=False, new_session_passive=False, old_session_passive=True)
  assert not early and ps.relay_intercepts and ps.modes[-2:] == ["elm327", "hyundaiCanfd"]


@pytest.mark.parametrize("pandad_first", [True, False])
@pytest.mark.parametrize("old_passive", [False, True])
def test_with_the_gates_dropped_pandad_waits_for_the_new_card(pandad_first, old_passive):
  ps, early = _restart(drop_gates=True, new_session_passive=True, old_session_passive=old_passive, pandad_first=pandad_first)
  assert not early and not ps.relay_intercepts       # capture session: elm327, the factory camera drives the car
  assert ps.modes[-1] == "elm327"
  ps, early = _restart(drop_gates=True, new_session_passive=False, old_session_passive=old_passive, pandad_first=pandad_first)
  assert not early and ps.relay_intercepts           # normal session: armed by ITS card, not before
  assert ps.modes[-2:] == ["elm327", "hyundaiCanfd"]


def test_drop_removes_both_gates_and_nothing_else():
  p = _Params({"ControlsReady": True, "FirmwareQueryDone": True, "CarParams": "x", "StockLfaCapture": True})
  assert sg.drop(p) == ("ControlsReady", "FirmwareQueryDone")
  assert p.removed == ["ControlsReady", "FirmwareQueryDone"] and set(p.d) == {"CarParams", "StockLfaCapture"}
  sg.DROP_SESSION_GATES = False
  p = _Params({"ControlsReady": True, "FirmwareQueryDone": True})
  assert sg.drop(p) == () and p.removed == []


# ---------------------------------------------------------------- manager's own loop body, run step by step
def _run_manager_loop(steps, params):
  """manager.py's loop body (from sm.update to ensure_running) against a scripted deviceState / pandaStates.
  Returns what was done in each step."""
  src = open(f"{REPO}/openpilot/system/manager/manager.py").read()
  body = src[src.index("    sm.update(1000)\n"):src.index("    running = ' '.join(")]
  code = compile(textwrap.dedent(body), "manager_loop", "exec")
  unknown = object()
  done = []

  class _SM(dict):
    def update(self, timeout): pass

  class _P(_Params):
    def clear_all(self, flag): done[-1].append(f"clear {flag}")
    def remove(self, k): done[-1].append(f"remove {k}"); super().remove(k)

  p = _P(params)
  ns = dict(sm=_SM(), log=types.SimpleNamespace(PandaState=types.SimpleNamespace(PandaType=types.SimpleNamespace(unknown=unknown))),
            session_gates=sg, stock_lfa_capture=slc, params=p,
            ParamKeyFlag=types.SimpleNamespace(CLEAR_ON_ONROAD_TRANSITION="onroad", CLEAR_ON_OFFROAD_TRANSITION="offroad",
                                               CLEAR_ON_IGNITION_ON="ignition"),
            cloudlog=types.SimpleNamespace(warning=lambda m: done[-1].append(f"log {m}")),
            managed_processes={}, ignore=[],
            ensure_running=lambda procs, started, **kw: done[-1].append(f"run {started} capture={slc.capture_requested(p)}"),
            started_prev=False, ignition_prev=False)
  for started, ignition in steps:
    done.append([])
    ns["sm"]["deviceState"] = types.SimpleNamespace(started=started)
    ns["sm"]["carParams"] = None
    ns["sm"]["pandaStates"] = [types.SimpleNamespace(pandaType="cuatro", ignitionLine=ignition, ignitionCan=False)]
    exec(code, ns)
  return done, p


def test_manager_loop_drops_the_gates_as_the_session_ends():
  done, p = _run_manager_loop([
    (False, False),     # 0 device up, car off
    (False, True),      # 1 ignition on
    (True, True),       # 2 onroad
    (True, True),       # 3
    (False, True),      # 4 onroad cycle: offroad with the ignition on
    (False, True),      # 5
    (True, True),       # 6 the cycle is over
  ], {"ControlsReady": True, "FirmwareQueryDone": True})
  assert done[2] == ["clear onroad", "run True capture=False"]
  assert done[3] == ["run True capture=False"]
  # the gates go in the very step the processes are stopped in - before manager can be held up collecting them
  assert done[4] == ["clear offroad", "remove ControlsReady", "remove FirmwareQueryDone", "run False capture=False"]
  assert not p.get_bool("ControlsReady") and not p.get_bool("FirmwareQueryDone")
  assert done[5] == ["run False capture=False"]
  assert done[6] == ["clear onroad", "run True capture=False"]
  src = open(f"{REPO}/openpilot/system/manager/manager.py").read()
  loop = src[src.index("  while True:\n    sm.update(1000)"):src.index("    # send managerState")]
  assert loop.index("session_gates.drop(params)") < loop.index("ensure_running(managed_processes.values(), started, params=params")
  # the restart timing itself is the one the car has always had
  hw = open(f"{REPO}/openpilot/system/hardware/hardwared.py").read()
  assert re.search(r"^ONROAD_CYCLE_TIME = 1\b", hw, re.M) and "  offroad_cycle_count = 0\n" in hw
  assert "started = sm['deviceState'].started\n" in loop


def test_manager_loop_starts_an_armed_capture_before_card():
  done, p = _run_manager_loop([
    (False, False),
    (True, True),       # the device booted with the car already on: ignition and onroad in one step
    (True, True),
    (False, False),     # ignition off
  ], {"StockLfaCaptureNext": True})
  assert done[1] == ["clear onroad", "clear ignition", "log stock LFA capture: started with the ignition", "run True capture=True"]
  assert done[2] == ["run True capture=True"]
  assert done[3][-1] == "run False capture=False" and "log stock LFA capture: ended with the ignition" in done[3]
  assert not p.get_bool("StockLfaCaptureNext") and not p.get_bool("StockLfaCapture")


# ---------------------------------------------------------------- "next start": armed, starts with the ignition
def test_armed_capture_starts_with_the_ignition_and_is_used_up():
  p = _Params({"StockLfaCaptureNext": True})
  assert slc.on_ignition(p, False, False) is None and not p.get_bool("StockLfaCapture")     # car still off
  assert slc.on_ignition(p, True, False) == "started"                                        # ignition on
  assert p.get_bool("StockLfaCapture") is True and p.get_bool("StockLfaCaptureNext") is False
  assert slc.capture_requested(p) is True
  assert slc.on_ignition(p, True, True) is None
  assert slc.on_ignition(p, False, True) == "ended" and not p.get_bool("StockLfaCapture")    # ignition off
  assert slc.on_ignition(p, True, False) is None and not p.get_bool("StockLfaCapture")       # the next drive is normal


def test_arming_does_nothing_in_the_drive_it_is_set_in():
  p = _Params()
  slc.on_ignition(p, True, False)
  p.put_bool("StockLfaCaptureNext", True)                       # armed while driving
  for _ in range(5):
    assert slc.on_ignition(p, True, True) is None
  assert slc.capture_requested(p) is False and p.get_bool("StockLfaCaptureNext") is True


def test_cold_boot_with_the_ignition_already_on_counts_as_the_rising_edge():
  """the device boots after the key is turned: manager's first loop with a panda already reports the ignition on"""
  p = _Params({"StockLfaCaptureNext": True})
  ignition_prev = False                                         # manager_thread starts with this
  assert slc.on_ignition(p, True, ignition_prev) == "started"


def test_kill_switch_uses_the_arming_up_without_a_capture_drive():
  slc.STOCK_LFA_CAPTURE = False
  p = _Params({"StockLfaCaptureNext": True})
  assert slc.on_ignition(p, True, False) is None
  assert not p.get_bool("StockLfaCapture") and not p.get_bool("StockLfaCaptureNext")   # no stale arming left for later


def test_manager_arms_before_it_starts_card():
  src = open(f"{REPO}/openpilot/system/manager/manager.py").read()
  assert src.index("stock_lfa_capture.on_ignition(params, ignition, ignition_prev)") < \
         src.index("ensure_running(managed_processes.values(), started, params=params")
  keys = open(f"{REPO}/openpilot/common/params_keys.h").read()
  m = re.search(r'\{"StockLfaCaptureNext", \{([^}]*)\}\}', keys)
  assert m and m.group(1).replace(" ", "") == 'PERSISTENT,BOOL,"0"'       # survives the boot between the two drives
  assert keys.isascii()
  assert slc.PARAM_NEXT == "StockLfaCaptureNext"


# ---------------------------------------------------------------- the second menu entry
def test_next_start_toggle_arms_without_touching_this_drive(dev):
  dev.ui_state.params = _Params()
  dev.restart_needed_callback.reset_mock()
  me = _screen(dev)
  dev.DeveloperLayoutMici._on_stock_next_toggled(me, True)
  assert dev.ui_state.params.get_bool("StockLfaCaptureNext") is True
  assert dev.ui_state.params.get_bool("StockLfaCapture") is False
  dev.restart_needed_callback.assert_not_called()               # no onroad cycle: nothing is handed over now
  assert me._stock_next_toggle.checked is True and me._stock_next_toggle.value == dev.STOCK_LFA_NEXT_TEXT[True]
  dev.DeveloperLayoutMici._on_stock_next_toggled(me, False)     # changed my mind
  assert dev.ui_state.params.get_bool("StockLfaCaptureNext") is False
  assert me._stock_next_toggle.checked is False and me._stock_next_toggle.value == dev.STOCK_LFA_NEXT_TEXT[False]
  dev.restart_needed_callback.assert_not_called()


def test_screen_follows_the_arming_turning_into_the_drive(dev):
  dev.ui_state.params = _Params({"StockLfaCaptureNext": True})
  me = _screen(dev)
  dev.DeveloperLayoutMici._show_stock_capture(me)
  assert (me._stock_capture_toggle.checked, me._stock_next_toggle.checked) == (False, True)
  slc.on_ignition(dev.ui_state.params, True, False)             # the next ignition on
  dev.DeveloperLayoutMici._show_stock_capture(me)
  assert (me._stock_capture_toggle.checked, me._stock_next_toggle.checked) == (True, False)
  assert me._stock_capture_toggle.value == dev.STOCK_LFA_CAPTURE_TEXT[True]
  n = me._stock_capture_toggle.layouts
  dev.DeveloperLayoutMici._show_stock_capture(me)
  assert me._stock_capture_toggle.layouts == n                  # an unchanged text is not laid out again


def test_next_start_toggle_is_the_entry_after_capture(dev):
  src = open(dev.__file__).read()
  assert 'BigToggle("stock LFA next start"' in src
  assert dev.STOCK_LFA_CAPTURE_NEXT_PARAM == slc.PARAM_NEXT
  assert "self._stock_next_toggle.set_enabled" not in src       # arming changes nothing now: allowed at any time
  for text in list(dev.STOCK_LFA_CAPTURE_TEXT.values()) + list(dev.STOCK_LFA_NEXT_TEXT.values()):
    assert len(text) <= 46                                      # the length the first version's sub-text had
