"""Phase 70b (report §93): the stock-LFA capture switch without ADAS warnings.
 - leaving onroad with the ignition on, manager acts one step later: the panda releases the harness relay first and only
   then are the onroad processes stopped (openpilot's 0x110 / 0x362 must not end before the camera's are back) - the
   order a pulled comma cable has
 - "stock LFA next start": the arming becomes the drive's mode on the rising edge of the ignition, so that drive never
   switches the relay at all"""
import re
import types

import pytest

from phase_tests.harness import make_cp  # noqa: F401 (path setup side effect)
import openpilot.common.relay_first_stop as rfs
import openpilot.common.stock_lfa_capture as slc
from phase_tests import test_phase70 as t70
from phase_tests.test_phase70 import _Params, _screen

dev = t70.dev   # the fixture: the comma 4 Developer screen module with its UI imports stubbed

REPO = __import__("os").path.dirname(__import__("os").path.dirname(__import__("os").path.abspath(__file__)))


@pytest.fixture(autouse=True)
def _restore():
  old = (rfs.RELAY_FIRST_STOP, slc.STOCK_LFA_CAPTURE)
  yield
  rfs.RELAY_FIRST_STOP, slc.STOCK_LFA_CAPTURE = old


def _ps(mode, panda_type="cuatro"):
  return types.SimpleNamespace(safetyModel=mode, pandaType=panda_type)


# ---------------------------------------------------------------- is the relay out?
@pytest.mark.parametrize("states, want", [
  ([], True),                                             # no panda: nothing to wait for
  ([_ps("hyundaiCanfd")], False),                         # car mode: the relay intercepts
  ([_ps("noOutput")], True),
  ([_ps("elm327")], True),
  ([_ps("silent")], True),
  ([_ps("hyundaiCanfd", "unknown")], True),               # a panda that is not there does not count
  ([_ps("noOutput"), _ps("hyundaiCanfd")], False),        # every connected panda must have let go
])
def test_relay_released(states, want):
  assert rfs.relay_released(states) is want


def test_released_modes_are_the_ones_panda_releases_the_relay_in():
  src = open(f"{REPO}/panda/board/main.c").read()
  body = src[src.index("void set_safety_mode("):src.index("bool is_car_safety_mode(")]
  for case in ("SAFETY_SILENT", "SAFETY_NOOUTPUT", "SAFETY_ELM327"):
    assert re.search(rf"case {case}:\s+set_intercept_relay\(false, false\);", body), case
  assert re.search(r"default:\s+set_intercept_relay\(true, false\);", body)
  assert set(rfs.RELAY_RELEASED_MODES) == {"silent", "noOutput", "elm327"}


def test_pandad_releases_the_relay_off_devicestate_within_its_10hz_loop():
  """what the hold waits for: pandad drops to noOutput when deviceState.started goes false, in its 10 Hz state loop"""
  src = open(f"{REPO}/openpilot/selfdrive/pandad/pandad.cc").read()
  assert 'is_onroad = sm["deviceState"].getDeviceState().getStarted();' in src
  assert "bool should_close_relay = !ignition_local || !is_onroad;" in src
  assert re.search(r"if \(should_close_relay && .*NO_OUTPUT\)\)\) \{\s+panda->set_safety_model\(cereal::CarParams::SafetyModel::NO_OUTPUT\);", src)
  assert "if (rk.frame() % 10 == 0) {" in src and 'RateKeeper rk("pandad", 100);' in src


# ---------------------------------------------------------------- the hold
def test_started_passes_through_when_nothing_is_to_be_handed_over():
  r = rfs.RelayFirstStop()
  assert r.update(False, False, True, 0.0) is False and r.event is None      # boot, car off
  assert r.update(False, True, True, 1.0) is False                           # ignition on, not onroad yet
  assert r.update(True, True, True, 2.0) is True                             # onroad (elm327: still released)
  assert r.update(True, True, False, 3.0) is True and r.hold_t0 is None      # openpilot has the relay
  assert r.event is None


def test_onroad_cycle_stops_the_processes_only_after_the_relay_is_out():
  r = rfs.RelayFirstStop()
  r.update(True, True, False, 10.0)
  # deviceState says offroad (onroad cycle), the panda still intercepts: openpilot keeps running
  assert r.update(False, True, False, 10.5) is True and r.hold_t0 == 10.5
  assert "holding" in r.event
  # pandad has switched to noOutput: now manager may stop the processes
  assert r.update(False, True, True, 11.0) is False and r.hold_t0 is None
  assert "relay released" in r.event and "0.50 s" in r.event
  assert r.update(False, True, True, 11.5) is False and r.event is None
  assert r.update(True, True, True, 12.5) is True                             # the cycle is over: onroad again


def test_hold_waits_no_longer_than_the_limit():
  r = rfs.RelayFirstStop()
  r.update(True, True, False, 0.0)
  assert r.update(False, True, False, 1.0) is True
  assert r.update(False, True, False, 1.0 + rfs.RELAY_FIRST_HOLD_MAX_S - 0.01) is True
  assert r.update(False, True, False, 1.0 + rfs.RELAY_FIRST_HOLD_MAX_S) is False and "timeout" in r.event
  assert 0.3 <= rfs.RELAY_FIRST_HOLD_MAX_S <= 1.0     # pandad needs ~0.1 s; manager's loop is 2 Hz


def test_missed_offroad_window_keeps_the_session_and_asks_again():
  """a stalled loop: deviceState is onroad again while manager still holds. pandad has re-armed the running session by
  then (relay in intercept), so stopping it now would cut its frames - it keeps running and a new cycle is asked for"""
  r = rfs.RelayFirstStop()
  r.update(True, True, False, 0.0)
  assert r.update(False, True, False, 1.0) is True and r.retry is False
  assert r.update(True, True, False, 3.5) is True and r.retry is True and r.hold_t0 is None
  assert "missed" in r.event and "new onroad cycle" in r.event
  assert r.update(True, True, False, 4.0) is True and r.retry is False       # asked once
  # the new cycle goes the normal way
  assert r.update(False, True, False, 4.5) is True
  assert r.update(False, True, True, 5.0) is False and r.retries == 0


def test_missed_windows_do_not_ask_forever():
  r = rfs.RelayFirstStop()
  r.update(True, True, False, 0.0)
  asked = 0
  for k in range(6):
    assert r.update(False, True, False, 10.0 * k + 1.0) is True
    assert r.update(True, True, False, 10.0 * k + 5.0) is True
    asked += r.retry
  assert asked == rfs.RELAY_FIRST_MAX_RETRIES == 2


def test_session_gates_are_dropped_when_the_session_ends():
  """pandad arms the safety mode on FirmwareQueryDone + ControlsReady and reads deviceState by itself: left set until the
  clear at the next onroad edge, a pandad tick just before that clear would arm the ended session's CarParams"""
  assert set(rfs.SESSION_GATES) == {"ControlsReady", "FirmwareQueryDone"}
  pandad = open(f"{REPO}/openpilot/selfdrive/pandad/panda_safety.cc").read()
  for gate in rfs.SESSION_GATES:
    assert f'params_.getBool("{gate}")' in pandad
  src = open(f"{REPO}/openpilot/system/manager/manager.py").read()
  assert re.search(r"elif not started and started_prev:\n\s+params\.clear_all\(ParamKeyFlag\.CLEAR_ON_OFFROAD_TRANSITION\)\n(\s+#.*\n)+"
                   r"\s+for gate in relay_first_stop\.SESSION_GATES:\n\s+params\.remove\(gate\)", src)
  # a passive card never sets ControlsReady again, so nothing would correct a wrongly armed panda
  card = open(f"{REPO}/openpilot/selfdrive/car/card.py").read()
  assert "if not self.CP.passive and initialized:\n      self.controls_update(" in card
  assert card.count('put_bool("ControlsReady", True)') == 1
  assert card.index("def controls_update") < card.index('put_bool("ControlsReady", True)') < card.index("  def step(self):")


def test_ignition_off_is_not_held():
  r = rfs.RelayFirstStop()
  r.update(True, True, False, 0.0)
  assert r.update(False, False, False, 1.0) is False and r.hold_t0 is None    # pandad releases the relay by itself
  r.update(True, True, False, 2.0)
  assert r.update(False, True, False, 3.0) is True                            # an onroad cycle ...
  assert r.update(False, False, False, 3.2) is False and "ignition off" in r.event   # ... and the key is turned


def test_no_hold_when_the_relay_is_already_out():
  r = rfs.RelayFirstStop()
  r.update(True, True, True, 0.0)            # a passive (capture) session: noOutput
  assert r.update(False, True, True, 1.0) is False and r.hold_t0 is None and r.event is None


def test_kill_switch_restores_the_immediate_stop():
  rfs.RELAY_FIRST_STOP = False
  r = rfs.RelayFirstStop()
  r.update(True, True, False, 0.0)
  assert r.update(False, True, False, 1.0) is False and r.hold_t0 is None
  rfs.RELAY_FIRST_STOP = True
  r = rfs.RelayFirstStop()
  r.update(True, True, False, 0.0)
  assert r.update(False, True, False, 1.0) is True              # holding ...
  rfs.RELAY_FIRST_STOP = False
  assert r.update(False, True, False, 1.1) is False and "disabled" in r.event   # ... and switched off meanwhile


def test_timeline_of_an_onroad_cycle():
  """the whole handover on the real clocks: hardwared reports offroad at T for the cycle time; pandad releases the relay
  at its next 10 Hz tick and the pandaStates of that tick still carries the old mode (the health is read before the
  switch), so manager sees the release one tick later; manager's loop runs on deviceState (the edges plus 2 Hz). card
  must be alive when the relay goes out, and the processes must keep a second to exit before they are started again."""
  src = open(f"{REPO}/openpilot/system/hardware/hardwared.py").read()
  cycle = float(re.search(r"^ONROAD_CYCLE_TIME = ([0-9.]+)", src, re.M).group(1))
  assert cycle >= rfs.RELAY_FIRST_HOLD_MAX_S + 1.0
  pandad = open(f"{REPO}/openpilot/selfdrive/pandad/pandad.cc").read()
  body = pandad[pandad.index("std::optional<bool> send_panda_states("):pandad.index("void process_panda_state(")]
  assert body.index("panda->get_state()") < body.index("should_close_relay") < body.index("fill_panda_state(ps, panda->hw_type, health)")
  after = []; margin = []
  for pandad_phase in (0.001, 0.02, 0.05, 0.08, 0.10):          # where pandad's 10 Hz tick falls after the edge
    for ds_phase in (0.01, 0.1, 0.2, 0.3, 0.4, 0.5):            # where the 2 Hz deviceState falls after the edge
      r = rfs.RelayFirstStop()
      r.update(True, True, False, 0.0)
      t_off = 1.0; t_release = t_off + pandad_phase; t_seen = t_release + 0.1; t_on = t_off + cycle
      ticks = sorted({t_off, t_on} | {round(t_off + ds_phase + 0.5 * k, 3) for k in range(8)})
      card_alive = True; t_stop = t_start = None
      for t in ticks:
        started = not (t_off <= t < t_on)
        eff = r.update(started, True, t >= t_seen, t)
        assert not r.retry
        if card_alive and not eff:
          card_alive = False; t_stop = t
        elif not card_alive and eff and t_start is None:
          t_start = t
      assert t_stop is not None and t_start is not None, (pandad_phase, ds_phase)
      after.append(t_stop - t_release); margin.append(t_start - t_stop)
  assert min(after) >= 0.1                 # openpilot is still sending when the relay goes out, every time
  assert max(after) <= 0.7                 # and is stopped soon after
  assert min(margin) >= 1.0                # time for the processes to exit before the restart


def test_longer_onroad_cycle_does_not_delay_the_start_after_boot():
  src = open(f"{REPO}/openpilot/system/hardware/hardwared.py").read()
  cycle = float(re.search(r"^ONROAD_CYCLE_TIME = ([0-9.]+)", src, re.M).group(1))
  assert "offroad_cycle_count = -int((ONROAD_CYCLE_TIME - 1) * SERVICE_LIST['pandaStates'].frequency)" in src
  assert 'onroad_conditions["not_onroad_cycle"] = (sm.frame - offroad_cycle_count) >= ONROAD_CYCLE_TIME * SERVICE_LIST[\'pandaStates\'].frequency' in src
  hz = 10
  start = -int((cycle - 1) * hz)
  assert next(f for f in range(100) if (f - start) >= cycle * hz) == 1 * hz          # boot: one second, as with the 1 s cycle
  assert sum(1 for f in range(500, 600) if (f - 500) < cycle * hz) == cycle * hz     # a requested cycle: the full time


def test_manager_acts_on_the_held_value_everywhere():
  src = open(f"{REPO}/openpilot/system/manager/manager.py").read()
  loop = src[src.index("  while True:\n    sm.update(1000)"):src.index("    # send managerState")]
  assert "relay_first = relay_first_stop.RelayFirstStop()" in src
  assert re.search(r"started = relay_first\.update\(sm\['deviceState'\]\.started, ignition, relay_first_stop\.relay_released\(sm\['pandaStates'\]\),\s+time\.monotonic\(\)\)", loop)
  assert loop.count("sm['deviceState'].started") == 1                       # nothing in the loop reads the raw value
  assert loop.index("ignition = any(") < loop.index("relay_first.update(") < loop.index("CLEAR_ON_OFFROAD_TRANSITION")
  assert "ensure_running(managed_processes.values(), started, params=params" in loop
  assert 'cloudlog.warning(f"relay-first stop: {relay_first.event}")' in loop
  assert re.search(r"if relay_first\.retry:\n\s+params\.put_bool\(\"OnroadCycleRequested\", True\)", loop)


def _run_manager_loop(steps, params):
  """manager.py's own loop body (from sm.update to ensure_running), run step by step against a scripted deviceState /
  pandaStates. Returns what was done in each step."""
  import textwrap
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
            relay_first=rfs.RelayFirstStop(), relay_first_stop=rfs, stock_lfa_capture=slc, params=p,
            ParamKeyFlag=types.SimpleNamespace(CLEAR_ON_ONROAD_TRANSITION="onroad", CLEAR_ON_OFFROAD_TRANSITION="offroad",
                                               CLEAR_ON_IGNITION_ON="ignition"),
            cloudlog=types.SimpleNamespace(warning=lambda m: done[-1].append(f"log {m}")),
            time=types.SimpleNamespace(monotonic=lambda: ns["now"]), managed_processes={}, ignore=[],
            ensure_running=lambda procs, started, **kw: done[-1].append(f"run {started} capture={slc.capture_requested(p)}"),
            started_prev=False, ignition_prev=False)
  for now, started, ignition, mode in steps:
    done.append([])
    ns["now"] = now
    ns["sm"]["deviceState"] = types.SimpleNamespace(started=started)
    ns["sm"]["carParams"] = None
    ns["sm"]["pandaStates"] = [types.SimpleNamespace(safetyModel=mode, pandaType="cuatro", ignitionLine=ignition, ignitionCan=False)]
    exec(code, ns)
  return done, p


def test_manager_loop_hands_the_car_over_relay_first():
  done, p = _run_manager_loop([
    (0.0, False, False, "noOutput"),        # 0 device up, car off
    (0.5, False, True, "noOutput"),         # 1 ignition on
    (1.0, True, True, "elm327"),            # 2 onroad
    (9.0, True, True, "hyundaiCanfd"),      # 3 openpilot has the relay
    (20.0, False, True, "hyundaiCanfd"),    # 4 onroad cycle asked: deviceState offroad, the relay still in intercept
    (20.5, False, True, "noOutput"),        # 5 pandad has let it go
    (21.0, False, True, "noOutput"),        # 6
    (22.0, True, True, "elm327"),           # 7 the cycle is over
  ], {"ControlsReady": True, "FirmwareQueryDone": True})
  assert done[2] == ["clear onroad", "run True capture=False"]
  assert done[3] == ["run True capture=False"]
  # the step the old code stopped everything in: nothing is cleared, nothing is stopped
  assert done[4] == ["log relay-first stop: holding the onroad processes until the panda releases the relay", "run True capture=False"]
  # the relay is out: the offroad transition, with the session's gates dropped before pandad can read them again
  assert done[5] == ["log relay-first stop: acting on offroad after 0.50 s (relay released)", "clear offroad",
                     "remove ControlsReady", "remove FirmwareQueryDone", "run False capture=False"]
  assert not p.get_bool("ControlsReady") and not p.get_bool("FirmwareQueryDone")
  assert done[6] == ["run False capture=False"]
  assert done[7] == ["clear onroad", "run True capture=False"]


def test_manager_loop_starts_an_armed_capture_before_card():
  done, p = _run_manager_loop([
    (0.0, False, False, "noOutput"),
    (0.5, True, True, "elm327"),            # the device booted with the car already on: ignition and onroad in one step
    (1.0, True, True, "elm327"),
    (60.0, False, False, "noOutput"),       # ignition off
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
