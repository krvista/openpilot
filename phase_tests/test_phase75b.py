"""Phase 75 (report §103), controls / MADS / driver-information side.
75j  controlsd: the lane-centre term is not added again while the model action is non-finite (the held command already
     contains it).
75k  latcontrol_angle: the 'Turn Exceeds Steering Limit' saturation path works again as upstream; not while the car
     controller holds the request passive by design or the factory LFA steers.
75l  CarControlSP.latGranted from pandaStates (stale / missing = granted); the MADS lateral mismatch counter decays
     while the panda agrees.
75m  MADS on the CCNC angle platform: the LFA button is no enable source while SCC main is off (no-entry text instead);
     the rising edge of cruiseState.enabled enables lateral when MADS is off and main is on.
75n  minimum steer speed with hysteresis (off at 4 km/h, on above 5 km/h).
75o  driver information: one prompt per passive episode, 'factory LFA active' / 'hold-off paused' texts, the paused
     presentation while the factory LFA steers.
Every change has a kill switch (module constants)."""
import importlib
import os
import shutil
import subprocess
import sys
import tempfile
import types
from unittest.mock import MagicMock

import numpy as np
import pytest

from phase_tests.harness import Sim, make_cp
from phase_tests.harness_noncontrol import FakeParams
from phase_tests import test_noncontrol_controlsd_state as H
from phase_tests.test_phase73 import model
from opendbc.car import structs
from opendbc.car.hyundai.values import HyundaiFlags, CarControllerParams as P
from opendbc.car.vehicle_model import VehicleModel
import opendbc.car.hyundai.carcontroller as ccmod
import openpilot.selfdrive.controls.controlsd as cd
import openpilot.selfdrive.controls.lib.latcontrol_angle as lca
import openpilot.sunnypilot.selfdrive.controls.controlsd_ext as cx
import openpilot.sunnypilot.mads.mads as mm
import openpilot.sunnypilot.selfdrive.selfdrived.lateral_info as li
from openpilot.cereal import log, custom
from openpilot.selfdrive.selfdrived.alertmanager import AlertManager
from openpilot.selfdrive.selfdrived.events import Events, ET
from openpilot.selfdrive.selfdrived.state import StateMachine as SSStateMachine
from openpilot.sunnypilot.selfdrive.selfdrived.events import EventsSP, EVENTS_SP
from openpilot.sunnypilot.mads.mads import ModularAssistiveDrivingSystem

KPH = 1 / 3.6
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
State = custom.ModularAssistiveDrivingSystem.ModularAssistiveDrivingSystemState
EventName = log.OnroadEvent.EventName
EventNameSP = custom.OnroadEventSP.EventName
ButtonType = structs.CarState.ButtonEvent.Type
SafetyModel = structs.CarParams.SafetyModel
AudibleAlert = structs.CarControl.HUDControl.AudibleAlert
AudibleAlertSP = custom.SelfdriveStateSP.AudibleAlert


def _restore(mod, names):
  saved = {k: getattr(mod, k) for k in names}
  yield mod
  for k, v in saved.items():
    setattr(mod, k, v)


@pytest.fixture
def cdk():
  yield from _restore(cd, ("LANE_CENTER_NONFINITE_GUARD", "LAT_MIN_SPEED_HYST_MS"))


@pytest.fixture
def lak():
  yield from _restore(lca, ("SAT_CHECK_UPSTREAM",))


@pytest.fixture
def cxk():
  yield from _restore(cx, ("LAT_GRANT_FROM_PANDA", "PAUSED_WHILE_FOLLOW_BAND"))


@pytest.fixture
def mk():
  yield from _restore(mm, ("LKAS_BUTTON_NEEDS_MAIN", "CRUISE_EDGE_ENABLE", "LATERAL_MISMATCH_DECAY_FRAMES"))


@pytest.fixture
def lik():
  yield from _restore(li, ("PAUSED_PROMPT", "PAUSED_PROMPT_PARKING_MIN_KPH", "FACTORY_LFA_INFO"))


@pytest.fixture
def params():
  yield from _restore(P, ("LAT_GRANT_GATE",))


def test_shipped_values():
  assert cd.LANE_CENTER_NONFINITE_GUARD is True and cd.LAT_MIN_SPEED_HYST_MS == pytest.approx(1.0 / 3.6)
  assert lca.SAT_CHECK_UPSTREAM is True and cx.LAT_GRANT_FROM_PANDA is True and cx.PAUSED_WHILE_FOLLOW_BAND is True
  assert mm.LKAS_BUTTON_NEEDS_MAIN is True and mm.CRUISE_EDGE_ENABLE is True and mm.LATERAL_MISMATCH_DECAY_FRAMES == 5
  assert li.PAUSED_PROMPT is True and li.FACTORY_LFA_INFO is True
  assert (li.PAUSED_PROMPT_MIN_KPH, li.PAUSED_PROMPT_DWELL_S, li.PAUSED_PROMPT_DWELL_CRUISE_S) == (15.0, 1.0, 0.1)
  assert (li.PAUSED_PROMPT_PARKING_MIN_KPH, li.PAUSED_PROMPT_MIN_GAP_S, li.PAUSED_EPISODE_END_S) == (25.0, 20.0, 2.0)
  # the flag values copied into the device-side modules are the car controller's
  assert cx.GW_SOLO_LFA_ACTIVE == ccmod.GW_SOLO | ccmod.GW_LFA_ACTIVE
  assert (li.GW_LFA_ACTIVE, li.GW_SOLO, li.GW_GAVE_UP) == (ccmod.GW_LFA_ACTIVE, ccmod.GW_SOLO, ccmod.GW_GAVE_UP)
  assert li.PR_PARKING == ccmod.PR_PARKING


# ------------------------------------------------------------------ 75j: non-finite model action
def nan_run(n_nan):
  """76 km/h, car 25 cm left of the lane centre until the term has settled (integral at its cap), then n_nan frames of
  a non-finite action, then the model is back. Returns (command before, commands during, commands after, states)."""
  s = H.mk_controls(); s._steer_legacy = False
  s.sm['carState'].vEgo = 21.0
  good = model(centre=0.25)
  bad = model(centre=0.25); bad.action.desiredCurvature = float('nan')

  def run(m, n):
    out = []
    for _ in range(n):
      s.sm['modelV2'] = m
      CC, _ = cd.Controls.state_control(s)
      out.append(float(CC.actuators.curvature))
    return np.array(out)
  k0 = run(good, 6000)[-1]
  st0 = (s._lc_p, s._lc_i)
  during = run(bad, n_nan)
  st1 = (s._lc_p, s._lc_i)
  after = run(good, 300)
  return k0, during, after, st0, st1


def test_nan_action_leaves_the_command_within_the_pre_nan_value():
  k0, during, after, st0, st1 = nan_run(20)                           # 0.2 s = the pure hold
  assert k0 > 2.0e-4 and st0[1] == pytest.approx(cd.LANE_CENTER_I_CAP)   # saturated term (integral at its cap)
  assert np.all(np.abs(during) <= abs(k0) * (1.0 + 1e-9))
  assert during[-1] == pytest.approx(k0, rel=1e-6)                    # held, not grown
  assert st1 == st0                                                   # the two states are kept, not zeroed
  assert np.max(np.abs(after)) <= abs(k0) * 1.02 and after[-1] == pytest.approx(k0, rel=0.02)   # no overshoot on recovery


def test_nan_action_for_one_model_frame_changes_nothing():
  k0, during, after, _, _ = nan_run(5)
  assert np.all(during == pytest.approx(k0, rel=1e-9)) and np.max(np.abs(after)) <= abs(k0) * 1.02


def test_long_nan_ramps_to_straight_and_stays_bounded():
  k0, during, _, _, _ = nan_run(260)
  assert np.max(np.abs(during)) <= abs(k0) * (1.0 + 1e-9) and abs(during[-1]) < 1e-9


def test_nan_guard_kill_switch_reproduces_the_runaway(cdk):
  cdk.LANE_CENTER_NONFINITE_GUARD = False
  k0, during, _, _, _ = nan_run(20)
  assert during[-1] > 5.0 * k0                                        # the pre-75 behaviour: the term re-added every frame


# ------------------------------------------------------------------ 75n: minimum steer speed hysteresis
def lat_trace(v_kph_list, standstill_at=()):
  s = H.mk_controls(); s._steer_legacy = False
  s.CP.minSteerSpeed = 5.0 / 3.6; s.CP.steerAtStandstill = False
  out = []
  for i, v in enumerate(v_kph_list):
    s.sm['carState'].vEgo = v * KPH
    s.sm['carState'].standstill = i in standstill_at
    CC, _ = cd.Controls.state_control(s)
    out.append(bool(CC.latActive))
  return out


def test_lateral_comes_on_above_5_and_goes_off_at_4_kph():
  up = [0.0, 3.0, 4.5, 5.0, 5.1, 6.0]
  assert lat_trace(up) == [False, False, False, False, True, True]
  down = [6.0, 5.0, 4.5, 4.1, 4.0, 3.9, 4.5, 5.0, 5.05]
  assert lat_trace(down) == [True, True, True, True, False, False, False, False, True]


def test_no_flapping_around_5_kph(cdk):
  dither = [6.0] + [5.0 + 0.3 * (-1) ** i for i in range(200)]       # 4.7 / 5.3 km/h alternating
  assert sum(np.diff(np.array(lat_trace(dither), int)) != 0) == 0
  cdk.LAT_MIN_SPEED_HYST_MS = 0.0                                     # kill: the single 5 km/h threshold
  assert sum(np.diff(np.array(lat_trace(dither), int)) != 0) >= 100
  assert lat_trace([6.0, 5.0, 4.9]) == [True, False, False]


def test_standstill_flag_still_switches_lateral_off_and_the_low_floor_is_kept():
  assert lat_trace([6.0, 6.0, 6.0], standstill_at=(1,)) == [True, False, True]
  s = H.mk_controls()                                                 # a car without a minimum steer speed: 0.3 m/s floor
  for v, want in ((1.0, True), (0.31, True), (0.29, False), (0.31, True)):
    s.sm['carState'].vEgo = v
    s.CP.steerAtStandstill = False
    assert bool(cd.Controls.state_control(s)[0].latActive) is want


# ------------------------------------------------------------------ 75k: saturation alert path
def make_lac():
  CP = make_cp()
  lac = lca.LatControlAngle.__new__(lca.LatControlAngle)
  lac.sat_check_min_speed = 5.0; lac._trim_7a6 = False; lac.use_steer_limited_by_safety = True; lac.dt = 0.01
  lac._roll_lp = 0.0; lac._roll_lp_init = False; lac._fb_integ = 0.0; lac._des_slow = 0.0; lac._fb_err_lp = 0.0
  lac.sat_time = 0.0; lac.sat_limit = 0.4; lac.direct = True; lac.request_paused = False
  return lac, VehicleModel(CP)


def sat_frames(n, limited=True, v=20.0, pressed=False, paused=False, curv_limited=False, lac_vm=None):
  """n frames with the car controller not delivering the command (carOutput differs); returns the saturated flags"""
  lac, VM = lac_vm or make_lac()
  out = []
  for i in range(n):
    lac.request_paused = paused(i) if callable(paused) else paused
    CS = types.SimpleNamespace(vEgo=v, steeringAngleDeg=0.0, steeringPressed=pressed(i) if callable(pressed) else pressed)
    _, _, lg = lac.update(True, CS, VM, types.SimpleNamespace(roll=0.0, angleOffsetDeg=0.0), limited, 1e-3, None, curv_limited, 0.2)
    out.append(bool(lg.saturated))
  return out


def test_undelivered_command_raises_saturation_after_the_limit_time(lak):
  sat = sat_frames(100)
  assert not any(sat[:38]) and all(sat[41:])                          # steerLimitTimer 0.4 s
  assert all(sat_frames(100, limited=False, curv_limited=True)[41:])  # the curvature limit counts as well
  assert not any(sat_frames(100, limited=False))
  lak.SAT_CHECK_UPSTREAM = False                                      # kill: the pre-75 call, never from this input
  assert not any(sat_frames(300))
  assert all(sat_frames(100, limited=False, curv_limited=True)[41:])  # (the curvature-limit path worked before as well)
  lak.SAT_CHECK_UPSTREAM = True
  leg = make_lac(); leg[0].direct = False                             # legacy profile: unchanged
  assert not any(sat_frames(300, lac_vm=leg))


def test_no_saturation_while_the_driver_steers_or_at_low_speed():
  assert not any(sat_frames(500, pressed=True))
  assert not any(sat_frames(500, pressed=lambda i: i % 20 < 10))      # hands on, pressing half of the time
  # (shorter presses can fill the timer, as upstream; selfdrived then holds the alert back for 2 s after any press)
  src = open(os.path.join(ROOT, "openpilot/selfdrive/selfdrived/selfdrived.py")).read()
  assert "recent_steer_pressed = (self.sm.frame - self.last_steering_pressed_frame)*DT_CTRL < 2.0" in src
  assert "if lac.active and not recent_steer_pressed and not self.CP.notCar:" in src
  assert not any(sat_frames(500, v=4.9))
  assert not any(sat_frames(500, limited=False))                      # the command is delivered


def test_no_saturation_while_the_request_is_paused_and_the_timer_runs_down():
  assert not any(sat_frames(500, paused=True))
  assert not any(sat_frames(500, paused=True, curv_limited=True))
  lv = make_lac()
  assert sat_frames(60, lac_vm=lv)[-1] and lv[0].sat_time == pytest.approx(0.4)
  rest = sat_frames(60, paused=True, lac_vm=lv)
  assert not any(rest[1:]) and lv[0].sat_time == 0.0                  # paused: the timer runs down
  assert not any(sat_frames(38, lac_vm=lv)) and sat_frames(5, lac_vm=lv)[-1]   # and has to fill again afterwards


class _FSM(dict):
  """SubMaster stand-in with the freshness maps"""
  def __init__(self, data, stale=()):
    super().__init__(data)
    self.alive = {k: k not in stale for k in data}
    self.seen = dict.fromkeys(data, True)
    self.valid = dict.fromkeys(data, True)
    self.logMonoTime = {}
    self.updated = {}

  def all_checks(self, s=None):
    return True


def cs_sp(paused=False, reason=0, gw=0):
  m = custom.CarStateSP.new_message()
  m.lateralControlPaused = paused; m.lateralPausedReason = reason; m.gatewayFlags = gw
  return m


@pytest.mark.parametrize("kw,want", [(dict(), False), (dict(paused=True), True), (dict(reason=ccmod.PR_NOT_GRANTED), True),
                                     (dict(reason=ccmod.PR_WHEEL_OUTRUN), True),
                                     (dict(gw=ccmod.GW_LFA_ACTIVE | ccmod.GW_SOLO), True),
                                     (dict(gw=ccmod.GW_LFA_ACTIVE | ccmod.GW_LFA_ON | ccmod.GW_VALID), False),   # legacy profile: it is meant to steer
                                     (dict(gw=ccmod.GW_LFA_ON | ccmod.GW_VALID | ccmod.GW_SOLO), False),
                                     (dict(gw=ccmod.GW_LFA_ON | ccmod.GW_FOLLOW | ccmod.GW_SOLO), True),          # icon 3, band binding
                                     (dict(gw=ccmod.GW_LFA_ON | ccmod.GW_FOLLOW), False)])
def test_request_paused_from_car_state_sp(kw, want):
  assert cx.ControlsExt.get_request_paused(_FSM({'carStateSP': cs_sp(**kw)})) is want


def test_follow_band_while_the_factory_lfa_releases_does_not_fill_the_saturation_timer(cxk):
  # real car controller: 79 km/h, command 6 deg, wheel 1 deg, icon 2 for 0.8 s, releasing (3) for 1.0 s, then off
  def worst_timer():
    sim = Sim(solo=True); lac, VM = make_lac(); worst = 0.0; flags = set()
    for i in range(500):
      st = 2 if 100 <= i < 180 else (3 if 180 <= i < 280 else 0)
      cmd = 6.0 if i >= 100 else 1.0
      sim.step(v=79.0 * KPH, cmd=cmd, wheel=1.0, lfa_state=st)
      if 185 <= i < 280:
        flags.add(sim.cc.gw_flags & (ccmod.GW_SOLO | ccmod.GW_FOLLOW | ccmod.GW_LFA_ACTIVE))
        assert sim.cc.lat_passive_reason == 0 and not sim.cc.lat_passive_indicated
      lac.request_paused = cx.ControlsExt.get_request_paused(_FSM({'carStateSP': cs_sp(
        paused=sim.cc.lat_passive_indicated, reason=sim.cc.lat_passive_reason, gw=sim.cc.gw_flags)}))
      CS = types.SimpleNamespace(vEgo=79.0 * KPH, steeringAngleDeg=1.0, steeringPressed=False)
      limited = abs(cmd - sim.tx_angle()) > 2.5                        # (controlsd: command vs carOutput)
      lac.update(True, CS, VM, types.SimpleNamespace(roll=0.0, angleOffsetDeg=0.0), limited, 1e-3, None, False, 0.2)
      worst = max(worst, float(lac.sat_time))
      assert lac.sat_time == 0.0 or i >= 280 or not cx.PAUSED_WHILE_FOLLOW_BAND
    assert flags == {ccmod.GW_SOLO | ccmod.GW_FOLLOW}                  # the band binds, nothing else says "passive"
    return worst
  assert worst_timer() < 0.3                                          # only the exit taper counts, well short of 0.4 s
  cxk.PAUSED_WHILE_FOLLOW_BAND = False                                # kill: the timer fills during the hand-over
  assert worst_timer() == pytest.approx(0.4)


def test_request_paused_is_false_without_a_fresh_car_state_sp():
  assert cx.ControlsExt.get_request_paused(_FSM({'carStateSP': cs_sp(paused=True)}, stale=('carStateSP',))) is False
  assert cx.ControlsExt.get_request_paused(H._SM({})) is False        # a SubMaster without the service
  never = _FSM({'carStateSP': cs_sp(paused=True)}); never.seen['carStateSP'] = False
  assert cx.ControlsExt.get_request_paused(never) is False


def test_controlsd_hands_the_paused_flag_to_the_lateral_controller():
  s = H.mk_controls()
  base = dict(s.sm); base['carStateSP'] = cs_sp(paused=True, reason=ccmod.PR_PARKING)
  s.sm = _FSM(base); s.sm.valid['driverAssistance'] = True
  s.LaC.request_paused = False
  cd.Controls.state_control(s)
  assert s.LaC.request_paused is True
  s.sm['carStateSP'] = cs_sp()
  cd.Controls.state_control(s)
  assert s.LaC.request_paused is False


def test_car_controller_passive_states_reach_the_saturation_gate():
  """the real car controller under the parking latch on the road -> card's mirror -> controlsd's gate"""
  sim = Sim(solo=True)
  for _ in range(1500):
    sim.step(v=6.0 * KPH, lfa_state=0)                                # lot crawl from boot: parking latch
  for _ in range(300):
    sim.step(v=25.0 * KPH, cmd=20.0, wheel=5.0, lfa_state=0)
  assert sim.cc.parking_mode_active and not sim.effective_lat_active()
  sp = cs_sp(paused=sim.cc.lat_passive_indicated, reason=sim.cc.lat_passive_reason, gw=sim.cc.gw_flags)
  assert cx.ControlsExt.get_request_paused(_FSM({'carStateSP': sp})) is True
  # the request is not delivered (|command - transmitted| > 2.5 deg) and yet no saturation
  assert abs(20.0 - sim.tx_angle()) > cd.STEER_ANGLE_SATURATION_THRESHOLD
  assert not any(sat_frames(300, v=25.0 * KPH, paused=True))


# ------------------------------------------------------------------ 75l: lateral grant
def ps(lat=True, mode=SafetyModel.hyundaiCanfd, rx_invalid=False):
  return types.SimpleNamespace(controlsAllowedLateral=lat, safetyModel=mode, safetyRxChecksInvalid=rx_invalid)


def test_lat_granted_follows_the_pandas_lateral_permission(cxk):
  g = cx.ControlsExt.get_lat_granted
  assert g(_FSM({'pandaStates': [ps(True)]})) is True
  assert g(_FSM({'pandaStates': [ps(False)]})) is False
  assert g(_FSM({'pandaStates': [ps(True), ps(False)]})) is False     # every panda in a car safety mode
  cxk.LAT_GRANT_FROM_PANDA = False
  assert g(_FSM({'pandaStates': [ps(False)]})) is True                # kill switch


def test_lat_granted_cannot_deadlock_on_a_missing_or_stale_panda_state():
  g = cx.ControlsExt.get_lat_granted
  assert g(_FSM({'pandaStates': [ps(False)]}, stale=('pandaStates',))) is True
  never = _FSM({'pandaStates': [ps(False)]}); never.seen['pandaStates'] = False
  assert g(never) is True
  assert g(_FSM({'pandaStates': []})) is True
  assert g(H._SM({})) is True                                         # a SubMaster without the service
  for mode in (SafetyModel.silent, SafetyModel.noOutput):             # not in a car safety mode: as the MADS check
    assert g(_FSM({'pandaStates': [ps(False, mode=mode)]})) is True


def ext_sm(panda_states, stale=()):
  model_v2 = log.ModelDataV2.new_message()
  return _FSM({'radarState': log.RadarState.new_message(), 'selfdriveStateSP': custom.SelfdriveStateSP.new_message(),
               'modelV2': model_v2, 'pandaStates': panda_states, 'carState': types.SimpleNamespace(canValid=True)}, stale=stale)


def test_state_control_ext_fills_the_field_and_it_reaches_the_car_controller():
  from openpilot.selfdrive.car.helpers import convert_carControlSP
  ext = types.SimpleNamespace(get_lead_data=cx.ControlsExt.get_lead_data, get_lat_granted=cx.ControlsExt.get_lat_granted,
                              _steer_legacy=False)
  for lat, stale, want in ((True, (), True), (False, (), False), (False, ('pandaStates',), True)):
    msg = cx.ControlsExt.state_control_ext(ext, ext_sm([ps(lat)], stale=stale))
    assert msg.latGranted is want
    assert convert_carControlSP(msg.as_reader()).latGranted is want
  assert 'pandaStates' in open(os.path.join(ROOT, "openpilot/sunnypilot/selfdrive/controls/controlsd_ext.py")).read().split(
    "self.sm_services_ext = ")[1].split("\n")[0]


# the real panda safety code with the MADS heartbeat, built on the fly; skipped where there is no C compiler
@pytest.fixture(scope="module")
def panda():
  cc_bin = shutil.which("cc")
  if cc_bin is None:
    pytest.skip("no C compiler")
  from opendbc.safety.tests.libsafety import libsafety_py
  from opendbc.safety.tests.common import CANPackerSafety
  from opendbc.safety import ALTERNATIVE_EXPERIENCE
  from opendbc.car.hyundai.values import HyundaiSafetyFlags
  from opendbc.sunnypilot.car.hyundai.values import ANGLE_STEERING_MODEL_BY_CAR, encode_angle_model_id
  root = os.path.join(ROOT, "opendbc_repo")
  tmp = tempfile.mkdtemp()
  try:
    obj, lib = os.path.join(tmp, "safety.os"), os.path.join(tmp, "libsafety.so")
    try:
      subprocess.check_call([cc_bin, "-fPIC", "-nostdlib", "-fno-builtin", "-std=gnu11", "-Wno-pointer-to-int-cast", "-O0", "-I", root,
                             "-c", os.path.join(root, "opendbc/safety/tests/libsafety/safety.c"), "-o", obj],
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
      subprocess.check_call([cc_bin, "-shared", obj, "-o", lib], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except (subprocess.CalledProcessError, OSError):
      pytest.skip("the safety library does not build here")
    libsafety_py.load(lib)
    S = libsafety_py.libsafety
    packer = CANPackerSafety("hyundai_canfd_generated")
    param = (HyundaiSafetyFlags.EV_GAS | HyundaiSafetyFlags.CANFD_LKA_STEER_MSG | HyundaiSafetyFlags.CANFD_LKA_STEER_MSG_ALT |
             HyundaiSafetyFlags.CANFD_ALT_BUTTONS | HyundaiSafetyFlags.CCNC)

    def init():
      S.set_current_safety_param_sp(encode_angle_model_id(ANGLE_STEERING_MODEL_BY_CAR["HYUNDAI_IONIQ_6_N"]))
      S.set_safety_hooks(structs.CarParams.SafetyModel.hyundaiCanfd, int(param))
      S.init_tests()
      S.set_alternative_experience(ALTERNATIVE_EXPERIENCE.ENABLE_MADS)
      S.mads_apply_alternative_experience(ALTERNATIVE_EXPERIENCE.ENABLE_MADS)

    yield S, packer, init
  finally:
    shutil.rmtree(tmp, ignore_errors=True)


def panda_chain(panda, mads_on, main_on, n, v=60.0 * KPH, cmd=6.0, lat_fn=None):
  """The whole chain at 100 Hz: the real panda (1 Hz heartbeat check, its lateral permission published as a 10 Hz
  pandaStates), controlsd's get_lat_granted and the real car controller, whose frames go back into the panda.
  mads_on(i) / main_on(i): selfdrived's MADS state and SCC main per frame. Returns per frame
  (lateral requested, panda allows, granted flag, frame active, frame rejected, passive reason).
  lat_fn(mads, main): what controlsd requests (default: MADS on and main on)."""
  S, packer, init = panda
  init()
  sim = Sim(solo=True)
  t = 0; wheel = 0.0; rej_last = False; published = [ps(True)]; out = []
  for i in range(-300, n):
    j = max(i, 0)
    mads, main = (True, True) if i < 0 else (bool(mads_on(j)), bool(main_on(j)))
    S.set_heartbeat_engaged_mads(mads)
    if i % 100 == 0:
      S.mads_heartbeat_engaged_check()                                # the panda evaluates the heartbeat at 1 Hz
    S.safety_rx_hook(packer.make_can_msg_safety("SCC_CONTROL", 1, {"MainMode_ACC": int(main), "ACCMode": 0}))
    S.safety_rx_hook(packer.make_can_msg_safety("WHEEL_SPEEDS", 1, {f"WHL_Spd{p}Val": v * 3.6 for p in ("FL", "FR", "RL", "RR")}))
    S.safety_rx_hook(packer.make_can_msg_safety("MDPS", 1, {"STEERING_ANGLE_2": wheel}))
    if i % 10 == 0:                                                   # pandaStates: 10 Hz
      published = [ps(bool(S.get_controls_allowed_lateral()))]
    granted = cx.ControlsExt.get_lat_granted(_FSM({'pandaStates': published}))
    lat = bool(lat_fn(mads, main)) if lat_fn is not None else (mads and main)
    sim.step(v=v, cmd=cmd if i >= 0 else 0.0, wheel=wheel, lat_active=lat, lfa_state=0, lat_granted=granted, tx_rejected=rej_last)
    t += 10000; S.set_timer(t)
    ok = bool(S.safety_tx_hook(packer.make_can_msg_safety("LKAS_ALT", 0, sim.lkas_alt())))
    rej_last = not ok
    if ok and sim.effective_lat_active():
      wheel += (sim.tx_angle() - wheel) * 0.07
    if i >= 0:
      out.append((lat, bool(S.get_controls_allowed_lateral()), granted, sim.effective_lat_active(), not ok,
                  sim.cc.lat_passive_reason))
  return np.array(out)


@pytest.mark.parametrize("phase", [5, 35, 65, 95])
def test_button_re_enable_waits_for_the_panda_without_a_rejected_frame(panda, params, cxk, phase):
  """MADS off for 5 s (LFA button), on again `phase` frames after one of the panda's 1 Hz heartbeat checks"""
  on_at = 500 + phase
  o = panda_chain(panda, lambda i: i < 0 or i >= on_at, lambda i: True, 800)
  assert not o[400:on_at, 1].any()                                    # the panda dropped lateral 2-3 s after the switch-off
  grant = on_at + int(np.argmax(o[on_at:, 1]))
  assert 0 < grant - on_at <= 100                                     # re-granted at its next 1 Hz check
  assert o[:, 4].sum() == 0                                           # no frame rejected
  assert not o[on_at:grant, 3].any()                                  # passive frames while waiting
  first = on_at + int(np.argmax(o[on_at:, 3]))
  assert 0 <= first - grant <= 11                                     # steering starts within ~0.1 s of the grant
  assert o[first:, 3].all()
  cxk.LAT_GRANT_FROM_PANDA = False                                    # kill: active frames into the closed panda
  k = panda_chain(panda, lambda i: i < 0 or i >= on_at, lambda i: True, 800)
  assert k[on_at:, 4].sum() >= 100 - phase - 2


def test_normal_engage_on_the_main_edge_is_delayed_by_at_most_a_tenth(panda, params, cxk):
  for on_at in (203, 207, 211):                                       # anywhere inside a pandaStates period
    o = panda_chain(panda, lambda i: i >= on_at, lambda i: i >= on_at, 400)   # SCC main on -> MADS enables with it
    first = int(np.argmax(o[:, 3]))
    assert o[on_at, 1] and 0 <= first - on_at <= 11 and o[first:, 3].all()
    assert o[:, 4].sum() == 0


def test_main_off_never_grants_and_never_sends_an_active_frame(panda, params, cxk):
  """the old LFA-button-with-main-off case seen from the car controller: passive frames only, nothing rejected"""
  o = panda_chain(panda, lambda i: True, lambda i: False, 400)
  assert not o[50:, 1].any() and not o[50:, 3].any() and o[50:, 4].sum() == 0
  # lateral requested all the same (MADS enabled by the LFA button with main off, as before 75m): the grant gate
  # keeps the frames passive, nothing is rejected
  o = panda_chain(panda, lambda i: True, lambda i: False, 400, lat_fn=lambda mads, main: mads)
  assert o[50:, 0].all() and not o[50:, 1].any() and not o[50:, 2].any()
  assert not o[50:, 3].any() and o[50:, 4].sum() == 0 and np.all(o[50:, 5] == ccmod.PR_NOT_GRANTED)
  cxk.LAT_GRANT_FROM_PANDA = False                                    # kill (controls side): active frames, all rejected
  k = panda_chain(panda, lambda i: True, lambda i: False, 400, lat_fn=lambda mads, main: mads)
  assert k[50:, 4].sum() >= 300
  cxk.LAT_GRANT_FROM_PANDA = True
  params.LAT_GRANT_GATE = False                                       # kill (car side)
  k = panda_chain(panda, lambda i: True, lambda i: False, 400, lat_fn=lambda mads, main: mads)
  assert k[50:, 4].sum() >= 300


# ------------------------------------------------------------------ MADS (75l counter, 75m)
CCNC_FLAGS = (HyundaiFlags.CANFD | HyundaiFlags.CCNC | HyundaiFlags.CANFD_LKA_STEER_MSG_ALT).value


def mk_mads(flags=CCNC_FLAGS, uem=True, main_cruise_allowed=True):
  FakeParams.reset()
  p = FakeParams()
  p.put_bool('Mads', True); p.put_bool('MadsMainCruiseAllowed', main_cruise_allowed)
  p.put_bool('MadsUnifiedEngagementMode', uem)
  CP = structs.CarParams(); CP.brand = 'hyundai'; CP.flags = flags; CP.passive = False
  sd = types.SimpleNamespace(CP=CP, CP_SP=structs.CarParamsSP(), params=p, enabled=False, initialized=True,
                             events=Events(), events_sp=EventsSP(), state_machine=SSStateMachine(),
                             CS_prev=structs.CarState(), sm={'pandaStates': [ps(True)]}, AM=AlertManager(), frame=0)
  return ModularAssistiveDrivingSystem(sd), sd


def mk_cs(main=True, cruise=False, v=15.0, lkas=False):
  CS = structs.CarState()
  CS.cruiseState.available = main; CS.cruiseState.enabled = cruise; CS.vEgo = v
  if lkas:
    be = structs.CarState.ButtonEvent(); be.type = ButtonType.lkas; be.pressed = True
    CS.buttonEvents = [be]
  return CS


def frame(mads, sd, CS, events=()):
  """one selfdrived cycle: events, the real selfdrive state machine, MADS, alerts. Returns the current alert."""
  sd.events.clear(); sd.events_sp.clear()
  for e in events:
    sd.events.add(e)
  sd.state_machine.update(sd.events)
  mads.update(CS)
  sd.frame += 1
  types_ = sd.state_machine.current_alert_types
  sd.AM.add_many(sd.frame, sd.events.create_alerts(types_, []) + sd.events_sp.create_alerts(types_, [None, CS, {}, True, 0, None]))
  sd.AM.process_alerts(sd.frame, set())
  sd.CS_prev = CS
  return sd.AM.current_alert


def enabled_then_off_by_button():
  mads, sd = mk_mads()
  frame(mads, sd, mk_cs())                                            # boot enable with main on
  assert mads.enabled
  frame(mads, sd, mk_cs(lkas=True))
  assert not mads.enabled
  return mads, sd


def test_platform_detection():
  assert mk_mads()[0].main_gated and mk_mads()[0].allow_always
  assert not mk_mads(flags=HyundaiFlags.CANFD.value)[0].main_gated
  assert not mk_mads(flags=(HyundaiFlags.CANFD | HyundaiFlags.CCNC).value)[0].main_gated
  assert not mk_mads(flags=(HyundaiFlags.CANFD | HyundaiFlags.CANFD_LKA_STEER_MSG_ALT).value)[0].main_gated


def test_lfa_button_with_main_off_is_refused_with_a_text(mk):
  mads, sd = mk_mads()
  sd.sm['pandaStates'] = [ps(False)]                                  # main off: the panda never grants
  for _ in range(20):
    frame(mads, sd, mk_cs(main=False))
  alert = frame(mads, sd, mk_cs(main=False, lkas=True))
  assert not mads.enabled and mads.state_machine.state == State.disabled
  assert sd.events_sp.has(EventNameSP.cruiseMainRequired) and not sd.events_sp.has(EventNameSP.lkasEnable)
  assert "Switch Cruise Main On" in (alert.alert_text_1, alert.alert_text_2) and alert.audible_alert == AudibleAlert.refuse
  for _ in range(320):                                                # the text stays 3 s, MADS stays off, no mismatch
    last = frame(mads, sd, mk_cs(main=False))
    assert not mads.enabled and mads.lateral_mismatch_counter == 0
    assert not sd.events_sp.has(EventNameSP.controlsMismatchLateral)
  assert last.alert_text_1 == "" and last.alert_text_2 == ""


def test_lfa_button_with_main_off_kill_switch_reproduces_the_mismatch(mk):
  mk.LKAS_BUTTON_NEEDS_MAIN = False
  mads, sd = mk_mads()
  sd.sm['pandaStates'] = [ps(False)]
  frame(mads, sd, mk_cs(main=False))
  frame(mads, sd, mk_cs(main=False, lkas=True))
  assert mads.enabled                                                 # the pre-75 behaviour ...
  tripped = None
  for i in range(260):
    alert = frame(mads, sd, mk_cs(main=False))
    if tripped is None and sd.events_sp.has(EventNameSP.controlsMismatchLateral):
      tripped = i; text = (alert.alert_text_1, alert.alert_text_2)
  assert tripped is not None and 195 <= tripped <= 205 and "Controls Mismatch: Lateral" in text   # ... 2 s later the red alert
  assert not mads.enabled


def test_lfa_button_with_main_on_is_unchanged():
  mads, sd = enabled_then_off_by_button()
  frame(mads, sd, mk_cs(lkas=True))
  assert mads.enabled and not sd.events_sp.has(EventNameSP.cruiseMainRequired)
  frame(mads, sd, mk_cs(lkas=True))
  assert not mads.enabled                                             # and off again


def test_other_platforms_keep_the_button_without_main():
  mads, sd = mk_mads(flags=HyundaiFlags.CANFD.value)
  frame(mads, sd, mk_cs(main=False))
  frame(mads, sd, mk_cs(main=False, lkas=True))
  assert mads.enabled and not sd.events_sp.has(EventNameSP.cruiseMainRequired)


def test_main_on_edge_and_boot_enable_still_work():
  mads, sd = mk_mads()
  for _ in range(50):
    frame(mads, sd, mk_cs(main=False))
    assert not mads.enabled
  frame(mads, sd, mk_cs(main=True))
  assert mads.enabled


def test_paused_mads_does_not_resume_while_main_is_off(mk):
  def paused_main_off():
    mads, sd = mk_mads()
    frame(mads, sd, mk_cs())
    mads.state_machine.state = State.paused                           # e.g. brake / gear pause
    sd.CS_prev = mk_cs(main=False)
    frame(mads, sd, mk_cs(main=False))
    return mads, sd
  mads, sd = paused_main_off()
  assert mads.state_machine.state == State.paused
  frame(mads, sd, mk_cs(main=True))                                   # main on: resumes
  assert mads.state_machine.state == State.enabled
  mk.LKAS_BUTTON_NEEDS_MAIN = False
  assert paused_main_off()[0].state_machine.state == State.enabled    # pre-75: active without a grant


def test_cruise_engagement_switches_lateral_back_on(mk):
  mads, sd = enabled_then_off_by_button()
  for _ in range(300):
    frame(mads, sd, mk_cs())
  assert not mads.enabled                                             # stays off by itself
  alert = frame(mads, sd, mk_cs(cruise=True))
  assert mads.enabled and mads.active and sd.events_sp.has(EventNameSP.lkasEnable)
  assert ET.ENABLE in sd.state_machine.current_alert_types and alert.audible_alert == AudibleAlert.engage
  frame(mads, sd, mk_cs(cruise=True))
  assert not sd.events_sp.has(EventNameSP.lkasEnable)                 # the edge only
  mk.CRUISE_EDGE_ENABLE = False
  mads, sd = enabled_then_off_by_button()
  frame(mads, sd, mk_cs(cruise=True))
  assert not mads.enabled                                             # kill switch


def test_cruise_edge_needs_main_and_unified_engagement_and_this_platform(mk):
  mk.LKAS_BUTTON_NEEDS_MAIN = False                                   # (so that only the cruise-edge rule is looked at)
  mads, sd = mk_mads()
  frame(mads, sd, mk_cs(main=False))
  frame(mads, sd, mk_cs(main=False, cruise=True))                     # a cruise edge without main: never
  assert not mads.enabled
  mads, sd = mk_mads(uem=False)
  frame(mads, sd, mk_cs()); frame(mads, sd, mk_cs(lkas=True))
  frame(mads, sd, mk_cs(cruise=True))
  assert not mads.enabled                                             # Unified Engagement Mode off: 'cruise only' is kept
  mads, sd = mk_mads(flags=HyundaiFlags.CANFD.value)
  frame(mads, sd, mk_cs()); frame(mads, sd, mk_cs(lkas=True))
  frame(mads, sd, mk_cs(cruise=True))
  assert not mads.enabled                                             # other platforms: their own engagement path


def test_cruise_edge_respects_no_entry_and_does_nothing_while_enabled():
  mads, sd = enabled_then_off_by_button()
  frame(mads, sd, mk_cs(cruise=True), events=[EventName.doorOpen])    # a no-entry condition
  assert mads.state_machine.state != State.enabled and not mads.active
  mads, sd = mk_mads()
  frame(mads, sd, mk_cs())
  frame(mads, sd, mk_cs(cruise=True))
  assert mads.enabled and not sd.events_sp.has(EventNameSP.lkasEnable)


def test_cruise_edge_is_held_back_when_the_panda_would_not_grant():
  mads, sd = enabled_then_off_by_button()
  sd.sm['pandaStates'] = [ps(False, rx_invalid=True)]
  frame(mads, sd, mk_cs(cruise=True))
  assert not mads.enabled                                             # RX checks invalid
  # after a mismatch trip only a main-on edge re-arms
  mads, sd = mk_mads()
  frame(mads, sd, mk_cs())
  sd.sm['pandaStates'] = [ps(False)]
  for _ in range(205):
    frame(mads, sd, mk_cs())
  assert not mads.enabled and mads.cruise_edge_blocked
  sd.sm['pandaStates'] = [ps(True)]
  frame(mads, sd, mk_cs(cruise=True))
  assert not mads.enabled
  frame(mads, sd, mk_cs(main=False)); frame(mads, sd, mk_cs(main=True))   # main off / on
  assert not mads.cruise_edge_blocked


def gap(mads, sd, frames, allowed):
  sd.sm['pandaStates'] = [ps(allowed)]
  for _ in range(frames):
    frame(mads, sd, mk_cs())


def test_mismatch_counter_decays_while_the_panda_agrees(mk):
  mads, sd = mk_mads(); frame(mads, sd, mk_cs())
  gap(mads, sd, 60, False)
  assert mads.lateral_mismatch_counter == 60                          # a 0.6 s grant wait
  gap(mads, sd, 100, True)
  assert mads.lateral_mismatch_counter == 40                          # one count per 5 agreeing frames
  gap(mads, sd, 6000, True)
  assert mads.lateral_mismatch_counter == 0
  gap(mads, sd, 190, False)                                           # a later 1.9 s gap alone does not trip
  assert mads.enabled and mads.lateral_mismatch_counter == 190
  gap(mads, sd, 12, False)
  assert not mads.enabled                                             # 2 s in a row still does
  mk.LATERAL_MISMATCH_DECAY_FRAMES = 0                                # kill: the leftovers add up as before
  mads, sd = mk_mads(); frame(mads, sd, mk_cs())
  gap(mads, sd, 60, False); gap(mads, sd, 6000, True)
  assert mads.lateral_mismatch_counter == 60
  gap(mads, sd, 139, False)
  assert mads.enabled
  gap(mads, sd, 3, False)
  assert not mads.enabled


def test_a_flapping_grant_still_trips():
  mads, sd = mk_mads(); frame(mads, sd, mk_cs())
  for _ in range(400):                                                # lateral permission off 30 % of the time
    gap(mads, sd, 3, False); gap(mads, sd, 7, True)
    if not mads.enabled:
      break
  assert not mads.enabled
  # agreeing frames are counted in a row: 1 off / 4 on never reaches the 5 agreeing frames of a decay step
  mads, sd = mk_mads(); frame(mads, sd, mk_cs())
  for _ in range(100):
    gap(mads, sd, 1, False); gap(mads, sd, 4, True)
  assert mads.enabled and mads.lateral_mismatch_counter == 100 and mads.lateral_agree_frames == 4
  for _ in range(105):
    gap(mads, sd, 1, False); gap(mads, sd, 4, True)
  assert not mads.enabled
  mads, sd = mk_mads(); frame(mads, sd, mk_cs())                      # 2 off / 9 on: one step back per cycle, still growing
  for _ in range(50):
    gap(mads, sd, 2, False); gap(mads, sd, 9, True)
  assert mads.enabled and mads.lateral_mismatch_counter == 50


# ------------------------------------------------------------------ 75o: driver information
def info_cs(v_kph=30.0, pressed=False, cruise=False):
  return types.SimpleNamespace(vEgo=v_kph * KPH, steeringPressed=pressed, cruiseState=types.SimpleNamespace(enabled=cruise))


def info_run(seq, L=None):
  """seq: (frames, dict(paused, reason, gw, v_kph, pressed, cruise, lat, fresh)); returns [(frame, event)] and the object"""
  L = L or li.LateralInfo(); ev = []; i = 0
  for n, kw in seq:
    sp = None if not kw.get("fresh", True) else cs_sp(paused=kw.get("paused", False), reason=kw.get("reason", ccmod.PR_SCENARIO),
                                                      gw=kw.get("gw", 0))
    for _ in range(n):
      for e in L.update(info_cs(kw.get("v_kph", 30.0), kw.get("pressed", False), kw.get("cruise", False)), sp, kw.get("lat", True)):
        ev.append((i, e))
      i += 1
  return ev, L


def prompts(seq):
  return [i for i, e in info_run(seq)[0] if e == EventNameSP.lateralPaused]


def test_prompt_after_one_second_passive_hands_off():
  assert prompts([(500, dict(paused=True))]) == [99]                  # once, 1.0 s into the episode
  assert prompts([(95, dict(paused=True)), (300, dict())]) == []      # shorter than the dwell: nothing
  assert prompts([(500, dict(paused=False))]) == []


def test_prompt_at_once_with_cruise_engaged():
  assert prompts([(200, dict()), (500, dict(paused=True, cruise=True))]) == [209]
  # cruise is engaged while already passive (the logged departure road): 0.1 s after the engagement
  assert prompts([(50, dict(paused=True)), (500, dict(paused=True, cruise=True))]) == [50]


def test_never_while_the_driver_is_steering():
  assert prompts([(3000, dict(paused=True, pressed=True))]) == []
  assert prompts([(3000, dict(paused=True, pressed=True, cruise=True))]) == []
  seq = [(40, dict(paused=True, pressed=True)), (50, dict(paused=True))] * 30   # pressing every 0.9 s
  assert prompts(seq) == []
  # let go: one second after the last press
  assert prompts([(300, dict(paused=True, pressed=True)), (400, dict(paused=True))]) == [300 + 99]
  assert prompts([(300, dict(paused=True, pressed=True, cruise=True)), (400, dict(paused=True, cruise=True))]) == [300 + 99]


def test_level_based_an_episode_that_starts_slow_prompts_once_above_15_kph():
  assert prompts([(1000, dict(paused=True, v_kph=14.0))]) == []
  assert prompts([(1000, dict(paused=True, v_kph=8.0)), (500, dict(paused=True, v_kph=16.0))]) == [1000 + 99]


def test_one_prompt_per_episode_and_rate_limited():
  flicker = [(150, dict(paused=True)), (50, dict()), (150, dict(paused=True)), (150, dict()), (300, dict(paused=True))]
  assert prompts(flicker) == [99]                                     # gaps under 2 s are the same episode
  two = [(150, dict(paused=True)), (300, dict()), (400, dict(paused=True))]
  assert prompts(two) == [99]                                         # a new episode 3 s later: inside the 20 s gap
  assert prompts([(6000, dict(paused=True))]) == [99]                 # one 60 s episode (the latch on the road): one prompt
  assert prompts([(6000, dict(paused=True, cruise=True))]) == [9]
  far = [(150, dict(paused=True)), (2100, dict()), (400, dict(paused=True))]
  assert prompts(far) == [99, 150 + 2100 + 99]
  # worst case, the flag toggling for ten minutes: at most one prompt per 20 s
  storm = [(150, dict(paused=True)), (250, dict())] * 150
  p = prompts(storm)
  assert len(p) <= 31 and min(np.diff(p)) >= 2000


def test_parking_latch_needs_cruise_or_road_speed(lik):
  park = dict(paused=True, reason=ccmod.PR_PARKING)
  assert prompts([(1000, dict(park, v_kph=19.0))]) == []              # inside the car park
  assert prompts([(300, dict(park, v_kph=19.0)), (300, dict(park, v_kph=27.0))]) == [300 + 99]
  assert prompts([(300, dict(park, v_kph=19.0)), (300, dict(park, v_kph=19.0, cruise=True))]) == [300 + 9]
  lik.PAUSED_PROMPT_PARKING_MIN_KPH = 15.0
  assert prompts([(1000, dict(park, v_kph=19.0))]) == [99]


def test_no_car_state_sp_no_prompt_and_the_kill_switch(lik):
  assert info_run([(1000, dict(paused=True, fresh=False, gw=ccmod.GW_LFA_ACTIVE | ccmod.GW_SOLO))])[0] == []
  lik.PAUSED_PROMPT = False
  assert prompts([(1000, dict(paused=True))]) == []


def test_factory_lfa_active_and_hold_off_paused(lik):
  act = dict(gw=ccmod.GW_LFA_ACTIVE | ccmod.GW_LFA_ON | ccmod.GW_VALID | ccmod.GW_SOLO)
  ev, _ = info_run([(90, act), (100, dict())])                        # a normal hold-off (under 1 s): nothing
  assert ev == []
  ev, _ = info_run([(300, act), (100, dict())])
  f = [i for i, e in ev if e == EventNameSP.factoryLfaActive]
  assert f == list(range(99, 300))                                    # shown from 1.0 s until it ends
  assert info_run([(300, dict(act, lat=False))])[0] == []             # lateral off: not openpilot's turn anyway
  assert info_run([(300, dict(gw=act["gw"] & ~ccmod.GW_SOLO))])[0] == []   # legacy profile: it is meant to steer
  ev, _ = info_run([(100, dict()), (1200, dict(gw=ccmod.GW_GAVE_UP)), (100, dict()), (50, dict(gw=ccmod.GW_GAVE_UP))])
  assert [i for i, e in ev if e == EventNameSP.lfaHoldOffPaused] == [100, 1400]   # once per pause
  lik.FACTORY_LFA_INFO = False
  assert info_run([(300, dict(act, gw=act["gw"] | ccmod.GW_GAVE_UP))])[0] == []


def alert_for(event, et, CS, reason=0):
  ev = EventsSP(); ev.add(event)
  sm = {'carStateSP': cs_sp(reason=reason)}
  return ev.create_alerts([et], [None, CS, sm, True, 0, None])[0]


def test_alert_texts_and_sounds():
  a = alert_for(EventNameSP.lateralPaused, ET.WARNING, info_cs(), reason=ccmod.PR_PARKING)
  assert (a.alert_text_1, a.alert_text_2) == ("Steering Paused", "Parking Mode")
  assert a.audible_alert == AudibleAlertSP.promptSingleLow and a.visual_alert == structs.CarControl.HUDControl.VisualAlert.none
  loud = alert_for(EventNameSP.lateralPaused, ET.WARNING, info_cs(cruise=True), reason=ccmod.PR_SCENARIO)
  assert loud.audible_alert == AudibleAlert.prompt and loud.alert_text_2 == "Low Speed"
  assert alert_for(EventNameSP.lateralPaused, ET.WARNING, info_cs(), reason=99).alert_text_2 == "Steer Manually"
  f = alert_for(EventNameSP.factoryLfaActive, ET.WARNING, info_cs())
  assert f.alert_text_1 == "Factory LFA Active" and f.audible_alert == AudibleAlert.none
  h = alert_for(EventNameSP.lfaHoldOffPaused, ET.PERMANENT, info_cs())
  assert h.alert_text_1 == "LFA Hold-Off Paused" and h.audible_alert == AudibleAlert.none
  for e in (EventNameSP.cruiseMainRequired, EventNameSP.lateralPaused, EventNameSP.factoryLfaActive, EventNameSP.lfaHoldOffPaused):
    assert e in EVENTS_SP
    for et in EVENTS_SP[e]:                                           # display only: none of them changes the state machines
      assert et in (ET.WARNING, ET.PERMANENT)
  names = custom.OnroadEventSP.EventName.schema.enumerants
  assert (names["cruiseMainRequired"], names["lateralPaused"], names["factoryLfaActive"], names["lfaHoldOffPaused"]) == (25, 26, 27, 28)
  assert names["laneChangeRoadEdge"] == 24


def test_one_frame_prompt_is_shown_for_three_seconds_and_sounds_once():
  AM = AlertManager(); ev = EventsSP(); shown = []
  for f in range(1, 500):
    ev.clear()
    if f == 10:
      ev.add(EventNameSP.lateralPaused)
    AM.add_many(f, ev.create_alerts([ET.PERMANENT, ET.WARNING], [None, info_cs(), {'carStateSP': cs_sp(reason=ccmod.PR_PARKING)}, True, 0, None]))
    AM.process_alerts(f, set())
    shown.append(AM.current_alert.alert_text_1 == "Steering Paused")
  assert sum(shown) == 301 and shown[9] and not shown[8] and not shown[320]
  # lateral off again (no WARNING alert type): the text is cleared at once
  AM.process_alerts(11, {ET.WARNING})
  assert AM.current_alert.alert_text_1 == ""


def test_real_car_controller_parking_latch_on_the_road_with_cruise_prompts():
  """the logged case: left the car park under the parking latch, cruise engaged at 25 km/h, nobody steers"""
  sim = Sim(solo=True); L = li.LateralInfo(); ev = []

  def go(n, v_kph, cruise):
    for _ in range(n):
      sim.step(v=v_kph * KPH, lfa_state=0, cruise_enabled=cruise)
      sp = cs_sp(paused=sim.cc.lat_passive_indicated, reason=sim.cc.lat_passive_reason, gw=sim.cc.gw_flags)
      for e in L.update(info_cs(v_kph, cruise=cruise), sp, True):
        ev.append((e, sim.cc.lat_passive_reason))
  go(1500, 6.0, False); go(600, 19.0, False)                          # the lot: latched, no prompt
  assert sim.cc.parking_mode_active and ev == []
  go(300, 25.0, True)                                                 # on the road, cruise engaged
  assert sim.cc.parking_mode_active and not sim.effective_lat_active()
  assert ev == [(EventNameSP.lateralPaused, ccmod.PR_PARKING)]
  go(900, 60.0, True)                                                 # the latch releases: steering, nothing more
  assert sim.effective_lat_active() and len(ev) == 1


def test_selfdrived_wiring():
  src = open(os.path.join(ROOT, "openpilot/selfdrive/selfdrived/selfdrived.py")).read()
  assert "ignore += ['carStateSP']" in src                            # can never raise commIssue
  assert src.index("ignore += ['carStateSP']") < src.index("ignore_liveness = ignore + ['deviceState']")
  assert "'longitudinalPlanSP', 'carStateSP']" in src
  assert "self.lateral_info.update(CS, cs_sp, self.sm['carControl'].latActive)" in src
  assert "self.sm.alive['carStateSP'] and self.sm.seen['carStateSP']" in src


def test_selfdrived_lines_feed_the_events_from_a_fresh_car_state_sp():
  # the lines added to SelfdriveD.update_events, run against a stub self with a real LateralInfo and EventsSP
  src = open(os.path.join(ROOT, "openpilot/selfdrive/selfdrived/selfdrived.py")).read()
  start = src.index("    cs_sp = self.sm['carStateSP'] if ")
  block = src[start:src.index("\n\n", start)]
  assert block.count("\n") == 2 and "self.events_sp.add(e)" in block
  code = compile("\n".join(ln[4:] for ln in block.splitlines()), "selfdrived_lines", "exec")

  def run_lines(n, sp, stale=(), seen=True, lat=True):
    sm = _FSM({'carStateSP': sp, 'carControl': types.SimpleNamespace(latActive=lat)}, stale=stale)
    sm.seen['carStateSP'] = seen
    me = types.SimpleNamespace(sm=sm, lateral_info=li.LateralInfo(), events_sp=EventsSP()); names = []
    for _ in range(n):
      me.events_sp.clear()
      exec(code, {"self": me, "CS": info_cs()})
      names += list(me.events_sp.names)
    return names
  paused = cs_sp(paused=True, reason=ccmod.PR_SCENARIO)
  assert run_lines(300, paused) == [EventNameSP.lateralPaused]
  assert run_lines(300, paused, stale=('carStateSP',)) == [] and run_lines(300, paused, seen=False) == []
  assert run_lines(300, cs_sp()) == []
  lfa = cs_sp(gw=ccmod.GW_SOLO | ccmod.GW_LFA_ACTIVE)
  assert run_lines(150, lfa) == [EventNameSP.factoryLfaActive] * 51 and run_lines(150, lfa, lat=False) == []


def test_hands_on_detection_and_attention_warning_are_not_touched():
  """nothing of the driver information goes to the car: the new modules do not send, and the LKAS_ALT of a passive
  frame is the same with and without the prompt logic (it lives in selfdrived only)"""
  for f in ("openpilot/sunnypilot/selfdrive/selfdrived/lateral_info.py",):
    src = open(os.path.join(ROOT, f)).read()
    assert "sendcan" not in src and "PubMaster" not in src and "hudControl" not in src
  for e in (EventNameSP.lateralPaused, EventNameSP.factoryLfaActive, EventNameSP.lfaHoldOffPaused):
    for a in EVENTS_SP[e].values():
      al = a(None, info_cs(), {'carStateSP': cs_sp()}, True, 0, None) if callable(a) else a
      assert al.visual_alert == structs.CarControl.HUDControl.VisualAlert.none


# ------------------------------------------------------------------ UI status
STUBS = ("pyray", "openpilot.system.ui", "openpilot.selfdrive.ui.sunnypilot.layouts", "openpilot.sunnypilot.models",
         "openpilot.sunnypilot.sunnylink")


class _StubFinder(importlib.abc.MetaPathFinder, importlib.abc.Loader):
  def find_spec(self, name, path, target=None):
    if name.startswith(STUBS):
      return importlib.machinery.ModuleSpec(name, self, is_package=True)
    return None

  def create_module(self, spec):
    m = MagicMock(); m.__path__ = []; m.__spec__ = spec; return m

  def exec_module(self, module):
    pass


@pytest.fixture
def ui(monkeypatch):
  name = "openpilot.selfdrive.ui.sunnypilot.ui_state"
  for k in [k for k in sys.modules if k.startswith(STUBS) or k == name]:
    monkeypatch.delitem(sys.modules, k)
  monkeypatch.setattr(sys, "meta_path", [_StubFinder()] + sys.meta_path)
  mod = importlib.import_module(name)
  yield mod
  for k in [k for k in sys.modules if k.startswith(STUBS) or k == name]:
    monkeypatch.delitem(sys.modules, k, raising=False)


def test_ui_shows_the_paused_presentation_while_the_factory_lfa_steers(ui):
  ss = types.SimpleNamespace(state=log.SelfdriveState.OpenpilotState.disabled, enabled=False)

  def status(gw, mads_enabled=True, paused=False):
    sp = types.SimpleNamespace(mads=types.SimpleNamespace(state=State.enabled if mads_enabled else State.disabled,
                                                          available=True, enabled=mads_enabled))
    return ui.UIStateSP.update_status(ss, sp, [], cs_sp(paused=paused, gw=gw))
  assert ui.GW_SOLO_LFA_ACTIVE == ccmod.GW_SOLO | ccmod.GW_LFA_ACTIVE
  assert status(0) == "lat_only"
  assert status(ccmod.GW_LFA_ON | ccmod.GW_VALID | ccmod.GW_SOLO | ccmod.GW_GAVE_UP) == "lat_only"   # on / hold-off paused: openpilot still steers
  assert status(ccmod.GW_LFA_ACTIVE | ccmod.GW_LFA_ON | ccmod.GW_SOLO) == "override"
  assert status(ccmod.GW_LFA_ACTIVE | ccmod.GW_LFA_ON) == "lat_only"  # legacy profile: the factory LFA is meant to steer
  assert status(ccmod.GW_LFA_ACTIVE | ccmod.GW_SOLO, mads_enabled=False) == "disengaged"
  assert status(0, paused=True) == "override"                         # (the Phase 27 flag as before)
  sp = types.SimpleNamespace(mads=types.SimpleNamespace(state=State.enabled, available=True, enabled=True))
  assert ui.UIStateSP.update_status(ss, sp, [], None) == "lat_only"   # no carStateSP
  assert ui.UIStateSP.update_status(ss, sp, [], types.SimpleNamespace(lateralControlPaused=False)) == "lat_only"
