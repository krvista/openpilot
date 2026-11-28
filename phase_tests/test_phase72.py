"""Phase 72 (report §96-97): openpilot steers alone.
The MDPS is commanded by the vehicle's ADAS controller (ADAS_CMD_35_10ms), which relays our LKAS_ALT request only while
the factory LFA is not active. Up to Phase 71 the 'MADS <-> cluster sync pulse' switched the factory LFA on with MADS.
72a  hold-off: the factory LFA (LFAHDA_CLUSTER.LFA_ICON) is kept switched off with LFA button pulses on its own state;
     the camera's button bit is no longer forwarded to the controller.
72b  direct profile: no trim / stall kick / entry boost / exit lead / compensator in the car controller; controlsd
     hands the model's action through (no lead layers, entry assist, extra low-pass); latcontrol without the 7a trim;
     lagd learns the relayed plant (seed 0.32 s, published <= 0.5 s, factory-LFA windows excluded).
72c  follow band: while the factory LFA is active / releasing the request stays within 1 deg of the wheel.
The Developer toggle 'stock LFA with op (old)' and its param were removed in Phase 74; the pre-72 behaviour itself
(the 'legacy' profile, its layers and the switches CarControlSP.stockLfaWithOp / steer_profile.SOLO_DEFAULT /
LFA_HOLD_OFF) in Phase 89 - what was a comparison against it here is now the solo half alone."""
import types

import numpy as np
import pytest

from phase_tests.harness import Sim
from phase_tests.harness_noncontrol import FakeParams  # noqa: F401 (stub install side effect)
from opendbc.car import structs
from opendbc.car.hyundai.values import CarControllerParams as P
import opendbc.car.hyundai.carcontroller as ccmod
import openpilot.selfdrive.controls.controlsd as cd

KPH = 1 / 3.6


# ------------------------------------------------------------------ the car's side: factory LFA switch
class FactoryLfa:
  """The ADAS controller's LFA switch as the logs show it: a rising edge of the LFA button bit toggles it 0.1 s later;
  switching off goes through 'releasing' (3) for 1 s; a pulse while releasing switches it back on (the case the
  hold-off must never cause); auto_on() = the car switching it on by itself (cruise engaged in an HDA zone)."""

  def __init__(self, state=0, ignore=False):
    self.state = state; self.ignore = ignore; self._prev = 0; self._due = None; self._rel = 0; self.pulses = []; self.n = 0

  def auto_on(self):
    self.state = 2; self._rel = 0

  def step(self, bit):
    self.n += 1
    if bit and not self._prev:
      self.pulses.append(self.n)
      if not self.ignore:
        self._due = self.n + 10
    self._prev = bit
    if self._due is not None and self.n >= self._due:
      self._due = None
      if self.state in (1, 2):
        self.state = 3; self._rel = 100
      else:                       # off, or releasing: on again
        self.state = 2; self._rel = 0
    if self.state == 3:
      self._rel -= 1
      if self._rel <= 0:
        self.state = 0
    return self.state


def run(sim, car, n, mads=True, **kw):
  """n frames with the car's LFA state fed back; returns the per-frame button bit"""
  sim.cc._cc_sp.mads.enabled = bool(mads)
  bits = []
  for _ in range(n):
    sim.step(lfa_state=car.state, **kw)
    bit = int(sim.lkas_alt()["LFA_BUTTON"]); bits.append(bit)
    car.step(bit)
  return bits


@pytest.fixture
def params():
  saved = {k: getattr(P, k) for k in ("GW_FOLLOW_BAND_DEG", "LFA_HOLD_OFF_MAX_PULSES")}
  yield P
  for k, v in saved.items():
    setattr(P, k, v)


# ------------------------------------------------------------------ 72a hold-off
def test_shipped_values():
  assert P.LFA_HOLD_OFF_CONFIRM_FRAMES == 30
  assert P.LFA_HOLD_OFF_WAIT_FRAMES == [150, 300, 600, 1000] and P.LFA_HOLD_OFF_MAX_PULSES == 8
  assert P.LFA_HOLD_OFF_WINDOW_FRAMES == 6000 and P.LFA_HOLD_OFF_GIVE_UP_FRAMES == 12000
  assert P.GW_FOLLOW_BAND_DEG == 1.0 and P.GW_RELAY_TOL_DEG == 0.5 and P.GW_RELAY_HIST == 8


def test_factory_lfa_found_on_is_switched_off_with_one_pulse():
  sim, car = Sim(), FactoryLfa(state=2)
  bits = run(sim, car, 600)
  assert len(car.pulses) == 1 and 30 <= car.pulses[0] <= 33          # 0.3 s of 'on', then the pulse
  assert sum(bits) == ccmod.LFA_SYNC_PULSE_FRAMES                    # one pulse = 2 frames
  assert car.state == 0
  assert sum(run(sim, car, 3000)) == 0                               # off stays off: nothing more is sent


def test_standby_counts_as_on_and_releasing_or_unknown_never_pulse():
  sim, car = Sim(), FactoryLfa(state=1)
  run(sim, car, 200)
  assert len(car.pulses) == 1 and car.state in (3, 0)
  for st in (3, -1):
    sim2 = Sim()
    bits = []
    for _ in range(1500):
      sim2.step(lfa_state=st); bits.append(int(sim2.lkas_alt()["LFA_BUTTON"]))
    assert sum(bits) == 0, st


def test_no_pulse_while_the_release_is_in_progress():
  # the car goes 2 -> 3 (1 s) -> 0 after our pulse; a second pulse inside that second would switch it back on
  sim, car = Sim(), FactoryLfa(state=2)
  run(sim, car, 400)
  assert len(car.pulses) == 1 and car.state == 0


def test_car_switching_it_on_again_is_answered():
  sim, car = Sim(), FactoryLfa(state=2)
  run(sim, car, 500)
  assert car.state == 0
  car.auto_on()                                                      # cruise engaged on a motorway: HDA switches the LFA on
  run(sim, car, 300)
  assert len(car.pulses) == 2 and car.pulses[1] - 500 <= 35 and car.state in (3, 0)
  run(sim, car, 200)
  assert car.state == 0


def test_ignored_pulses_are_retried_with_backoff():
  sim, car = Sim(), FactoryLfa(state=1, ignore=True)        # e.g. at standstill right after the start
  run(sim, car, 6200)
  gaps = np.diff(car.pulses).tolist()                                # (the wait starts after the pulse's second frame)
  assert gaps[:3] == [151, 301, 601] and all(g == 1001 for g in gaps[3:])
  assert len(car.pulses) <= P.LFA_HOLD_OFF_MAX_PULSES + 1 and sim.cc.lfa_hold_give_up == 0   # steady retries never trip the limit
  car.ignore = False
  run(sim, car, 1200)
  assert car.state == 0
  assert sim.cc.lfa_hold_tries == 0                                  # seen off: the backoff starts over


def test_a_car_that_keeps_switching_it_on_makes_the_hold_off_pause():
  sim, car = Sim(), FactoryLfa(state=2)
  for _ in range(40):                                                # every time it is off, the car switches it on again
    run(sim, car, 160)
    if car.state == 0:
      car.auto_on()
  assert len(car.pulses) == P.LFA_HOLD_OFF_MAX_PULSES
  assert sim.cc.lfa_hold_give_up > 0 and sim.cc.gw_flags & ccmod.GW_GAVE_UP
  n = len(car.pulses)
  run(sim, car, 3000)                                                # paused: not a single pulse
  assert len(car.pulses) == n
  run(sim, car, P.LFA_HOLD_OFF_GIVE_UP_FRAMES)                       # after the pause it tries again
  assert len(car.pulses) > n


def test_camera_button_bit_is_not_forwarded():
  sim = Sim()
  sim.step(lfa_state=0, cam_lfa_button=1)
  assert sim.lkas_alt()["LFA_BUTTON"] == 0


def test_mads_edges_send_no_pulse():
  sim = Sim(); bits = []
  for i in range(300):
    sim.cc._cc_sp.mads.enabled = i >= 100
    sim.step(lfa_state=0); bits.append(int(sim.lkas_alt()["LFA_BUTTON"]))
  assert sum(bits) == 0


# ------------------------------------------------------------------ 72b direct profile (car controller)
def curve(sim, n=900, cmd=8.0, wheel_frac=0.6, v=20.0, **kw):
  """a sustained curve command with the wheel delivering only part of it (what the trim was built for)"""
  for i in range(n):
    c = cmd * min(i / 100.0, 1.0)
    sim.step(v=v, cmd=c, wheel=c * wheel_frac, **kw)
  return sim


def test_request_is_the_command():
  sim = curve(Sim(), lfa_state=0)
  assert abs(sim.tx_angle() - 8.0) <= 0.25                           # the request IS the command (hysteresis band 0.15)
  assert not (sim.cc.steer_debug_flags & (ccmod.SF_ENTRY_BOOST | ccmod.SF_MDPS_COMP))


# ------------------------------------------------------------------ 72c follow band
def test_request_stays_with_the_wheel_while_the_factory_lfa_is_active():
  sim = Sim()
  for i in range(400):
    sim.step(v=25.0, cmd=7.0, wheel=1.0, lfa_state=2)
    if i > 50:
      assert abs(sim.tx_angle() - 1.0) <= P.GW_FOLLOW_BAND_DEG + 0.05
  assert sim.cc.gw_follow and sim.cc.gw_flags & ccmod.GW_FOLLOW and sim.cc.gw_flags & ccmod.GW_LFA_ACTIVE


def test_handover_has_no_stored_step():
  """factory LFA active with our command 6 deg away, then it releases (3) and is off (0): what the MDPS gets from us"""
  def drive():
    sim = Sim(); tx = []; wh = []
    w = 1.0
    for i in range(900):
      st = 2 if i < 300 else (3 if i < 400 else 0)
      sim.step(v=25.0, cmd=7.0, wheel=w, lfa_state=st)
      tx.append(sim.tx_angle()); wh.append(w)
      if st != 2 and i >= 350:                                       # relayed from 0.5 s after the release starts
        w += (tx[-1] - w) * 0.07                                     # the MDPS: ~0.13 s lag
    return np.array(tx), np.array(wh)
  solo, w_s = drive()
  assert abs(solo[349] - 1.0) <= 1.05                                # at the switch-over: within the band
  assert np.all(np.abs(solo[300:400] - w_s[300:400]) <= P.GW_FOLLOW_BAND_DEG + 0.05)   # with the wheel while releasing
  assert solo[-1] == pytest.approx(7.0, abs=0.3)                     # and afterwards it does go to the command
  assert np.max(np.abs(w_s[360:] - w_s[350:-10])) <= 1.0             # the wheel never moves more than 1 deg in 0.1 s


def test_follow_band_kill_switch_and_states_without_it(params):
  for st in (0, 1, -1):
    sim = Sim()
    for _ in range(400):
      sim.step(v=25.0, cmd=7.0, wheel=1.0, lfa_state=st)
    assert sim.tx_angle() > 5.0 and not sim.cc.gw_follow, st
  params.GW_FOLLOW_BAND_DEG = 0.0
  sim = Sim()
  for _ in range(400):
    sim.step(v=25.0, cmd=7.0, wheel=1.0, lfa_state=2)
  assert sim.tx_angle() > 5.0


# ------------------------------------------------------------------ relay check and telemetry
def test_relay_flag_follows_the_controllers_request():
  sim = Sim(); hist = [0.0] * 4
  for i in range(300):
    c = 3.0 * np.sin(i / 60.0)
    sim.step(v=25.0, cmd=c, wheel=c, lfa_state=0, gw_req_angle=hist[-3])       # the controller repeats us 30 ms later
    hist.append(sim.tx_angle())
  f = sim.cc.gw_flags
  assert f & ccmod.GW_RELAY and f & ccmod.GW_VALID and f & ccmod.GW_SOLO and not f & ccmod.GW_LFA_ON
  for _ in range(50):
    sim.step(v=25.0, cmd=6.0, wheel=0.0, lfa_state=2, gw_req_angle=-4.0)        # its own request, far from ours
  f = sim.cc.gw_flags
  assert not f & ccmod.GW_RELAY and f & ccmod.GW_LFA_ACTIVE and f & ccmod.GW_LFA_ON
  sim.step(v=25.0, cmd=0.0, wheel=0.0, lfa_state=-1)                            # nothing received
  assert not sim.cc.gw_flags & ccmod.GW_VALID and not sim.cc.gw_flags & ccmod.GW_RELAY
  bits = [ccmod.GW_VALID, ccmod.GW_LFA_ON, ccmod.GW_LFA_ACTIVE, ccmod.GW_RELAY, ccmod.GW_SOLO, ccmod.GW_PULSE, ccmod.GW_FOLLOW, ccmod.GW_GAVE_UP]
  assert sorted(bits) == [1 << k for k in range(8)]


def test_schema_and_plumbing():
  from openpilot.cereal import custom
  from openpilot.selfdrive.car.helpers import convert_carControlSP
  import openpilot.selfdrive.locationd.lagd as lagd
  # the profile field is unused since Phase 89 (never written) but stays in the schema and the dataclass, in step
  assert structs.CarControlSP().stockLfaWithOp is False
  assert convert_carControlSP(custom.CarControlSP.new_message().as_reader()).stockLfaWithOp is False
  m = custom.CarControlSP.new_message(); m.stockLfaWithOp = True      # e.g. a log from before Phase 74
  assert convert_carControlSP(m.as_reader()).stockLfaWithOp is True
  s = custom.CarStateSP.new_message(); s.gatewayFlags = 0b10010101
  assert s.gatewayFlags == 0b10010101
  assert lagd.GW_LFA_ACTIVE == ccmod.GW_LFA_ACTIVE
  import os
  root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
  assert "StockLfaWithOp" not in open(os.path.join(root, "openpilot/common/params_keys.h")).read()   # Phase 74
  card = open(os.path.join(root, "openpilot/selfdrive/car/card.py")).read()
  assert "CS_SP.gatewayFlags = int(getattr(self.CI.CC, 'gw_flags', 0)) & 0xFF" in card
  ext = open(os.path.join(root, "openpilot/sunnypilot/selfdrive/controls/controlsd_ext.py")).read()
  assert "stockLfaWithOp" not in ext                                 # Phase 89: nothing writes it


def test_carstate_reads_the_two_messages_without_gating_can_valid():
  from opendbc.car import Bus
  from opendbc.car.hyundai.interface import CarInterface
  from phase_tests.harness import make_cp
  CI = CarInterface(make_cp(), structs.CarParamsSP())
  st = CI.can_parsers[Bus.pt].message_states
  for addr in (0x1E0, 0xCB):
    assert addr in st and st[addr].ignore_alive and st[addr].ignore_counter
  assert CI.CS.lfa_state == -1 and CI.CS.gw_req_valid is False       # nothing received yet = unknown


# ------------------------------------------------------------------ 72b controlsd direct path
def test_direct_path_is_the_models_action():
  from phase_tests import test_noncontrol_controlsd_state as H
  s = H.mk_controls(); s._model_lat_delay = 0.32
  m = H.mk_model(fallback=0.0031)
  assert cd.Controls._lookahead_curvature(s, m) == float(m.action.desiredCurvature)


def test_direct_state_control_has_no_lowpass_and_sets_the_flag():
  from phase_tests import test_noncontrol_controlsd_state as H
  s = H.mk_controls()
  s.sm['carState'].vEgo = 8.0                                        # 29 km/h (below the lane-centre term's speed)
  s.sm['modelV2'] = H.mk_model()
  outs = []
  for i in range(60):
    s.sm['modelV2'].action.desiredCurvature = 0.0 if i < 5 else 0.004
    s.sm.logMonoTime['modelV2'] = i                                  # a new model frame (per-frame values are cached)
    CC, _ = cd.Controls.state_control(s)
    outs.append(float(CC.actuators.curvature))
  assert s.steer_flags & cd.STEER_FLAG_DIRECT
  assert not s.steer_flags & (cd.STEER_FLAG_ENTRY_ASSIST | cd.STEER_FLAG_LOOKAHEAD_CAP | cd.STEER_FLAG_LOOKAHEAD_ORIENT
                              | cd.STEER_FLAG_LOOKAHEAD_BUDGET)
  # no low-pass: only clip_curvature's rate limit (MAX_LATERAL_JERK / v^2 per second) between the action and the command
  from openpilot.selfdrive.controls.lib.drive_helpers import MAX_LATERAL_JERK
  step = MAX_LATERAL_JERK / 8.0 ** 2 * 0.01
  assert outs[5] == pytest.approx(min(step, 0.004), rel=1e-6) and outs[20] == pytest.approx(min(16 * step, 0.004), rel=1e-6)
  assert outs[-1] == pytest.approx(0.004, abs=2e-4)


def test_latcontrol_is_pure_feed_forward():
  from openpilot.selfdrive.controls.lib import latcontrol_angle as la
  CP = types.SimpleNamespace(brand="hyundai", steerControlType="angle", steerLimitTimer=1.0)
  lac = la.LatControlAngle(CP, None, None, 0.01)
  VM = types.SimpleNamespace(calc_curvature=lambda a, v, r: a / 2.7 / 15.0,
                             get_steer_from_curvature=lambda k, v, r: k * 2.7 * 15.0)
  CS = types.SimpleNamespace(steeringAngleDeg=0.0, vEgo=20.0, steeringPressed=False)
  prm = types.SimpleNamespace(roll=0.0, angleOffsetDeg=0.0)
  outs = [lac.update(True, CS, VM, prm, False, 0.0005, None, False, 0.3)[1] for _ in range(300)]
  assert len(set(outs)) == 1                                         # the wheel never delivers it: nothing winds up
  assert outs[0] == pytest.approx(np.degrees(-0.0005 * 2.7 * 15.0))


# ------------------------------------------------------------------ 72b lagd
def test_lagd_solo_profile():
  import openpilot.selfdrive.locationd.lagd as lagd
  CP = types.SimpleNamespace(brand="hyundai", steerControlType="angle")
  solo = lagd.lagd_profile(CP)
  assert solo["initial_lag_seed"] == lagd.LAGD_SOLO_SEED_S == 0.32
  assert solo["max_lag"] == lagd.MAX_LAG and solo["publish_max"] == lagd.LAGD_SOLO_PUBLISH_MAX_S == 0.50
  assert solo["min_vego"] == pytest.approx(60 * KPH) and solo["min_ncc"] == 0.9 and solo["min_confidence"] == 0.5
  assert solo["min_press_sec"] == 0.5 and solo["press_free_sec"] == lagd.LAGD_PRESS_FREE_S == 60.0
  assert solo["min_recovery_buffer_sec"] == 1.0 and len(solo) == 9
  assert lagd.lagd_profile(types.SimpleNamespace(brand="toyota", steerControlType="torque")) == {}


def _cached_lag(lag):
  import openpilot.cereal.messaging as messaging
  from opendbc.car.structs import car
  import openpilot.selfdrive.locationd.lagd as lagd
  msg = messaging.new_message('lateralDelay')
  ld = msg.lateralDelay
  ld.lateralDelayEstimate = lag; ld.lateralDelay = lag; ld.validBlocks = 12; ld.status = 'estimated'; ld.version = lagd.VERSION
  CP = car.CarParams.new_message(carFingerprint="HYUNDAI_IONIQ_6_N")

  class _P:
    def __init__(self):
      self.store = {"LiveDelay": msg.to_bytes(), "CarParamsPrevRoute": CP.to_bytes()}
    def get(self, k, **kw):
      return self.store.get(k)
    def remove(self, k):
      self.store.pop(k, None)
  return lagd, _P(), CP


@pytest.mark.parametrize("lag,solo,kept", [(0.76, True, False), (0.33, True, True), (0.33, False, False), (0.76, False, True),
                                           (0.76, None, True), (0.33, None, True)])
def test_cached_delay_of_the_other_profile_is_not_carried_over(lag, solo, kept):
  lagd, p, CP = _cached_lag(lag)
  out = lagd.retrieve_initial_lag(p, CP, solo) if solo is not None else lagd.retrieve_initial_lag(p, CP)
  if kept:
    assert out is not None and out[0] == pytest.approx(lag) and out[1] == 12
  else:
    assert out is None and "LiveDelay" not in p.store


def test_lagd_does_not_learn_from_factory_lfa_windows():
  import openpilot.selfdrive.locationd.lagd as lagd
  CP = types.SimpleNamespace(brand="hyundai", steerControlType="angle", steerActuatorDelay=0.1)
  est = lagd.LateralLagEstimator(CP, 0.05, **lagd.lagd_profile(CP))
  assert est.initial_lag == pytest.approx(0.32)
  assert "carStateSP" in est.inputs
  est.handle_log(1.0, "carStateSP", types.SimpleNamespace(gatewayFlags=lagd.GW_LFA_ACTIVE | 1))
  assert est.factory_lfa_active is True
  est.lat_active = True; est.t = 5.0
  est.calibrator = types.SimpleNamespace(calib_valid=True)
  est.pose_valid = True; est.v_ego = 30.0; est.yaw_rate = 0.01; est.desired_curvature = 0.01 / 30.0
  est.update_points()
  assert est.last_lat_inactive_t == 5.0                              # treated like "lateral not active"
  est.handle_log(6.0, "carStateSP", types.SimpleNamespace(gatewayFlags=1))
  assert est.factory_lfa_active is False
  est.handle_log(7.0, "carStateSP", types.SimpleNamespace())         # a log without the field
  assert est.factory_lfa_active is False


def test_published_delay_is_capped_in_solo():
  import openpilot.selfdrive.locationd.lagd as lagd
  CP = types.SimpleNamespace(brand="hyundai", steerControlType="angle", steerActuatorDelay=0.1)
  est = lagd.LateralLagEstimator(CP, 0.05, **lagd.lagd_profile(CP))
  assert est.publish_max == 0.50 and est.max_lag == lagd.MAX_LAG
  est.reset(0.62, est.min_valid_block_count)                         # a drive where the factory LFA could not be held off
  assert est.get_msg(True).lateralDelay.lateralDelay == pytest.approx(0.50)


# ------------------------------------------------------------------ Phase 74: the developer toggle is gone
def test_no_developer_toggle_for_the_old_behaviour():
  import os
  root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
  src = open(os.path.join(root, "openpilot/selfdrive/ui/mici/layouts/settings/developer.py")).read()
  for gone in ("StockLfaWithOp", "_stock_with_op_toggle", "STOCK_LFA_WITH_OP", "_on_stock_with_op_toggled", 'BigToggle("stock LFA with op'):
    assert gone not in src, gone
  assert 'BigToggle("stock LFA capture"' in src                      # the capture toggle stays
