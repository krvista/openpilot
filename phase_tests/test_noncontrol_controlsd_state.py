"""End-to-end guards on controlsd's RECURSIVE lateral state.

The audit's _lookahead_curvature tests only check the helper's return value.
These drive the real Controls.state_control() so they actually prove the thing
that matters: a single non-finite model frame must not permanently latch
_lat_cmd_lp / _absdc_slow / _klane_lp / desired_curvature (which zeroes the
steering command for the rest of the drive via the actuator finite-guard).
"""
import math
import types

import pytest

from phase_tests.harness_noncontrol import FakeParams  # noqa: F401 (stub install side effect)

from cereal import car, log
import openpilot.selfdrive.controls.controlsd as cd

N_PTS = 20
K = 0.006


def mk_model(fallback=K, lane_nan_idx=None):
  xs = [2.0 * i for i in range(N_PTS)]
  ys = [0.5 * K * x * x for x in xs]
  yl = [-1.8 + y for y in ys]
  yr = [1.8 + y for y in ys]
  if lane_nan_idx is not None:
    yl[lane_nan_idx] = float('nan')
  ln = lambda Y: types.SimpleNamespace(x=list(xs), y=Y)  # noqa: E731
  return types.SimpleNamespace(
    action=types.SimpleNamespace(desiredCurvature=fallback),
    position=types.SimpleNamespace(x=xs, y=ys, yStd=[0.01] * N_PTS),
    laneLines=[ln(yl), ln(yl), ln(yr), ln(yr)],
    laneLineProbs=[0.9, 0.95, 0.95, 0.9],
    meta=types.SimpleNamespace(laneChangeState=log.LaneChangeState.off,
                               laneChangeDirection=log.LaneChangeDirection.none),
  )


class _SM(dict):
  valid = {'driverAssistance': True}
  updated = {'liveCalibration': False, 'livePose': False}
  logMonoTime: dict = {}

  def all_checks(self, s=None):
    return True


class _VM:
  def update_params(self, x, sr):
    pass

  def calc_curvature(self, angle, v, roll):
    return 0.0

  def get_steer_from_curvature(self, curv, v, roll):
    return curv * 15.0


class _LoC:
  long_control_state = log.LongitudinalPersonality.standard

  def reset(self):
    pass

  def update(self, *a):
    return 0.0


class _LaC:
  def reset(self):
    pass

  def update(self, active, CS, VM, lp, lim, desired_curvature, pose, curv_limited, delay):
    lg = log.ControlsState.LateralAngleState.new_message()
    lg.active = active
    return 0.0, math.degrees(VM.get_steer_from_curvature(-desired_curvature, CS.vEgo, 0.0)), lg


def mk_controls():
  s = cd.Controls.__new__(cd.Controls)
  s.sm = _SM({
    'carState': types.SimpleNamespace(vEgo=20.0, steeringAngleDeg=0.0, standstill=False,
                                      steerFaultTemporary=False, steerFaultPermanent=False,
                                      leftBlinker=False, rightBlinker=False, leftBlindspot=False, rightBlindspot=False, vCruise=100.0,
                                      vCruiseCluster=100.0, canValid=True, steeringPressed=False),
    'liveParameters': types.SimpleNamespace(stiffnessFactor=1.0, steerRatio=15.0, angleOffsetDeg=0.0, roll=0.0),
    'liveDelay': types.SimpleNamespace(lateralDelay=0.2),
    'longitudinalPlan': types.SimpleNamespace(aTarget=0.0, shouldStop=False, hasLead=False),
    'modelV2': mk_model(),
    'selfdriveState': types.SimpleNamespace(enabled=True, active=True, personality=None, alertHudVisual=None),
    'onroadEvents': [],
    'driverAssistance': types.SimpleNamespace(leftLaneDeparture=False, rightLaneDeparture=False),
  })
  s.CP = types.SimpleNamespace(steerControlType=car.CarParams.SteerControlType.angle,
                               minSteerSpeed=0.0, steerAtStandstill=True,
                               openpilotLongitudinalControl=False,
                               lateralTuning=types.SimpleNamespace(which=lambda: 'angle'))
  s.CP_SP = types.SimpleNamespace(pcmCruiseSpeed=True)
  s.CI = types.SimpleNamespace(get_pid_accel_limits=lambda *a: (-1.0, 1.0))
  s.VM, s.LoC, s.LaC = _VM(), _LoC(), _LaC()
  s.get_lat_active = lambda sm: True
  s.calibrated_pose = None
  s.steer_limited_by_safety = False
  s.curvature = s.desired_curvature = s.predicted_lat_accel_ratio = 0.0
  s._lat_cmd_lp = s._klane_lp = s._absdc_slow = 0.0
  s.bsm_guard = cd.BsmLaneGuard(0.01)   # Phase 37b (Controls built via __new__)
  s.lane_dropout = False; s.lane_dropout_clear_frames = 0; s.lane_dropout_frames = 0; s.lane_dropout_cooldown = 0   # Phase 39
  s._model_nonfinite_frames = 0
  return s


def run(s, n, model):
  out = None
  for _ in range(n):
    s.sm['modelV2'] = model
    CC, _ = cd.Controls.state_control(s)
    out = (float(CC.actuators.curvature), float(CC.actuators.steeringAngleDeg))
  return out


class TestRecursiveStatePoisoning:
  def test_single_nonfinite_model_frame_fully_recovers(self):
    s = mk_controls()
    run(s, 200, mk_model())
    steady = s.desired_curvature
    assert steady == pytest.approx(K, rel=1e-3)
    run(s, 1, mk_model(fallback=float('nan')))
    out = run(s, 400, mk_model())
    for v in (s._lat_cmd_lp, s._absdc_slow, s._klane_lp, s.desired_curvature):
      assert math.isfinite(v)
    # command back to steady, i.e. NOT stuck at the finite-guard's 0.0
    assert out[0] == pytest.approx(steady, rel=1e-6)
    assert abs(out[1]) > 1.0

  def test_single_nan_lane_line_does_not_kill_entry_assist(self):
    # NaN at a sampled interp node (x=0) poisons _klane_lp permanently,
    # silently disabling Phase 7c entry assist for the rest of the drive.
    s = mk_controls()
    run(s, 200, mk_model())
    run(s, 1, mk_model(lane_nan_idx=0))
    run(s, 400, mk_model())
    assert math.isfinite(s._klane_lp)
    assert abs(s._klane_lp) > 1e-6

  def test_sustained_nonfinite_model_does_not_latch_a_stale_turn(self):
    s = mk_controls()
    run(s, 200, mk_model())
    assert s.desired_curvature == pytest.approx(K, rel=1e-3)
    nan_model = mk_model(fallback=float('nan'))
    held = run(s, int(cd.MODEL_NONFINITE_HOLD_S / cd.DT_CTRL), nan_model)
    assert held[0] == pytest.approx(K, rel=1e-3)  # brief hold: no jerk
    out = run(s, int(3.0 / cd.DT_CTRL), nan_model)
    assert math.isfinite(out[0]) and abs(out[0]) < 0.05 * K


def mk_model_dropout(k_plan, lane_prob):
  m = mk_model(fallback=k_plan)
  m.laneLineProbs = [lane_prob, lane_prob, lane_prob, lane_prob]
  return m


class TestLaneDropoutLatch:
  # Phase 39: route 00000004 seg 8 — both lane probs ~0.13 for 7 s while the plan swung to
  # -11.8e-3; the 6g-2 blend half-passed it. The latch must hold/decay instead.
  def test_dropout_holds_and_decays_instead_of_following_the_plan(self):
    s = mk_controls()
    run(s, 200, mk_model())                       # steady on K with good lanes
    steady = s.desired_curvature
    out = run(s, 300, mk_model_dropout(k_plan=4 * K, lane_prob=0.13))   # 3 s dropout with a 4x plan spike
    assert s.lane_dropout
    assert abs(out[0]) < 0.5 * abs(steady) + 1e-9, (out[0], steady)   # decaying toward measured (0.0), not toward 4K
    assert abs(out[0] - 4 * K) > 3 * K

  def test_exit_needs_hi_for_half_a_second_then_plan_resumes(self):
    s = mk_controls()
    run(s, 200, mk_model())
    run(s, 100, mk_model_dropout(k_plan=K, lane_prob=0.13))
    assert s.lane_dropout
    run(s, 20, mk_model_dropout(k_plan=K, lane_prob=0.5))        # 0.2 s above HI: still latched
    assert s.lane_dropout
    run(s, 40, mk_model_dropout(k_plan=K, lane_prob=0.5))        # 0.6 s: released
    assert not s.lane_dropout
    out = run(s, 300, mk_model())
    assert out[0] == pytest.approx(K, rel=1e-2)

  def test_reconnection_band_keeps_6g1_floor(self):
    # lane_min in [0.20, 0.30] must NOT latch (6g-1: freezing there ran the car wide at corner entry)
    s = mk_controls()
    run(s, 200, mk_model())
    run(s, 50, mk_model_dropout(k_plan=K, lane_prob=0.25))
    assert not s.lane_dropout

  def test_kill_restores_blend(self):
    old = cd.LANE_DROPOUT_LATCH
    try:
      cd.LANE_DROPOUT_LATCH = False
      s = mk_controls(); run(s, 200, mk_model())
      out = run(s, 300, mk_model_dropout(k_plan=4 * K, lane_prob=0.13))
      assert not s.lane_dropout and abs(out[0]) > 1.2 * K        # blend moves TOWARD the spike (pre-39); the latch would decay below K
    finally:
      cd.LANE_DROPOUT_LATCH = old

  def test_max_duration_hands_back_to_blend(self):
    s = mk_controls(); run(s, 200, mk_model())
    run(s, 250, mk_model_dropout(k_plan=K, lane_prob=0.13))     # 2.5 s: still latched
    assert s.lane_dropout
    run(s, 100, mk_model_dropout(k_plan=K, lane_prob=0.13))     # 3.5 s: released even though still < LO
    assert not s.lane_dropout

  def test_exit_counter_resets_on_in_band_frame(self):
    s = mk_controls(); run(s, 200, mk_model())
    run(s, 100, mk_model_dropout(k_plan=K, lane_prob=0.13))
    for _ in range(4):                                          # 4 x (0.4 s above HI, 1 frame in band): never 0.5 s continuous
      run(s, 40, mk_model_dropout(k_plan=K, lane_prob=0.5)); run(s, 1, mk_model_dropout(k_plan=K, lane_prob=0.25))
    assert s.lane_dropout
