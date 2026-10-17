"""Phase 56 (report §56/§57): turn exits. While the car is unwinding from a turn it actually made (its curvature peaked
>= TURN_EXIT_K within TURN_EXIT_S and the model action is now <= half of that peak): (a) no lane-dropout latch,
(b) a command moving toward straight is not rate-capped by Phase 41; and (c) a held dropout ends when the driver turns
against it (opposing torque has the held curvature's sign: angle = -curvature)."""
import math

import pytest

from phase_tests.harness_noncontrol import FakeParams  # noqa: F401 (stub install side effect)

import openpilot.selfdrive.controls.controlsd as cd
from phase_tests.test_noncontrol_controlsd_state import mk_controls, mk_model, run, K
from phase_tests.test_phase49 import mk_lanes


def test_shipped_constants():
  assert cd.TURN_EXIT_K == 8e-3 and cd.TURN_EXIT_S == 5.0 and cd.TURN_EXIT_RATIO == 2.0
  assert cd.LANE_DROPOUT_COUNTER_TQ == 200.0


def test_turn_exit_window_tracks_the_cars_curvature():
  s = mk_controls(); run(s, 100, mk_model())
  assert not s._turn_exit_update(0.0, K)
  for _ in range(50):
    s._turn_exit_update(0.03, 0.03)              # the car turns (30e-3)
  assert not s._turn_exit_update(0.03, 0.02)      # still turning: action not below half the peak
  assert s._turn_exit_update(0.004, 0.004)        # unwinding
  for _ in range(int(cd.TURN_EXIT_S / cd.DT_CTRL) + 20):
    s._turn_exit_update(0.004, 0.004)
  assert not s._turn_exit_update(0.004, 0.004)    # the turn has left the 5 s window


def test_model_spike_alone_is_not_a_turn():
  s = mk_controls(); run(s, 100, mk_model())
  for _ in range(100):
    s._turn_exit_update(0.001, 0.05)             # action spikes, the car does not turn
  assert not s._turn_exit_update(0.001, 0.002)


def test_no_dropout_latch_while_unwinding_from_a_turn():
  s = mk_controls(); run(s, 200, mk_model())
  for _ in range(100):
    s._turn_exit_update(0.03, 0.03)
  s.sm['carState'].steeringAngleDeg = 0.0
  run(s, 60, mk_lanes(K, 0.05, 0.05))             # turned, now lines gone and action K (< half the peak)
  assert not s.lane_dropout
  # same blackout without the turn latches as before
  s2 = mk_controls(); run(s2, 200, mk_model())
  run(s2, 60, mk_lanes(K, 0.05, 0.05))
  assert s2.lane_dropout


def test_counter_steer_releases_a_hold():
  s = mk_controls(); run(s, 200, mk_model())
  run(s, 60, mk_lanes(K, 0.05, 0.05))
  assert s.lane_dropout and s.lane_dropout_hold_k > 0
  s.sm['carState'].steeringTorque = -250.0        # same direction as the held angle (-curvature): not a counter-steer
  run(s, 1, mk_lanes(K, 0.05, 0.05))
  assert s.lane_dropout
  s.sm['carState'].steeringTorque = 150.0         # opposing but light
  run(s, 1, mk_lanes(K, 0.05, 0.05))
  assert s.lane_dropout
  s.sm['carState'].steeringTorque = 250.0         # opposing, >= 200
  run(s, 1, mk_lanes(K, 0.05, 0.05))
  assert not s.lane_dropout and not s.lane_dropout_armed


def test_kill_switches():
  saved = (cd.TURN_EXIT_K, cd.LANE_DROPOUT_COUNTER_TQ)
  try:
    cd.TURN_EXIT_K, cd.LANE_DROPOUT_COUNTER_TQ = 1e9, 1e9
    s = mk_controls(); run(s, 200, mk_model())
    for _ in range(100):
      s._turn_exit_update(0.03, 0.03)
    run(s, 60, mk_lanes(K, 0.05, 0.05))
    assert s.lane_dropout
    s.sm['carState'].steeringTorque = 1000.0
    run(s, 1, mk_lanes(K, 0.05, 0.05))
    assert s.lane_dropout
  finally:
    cd.TURN_EXIT_K, cd.LANE_DROPOUT_COUNTER_TQ = saved


def test_unwinding_is_not_rate_capped():
  cap = cd.LowConfRateCap(cd.DT_CTRL)
  k_per_deg = 1e-4
  # what controlsd passes during a turn exit: blinker_or_lc | unwinding -> the cap returns the new command untouched
  assert cap.update(0.002, 0.008, 0.1, False, True, 12.0, k_per_deg) == 0.002
  # outside the window the 6 deg/s cap still applies
  assert cap.update(0.002, 0.008, 0.1, False, False, 12.0, k_per_deg) == pytest.approx(0.008 - 6.0 * k_per_deg * cd.DT_CTRL)
  src = open(cd.__file__).read()
  assert "unwinding = TURN_EXIT_SOFT_EXEMPT and turn_exit and abs(new_desired_curvature) < abs(self.desired_curvature)" in src
  assert "blinker_or_lc or unwinding" in src


def test_unwinding_is_not_frozen_by_the_low_confidence_blend():
  saved = cd.LOWCONF_CURVE_K
  cd.LOWCONF_CURVE_K = [1e9, 1e9]          # Phase 57 off: this test is about the 56d exemption alone
  try:
    _blend_test()
  finally:
    cd.LOWCONF_CURVE_K = saved


def _blend_test():
  def unwind(turned):
    s = mk_controls(); run(s, 200, mk_model(fallback=0.02))           # op turning at 20e-3
    if turned:
      for _ in range(100):
        s._turn_exit_update(0.03, 0.03)                              # ... and the car really turned
    s.desired_curvature = s._lat_cmd_lp = 0.02
    flags = 0
    for _ in range(30):                                              # lines weak (blend, no dropout), model unwound
      run(s, 1, mk_lanes(0.004, 0.2, 0.2))
      flags |= s.steer_flags
    return s.desired_curvature, flags
  k_turn, f_turn = unwind(True)
  k_plain, f_plain = unwind(False)
  assert f_plain & cd.STEER_FLAG_CONF_BLEND and not f_turn & cd.STEER_FLAG_CONF_BLEND
  assert abs(k_turn) < abs(k_plain)                                  # the blend held the plain case nearer the turn


def test_turn_exit_unwinding_lead_gets_the_full_weight():
  from phase_tests.test_phase55 import call, model, settled, DB
  s = settled(); s._turn_exit = True
  assert call(s, model(0.003, 0.002, t_step=0.95)) == pytest.approx(0.003 - 0.5 * (0.001 - DB), rel=1e-6)
  s = settled(); s._turn_exit = False
  assert call(s, model(0.003, 0.002, t_step=0.95)) == pytest.approx(0.003 - 0.2 * (0.001 - DB), rel=1e-6)


def test_phase57_curve_fades_the_low_confidence_restraints():
  """Phase 57: with |action| >= 3e-3 the 41 cap runs at 30 deg/s and the 6g blend floor is 1 — the command ramps toward
  the model 5x faster through a low-line curve than near-straight (1.2e-3), where both restraints still act."""
  assert cd.LOWCONF_CURVE_K == [1.5e-3, 3.0e-3]
  from phase_tests.test_phase41 import _model
  slope = {}; flags = {}
  for k in (0.0012, 0.006):
    s = mk_controls(); s.sm['carState'].vEgo = 12.0
    run(s, 200, _model(0.0, [0.9, 0.95, 0.95, 0.9]))
    f = 0; hist = []
    for _ in range(25):
      run(s, 1, _model(k, [0.9, 0.2, 0.95, 0.9]))
      f |= s.steer_flags; hist.append(s.lowconf_cap.last_out)
    slope[k] = (hist[-1] - hist[14]) / 10.0; flags[k] = f
  assert flags[0.0012] & cd.STEER_FLAG_CONF_BLEND and flags[0.0012] & cd.STEER_FLAG_LOWCONF_CAPPED
  assert not flags[0.006] & cd.STEER_FLAG_CONF_BLEND
  assert slope[0.006] > 3.0 * slope[0.0012]            # 30 deg/s vs 6 deg/s own-state ramps


def test_phase57_cap_ramps_from_its_own_output():
  """The cap used to re-anchor to the low-passed command every frame (effective rate alpha * cap). Own-state: the
  pre-LP command advances by the full step each frame."""
  from phase_tests.test_phase41 import _model
  import openpilot.selfdrive.controls.lib.lowconf_rate_cap as lrc
  s = mk_controls(); s.sm['carState'].vEgo = 12.0
  run(s, 200, _model(0.0, [0.9, 0.95, 0.95, 0.9]))
  hist = []
  for _ in range(30):
    run(s, 1, _model(0.0012, [0.9, 0.2, 0.95, 0.9]))
    hist.append(s.lowconf_cap.last_out)
  step = 6.0 * lrc.NOMINAL_K_PER_DEG * cd.DT_CTRL          # the harness VM gives no k/deg -> nominal
  assert hist[20] - hist[10] == pytest.approx(10 * step, rel=0.05)
  saved = lrc.LOWCONF_CAP_OWN_STATE
  lrc.LOWCONF_CAP_OWN_STATE = False                        # kill: the old crawl (a few % of the step per frame)
  try:
    s = mk_controls(); s.sm['carState'].vEgo = 12.0
    run(s, 200, _model(0.0, [0.9, 0.95, 0.95, 0.9]))
    h2 = []
    for _ in range(30):
      run(s, 1, _model(0.0012, [0.9, 0.2, 0.95, 0.9]))
      h2.append(s.desired_curvature)
    assert h2[20] - h2[10] < 0.3 * 10 * step
  finally:
    lrc.LOWCONF_CAP_OWN_STATE = saved
