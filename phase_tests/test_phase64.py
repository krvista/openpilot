"""Phase 64 (report §76).
64a: lagd CCNC profile — Hyundai angle-steering (CCNC) cars learn with NCC >= 0.9, confidence >= 0.5, >= 60 km/h, a 1 s
recovery, and only a press held >= 0.5 s counts as steering (short HOD "hands on" touches do not blank the learner).
64b: controlsd's Phase 60 delay shift follows the delay modeld is actually told: E_eff = E(v) + 0.3 - model delay, and the
plan reference times move with the model delay, so a learned (estimated) lagd value is not counted twice."""
import types

import numpy as np
import pytest

from phase_tests.harness_noncontrol import FakeParams  # noqa: F401 (stub install side effect)

import openpilot.selfdrive.controls.controlsd as cd
import openpilot.selfdrive.locationd.lagd as lagd
from phase_tests.test_phase55 import model, call, settled
from phase_tests.test_phase60 import ramp_plan


@pytest.fixture(autouse=True)
def _restore():
  names = ("LOOKAHEAD_DELAY_EXTRA_S", "LOOKAHEAD_DELAY_FOLLOW_MODEL")
  saved = {nm: getattr(cd, nm) for nm in names}
  prof = lagd.CCNC_PROFILE
  yield
  for nm, val in saved.items():
    setattr(cd, nm, val)
  lagd.CCNC_PROFILE = prof
  FakeParams.reset()


# ---------------------------------------------------------------- 64a lagd
def cp(brand="hyundai", sct="angle"):
  return types.SimpleNamespace(brand=brand, steerControlType=sct, steerActuatorDelay=0.1)


def test_profile_only_for_ccnc_angle_cars():
  p = lagd.lagd_profile(cp())
  assert p["min_ncc"] == 0.9 and p["min_confidence"] == 0.5 and p["min_press_sec"] == 0.5
  assert p["min_vego"] == pytest.approx(60.0 / 3.6) and p["min_recovery_buffer_sec"] == 1.0
  assert lagd.lagd_profile(cp(sct="torque")) == {}
  assert lagd.lagd_profile(cp(brand="toyota")) == {}
  lagd.CCNC_PROFILE = False
  assert lagd.lagd_profile(cp()) == {}


def test_seed_sets_the_pre_learning_delay():
  e = lagd.LateralLagEstimator(cp(), 0.05, **lagd.lagd_profile(cp()))
  assert e.initial_lag == pytest.approx(lagd.LAGD_SEED_S)           # 0.55, not steerActuatorDelay + 0.2 = 0.30
  msg = e.get_msg(True)                                             # before any learning
  assert str(msg.lateralDelay.status) == "unestimated"
  assert msg.lateralDelay.lateralDelay == pytest.approx(lagd.LAGD_SEED_S)   # what modeld receives


def test_seed_kill_switch_restores_stock_base():
  e = lagd.LateralLagEstimator(cp(), 0.05)                          # no profile kwargs
  assert e.initial_lag == pytest.approx(0.1 + 0.2)                  # steerActuatorDelay + 0.2
  assert lagd.lagd_profile(cp())["initial_lag_seed"] == lagd.LAGD_SEED_S


def test_learned_value_overrides_the_seed():
  # once estimated, modeld gets the learned block mean, not the seed (adaptive)
  e = lagd.LateralLagEstimator(cp(), 0.05, **lagd.lagd_profile(cp()))
  e.reset(0.60, lagd.BLOCK_NUM_NEEDED + 2)                          # as if learned 0.60 over >= 5 blocks
  msg = e.get_msg(True)
  assert str(msg.lateralDelay.status) == "estimated"
  assert msg.lateralDelay.lateralDelay == pytest.approx(0.60, abs=1e-6)
  assert msg.lateralDelay.lateralDelay != pytest.approx(lagd.LAGD_SEED_S)


def test_estimator_takes_the_profile():
  e = lagd.LateralLagEstimator(cp(), 0.05, **lagd.lagd_profile(cp()))
  assert e.min_ncc == 0.9 and e.min_confidence == 0.5 and e.min_press_sec == 0.5
  assert e.min_vego == pytest.approx(60.0 / 3.6) and e.min_recovery_buffer_sec == 1.0
  s = lagd.LateralLagEstimator(cp(), 0.05)
  assert s.min_ncc == lagd.MIN_NCC and s.min_press_sec == 0.0 and s.min_vego == lagd.MIN_VEGO


def press_trace(e, pattern, dt=0.01):
  out = []
  for i, p in enumerate(pattern):
    e.handle_log(i * dt, "carState", types.SimpleNamespace(steeringPressed=bool(p), vEgo=25.0))
    out.append(e.steering_pressed)
  return np.array(out)


def test_short_hod_touch_is_not_steering():
  e = lagd.LateralLagEstimator(cp(), 0.05, **lagd.lagd_profile(cp()))
  tr = press_trace(e, [0] * 10 + [1] * 30 + [0] * 10)          # 0.3 s touch
  assert not tr.any()


def test_held_press_counts_after_half_a_second():
  e = lagd.LateralLagEstimator(cp(), 0.05, **lagd.lagd_profile(cp()))
  tr = press_trace(e, [0] * 10 + [1] * 100 + [0] * 10)         # 1.0 s press
  assert not tr[:59].any() and tr[60:110].all() and not tr[110:].any()


def test_stock_press_is_immediate():
  e = lagd.LateralLagEstimator(cp(), 0.05)
  tr = press_trace(e, [0] * 10 + [1] * 30 + [0] * 10)
  assert tr[10:40].all() and not tr[40:].any()


# ---------------------------------------------------------------- 64b controlsd
def with_delay(d):
  s = settled(20.0)
  s._model_lat_delay = d
  return s


def test_untold_delay_keeps_phase60():
  cd.LOOKAHEAD_DELAY_EXTRA_S = [0.3] * 5
  m = ramp_plan(0.001, 0.005)
  a = with_delay(0.3); out_a = call(a, m)
  b = settled(20.0); out_b = call(b, m)                     # no attribute = the 0.3 base
  assert a._delay_extra_eff == pytest.approx(0.3) and out_a == pytest.approx(out_b, abs=1e-12)


def test_learned_delay_is_not_counted_twice():
  cd.LOOKAHEAD_DELAY_EXTRA_S = [0.3] * 5
  m = ramp_plan(0.001, 0.005)
  s = with_delay(0.57)
  call(s, m)
  assert s._delay_extra_eff == pytest.approx(0.03, abs=1e-9)
  s = with_delay(0.6)
  call(s, m)
  assert s._delay_extra_eff == 0.0 and s._delay_match == 0.0


def test_total_read_point_is_kept():
  # the delay-match reads the plan at REF + ref_shift + E_eff = 0.3 + E whatever the model delay
  cd.LOOKAHEAD_DELAY_EXTRA_S = [0.3] * 5
  m = ramp_plan(0.001, 0.005)                                # plan slope 0.002 / s
  s = with_delay(0.45)
  call(s, m)
  assert s._delay_extra_eff == pytest.approx(0.15)
  assert s._delay_match == pytest.approx(0.002 * 0.15, rel=0.05)


def test_orient_lead_moves_with_the_action():
  # a step in the plan at 1.1 s: seen by the lead window (0.3 .. 0.98 s) only once the window moves with a 0.6 s delay
  cd.LOOKAHEAD_DELAY_EXTRA_S = [0.0] * 5
  m = model(0.002, 0.006, t_step=1.1)
  out_base = call(with_delay(0.3), m)
  out_late = call(with_delay(0.6), m)
  assert out_late > out_base


def test_kill_switch():
  cd.LOOKAHEAD_DELAY_EXTRA_S = [0.3] * 5
  cd.LOOKAHEAD_DELAY_FOLLOW_MODEL = False
  m = ramp_plan(0.001, 0.005)
  s = with_delay(0.6)
  out = call(s, m)
  assert s._delay_extra_eff == pytest.approx(0.3)
  assert out == pytest.approx(call(settled(20.0), m), abs=1e-12)


class _Delay:
  def __init__(self, d):
    self.lateralDelay = d


def ctl(d):
  c = types.SimpleNamespace(params=FakeParams(), sm={"lateralDelay": _Delay(d)})
  c._update = lambda: cd.Controls._update_model_lat_delay(c)
  return c


def test_model_delay_source_follows_the_toggle():
  FakeParams().put_bool("LagdToggle", True)
  FakeParams().put("LagdValueCache", 0.3)
  c = ctl(0.57); c._update()
  assert c._model_lat_delay == pytest.approx(0.57)
  FakeParams().put_bool("LagdToggle", False)
  c = ctl(0.57); c._update()
  assert c._model_lat_delay == pytest.approx(0.3)            # toggle off: modeld uses the cached fixed value


def test_bad_delay_falls_back_to_base():
  FakeParams().put_bool("LagdToggle", True)
  FakeParams().put("LagdValueCache", 0.3)
  c = ctl(float("nan")); c._update()
  assert c._model_lat_delay == cd.LOOKAHEAD_DELAY_BASE_S
