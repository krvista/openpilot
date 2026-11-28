"""Phase 64 (report §76).
64a: lagd CCNC profile — Hyundai angle-steering (CCNC) cars learn with NCC >= 0.9, confidence >= 0.5, >= 60 km/h, a 1 s
recovery, and only a press held >= 0.5 s counts as steering (short HOD "hands on" touches do not blank the learner).
64b: controlsd tracks the delay modeld is actually told (_update_model_lat_delay: lagd's value with LagdToggle on, else
the cached fixed value). The Phase 60 delay shift that was corrected by it was removed in Phase 89 (and its tests); the
tracked delay still times the Phase 78 entry lead.
The lagd tests run on the only profile left since Phase 89 (seed 0.32 s, published <= 0.50 s)."""
import types

import numpy as np
import pytest

from phase_tests.harness_noncontrol import FakeParams  # noqa: F401 (stub install side effect)

import openpilot.selfdrive.controls.controlsd as cd
import openpilot.selfdrive.locationd.lagd as lagd


@pytest.fixture(autouse=True)
def _restore():
  prof = lagd.CCNC_PROFILE
  yield
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
  assert e.initial_lag == pytest.approx(lagd.LAGD_SOLO_SEED_S)      # 0.32, not steerActuatorDelay + 0.2 = 0.30
  msg = e.get_msg(True)                                             # before any learning
  assert str(msg.lateralDelay.status) == "unestimated"
  assert msg.lateralDelay.lateralDelay == pytest.approx(lagd.LAGD_SOLO_SEED_S)   # what modeld receives


def test_seed_kill_switch_restores_stock_base():
  e = lagd.LateralLagEstimator(cp(), 0.05)                          # no profile kwargs
  assert e.initial_lag == pytest.approx(0.1 + 0.2)                  # steerActuatorDelay + 0.2
  assert lagd.lagd_profile(cp())["initial_lag_seed"] == lagd.LAGD_SOLO_SEED_S


def test_learned_value_overrides_the_seed():
  # once estimated, modeld gets the learned block mean, not the seed (adaptive)
  e = lagd.LateralLagEstimator(cp(), 0.05, **lagd.lagd_profile(cp()))
  e.reset(0.40, lagd.BLOCK_NUM_NEEDED + 2)                          # as if learned 0.40 over >= 5 blocks
  msg = e.get_msg(True)
  assert str(msg.lateralDelay.status) == "estimated"
  assert msg.lateralDelay.lateralDelay == pytest.approx(0.40, abs=1e-6)
  assert msg.lateralDelay.lateralDelay != pytest.approx(lagd.LAGD_SOLO_SEED_S)


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
