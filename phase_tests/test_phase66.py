"""Phase 66 (report §85): lagd lag search widened and press-free window gate (CCNC profile).
The stock search [0.15, 0.65) returned every peak at or beyond its last sample as exactly 0.600 s (76 % of this car's
accepted estimates); the CCNC profile searches to LAGD_MAX_LAG_S, accepts an estimate only when no driver press of any
length happened within LAGD_PRESS_FREE_S (the correlation window), seeds 0.65 s and bumps the cache VERSION."""
import types

import numpy as np
import pytest

from phase_tests.harness_noncontrol import FakeParams  # noqa: F401 (stub install side effect)

import openpilot.selfdrive.locationd.lagd as lagd
from openpilot.cereal import log


def cp(brand="hyundai", sct="angle"):
  return types.SimpleNamespace(brand=brand, steerControlType=sct, steerActuatorDelay=0.1, carFingerprint="X")


def est(**kw):
  return lagd.LateralLagEstimator(cp(), 0.05, **{**lagd.lagd_profile(cp()), **kw})


def test_profile_values():
  p = lagd.lagd_profile(cp())
  assert p["max_lag"] == lagd.LAGD_MAX_LAG_S == 1.0
  assert p["press_free_sec"] == lagd.LAGD_PRESS_FREE_S == lagd.MOVING_WINDOW_SEC
  assert p["initial_lag_seed"] == lagd.LAGD_SEED_S == 0.65
  assert lagd.VERSION == 2
  s = lagd.LateralLagEstimator(cp(), 0.05)                      # stock: old search, no gate
  assert s.max_lag == lagd.MAX_LAG == 0.65 and s.press_free_sec == 0.0 and s.press_free()


def delayed_pair(lag_s, dt=0.05, n=1200, seed=0):
  rng = np.random.default_rng(seed)
  x = np.convolve(rng.standard_normal(n + 400), np.ones(40) / 40, mode="same")   # slow, curve-like command
  k = int(round(lag_s / dt))
  return x[400:], x[400 - k:len(x) - k], np.ones(n, dtype=bool)


def test_search_edge_clipped_the_estimate_and_the_wide_search_does_not():
  d, a, m = delayed_pair(0.80)
  lag_old, corr_old, _ = lagd.LateralLagEstimator.actuator_delay(d, a, m, 0.05, lagd.MIN_LAG, lagd.MAX_LAG)
  lag_new, corr_new, _ = lagd.LateralLagEstimator.actuator_delay(d, a, m, 0.05, lagd.MIN_LAG, lagd.LAGD_MAX_LAG_S)
  assert lag_old == pytest.approx(0.60, abs=1e-9)             # the search edge, not the 0.80 s of the signal
  assert lag_new == pytest.approx(0.80, abs=0.03)
  assert corr_new > corr_old


def test_wide_search_keeps_a_short_lag():
  d, a, m = delayed_pair(0.35)
  lag_new, _, _ = lagd.LateralLagEstimator.actuator_delay(d, a, m, 0.05, lagd.MIN_LAG, lagd.LAGD_MAX_LAG_S)
  assert lag_new == pytest.approx(0.35, abs=0.03)


def cs(pressed):
  return types.SimpleNamespace(steeringPressed=pressed, vEgo=25.0)


def test_press_free_gate_covers_the_whole_window():
  e = est()
  e.handle_log(10.0, "carState", cs(False)); assert e.press_free()
  e.handle_log(10.1, "carState", cs(True))                      # a 0.1 s touch: not "steering" for Phase 64a ...
  e.handle_log(10.2, "carState", cs(False))
  assert not e.steering_pressed
  for t in (10.3, 40.0, 69.9):
    e.handle_log(t, "carState", cs(False)); assert not e.press_free()   # ... but it blocks estimates for 60 s
  e.handle_log(70.2, "carState", cs(False)); assert e.press_free()


def test_gate_kill_switch_and_stock():
  e = est(press_free_sec=0.0)
  e.handle_log(1.0, "carState", cs(True)); e.handle_log(1.1, "carState", cs(False))
  assert e.press_free()


def test_published_value_is_capped_at_the_profile_search_end():
  e = est()
  e.reset(0.90, lagd.BLOCK_NUM_NEEDED + 2)
  assert e.get_msg(True).lateralDelay.lateralDelay == pytest.approx(0.90)       # was min(0.65, ...)
  s = lagd.LateralLagEstimator(cp(), 0.05); s.reset(0.90, lagd.BLOCK_NUM_NEEDED + 2)
  assert s.get_msg(True).lateralDelay.lateralDelay == pytest.approx(lagd.MAX_LAG)


def test_seed_before_learning():
  e = est()
  m = e.get_msg(True).lateralDelay
  assert str(m.status) == "unestimated" and m.lateralDelay == pytest.approx(0.65)


class _P:
  def __init__(self, lag_bytes, cp_bytes):
    self.d = {"LiveDelay": lag_bytes, "CarParamsPrevRoute": cp_bytes}; self.removed = []
  def get(self, k, **kw): return self.d.get(k)
  def remove(self, k): self.removed.append(k)


def test_old_version_cache_is_dropped():
  from opendbc.car.structs import car
  CP = car.CarParams.new_message(carFingerprint="X")
  e = est(); e.reset(0.60, 7)
  msg = e.get_msg(True); msg.lateralDelay.version = 1          # cache written by the clipped search
  p = _P(msg.to_bytes(), CP.to_bytes())
  assert lagd.retrieve_initial_lag(p, CP) is None and p.removed == ["LiveDelay"]
  msg.lateralDelay.version = lagd.VERSION
  p = _P(msg.to_bytes(), CP.to_bytes())
  lag, blocks = lagd.retrieve_initial_lag(p, CP)
  assert blocks == 7 and lag == pytest.approx(0.60)
