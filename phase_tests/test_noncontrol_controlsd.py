"""Phase 6h-2 / 7c controlsd helpers — _lookahead_curvature (bounded-additive
lookahead) and _predicted_lat_accel_excess, exercised as the real unbound
methods on a minimal stub self."""
import math
import types

import numpy as np
import pytest

from phase_tests.harness_noncontrol import FakeParams  # noqa: F401 (stub install side effect)

import openpilot.selfdrive.controls.controlsd as cd


class StubSelf:
  def __init__(self, desired_curvature=0.002):
    self.desired_curvature = desired_curvature
    self._model_nonfinite_frames = 0


def mk_model(xs, ys, fallback=0.002):
  return types.SimpleNamespace(
    action=types.SimpleNamespace(desiredCurvature=fallback),
    position=types.SimpleNamespace(x=list(xs), y=list(ys)),
  )


def parabola_model(k=0.01, n=15, dx=2.0, fallback=0.002):
  """y = k/2 x^2 -> curvature ~ k at x=0."""
  xs = [dx * i for i in range(n)]
  ys = [0.5 * k * x * x for x in xs]
  return mk_model(xs, ys, fallback=fallback)


def lookahead(s, m, v, extra=0.1):
  return cd.Controls._lookahead_curvature(s, m, v, extra)


class TestLookaheadCurvature:
  def test_nan_fallback_holds_last_command(self):
    # a non-finite model action must not propagate — it would poison the
    # 6h-1 low-pass EMA permanently (defect fixed in controlsd.py)
    s = StubSelf(desired_curvature=0.0042)
    m = parabola_model(fallback=float('nan'))
    out = lookahead(s, m, 15.0)
    assert out == pytest.approx(0.0042)
    assert math.isfinite(out)

  def test_inf_fallback_holds_last_command(self):
    s = StubSelf(desired_curvature=-0.001)
    m = parabola_model(fallback=float('inf'))
    assert lookahead(s, m, 15.0) == pytest.approx(-0.001)

  def test_sustained_nonfinite_model_ramps_to_straight(self):
    # a model that keeps emitting non-finite actions has no valid plan and
    # nothing downstream faults on it, so the held command must NOT latch
    # forever — it has to reach straight inside MODEL_NONFINITE_RAMP_END_S.
    s = StubSelf(desired_curvature=0.006)
    m = parabola_model(fallback=float('nan'))
    hold_frames = int(cd.MODEL_NONFINITE_HOLD_S / cd.DT_CTRL)
    for _ in range(hold_frames):
      out = lookahead(s, m, 20.0)
      assert out == pytest.approx(0.006)         # pure hold, no jerk
      s.desired_curvature = out                  # closes the loop as controlsd does
    for _ in range(int(cd.MODEL_NONFINITE_RAMP_END_S / cd.DT_CTRL)):
      out = lookahead(s, m, 20.0)
      assert math.isfinite(out) and abs(out) <= 0.006 + 1e-12
      s.desired_curvature = out
    assert out == pytest.approx(0.0, abs=1e-12)

  def test_nonfinite_counter_resets_on_clean_frame(self):
    s = StubSelf(desired_curvature=0.006)
    for _ in range(500):
      lookahead(s, parabola_model(fallback=float('nan')), 20.0)
    lookahead(s, parabola_model(fallback=0.002), 20.0)
    assert s._model_nonfinite_frames == 0
    assert lookahead(s, parabola_model(fallback=float('nan')), 20.0) == pytest.approx(0.006)

  def test_empty_position_returns_fallback(self):
    m = mk_model([], [], fallback=0.003)
    assert lookahead(StubSelf(), m, 15.0) == pytest.approx(0.003)

  def test_short_position_returns_fallback(self):
    m = mk_model([0, 1, 2, 3], [0, 0, 0, 0], fallback=0.003)
    assert lookahead(StubSelf(), m, 15.0) == pytest.approx(0.003)

  def test_v_zero_no_division_error(self):
    m = parabola_model(fallback=0.003)
    # v=0 -> dist_ahead=0 < 0.3 -> fallback, and dk_max denominator is
    # floored at 5 m/s so no ZeroDivisionError anywhere
    assert lookahead(StubSelf(), m, 0.0) == pytest.approx(0.003)

  def test_nan_trajectory_returns_fallback(self):
    m = parabola_model(fallback=0.003)
    m.position.y[7] = float('nan')
    assert lookahead(StubSelf(), m, 15.0) == pytest.approx(0.003)

  def test_straight_gate_passthrough(self):
    m = parabola_model(k=0.01, fallback=0.0005)  # |fb| < 0.0008 gate
    assert lookahead(StubSelf(), m, 15.0) == pytest.approx(0.0005)

  def test_additive_bound_dk_max(self):
    # strongly curved trajectory vs small fallback: lead must clip at dk_max
    fb = 0.002
    v = 15.0
    m = parabola_model(k=0.05, fallback=fb)
    out = lookahead(StubSelf(), m, v, extra=0.1)
    # recompute the bound exactly as the code does
    base_s = float(np.interp(v, [5.6, 13.9, 27.8, 38.9], [0.08, 0.10, 0.13, 0.18]))
    boost_s = float(np.interp(abs(fb), [0.0008, 0.005], [0.0, 0.20]))
    t_ahead = min(base_s + boost_s + 0.1, cd.LOOKAHEAD_T_AHEAD_CAP + 0.1)
    dk_max = cd.LOOKAHEAD_JERK_BUDGET * max(t_ahead, 0.05) / max(v, 5.0) ** 2
    assert out == pytest.approx(fb + dk_max)  # clipped, positive direction
    assert abs(out - fb) <= dk_max + 1e-12

  def test_additive_bound_negative_direction(self):
    fb = -0.002
    m = parabola_model(k=-0.05, fallback=fb)
    out = lookahead(StubSelf(), m, 15.0)
    assert out < fb  # lead pushes further negative...
    assert abs(out - fb) <= cd.LOOKAHEAD_JERK_BUDGET * (cd.LOOKAHEAD_T_AHEAD_CAP + 0.1) / 25.0 + 1e-12

  def test_trajectory_shorter_than_lookahead_falls_back(self):
    # x range shorter than dist_ahead -> no extrapolation, fallback
    m = parabola_model(k=0.01, n=15, dx=0.05, fallback=0.003)
    assert lookahead(StubSelf(), m, 30.0) == pytest.approx(0.003)


class TestPredictedLatAccelExcess:
  def pred(self, m, v, look=1.5):
    return cd.Controls._predicted_lat_accel_excess(StubSelf(), m, v, lookahead_s=look)

  def test_low_speed_zero(self):
    m = parabola_model()
    assert self.pred(m, 1.9) == 0.0

  def test_short_trajectory_zero(self):
    m = mk_model([0, 1], [0, 0])
    assert self.pred(m, 15.0) == 0.0

  def test_nan_trajectory_zero(self):
    m = parabola_model()
    m.position.x[3] = float('nan')
    assert self.pred(m, 15.0) == 0.0

  def test_sane_trajectory_matches_geometry(self):
    k = 0.005
    v = 10.0
    m = parabola_model(k=k, n=24, dx=2.0)
    out = self.pred(m, v)
    # cubic fit of a parabola: curvature at dist d is exactly k (2nd deriv)
    expected = v * v * k / cd.LAT_ACCEL_ENVELOPE
    assert out == pytest.approx(expected, rel=0.05)
    assert out >= 0.0

  def test_result_finite_random_noise(self):
    rng = np.random.default_rng(0)
    for _ in range(50):
      n = int(rng.integers(0, 30))
      xs = np.sort(rng.uniform(0, 60, n))
      ys = rng.normal(0, 1.0, n)
      m = mk_model(xs.tolist(), ys.tolist())
      out = self.pred(m, float(rng.uniform(0, 40)))
      assert math.isfinite(out) and out >= 0.0
