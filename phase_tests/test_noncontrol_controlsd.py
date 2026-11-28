"""controlsd helpers — _lookahead_curvature (the model's action with the non-finite
hold; its bounded-additive lookahead lead was removed in Phase 89) and
_predicted_lat_accel_excess, exercised as the real unbound methods on a minimal
stub self."""
import math
import types

import numpy as np
import pytest

from phase_tests.harness_noncontrol import FakeParams  # noqa: F401 (stub install side effect)

import openpilot.selfdrive.controls.controlsd as cd


class StubSelf:
  _model_frame = cd.Controls._model_frame     # the per-model-frame cache is part of the real method's contract

  def __init__(self, desired_curvature=0.002):
    self.desired_curvature = desired_curvature
    self._model_nonfinite_frames = 0
    self._mf = None
    self.sm = types.SimpleNamespace(logMonoTime={})


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


def lookahead(s, m, v=None):
  return cd.Controls._lookahead_curvature(s, m)


class TestLookaheadCurvature:
  def test_nan_fallback_holds_last_command(self):
    # a non-finite model action must not propagate — it would poison the
    # recursive command state permanently (defect fixed in controlsd.py)
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

  def test_the_action_is_returned_whatever_the_plan_trajectory(self):
    for m in (mk_model([], [], fallback=0.003), mk_model([0, 1, 2, 3], [0, 0, 0, 0], fallback=0.003),
              parabola_model(k=0.05, fallback=0.003), parabola_model(k=-0.05, fallback=0.003),
              parabola_model(k=0.01, n=15, dx=0.05, fallback=0.003)):
      assert lookahead(StubSelf(), m) == 0.003
    m = parabola_model(fallback=0.003)
    m.position.y[7] = float('nan')
    assert lookahead(StubSelf(), m) == 0.003
    assert lookahead(StubSelf(), parabola_model(k=-0.05, fallback=-0.002)) == -0.002


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
