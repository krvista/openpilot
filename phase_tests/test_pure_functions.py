import math
import random

import numpy as np
import pytest

from phase_tests.harness import make_cp  # noqa: F401 (path setup side effect)
from opendbc.car.hyundai.carcontroller import compute_torque_reduction_gain, sp_smooth_angle
from opendbc.car.hyundai.values import CarControllerParams


def run_gain_seq(n, tq_fn, v_kph=50.0, err_fn=lambda i: 0.0, lat=True, blinker=False,
                 pressed=False, last=0.0):
  out = []
  for i in range(n):
    kw = {}
    if pressed:
      kw = dict(grip_full=CarControllerParams.ACIGAIN_GRIP_FULL_NM,
                grip_floor=CarControllerParams.ACIGAIN_GRIP_FLOOR,
                suppress_error_boost=True)
    last = compute_torque_reduction_gain(tq_fn(i), v_kph, lat, last, err_fn(i),
                                         blinker_on=blinker, **kw)
    out.append(last)
  return out


class TestACIGain:
  def test_output_bounds_random_sweep(self):
    rng = random.Random(0)
    last = 0.0
    for _ in range(20000):
      last = compute_torque_reduction_gain(
        rng.uniform(-800, 800), rng.uniform(0, 160), rng.random() < 0.9, last,
        rng.uniform(-5, 5), blinker_on=rng.random() < 0.3,
        grip_full=rng.choice([260.0, 350.0]), grip_floor=rng.choice([0.10, 0.19]),
        suppress_error_boost=rng.random() < 0.5)
      assert 0.0 <= last <= 1.0, f"gain out of [0,1]: {last!r}"

  def test_saturation_at_one_exact(self):
    # drive to the ceiling with a large error boost at high speed
    seq = run_gain_seq(2000, lambda i: 0.0, v_kph=130.0, err_fn=lambda i: 3.0)
    assert max(seq) <= 1.0
    assert min(seq) >= 0.0

  def test_rate_limits_per_frame(self):
    rng = random.Random(1)
    last = 0.0
    for i in range(5000):
      tq = rng.uniform(-700, 700)
      err = rng.uniform(-3, 3)
      lat = rng.random() < 0.8
      g = compute_torque_reduction_gain(tq, 60.0, lat, last, err)
      step = g - last
      rate_dn = float(np.interp(abs(tq), [0, 300, 700], [0.004, 0.01, 0.04]))
      rate_up = float(np.interp(abs(err), [0.5, 1.5], [0.004, 0.04]))
      # 0.004 quantization can add up to half an LSB on either side
      assert step <= rate_up + 0.0021, (i, step, rate_up)
      assert -step <= rate_dn + 0.0021, (i, step, rate_dn)
      last = g

  def test_blinker_only_lowers_ceiling(self):
    # steady state with and without blinker must satisfy g_blinker <= g_plain
    for tq in (0.0, 120.0, 180.0, 250.0, 310.0, 400.0):
      for v in (0.0, 20.0, 60.0, 120.0):
        g_plain = run_gain_seq(1500, lambda i: tq, v_kph=v)[-1]
        g_blink = run_gain_seq(1500, lambda i: tq, v_kph=v, blinker=True)[-1]
        assert g_blink <= g_plain + 1e-9, (tq, v, g_blink, g_plain)

  def test_inactive_decays_to_zero_never_negative(self):
    seq = run_gain_seq(300, lambda i: 100.0, lat=True)
    last = seq[-1]
    for i in range(2000):
      last = compute_torque_reduction_gain(100.0, 50.0, False, last, 0.0)
      assert last >= 0.0
    assert last == 0.0

  def test_kill_switch_blinker_flat(self, monkeypatch):
    # [0.45, 0.45] = pre-10b flat per comment: START == FULL degenerate interp
    monkeypatch.setattr(CarControllerParams, 'ACIGAIN_BLINKER_GATE_START_NM', 220.0)
    monkeypatch.setattr(CarControllerParams, 'ACIGAIN_BLINKER_GATE_FULL_NM', 220.0)
    g = run_gain_seq(1500, lambda i: 0.0, v_kph=120.0, blinker=True)[-1]
    # hands-off: interp at 0 Nm below START -> 0.45 ceiling
    assert g <= 0.45 + 1e-9

  def test_nan_raises_documenting_caller_contract(self):
    # The function is NOT NaN-safe (round() raises) — this is acceptable ONLY
    # because the single call site sanitizes all inputs (steer_torque_safe,
    # v_ego_safe, apply_angle_last chain). This test pins that contract: if
    # someone adds an unsanitized call path, the Sim-level NaN tests in
    # test_state_reset.py are the guard. Here we just document the behavior.
    with pytest.raises(ValueError):
      compute_torque_reduction_gain(float('nan'), 50.0, True, 0.5, 0.0)
    assert math.isfinite(compute_torque_reduction_gain(100.0, 50.0, True, 0.5, 0.0))


class TestSmoothAngle:
  def test_deadband_holds(self):
    assert sp_smooth_angle(5.0, 10.05, 10.0) == 10.0

  def test_output_between_last_and_target(self):
    rng = random.Random(2)
    for _ in range(5000):
      v = rng.uniform(0, 25)
      a = rng.uniform(-90, 90)
      last = rng.uniform(-90, 90)
      out = sp_smooth_angle(v, a, last)
      lo, hi = min(a, last), max(a, last)
      assert lo - 1e-9 <= out <= hi + 1e-9

  def test_sign_symmetry(self):
    rng = random.Random(3)
    for _ in range(2000):
      v = rng.uniform(0, 25)
      a = rng.uniform(-90, 90)
      last = rng.uniform(-90, 90)
      assert sp_smooth_angle(v, a, last) == pytest.approx(-sp_smooth_angle(v, -a, -last), abs=1e-12)
