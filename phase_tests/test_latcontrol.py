"""Phase 7a/7b curvature-trim integrator in latcontrol_angle: NaN robustness and
trim invariants (bounded, bleeds on press/inactive)."""
import math
import types

import numpy as np
import pytest

from phase_tests.harness import make_cp  # path setup + real CP
from opendbc.car.vehicle_model import VehicleModel
import openpilot.selfdrive.controls.lib.latcontrol_angle as lca

DT = 0.01


class FakeCS:
  def __init__(self, v=15.0, angle=0.0, pressed=False):
    self.vEgo = v
    self.steeringAngleDeg = angle
    self.steeringPressed = pressed


def make_lac():
  CP = make_cp()
  lac = lca.LatControlAngle.__new__(lca.LatControlAngle)
  # minimal base state used by update()
  lac.sat_check_min_speed = 5.0
  lac.use_steer_limited_by_safety = True
  lac.dt = DT
  lac._roll_lp = 0.0
  lac._roll_lp_init = False
  lac._fb_integ = 0.0
  lac._des_slow = 0.0
  lac._fb_err_lp = 0.0
  lac.sat_time = 0.0
  lac.sat_limit = getattr(CP, 'steerLimitTimer', 0.4)
  return lac, VehicleModel(CP)


def step(lac, VM, active=True, v=15.0, angle=0.0, pressed=False, desired=0.0, roll=0.0):
  params = types.SimpleNamespace(roll=roll, angleOffsetDeg=0.0)
  CS = FakeCS(v=v, angle=angle, pressed=pressed)
  _, out, _ = lac.update(active, CS, VM, params, False, desired, None, False, 0.1)
  return out


class TestLatFbInteg:
  def test_trim_bounded_by_cap(self):
    lac, VM = make_lac()
    for _ in range(5000):
      step(lac, VM, v=20.0, angle=0.0, desired=8e-4)
      cap = min(lca.LAT_FB_CAP, lca.LAT_FB_ACCEL_CAP / max(20.0, 5.0) ** 2)
      assert abs(lac._fb_integ) <= cap + 1e-12

  def test_trim_bleeds_while_pressed(self):
    lac, VM = make_lac()
    for _ in range(2000):
      step(lac, VM, v=20.0, desired=8e-4)
    t0 = abs(lac._fb_integ)
    assert t0 > 0
    for _ in range(100):
      step(lac, VM, v=20.0, desired=8e-4, pressed=True)
      assert abs(lac._fb_integ) <= t0
    assert abs(lac._fb_integ) < t0

  def test_nan_wheel_angle_single_frame_does_not_poison(self):
    lac, VM = make_lac()
    for _ in range(500):
      step(lac, VM, v=20.0, desired=6e-4, angle=1.0)
    # one NaN CAN frame on the measured angle
    out = step(lac, VM, v=20.0, desired=6e-4, angle=float('nan'))
    # recovery frames: integrator state and output must return to finite
    for _ in range(50):
      out = step(lac, VM, v=20.0, desired=6e-4, angle=1.0)
    assert np.isfinite(lac._fb_integ), "fb integrator permanently poisoned by one NaN frame"
    assert np.isfinite(lac._fb_err_lp)
    assert np.isfinite(out), "TX angle stuck non-finite after NaN frame passed"

  def test_nan_desired_curvature_does_not_poison(self):
    lac, VM = make_lac()
    for _ in range(500):
      step(lac, VM, v=20.0, desired=6e-4, angle=1.0)
    step(lac, VM, v=20.0, desired=float('nan'), angle=1.0)
    for _ in range(50):
      out = step(lac, VM, v=20.0, desired=6e-4, angle=1.0)
    assert np.isfinite(lac._fb_integ)
    assert np.isfinite(lac._des_slow)
    assert np.isfinite(out)

  def test_sign_symmetry(self):
    lp, VMp = make_lac()
    ln, VMn = make_lac()
    for i in range(1000):
      d = 6e-4 * math.sin(0.01 * i)
      a = 3.0 * math.sin(0.008 * i)
      step(lp, VMp, v=20.0, desired=d, angle=a)
      step(ln, VMn, v=20.0, desired=-d, angle=-a)
    assert lp._fb_integ == pytest.approx(-ln._fb_integ, abs=1e-15)


class TestLatFbInteg7a6:
  # Phase 7a-6: a constant sub-deadband bias must not wind the trim to the cap; a corner deficit still must.
  def test_sub_deadband_bias_does_not_wind_up(self):
    lac, VM = make_lac()
    for _ in range(3000):                                  # 30 s of a 0.1e-3 bias (route 7 hands-off median 0.14e-3)
      step(lac, VM, v=14.0, desired=1e-4)
    assert abs(lac._fb_integ) < 0.05e-3, lac._fb_integ

  def test_corner_deficit_still_reaches_cap(self):
    lac, VM = make_lac()
    for _ in range(300):                                   # 3 s of a 0.8e-3 corner deficit
      step(lac, VM, v=14.0, desired=8e-4)
    cap = min(lca.LAT_FB_CAP, lca.LAT_FB_ACCEL_CAP / 14.0 ** 2)
    assert abs(lac._fb_integ) >= 0.95 * cap, (lac._fb_integ, cap)

  def test_leak_settles_below_cap_for_moderate_bias(self):
    lac, VM = make_lac()
    for _ in range(6000):                                  # 60 s of 0.4e-3
      step(lac, VM, v=14.0, desired=4e-4)
    expected = lca.LAT_FB_KI * lca.LAT_FB_LEAK_TAU * (4e-4 - lca.LAT_FB_ERR_DEADBAND)   # 0.8e-3 steady state
    assert abs(abs(lac._fb_integ) - expected) < 0.1e-3, (lac._fb_integ, expected)

  def test_kill_switch_restores_7a5(self):
    old = (lca.LAT_FB_ERR_DEADBAND, lca.LAT_FB_LEAK_TAU)
    try:
      lca.LAT_FB_ERR_DEADBAND = 0.0; lca.LAT_FB_LEAK_TAU = 0.0
      lac, VM = make_lac()
      for _ in range(3000):
        step(lac, VM, v=14.0, desired=1e-4)
      cap = min(lca.LAT_FB_CAP, lca.LAT_FB_ACCEL_CAP / 14.0 ** 2)
      assert abs(lac._fb_integ) >= 0.95 * cap                 # 7a-5 behaviour: a constant bias winds to the cap
    finally:
      lca.LAT_FB_ERR_DEADBAND, lca.LAT_FB_LEAK_TAU = old

  def test_leak_scales_with_cap_so_highway_does_not_saturate_on_a_small_bias(self):
    # review: with a fixed leak tau the cap (0.5/v^2) shrank faster than the leak ceiling on the highway
    for v in (14.0, 30.0, 36.0):
      lac, VM = make_lac()
      for _ in range(6000):
        step(lac, VM, v=v, desired=3e-4)                   # 0.3e-3 steady bias, 60 s
      cap = min(lca.LAT_FB_CAP, lca.LAT_FB_ACCEL_CAP / v ** 2)
      assert abs(lac._fb_integ) < 0.9 * cap, (v, lac._fb_integ, cap)
