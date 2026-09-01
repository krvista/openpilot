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
