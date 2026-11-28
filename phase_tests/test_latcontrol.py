"""latcontrol_angle: pure feed-forward angle from the desired curvature (the Phase 7a/7b closed-loop curvature trim and
its tests were removed in Phase 89). What stays: the output is a function of this frame's inputs and the roll low-pass
only - nothing winds up, a non-finite frame does not poison later frames, left and right are symmetric."""
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
  return lca.LatControlAngle(CP, None, None, DT), VehicleModel(CP)


def step(lac, VM, active=True, v=15.0, angle=0.0, pressed=False, desired=0.0, roll=0.0):
  params = types.SimpleNamespace(roll=roll, angleOffsetDeg=0.0)
  CS = FakeCS(v=v, angle=angle, pressed=pressed)
  _, out, _ = lac.update(active, CS, VM, params, False, desired, None, False, 0.1)
  return out


class TestFeedForward:
  def test_output_is_the_vehicle_model_angle_and_nothing_winds_up(self):
    lac, VM = make_lac()
    want = math.degrees(VM.get_steer_from_curvature(-8e-4, 20.0, 0.0))
    outs = [step(lac, VM, v=20.0, angle=0.0, desired=8e-4) for _ in range(3000)]   # the wheel never delivers it
    assert outs[0] == pytest.approx(want) and len(set(outs)) == 1
    assert not any(hasattr(lac, name) for name in ("_fb_integ", "_des_slow", "_fb_err_lp", "_trim_7a6", "direct"))

  def test_inactive_returns_the_wheel_angle(self):
    lac, VM = make_lac()
    assert step(lac, VM, active=False, v=20.0, angle=3.7, desired=8e-4) == 3.7

  def test_nan_wheel_angle_single_frame_does_not_poison(self):
    lac, VM = make_lac()
    for _ in range(500):
      step(lac, VM, v=20.0, desired=6e-4, angle=1.0)
    step(lac, VM, v=20.0, desired=6e-4, angle=float('nan'))      # one NaN CAN frame on the measured angle
    out = step(lac, VM, v=20.0, desired=6e-4, angle=1.0)
    assert np.isfinite(out), "TX angle stuck non-finite after NaN frame passed"

  def test_nan_desired_curvature_does_not_poison(self):
    lac, VM = make_lac()
    for _ in range(500):
      step(lac, VM, v=20.0, desired=6e-4, angle=1.0)
    assert not np.isfinite(step(lac, VM, v=20.0, desired=float('nan'), angle=1.0))   # (controlsd zeroes a non-finite actuator)
    assert np.isfinite(step(lac, VM, v=20.0, desired=6e-4, angle=1.0))

  def test_nan_roll_holds_the_last_filtered_roll(self):
    lac, VM = make_lac()
    for _ in range(300):
      a = step(lac, VM, v=20.0, desired=6e-4, roll=0.03)
    assert step(lac, VM, v=20.0, desired=6e-4, roll=float('nan')) == a

  def test_sign_symmetry(self):
    lp, VMp = make_lac()
    ln, VMn = make_lac()
    for i in range(1000):
      d = 6e-4 * math.sin(0.01 * i)
      a = 3.0 * math.sin(0.008 * i)
      assert step(lp, VMp, v=20.0, desired=d, angle=a) == pytest.approx(-step(ln, VMn, v=20.0, desired=-d, angle=-a), abs=1e-12)
