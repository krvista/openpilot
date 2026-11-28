"""Phase 91: the low-speed 0.1 deg command hold is removed from the angle smoothing."""
import pytest

from opendbc.car.hyundai.carcontroller import sp_smooth_angle
from opendbc.car.hyundai.values import CarControllerParams as P
from opendbc.car.scalar import interp as _interp


def alpha(v):
  return min(float(_interp(v, P.SMOOTHING_ANGLE_VEGO_MATRIX, P.SMOOTHING_ANGLE_ALPHA_MATRIX)), 1.0)


def test_constants_are_gone():
  assert not hasattr(P, "SMOOTHING_ANGLE_DEADBAND_DEG")
  assert not hasattr(P, "SMOOTHING_ANGLE_DEADBAND_MAX_VEGO")


@pytest.mark.parametrize("v", [1.0, 5.0, 10.9, 11.0, 15.0, 30.0])
@pytest.mark.parametrize("gap", [0.01, 0.05, 0.099, 0.1, 0.5, -0.05, -0.5])
def test_plain_ema_at_every_speed(v, gap):
  last = 12.0
  assert sp_smooth_angle(v, last + gap, last) == pytest.approx(last + gap * alpha(v), abs=1e-12)


def test_small_steps_converge_instead_of_holding():
  x = 0.0
  for _ in range(200):
    x = sp_smooth_angle(5.0, 0.09, x)       # was held at 0.0 for ever
  assert x == pytest.approx(0.09, abs=1e-3)


def test_no_step_at_the_old_speed_edge():
  a = sp_smooth_angle(10.99, 10.05, 10.0); b = sp_smooth_angle(11.01, 10.05, 10.0)
  assert abs(a - b) < 1e-3
