"""Phase 63c (report §86-87): the curve trim may not exceed CURVE_TRIM_CMD_CAP_K (1.6) x the pre-trim request (floor
CURVE_TRIM_CMD_CAP_FLOOR_DEG 1.5°), on top of the Phase 63a speed cap. On the real 7a-7f drives the speed cap alone let a
1.6° highway-curve request carry a 2.9° trim (TX 3.5 x the command) while the wheel followed ~70 % of the command."""
import numpy as np
import pytest

from phase_tests.harness import Sim
from phase_tests.test_phase63 import params, KILL_63A
from opendbc.car.hyundai.values import CarControllerParams as P


def wind(cmd_deg, v_kph, wheel_deg, frames=1800, **kw):
  """Steady request against a wheel that stays put: returns (trim trace, the pre-trim request the controller used).
  The request inside the controller is the hysteresis-held command (0.15 deg less than cmd at >= 43 km/h)."""
  with params(ENTRY_BOOST_GAIN=0.0, **kw):
    sim = Sim()
    for _ in range(300):
      sim.step(v=v_kph / 3.6, tq=0.0, wheel=wheel_deg, cmd=cmd_deg)
    tr = []
    for _ in range(frames):
      sim.step(v=v_kph / 3.6, tq=0.0, wheel=wheel_deg, cmd=cmd_deg)
      tr.append(sim.cc.curve_trim)
  return np.array(tr), abs(float(sim.cc.cmd_hyst))


def bound(req):
  return max(P.CURVE_TRIM_CMD_CAP_FLOOR_DEG, P.CURVE_TRIM_CMD_CAP_K * req)


def speed_cap(v_kph):
  return float(np.interp(v_kph / 3.6, P.CURVE_TRIM_CAP_SPEEDS_MS, P.CURVE_TRIM_CAP_DEG))


def test_small_highway_request_is_bounded_by_the_command():
  # 85 km/h, 1.8° command (1.65° request), wheel at 1.1° (~65 %): speed cap ~3.1°, command bound 1.6 x 1.65 = 2.64°
  tr, req = wind(1.8, 85.0, 1.1)
  assert req == pytest.approx(1.65, abs=0.01)
  assert speed_cap(85.0) > bound(req) + 0.4                   # the test only means something if the new bound is the tighter one
  assert tr.max() <= bound(req) + 1e-6
  assert tr[-1] == pytest.approx(bound(req), abs=0.05)        # it still winds up to what the command allows


def test_kill_switch_restores_the_speed_cap():
  tr, _ = wind(1.8, 85.0, 1.1, CURVE_TRIM_CMD_CAP_K=0.0)
  assert tr[-1] == pytest.approx(speed_cap(85.0), abs=0.05)


def test_large_request_is_unchanged():
  # 60 km/h, 6° request, wheel at 3.6°: bound 9.6° > speed cap 4.86° -> same as before
  a, _ = wind(6.0, 60.0, 3.6)
  b, _ = wind(6.0, 60.0, 3.6, CURVE_TRIM_CMD_CAP_K=0.0)
  assert a[-1] == pytest.approx(b[-1], abs=1e-9) and a[-1] == pytest.approx(speed_cap(60.0), abs=0.05)


def test_bound_scales_with_the_request():
  t18 = wind(1.8, 85.0, 1.1)[0][-1]
  t22 = wind(2.2, 85.0, 1.4)[0][-1]
  assert t22 > t18 + 0.3                                      # 1.6 x 2.05 = 3.28 -> the speed cap 3.1 binds instead
  assert t22 == pytest.approx(speed_cap(85.0), abs=0.05)


def test_phase63a_kill_still_gates_small_requests():
  # with the 63a gates back at 4° the 1.6° request never arms the trim, bound or not
  tr, _ = wind(1.8, 85.0, 1.1, **KILL_63A)
  assert np.abs(tr).max() == 0.0
