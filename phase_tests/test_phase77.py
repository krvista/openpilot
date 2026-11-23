"""Phase 77 (route a7, report 107): lane-centre term - faster / larger integral and a proportional part on the car's
own offset by speed; engage taper starting faster after the driver's own hand."""
import numpy as np
import pytest

import openpilot.selfdrive.controls.controlsd as cd
from phase_tests.harness import Sim
from phase_tests.test_phase73 import model, term
from opendbc.car.hyundai.values import CarControllerParams as P

KPH = 1 / 3.6


@pytest.fixture
def keep():
  names = ("LANE_CENTER_KE", "LANE_CENTER_KI", "LANE_CENTER_I_CAP", "LANE_CENTER_WEIGHT")
  saved = {k: getattr(cd, k) for k in names}; t = P.ENGAGE_TAPER_DRIVER_START_DPS
  yield
  for k, v in saved.items():
    setattr(cd, k, v)
  P.ENGAGE_TAPER_DRIVER_START_DPS = t


def test_shipped_values():
  assert cd.LANE_CENTER_KI == 0.06e-3 and cd.LANE_CENTER_I_CAP == 0.30e-3
  assert cd.LANE_CENTER_KE == [0.0, 0.5e-3, 0.8e-3]
  assert cd.LANE_CENTER_KE_SPEED_MS == pytest.approx([45 * KPH, 60 * KPH, 80 * KPH])
  assert P.ENGAGE_TAPER_DRIVER_START_DPS == 10.0


def test_offset_part_by_speed_and_sign(keep):
  cd.LANE_CENTER_KI = 0.0
  e = 0.10 + cd.LANE_CENTER_BIAS_M                                    # lane centre 10 cm to the right of the car
  for v_kph, ke in ((40, 0.0), (45, 0.0), (60, 0.5e-3), (70, 0.65e-3), (80, 0.8e-3), (110, 0.8e-3)):
    out, s = term(model(centre=0.10), v=v_kph * KPH, n=400)
    assert s._lc_e == pytest.approx(ke * e, rel=0.02, abs=1e-9), v_kph
    assert out == pytest.approx(s._lc_p + s._lc_e, rel=1e-6) and out > 0.0
  _, s = term(model(centre=-0.10), v=80 * KPH, n=400)                 # centre to the left: steer left
  assert s._lc_e == pytest.approx(0.8e-3 * (-0.10 + cd.LANE_CENTER_BIAS_M), rel=0.02) and s._lc_e < 0.0


def test_offset_part_works_in_a_curve_where_the_integral_is_frozen(keep):
  m = model(k_plan=2.5e-3, centre=-0.12)                              # right curve, car 12 cm inside (right of the centre)
  out, s = term(m, v=80 * KPH, n=600)
  assert s._lc_i == 0.0
  assert s._lc_e == pytest.approx(0.8e-3 * (-0.12 + cd.LANE_CENTER_BIAS_M), rel=0.02)
  cd.LANE_CENTER_KE = [0.0, 0.0, 0.0]
  out0, s0 = term(m, v=80 * KPH, n=600)
  assert s0._lc_e == 0.0 and out < out0 < 0.0                         # the new part adds outward curvature


def test_both_proportional_parts_share_the_cap_and_low_pass(keep):
  cd.LANE_CENTER_KI = 0.0
  v = 80 * KPH; cap = cd.LANE_CENTER_ACC / v ** 2
  out, s = term(model(centre=3.0), v=v, n=600)
  assert out == pytest.approx(cap, rel=0.02)
  o1, s1 = term(model(centre=0.10), v=v, n=1)                         # first frame: low-passed, not a step
  assert abs(s1._lc_e) < 0.05 * 0.8e-3 * 0.07


def test_integral_is_faster_and_reaches_the_new_cap(keep):
  out, s = term(model(centre=0.25), v=21.0, n=300)
  assert s._lc_i == pytest.approx(0.06e-3 * (0.25 + cd.LANE_CENTER_BIAS_M) * 3.0, rel=0.02)
  out, s = term(model(centre=0.25), v=21.0, n=6000, s=s)
  assert s._lc_i == pytest.approx(0.30e-3)


def test_unusable_context_fades_the_offset_part_and_nan_is_safe(keep):
  _, s = term(model(centre=0.10), v=80 * KPH, n=400)
  assert s._lc_e > 0.0
  _, s = term(model(centre=0.10), v=80 * KPH, n=300, s=s, usable=False)
  assert abs(s._lc_e) < 1e-7
  bad = model(centre=0.10); bad.laneLines[1].y[0] = float('nan')
  out, s = term(bad, v=80 * KPH, n=10, s=s)
  assert np.isfinite(out) and np.isfinite(s._lc_e)


def after_hand(start_dps, v_kph=60.0):
  """the driver holds the wheel 6 deg off the command with a firm grip, lets go; the request then walks to the command"""
  P.ENGAGE_TAPER_DRIVER_START_DPS = start_dps
  sim = Sim(solo=True)
  for _ in range(200):
    sim.step(v=v_kph * KPH, cmd=0.0, wheel=0.0, lfa_state=0)
  for _ in range(150):
    sim.step(v=v_kph * KPH, cmd=6.0, wheel=0.0, tq=400.0, pressed=True, lfa_state=0)
  tx = []
  for _ in range(150):
    sim.step(v=v_kph * KPH, cmd=6.0, wheel=0.0, tq=0.0, pressed=False, lfa_state=0)
    tx.append(sim.cc.apply_angle_last)
  return np.array(tx), sim


def test_taper_starts_faster_after_the_drivers_hand(keep):
  new, sim = after_hand(10.0)
  old, _ = after_hand(4.0)
  assert sim.cc.taper_driver
  i = int(np.argmax(np.abs(np.diff(new)) > 1e-6))                     # first frame the request moves
  assert new[i + 20] - new[i] > old[i + 20] - old[i] + 0.5            # 0.2 s on: clearly further
  assert new[i + 20] - new[i] <= 0.2 * 16.0 + 0.2                     # still tapered (10 -> ~13 deg/s), not the full rate
  assert np.max(np.abs(np.diff(new))) <= 0.31


def test_other_anchors_keep_the_slow_start(keep):
  sim = Sim(solo=True); tx = []
  for _ in range(300):
    sim.step(v=42 * KPH, cmd=8.0, wheel=0.0, lat_active=False, lfa_state=0)
  for _ in range(40):
    sim.step(v=42 * KPH, cmd=8.0, wheel=0.0, lat_active=True, lfa_state=0); tx.append(sim.cc.apply_angle_last)
  assert not sim.cc.taper_driver and tx[20] - tx[0] <= 1.3
