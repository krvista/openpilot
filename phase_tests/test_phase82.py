"""Phase 82 (report 114): lane fit - a quadratic through the lane centre near the car gives offset, heading and curvature;
part of the difference between the curvature that centres the car and the model action is added (>= 45 km/h)."""
import numpy as np
import pytest

import openpilot.selfdrive.controls.controlsd as cd
from phase_tests.test_phase73 import model, term, XS

KPH = 1 / 3.6


@pytest.fixture
def keep():
  names = ("LANE_FIT_GAIN", "LANE_FIT_ARM_M", "LANE_CENTER_KI", "LANE_CENTER_WEIGHT", "LANE_CENTER_KE", "LANE_FIT_SPEED_MS")
  saved = {k: getattr(cd, k) for k in names}
  cd.LANE_CENTER_KI = 0.0; cd.LANE_CENTER_WEIGHT = 1e-12              # only the fit part (the weight must stay > 0)
  cd.LANE_FIT_SPEED_MS = [45 * KPH, 60 * KPH]                         # the Phase 82 gate (Phase 88 ships 40-55 km/h)
  yield
  for k, v in saved.items():
    setattr(cd, k, v)


def fit_part(m, v, n=400, **kw):
  out, s = term(m, v=v, n=n, **kw)
  return s._lc_f, out, s


def test_shipped_values():
  assert cd.LANE_FIT_GAIN == 0.35 and cd.LANE_FIT_ARM_M == 1.8 and cd.LANE_FIT_T_S == 1.5 and cd.LANE_FIT_RANGE_S == 1.0
  assert cd.LANE_FIT_SPEED_MS == pytest.approx([40 * KPH, 55 * KPH]) and cd.LANE_FIT_RANGE_M == (15.0, 45.0)   # Phase 88
  assert cd.LANE_CENTER_KE == [0.0, 0.0, 0.0, 0.0]                    # replaced by the fit
  assert cd.LANE_CENTER_KI == 0.06e-3 and cd.LANE_CENTER_I_CAP == 0.30e-3


def test_quadratic_fit_reads_offset_heading_and_curvature():
  m = model(k_plan=0.0, centre=0.20, k_lane=2e-3)
  for ln in m.laneLines:                                              # add a heading of 10 mrad to the lane
    ln.y = [y + 0.010 * x for y, x in zip(ln.y, XS)]
  c0, psi, k = cd._ModelFrame(m, 0).lane_fit(30.0)
  assert c0 == pytest.approx(0.20, abs=1e-6) and psi == pytest.approx(0.010, abs=1e-6) and k == pytest.approx(2e-3, rel=1e-6)
  bad = model(); bad.laneLines[1].y[3] = float('nan')
  assert cd._ModelFrame(bad, 0).lane_fit(30.0) is None
  assert cd._ModelFrame(model(), 0).lane_fit(1.0) is None             # fewer than 3 points


def test_offset_alone_follows_the_formula(keep):
  v = 70 * KPH; e = 0.10 + cd.LANE_CENTER_BIAS_M; L = v * 1.5
  f, out, s = fit_part(model(centre=0.10), v)                         # centre 10 cm to the right, straight lane, straight plan
  assert f == pytest.approx(0.35 * 1.432 * e / L ** 2, rel=0.02) and f > 0.0
  assert out == pytest.approx(f, rel=1e-3, abs=1e-9)
  f2, _, _ = fit_part(model(centre=-0.10), v)
  assert f2 < 0.0


def test_no_action_when_the_model_already_follows_the_lane(keep):
  v = 70 * KPH; k = 2.0e-3
  m = model(k_plan=k, centre=-cd.LANE_CENTER_BIAS_M, k_lane=k)        # on target, lane and action have the same curvature
  for ln in m.laneLines:
    ln.y = [y + cd.LANE_FIT_ARM_M * k * x for y, x in zip(ln.y, XS)]  # the heading a camera 1.8 m ahead of the axle sees
  f, _, _ = fit_part(m, v)
  assert abs(f) < 0.01 * 0.35 * k


def test_action_tighter_than_the_lane_is_eased_and_inside_offset_pushes_out(keep):
  v = 70 * KPH; k = 2.0e-3
  def curve(k_act, centre):
    m = model(k_plan=k_act, centre=centre - cd.LANE_CENTER_BIAS_M, k_lane=k)
    for ln in m.laneLines:
      ln.y = [y + cd.LANE_FIT_ARM_M * k * x for y, x in zip(ln.y, XS)]
    return fit_part(m, v)[0]
  assert curve(2.2e-3, 0.0) == pytest.approx(0.35 * (2.0e-3 - 2.2e-3), rel=0.03)      # action 10 % tighter: 35 % of it taken out
  inside = curve(2.0e-3, -0.12)                                        # right curve, car 12 cm right of (inside) the centre
  assert inside < 0.0 and inside == pytest.approx(0.35 * 1.432 * -0.12 / (v * 1.5) ** 2, rel=0.03)


def test_lever_arm_correction(keep):
  v = 70 * KPH; k = 2.0e-3
  m = model(k_plan=k, centre=-cd.LANE_CENTER_BIAS_M, k_lane=k)
  for ln in m.laneLines:
    ln.y = [y + 1.8 * k * x for y, x in zip(ln.y, XS)]
  cd.LANE_FIT_ARM_M = 0.0
  f, _, _ = fit_part(m, v)                                            # uncorrected: the heading part pushes to the inside
  assert f == pytest.approx(0.35 * 1.653 * 1.8 * k / (v * 1.5), rel=0.03) and f > 0.0


def test_speed_weight_and_gates(keep):
  m = model(centre=0.10)
  assert fit_part(m, 40 * KPH)[0] == 0.0 and fit_part(m, 45 * KPH)[0] == 0.0
  half = fit_part(m, 52.5 * KPH)[0]; e = 0.10 + cd.LANE_CENTER_BIAS_M
  assert half == pytest.approx(0.5 * 0.35 * 1.432 * e / (52.5 * KPH * 1.5) ** 2, rel=0.03)
  assert fit_part(model(centre=0.10, probs=0.4), 70 * KPH)[0] == 0.0  # lines not confident
  f, _, s = fit_part(m, 70 * KPH)
  assert f > 0.0
  _, s = term(m, v=70 * KPH, n=300, s=s, usable=False)             # blinker / press / lane change: fades out
  assert abs(s._lc_f) < 1e-7


def test_shares_the_cap_and_is_low_passed(keep):
  v = 80 * KPH; cap = cd.LANE_CENTER_ACC / v ** 2
  f, out, _ = fit_part(model(centre=3.0), v, n=600)
  assert out == pytest.approx(cap, rel=0.02)
  f1, _, _ = fit_part(model(centre=0.10), v, n=1)
  assert 0.0 < f1 < 0.05 * 0.35 * 1.432 * 0.07 / (v * 1.5) ** 2 * 2


def test_kill_switch_and_nan(keep):
  cd.LANE_FIT_GAIN = 0.0
  f, out, _ = fit_part(model(centre=0.10), 70 * KPH)
  assert f == 0.0 and abs(out) < 1e-9
  cd.LANE_FIT_GAIN = 0.35
  _, _, s = fit_part(model(centre=0.10), 70 * KPH)
  out, s = term(model(centre=0.10), v=70 * KPH, n=5, s=s, k=float('nan'))
  assert np.isfinite(out) and np.isfinite(s._lc_f)
