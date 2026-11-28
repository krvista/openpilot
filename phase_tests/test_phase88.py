"""Phase 88 (report 119 item 5): the lane fit comes in over 40-55 km/h (was 45-60)."""
import pytest

from phase_tests.harness_noncontrol import FakeParams  # noqa: F401 (stub install side effect)

import openpilot.selfdrive.controls.controlsd as cd
from phase_tests.test_noncontrol_controlsd_state import mk_controls, run
from phase_tests.test_phase73 import model, term, XS

KPH = 1 / 3.6
OLD = [45 * KPH, 60 * KPH]


@pytest.fixture
def keep():
  saved = cd.LANE_FIT_SPEED_MS
  yield
  cd.LANE_FIT_SPEED_MS = saved


@pytest.fixture
def fit_only():
  names = ("LANE_CENTER_KI", "LANE_CENTER_WEIGHT", "LANE_FIT_SPEED_MS")
  saved = {k: getattr(cd, k) for k in names}
  cd.LANE_CENTER_KI = 0.0; cd.LANE_CENTER_WEIGHT = 1e-12              # only the fit part (the weight must stay > 0)
  yield
  for k, v in saved.items():
    setattr(cd, k, v)


def fit(m, kph, n=400):
  return term(m, v=kph * KPH, n=n)[1]._lc_f


def curve(k_act, k=4.0e-3, centre=0.0):
  """lane of curvature k as a camera LANE_FIT_ARM_M ahead of the axle sees it; action k_act"""
  m = model(k_plan=k_act, centre=centre - cd.LANE_CENTER_BIAS_M, k_lane=k)
  for ln in m.laneLines:
    ln.y = [y + cd.LANE_FIT_ARM_M * k * x for y, x in zip(ln.y, XS)]
  return m


def test_shipped_values():
  assert cd.LANE_FIT_SPEED_MS == pytest.approx([40 * KPH, 55 * KPH])
  assert cd.LANE_FIT_SPEED_MS[0] >= cd.LANE_CENTER_SPEED_MS[1] - 5 * KPH - 1e-9   # not below 40: the whole term is still ramping
  assert cd.LANE_FIT_GAIN == 0.35 and cd.LANE_FIT_T_S == 1.5 and cd.LANE_FIT_RANGE_M == (15.0, 45.0)   # untouched


def test_speed_ramp_of_the_fit_part(fit_only):
  m = model(centre=0.10); e = 0.10 + cd.LANE_CENTER_BIAS_M
  full = lambda kph: 0.35 * 1.432 * e / (kph * KPH * 1.5) ** 2     # noqa: E731
  assert fit(m, 35) == 0.0 and fit(m, 40) == 0.0
  # 45 km/h: fit ramp 1/3, lane-centre speed ramp (30-45 km/h) complete
  assert fit(m, 45) == pytest.approx(full(45) / 3.0, rel=0.03)
  assert fit(m, 47.5) == pytest.approx(0.5 * full(47.5), rel=0.03)
  assert fit(m, 55) == pytest.approx(full(55), rel=0.03)
  assert fit(m, 42.5) == pytest.approx((2.5 / 15.0) * (12.5 / 15.0) * full(42.5), rel=0.03)   # both ramps multiply


def test_more_of_an_inside_cut_is_taken_out_between_40_and_60(fit_only):
  k = 4.0e-3; m = curve(1.1 * k)                                      # action 10 % tighter than the lane (right curve)
  new = {kph: fit(m, kph) for kph in (42, 45, 50, 55, 58)}
  cd.LANE_FIT_SPEED_MS = OLD
  old = {kph: fit(m, kph) for kph in (42, 45, 50, 55, 58)}
  assert old[42] == 0.0 and old[45] == 0.0
  for kph in (42, 45, 50, 55, 58):
    assert new[kph] < 0.0 and new[kph] <= old[kph]                    # eases the right curve: toward the outside
  assert new[50] == pytest.approx(2.0 * old[50], rel=0.03)            # 2/3 against 1/3 of the full part
  assert new[55] == pytest.approx(-0.35 * 0.1 * k, rel=0.03)          # full part: 35 % of the excess
  assert new[58] == pytest.approx(old[58] * (15.0 / 13.0), rel=0.03)


def test_unchanged_from_60_and_below_40(fit_only):
  ms = (model(centre=0.10), curve(4.4e-3), curve(4.0e-3, centre=-0.20))
  new = [[fit(m, kph) for kph in (30, 38, 40, 60, 72, 90, 110)] for m in ms]
  cd.LANE_FIT_SPEED_MS = OLD
  old = [[fit(m, kph) for kph in (30, 38, 40, 60, 72, 90, 110)] for m in ms]
  assert new == old


def test_fit_length_and_range_at_the_new_low_end(fit_only):
  # 40-54 km/h: the fit range is held at LANE_FIT_RANGE_M[0] = 15 m (v * 1 s would be 11-15 m), L = v * 1.5 s (16.7 m at 40)
  v = 44 * KPH; e = 0.10 + cd.LANE_CENTER_BIAS_M
  seen = []
  orig = cd._ModelFrame.lane_fit
  def spy(self, xf):
    seen.append(xf); return orig(self, xf)
  cd._ModelFrame.lane_fit = spy
  try:
    f = fit(model(centre=0.10), 44)
  finally:
    cd._ModelFrame.lane_fit = orig
  assert seen and set(seen) == {15.0}
  w = (44 - 40) / 15.0 * (44 - 30) / 15.0
  assert f == pytest.approx(w * 0.35 * 1.432 * e / (v * 1.5) ** 2, rel=0.03)


def test_shares_the_cap_at_low_speed(fit_only):
  v = 50 * KPH; cap = cd.LANE_CENTER_ACC / v ** 2
  out, s = term(model(centre=3.0), v=v, n=600)                        # absurd offset
  assert out == pytest.approx(cap, rel=0.02) and abs(s._lc_f) <= cap * 1.0001


def test_weak_lines_blinker_and_press_still_switch_it_off(fit_only):
  assert fit(model(centre=0.10, probs=0.4), 50) == 0.0                # lines not confident
  half = fit(model(centre=0.10, probs=0.65), 50); full = fit(model(centre=0.10, probs=0.8), 50)
  assert half == pytest.approx(0.5 * full, rel=0.05)
  assert fit(model(centre=0.10, width=3.4, width_far=6.0), 50) == 0.0   # lane widening by a third: no lane terms
  _, s = term(model(centre=0.10), v=50 * KPH, n=400)
  assert s._lc_f > 0.0
  _, s = term(model(centre=0.10), v=50 * KPH, n=300, s=s, usable=False)   # blinker / press / lane change / dropout hold
  assert abs(s._lc_f) < 1e-7


def test_no_step_while_the_speed_crosses_the_ramp(fit_only):
  m = curve(4.4e-3); s = None; prev = 0.0; worst = 0.0
  for i in range(3000):                                               # 35 -> 60 km/h in 30 s
    v = (35 + 25 * i / 3000) * KPH
    _, s = term(m, v=v, n=1, s=s)
    worst = max(worst, abs(s._lc_f - prev)); prev = s._lc_f
  assert prev < 0.0
  assert worst < 0.02 * abs(prev)                                     # per frame: under 2 % of where it ends


def test_state_control_telemetry_bit_and_kill_switch(keep):
  def drive(kph):
    s = mk_controls()
    s.sm['carState'].vEgo = kph * KPH
    out = run(s, 400, model(centre=0.30))
    return out[0], bool(s.steer_flags & cd.STEER_FLAG_LANE_FIT), s._lc_f
  new50 = drive(50); new72 = drive(72); new38 = drive(38)
  assert new50[1] and new72[1] and not new38[1] and new38[2] == 0.0
  cd.LANE_FIT_SPEED_MS = OLD                                          # kill switch: the Phase 82 gate
  old50 = drive(50); old44 = drive(44); old72 = drive(72); old38 = drive(38)
  assert not old44[1] and old44[2] == 0.0
  assert 0.0 < old50[2] < new50[2] and new50[2] == pytest.approx(2.0 * old50[2], rel=0.05)
  assert old72 == new72 and old38 == new38                            # outside 40-60 km/h nothing changed
  cd.LANE_FIT_SPEED_MS = [40 * KPH, 55 * KPH]
  assert drive(44)[1]
