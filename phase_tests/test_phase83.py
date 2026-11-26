"""Phase 83 (report 115): avoidance release - the lane terms let go while the plan aims away from its usual spot."""
import types

import numpy as np
import pytest

import openpilot.selfdrive.controls.controlsd as cd
from phase_tests.test_phase73 import model, term

KPH = 1 / 3.6
V = 70 * KPH


@pytest.fixture
def keep():
  names = ("LANE_AVOID_RELEASE", "LANE_CENTER_KI", "LANE_FIT_GAIN")
  saved = {k: getattr(cd, k) for k in names}
  yield
  for k, v in saved.items():
    setattr(cd, k, v)


def settled(centre=0.0, plan_shift=0.0, n=600, v=V):
  out, s = term(model(centre=centre, plan_shift=plan_shift), v=v, n=n)
  return out, s


def test_shipped_values():
  assert cd.LANE_AVOID_RELEASE is True and cd.LANE_AVOID_DEV_M == [0.20, 0.45] and cd.LANE_AVOID_CURVE == 0.25
  assert (cd.LANE_AVOID_USUAL_S, cd.LANE_AVOID_RECOVER_S, cd.LANE_AVOID_RESET_S, cd.LANE_AVOID_WARMUP_S) == (10.0, 1.5, 5.0, 3.0)


def test_steady_offset_between_plan_and_centre_is_the_usual_spot(keep):
  cd.LANE_CENTER_KI = 0.0
  out, s = settled(plan_shift=-0.30)                                  # the model always drives 30 cm left of the centre
  assert s._lc_wa == pytest.approx(1.0) and s._lc_usual == pytest.approx(-0.30, abs=0.01)
  cd.LANE_AVOID_RELEASE = False
  out0, _ = settled(plan_shift=-0.30)
  assert out == pytest.approx(out0, rel=1e-6) and out > 0.0           # the terms still pull toward the centre as before


def test_plan_moving_aside_releases_at_once_and_comes_back_slowly(keep):
  cd.LANE_CENTER_KI = 0.0
  _, s = settled()
  dodge = model(plan_shift=-0.60)                                     # plan suddenly aims 60 cm to the left
  out, s = term(dodge, v=V, n=1, s=s)
  assert s._lc_wa == 0.0
  out, s = term(dodge, v=V, n=200, s=s)                               # 2 s of it: nothing pulls back
  assert abs(out) < 2e-6 and s._lc_wa < 0.05
  cd.LANE_AVOID_RELEASE = False
  out0, _ = term(dodge, v=V, n=200, s=types.SimpleNamespace())
  assert out0 > 20e-6                                                 # without the release the terms would fight the dodge
  cd.LANE_AVOID_RELEASE = True
  _, s = term(model(), v=V, n=30, s=s)                                # plan back: 0.3 s later still mostly released
  assert 0.05 < s._lc_wa < 0.4
  _, s = term(model(), v=V, n=600, s=s)
  assert s._lc_wa == pytest.approx(1.0, abs=0.02)


def test_partial_release_between_the_two_thresholds(keep):
  _, s = settled()
  _, s = term(model(plan_shift=0.325), v=V, n=1, s=s)
  assert s._lc_wa == pytest.approx(0.5, abs=0.03)
  _, s2 = settled()
  _, s2 = term(model(plan_shift=0.15), v=V, n=50, s=s2)
  assert s2._lc_wa == pytest.approx(1.0)                              # below 20 cm: untouched


def test_curve_allowance(keep):
  v = 60 * KPH; k = 3.0e-3; L = v * cd.LANE_CENTER_T_S
  inside = 0.25 * k * L * L / 2.0                                      # the plan cutting the curve by the allowance (42 cm)
  _, s = term(model(k_plan=k, k_lane=k), v=v, n=600)
  _, s = term(model(k_plan=k, k_lane=k, plan_shift=inside + 0.10), v=v, n=20, s=s)
  assert s._lc_wa == pytest.approx(1.0)
  _, s = term(model(k_plan=k, k_lane=k, plan_shift=inside + 0.50), v=v, n=1, s=s)
  assert s._lc_wa == 0.0


def test_integral_is_held_not_bled_and_not_fed(keep):
  cd.LANE_FIT_GAIN = 0.0
  _, s = term(model(centre=0.20), v=V, n=1500)
  i0 = s._lc_i
  assert i0 > 0.0
  _, s = term(model(centre=0.20, plan_shift=-0.70), v=V, n=500, s=s)  # 5 s avoidance with an offset present
  assert s._lc_wa < 0.05 and s._lc_i == pytest.approx(i0, rel=1e-9)


def test_a_long_diversion_is_learned_slowly_and_a_gap_forgets_the_spot(keep):
  _, s = settled()
  _, s = term(model(plan_shift=-0.60), v=V, n=3000, s=s)              # 30 s: about 1.5 time constants of 20 s
  assert -0.60 < s._lc_usual < -0.40
  _, s = term(model(plan_shift=-0.60), v=V, n=3000, s=s)
  assert s._lc_wa > 0.9                                               # taken over as the new usual spot
  _, s = term(model(), v=V, n=int(cd.LANE_AVOID_RESET_S * 100) + 2, s=s, usable=False)
  assert s._lc_usual is None and s._lc_wa > 0.9


def test_not_seen_while_unusable_and_nan_is_safe(keep):
  _, s = settled()
  _, s = term(model(plan_shift=-0.60), v=V, n=5, s=s, usable=False)   # blinker / press: the term is off anyway
  assert s._lc_wa == pytest.approx(1.0)
  out, s = term(model(plan_shift=-0.60), v=V, n=5, s=s, k=float('nan'))
  assert np.isfinite(out) and np.isfinite(s._lc_wa)


def test_kill_switch(keep):
  cd.LANE_AVOID_RELEASE = False
  _, s = settled()
  _, s = term(model(plan_shift=-0.60), v=V, n=50, s=s)
  assert getattr(s, "_lc_wa", 1.0) == 1.0


def test_no_release_during_the_warm_up_and_the_usual_spot_is_an_average(keep):
  s = types.SimpleNamespace()
  _, s = term(model(plan_shift=0.50), v=V, n=1, s=s)                  # first usable frame happens to be far off
  _, s = term(model(plan_shift=0.0), v=V, n=200, s=s)                 # ... the next 2 s are not
  assert s._lc_wa == pytest.approx(1.0) and abs(s._lc_usual) < 0.01   # not released, and the first frame did not stick
  _, s = term(model(plan_shift=0.0), v=V, n=150, s=s)
  _, s = term(model(plan_shift=-0.60), v=V, n=1, s=s)                 # after the warm-up a dodge releases
  assert s._lc_wa == 0.0
