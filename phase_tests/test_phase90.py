"""Phase 90 (report 127/128): tight-curve trim - above 11e-3 1/m the direct-path command is 7 % less than the model asks."""
import math

import pytest

from phase_tests.harness_noncontrol import FakeParams  # noqa: F401 (stub install side effect)

import openpilot.selfdrive.controls.controlsd as cd
from phase_tests.test_noncontrol_controlsd_state import mk_controls, run
from phase_tests.test_phase73 import model

V = 11.0   # 40 km/h


@pytest.fixture
def keep():
  saved = cd.MODEL_K_TRIM
  yield
  cd.MODEL_K_TRIM = saved


def settled(k, trim, n=600, nan_frames=0):
  cd.MODEL_K_TRIM = trim
  s = mk_controls()
  s.sm['carState'].vEgo = V
  m = model(k_plan=k, centre=-cd.LANE_CENTER_BIAS_M)     # car on the lane-term target: the lane parts stay at zero
  out = run(s, n, m)[0]
  if nan_frames:
    bad = model(k_plan=k, centre=-cd.LANE_CENTER_BIAS_M); bad.action.desiredCurvature = float('nan')
    out = run(s, nan_frames, bad)[0]
  return out


def test_shipped_values():
  assert cd.MODEL_K_TRIM == 0.07 and cd.MODEL_K_TRIM_K == [8e-3, 11e-3]


def factor(k):
  return 1.0 - cd.MODEL_K_TRIM * min(max((abs(k) - 8e-3) / 3e-3, 0.0), 1.0)


@pytest.mark.usefixtures("keep")
@pytest.mark.parametrize("k", [3e-3, 7.9e-3, -7.9e-3])
def test_nothing_below_the_threshold(k):
  assert settled(k, 0.07) == settled(k, 0.0)


@pytest.mark.usefixtures("keep")
@pytest.mark.parametrize("k,f", [(9.5e-3, 0.965), (11e-3, 0.93), (14e-3, 0.93), (-14e-3, 0.93)])
def test_trim_fades_in_and_is_symmetric(k, f):
  off = settled(k, 0.0); on = settled(k, 0.07)
  assert abs(off) > 0.9 * abs(k) and math.copysign(1.0, on) == math.copysign(1.0, k)
  assert on / off == pytest.approx(f, abs=0.004)


@pytest.mark.usefixtures("keep")
def test_continuous_and_monotonic_in_the_model_curvature():
  ks = [7e-3 + i * 0.25e-3 for i in range(33)]            # 7e-3 .. 15e-3
  out = [k * (1.0 - 0.07 * min(max((k - 8e-3) / 3e-3, 0.0), 1.0)) for k in ks]
  assert all(b > a for a, b in zip(out, out[1:]))          # more asked is never less commanded
  assert max(b - a for a, b in zip(out, out[1:])) < 0.26e-3


@pytest.mark.usefixtures("keep")
def test_held_command_is_not_trimmed_again_while_the_model_output_is_bad():
  a = settled(14e-3, 0.07, nan_frames=20); b = settled(14e-3, 0.0, nan_frames=20)
  assert math.isfinite(a) and math.isfinite(b)
  assert a / b == pytest.approx(0.93, abs=0.01)            # still one trim, not 0.93 ** 20


@pytest.mark.usefixtures("keep")
def test_kill_switch():
  cd.MODEL_K_TRIM = 0.0
  s = mk_controls(); s.sm['carState'].vEgo = V
  assert run(s, 600, model(k_plan=14e-3, centre=-cd.LANE_CENTER_BIAS_M))[0] == pytest.approx(14e-3, rel=0.02)


# speed fade of the trim (lowturn review): none below 20 km/h, full from 30 km/h
def settled_at(k, trim, v, n=600):
  cd.MODEL_K_TRIM = trim
  s = mk_controls()
  s.sm['carState'].vEgo = v
  return run(s, n, model(k_plan=k, centre=-cd.LANE_CENTER_BIAS_M))[0]


@pytest.mark.usefixtures("keep")
@pytest.mark.parametrize("kph,share", [(5.0, 0.0), (19.9, 0.0), (25.0, 0.5), (30.0, 1.0), (40.0, 1.0)])
def test_trim_speed_fade(kph, share):
  assert cd.MODEL_K_TRIM_V_MS == [20.0 / 3.6, 30.0 / 3.6]
  for k in (14e-3, -14e-3):
    off = settled_at(k, 0.0, kph / 3.6); on = settled_at(k, 0.07, kph / 3.6)
    if share == 0.0:
      assert on == off
    else:
      assert on / off == pytest.approx(1.0 - 0.07 * share, abs=0.004)


@pytest.mark.usefixtures("keep")
def test_trim_speed_fade_kill_switch():
  saved = cd.MODEL_K_TRIM_V_MS
  try:
    cd.MODEL_K_TRIM_V_MS = [-2.0, -1.0]
    off = settled_at(14e-3, 0.0, 10.0 / 3.6); on = settled_at(14e-3, 0.07, 10.0 / 3.6)
    assert on / off == pytest.approx(0.93, abs=0.004)       # as Phase 90 without the fade
  finally:
    cd.MODEL_K_TRIM_V_MS = saved
