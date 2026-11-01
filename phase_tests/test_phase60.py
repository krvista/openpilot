"""Phase 60 (report §67): speed-scheduled delay match in controlsd. The base command (model action) is shifted along the
model's own plan: K + clip(k_plan(0.3 + E(v)) - k_plan(0.3), +-J |E| / v^2), E from LOOKAHEAD_DELAY_EXTRA_S by speed
(negative = the chain is faster than the model assumes). Off in lane changes and faded in with the orient path."""
import numpy as np
import pytest

from phase_tests.harness_noncontrol import FakeParams  # noqa: F401 (stub install side effect)

import openpilot.selfdrive.controls.controlsd as cd
from phase_tests.test_noncontrol_controlsd import StubSelf
from phase_tests.test_phase55 import model, call, settled


@pytest.fixture(autouse=True)
def _restore():
  names = ("LOOKAHEAD_DELAY_EXTRA_S", "LOOKAHEAD_DELAY_JERK", "LOOKAHEAD_ORIENT_T_S", "LOOKAHEAD_ORIENT_GATE")
  saved = {nm: getattr(cd, nm) for nm in names}
  yield
  for nm, val in saved.items():
    setattr(cd, nm, val)


def ramp_plan(k0, k1, v=20.0):
  """Plan curvature ramping linearly from k0 at t=0 to k1 at t=2 s (a curve entry the model plans)."""
  m = model(k0, k1, v=v)
  T = np.array(cd.ModelConstants.T_IDXS)
  m.orientationRate.z = [(k0 + (k1 - k0) * min(t / 2.0, 1.0)) * v for t in T]
  return m


def out_with(E, m, v=20.0, n=1):
  cd.LOOKAHEAD_DELAY_EXTRA_S = [E] * 5
  # orient path (>= 30 km/h): the 55c lead is a plan DIFFERENCE, identical for every E, so out(E) - out(0) is the shift
  s = settled(v)
  return call(s, m, v=v, n=n), s


def test_kill_switch_is_identity():
  m = ramp_plan(0.001, 0.005)
  base, s = out_with(0.0, m)
  assert s._delay_match == 0.0
  cd.LOOKAHEAD_DELAY_EXTRA_S = [0.0] * 5
  assert base == pytest.approx(call(settled(), m), abs=1e-12)


def test_positive_shift_reads_further_along_the_plan():
  m = ramp_plan(0.001, 0.005)                 # plan slope 0.002 / s
  base, _ = out_with(0.0, m)
  ahead, s = out_with(0.2, m)
  assert ahead - base == pytest.approx(0.002 * 0.2, rel=0.05)
  assert s._delay_match > 0


def test_negative_shift_reads_earlier():
  m = ramp_plan(0.001, 0.005)
  base, _ = out_with(0.0, m)
  behind, s = out_with(-0.1, m)
  assert behind - base == pytest.approx(-0.002 * 0.1, rel=0.05)


def test_shift_bounded_by_the_jerk_envelope():
  m = ramp_plan(0.0, 0.2)                     # absurd plan: 0.1 /m per s
  base, _ = out_with(0.0, m)
  ahead, _ = out_with(0.2, m)
  assert abs(ahead - base) <= cd.LOOKAHEAD_DELAY_JERK * 0.2 / 20.0 ** 2 + 1e-12


def test_off_in_lane_change_and_below_the_orient_ramp():
  m = ramp_plan(0.001, 0.005)
  m.meta.laneChangeState = cd.LaneChangeState.laneChangeStarting
  _, s = out_with(0.2, m)
  assert s._delay_match == 0.0
  m2 = ramp_plan(0.001, 0.005, v=6.0)
  _, s2 = out_with(0.2, m2, v=6.0)
  assert s2._delay_match == 0.0              # 21.6 km/h: below LOOKAHEAD_ORIENT_RAMP_V


def test_shipped_table_shape():
  assert len(cd.LOOKAHEAD_DELAY_EXTRA_S) == len(cd.LOOKAHEAD_DELAY_SPEEDS_MS)
  assert all(-0.3 <= e <= 0.3 for e in cd.LOOKAHEAD_DELAY_EXTRA_S)
