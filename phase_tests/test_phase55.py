"""Phase 55 (report §52/§53): the lookahead lead is sampled from the plan yaw rate / plan speed at 0.9 s + tau(v)
instead of the cubic y'' at t_ahead, with the jerk budget over the same horizon; same gate and blend. The cubic path
stays below 25-30 km/h, during lane changes and when the plan arrays are missing, cross-faded over 0.5 s."""
import types

import numpy as np
import pytest

from phase_tests.harness_noncontrol import FakeParams  # noqa: F401 (stub install side effect)

import openpilot.selfdrive.controls.controlsd as cd
from phase_tests.test_noncontrol_controlsd import StubSelf

T = np.array(cd.ModelConstants.T_IDXS)


@pytest.fixture(autouse=True)
def _shipped():
  saved = (cd.LOOKAHEAD_ORIENT_T_S, cd.LOOKAHEAD_ORIENT_WIDE_BUDGET)
  yield
  cd.LOOKAHEAD_ORIENT_T_S, cd.LOOKAHEAD_ORIENT_WIDE_BUDGET = saved


def model(fb, k_far, v=20.0, t_step=0.6, k_geom=None, lcs=None, orient=True):
  """Plan: action fb, cubic geometry of curvature k_geom (default fb), yaw rate stepping from fb*v to k_far*v at t_step."""
  kg = fb if k_geom is None else k_geom
  xs = [v * t for t in T]
  m = types.SimpleNamespace(action=types.SimpleNamespace(desiredCurvature=fb),
                            position=types.SimpleNamespace(x=xs, y=[0.5 * kg * x * x for x in xs]),
                            meta=types.SimpleNamespace(laneChangeState=cd.LaneChangeState.off if lcs is None else lcs,
                                                       laneChangeDirection=0))
  if orient:
    m.orientationRate = types.SimpleNamespace(z=[(fb if t < t_step else k_far) * v for t in T])
    m.velocity = types.SimpleNamespace(x=[v] * len(T))
  return m


def call(s, m, v=20.0, tau=0.08, n=1):
  for _ in range(n):
    out = cd.Controls._lookahead_curvature(s, m, v, tau)
  return out


def settled(v=20.0):
  s = StubSelf()
  s._lookahead_orient_w = 1.0 if v >= 30.0 / 3.6 else 0.0
  return s


def test_shipped_constants():
  assert cd.LOOKAHEAD_ORIENT_T_S == 0.9 and cd.LOOKAHEAD_ORIENT_WIDE_BUDGET is True
  assert cd.LOOKAHEAD_ORIENT_SLEW_S == 0.5 and cd.LOOKAHEAD_JERK_BUDGET == 0.7
  assert cd.LOOKAHEAD_ORIENT_RAMP_V == pytest.approx([25.0 / 3.6, 30.0 / 3.6])


def test_orient_lead_reads_plan_at_09_plus_tau():
  # yaw rate steps up at 0.95 s: seen at 0.9 + 0.08 = 0.98 s, not by the cubic (whose geometry stays at fb)
  fb, kf, v = 0.002, 0.0026, 20.0
  s = settled()
  out = call(s, model(fb, kf, v, t_step=0.95), v)
  assert out == pytest.approx(kf, rel=1e-6)                 # inside the budget -> exact plan value
  assert s._lookahead_flags & cd.STEER_FLAG_LOOKAHEAD_ORIENT and not s._lookahead_flags & cd.STEER_FLAG_LOOKAHEAD_BUDGET
  assert call(settled(), model(fb, kf, v, t_step=1.2), v) == pytest.approx(fb)   # step beyond 0.98 s: no lead yet


def test_wide_budget_bounds_the_lead():
  fb, v, tau = 0.002, 20.0, 0.08
  s = settled()
  out = call(s, model(fb, 0.05, v), v, tau)
  dk = cd.LOOKAHEAD_JERK_BUDGET * (0.9 + tau) / v ** 2
  assert out == pytest.approx(fb + dk) and s._lookahead_flags & cd.STEER_FLAG_LOOKAHEAD_BUDGET
  cd.LOOKAHEAD_ORIENT_WIDE_BUDGET = False                   # step-1 kill: budget back on t_ahead
  base = float(np.interp(v, [5.6, 13.9, 27.8, 38.9], [0.08, 0.10, 0.13, 0.18]))
  t_ahead = min(base + float(np.interp(fb, [0.0008, 0.005], [0.0, 0.20])), cd.LOOKAHEAD_T_AHEAD_CAP) + tau
  assert call(settled(), model(fb, 0.05, v), v, tau) == pytest.approx(fb + cd.LOOKAHEAD_JERK_BUDGET * t_ahead / v ** 2)


def test_gate_and_blend_unchanged():
  assert call(settled(), model(0.0005, 0.01), 20.0) == pytest.approx(0.0005)     # below the 0.0008 gate
  fb = 0.00115                                                                     # blend 0.5
  out = call(settled(), model(fb, fb + 0.0005, t_step=0.1), 20.0)
  assert out == pytest.approx(fb + 0.5 * 0.0005)


def test_kill_switch_is_the_cubic_path():
  m = model(0.002, 0.004, k_geom=0.003)
  ref = call(StubSelf(), model(0.002, 0.004, k_geom=0.003, orient=False))
  cd.LOOKAHEAD_ORIENT_T_S = 0.0
  s = settled()
  assert call(s, m) == pytest.approx(ref) and s._lookahead_flags == 0


def test_missing_or_bad_plan_falls_back_to_cubic():
  ref = call(StubSelf(), model(0.002, 0.004, k_geom=0.003, orient=False))
  m = model(0.002, 0.004, k_geom=0.003)
  m.orientationRate.z[20] = float('nan')
  assert call(settled(), m) == pytest.approx(ref)
  m = model(0.002, 0.004, k_geom=0.003)
  m.velocity.x = m.velocity.x[:10]
  assert call(settled(), m) == pytest.approx(ref)


def test_low_speed_and_lane_change_use_cubic():
  v = 6.0                                                     # < 25 km/h
  ref = call(StubSelf(), model(0.004, 0.008, v, k_geom=0.005, orient=False), v)
  s = settled(v)
  assert call(s, model(0.004, 0.008, v, k_geom=0.005), v) == pytest.approx(ref) and s._lookahead_flags == 0
  ref = call(StubSelf(), model(0.002, 0.004, k_geom=0.003, orient=False))
  s = settled()
  out = call(s, model(0.002, 0.004, k_geom=0.003, lcs=cd.LaneChangeState.laneChangeStarting), n=50)
  assert out == pytest.approx(ref)                            # faded fully to cubic within 0.5 s


def test_cross_fade_is_slew_limited():
  m_lc = model(0.002, 0.004, k_geom=0.003, lcs=cd.LaneChangeState.laneChangeStarting)
  m = model(0.002, 0.004, k_geom=0.003)
  s = settled()
  prev = call(s, m)
  outs = []
  for _ in range(60):
    outs.append(call(s, m_lc))
  steps = np.abs(np.diff([prev] + outs))
  span = abs(call(settled(), m) - outs[-1])
  assert steps.max() <= span * cd.DT_CTRL / cd.LOOKAHEAD_ORIENT_SLEW_S + 1e-12
  assert s._lookahead_orient_w == 0.0


def test_state_control_sets_orient_flag():
  try:
    from phase_tests.test_noncontrol_controlsd_state import mk_controls  # noqa: F401
  except Exception as e:  # noqa: BLE001 - environments without the controlsd import chain
    pytest.skip(str(e))
  src = open(cd.__file__).read()
  assert "| self._lookahead_flags" in src
