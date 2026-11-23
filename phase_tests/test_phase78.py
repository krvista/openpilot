"""Phase 78 (report 108-109): entry lead - the wheel starts 0.1 s earlier where a curve builds up, only there."""
import math
import types

import numpy as np
import pytest

import openpilot.selfdrive.controls.controlsd as cd
from openpilot.selfdrive.modeld.constants import ModelConstants

KPH = 1 / 3.6
TI = np.array(ModelConstants.T_IDXS, dtype=np.float64)


@pytest.fixture
def keep():
  names = ("ENTRY_LEAD_S", "ENTRY_LEAD_CAP", "ENTRY_LEAD_WASHOUT_S")
  saved = {k: getattr(cd, k) for k in names}
  yield
  for k, v in saved.items():
    setattr(cd, k, v)


def frame(v, k_of_t):
  """plan whose curvature over plan time is k_of_t(t): heading psi(t) = v * integral of k"""
  ts = np.linspace(0.0, 10.0, 2001); k = np.array([k_of_t(t) for t in ts])
  psi = np.concatenate(([0.0], np.cumsum(0.5 * (k[1:] + k[:-1]) * np.diff(ts)))) * v
  m = types.SimpleNamespace(orientation=types.SimpleNamespace(z=np.interp(TI, ts, psi).tolist()), action=types.SimpleNamespace(desiredCurvature=0.0),
                            position=types.SimpleNamespace(x=[0.0] * 33), laneLineProbs=[1.0] * 4,
                            meta=types.SimpleNamespace(laneChangeState=0, laneChangeDirection=0))
  mf = cd._ModelFrame(m, 0)
  t1 = 0.316 + cd.ENTRY_LEAD_FRAME_S
  act = 2.0 * float(np.interp(t1, TI, mf.psi_plan)) / (v * t1) - k_of_t(0.0)      # modeld's action for this plan
  return mf, act


def lead(v, k_of_t, n=60, usable=True, s=None):
  s = s if s is not None else types.SimpleNamespace(_model_lat_delay=0.316)
  mf, act = frame(v, k_of_t)
  CS = types.SimpleNamespace(vEgo=v)
  out = 0.0
  for _ in range(n):
    out = cd.Controls._entry_lead(s, mf, CS, act, usable)
  return out, act, s


def test_shipped_values():
  assert cd.ENTRY_LEAD_S == 0.10 and cd.ENTRY_LEAD_CAP == 0.15e-3 and cd.ENTRY_LEAD_K == [0.15e-3, 0.4e-3]
  assert cd.ENTRY_LEAD_WASHOUT_S == 0.6
  assert cd.ENTRY_LEAD_MIN_MS == pytest.approx(30 * KPH) and cd.STEER_FLAG_ENTRY_LEAD == 1 << 13


def test_building_curve_gets_the_action_of_a_tenth_of_a_second_later(keep):
  cd.ENTRY_LEAD_WASHOUT_S = 1e9                                       # the raw lead, without the washout
  v = 70 * KPH; ramp = lambda t: 0.4e-3 + 1.0e-3 * t                 # noqa: E731 - right curve building at 1e-3 / s
  out, act, _ = lead(v, ramp)
  assert out == pytest.approx(1.0e-3 * 0.10, rel=0.1)                # the action is the plan curvature at the told delay: 0.1 s later = +0.1e-3
  out_l, act_l, _ = lead(v, lambda t: -ramp(t))
  assert out_l == pytest.approx(-out, rel=1e-6) and act_l < 0.0      # mirror image for a left curve


def test_nothing_on_a_straight_in_a_steady_curve_or_on_the_exit():
  v = 70 * KPH
  assert lead(v, lambda t: 0.0)[0] == 0.0
  assert lead(v, lambda t: 0.02e-3 + 0.2e-3 * t)[0] == 0.0           # future action below 0.15e-3: weight 0
  assert abs(lead(v, lambda t: 2.0e-3)[0]) < 1e-9                    # settled
  assert lead(v, lambda t: max(2.0e-3 - 1.0e-3 * t, 0.0))[0] == 0.0  # unwinding
  assert lead(v, lambda t: -0.5e-3 + 1.0e-3 * t)[0] == 0.0           # crossing over from a left curve: not while still left


def test_washout_takes_the_lead_out_again_while_the_curve_still_builds():
  v = 70 * KPH; ramp = lambda t: 0.4e-3 + 1.0e-3 * t                 # noqa: E731
  s = None; tr = []
  for _ in range(300):
    out, _, s = lead(v, ramp, n=1, s=s); tr.append(out)
  tr = np.array(tr)
  assert 0.07e-3 < tr.max() < 0.1e-3 and np.argmax(tr) < 40          # most of the 0.1e-3 within 0.4 s ...
  assert tr[120] < 0.2 * tr.max() and tr[299] < 0.02 * tr.max()      # ... and gone again although the ramp goes on
  assert tr.min() >= 0.0                                             # never the other way
  out, _, s = lead(v, lambda t: 2.0e-3, n=50, s=s)                   # the lead ends: no reverse pulse
  assert abs(out) < 1e-9


def test_cap_speed_floor_and_context(keep):
  cd.ENTRY_LEAD_WASHOUT_S = 1e9
  v = 60 * KPH; steep = lambda t: 0.5e-3 + 8.0e-3 * t                # noqa: E731
  assert lead(v, steep)[0] == pytest.approx(cd.ENTRY_LEAD_CAP, rel=0.01)
  assert lead(25 * KPH, steep)[0] == 0.0
  assert lead(v, steep, usable=False)[0] == 0.0
  out, _, s = lead(v, steep)
  out2, _, _ = lead(v, steep, n=30, usable=False, s=s)               # lane change / press: fades within 0.3 s
  assert abs(out2) < 0.01 * cd.ENTRY_LEAD_CAP
  o1, _, _ = lead(v, steep, n=1)
  assert 0.0 < o1 < 0.25 * cd.ENTRY_LEAD_CAP                         # low-passed start, no step
  cd.ENTRY_LEAD_S = 0.0
  assert lead(v, steep)[0] == 0.0


def test_missing_or_non_finite_plan_is_safe():
  s = types.SimpleNamespace(_model_lat_delay=0.316); CS = types.SimpleNamespace(vEgo=20.0)
  for z in ([], [0.0] * 5, [float('nan')] * 33):
    m = types.SimpleNamespace(orientation=types.SimpleNamespace(z=z), action=types.SimpleNamespace(desiredCurvature=1e-3),
                              position=types.SimpleNamespace(x=[0.0] * 33), laneLineProbs=[1.0] * 4,
                              meta=types.SimpleNamespace(laneChangeState=0, laneChangeDirection=0))
    assert cd.Controls._entry_lead(s, cd._ModelFrame(m, 0), CS, 1e-3, True) == 0.0
  mf, act = frame(20.0, lambda t: 0.4e-3 + 1e-3 * t)
  assert cd.Controls._entry_lead(s, mf, CS, float('nan'), True) == 0.0
  assert math.isfinite(s._entry_lead_lp)
