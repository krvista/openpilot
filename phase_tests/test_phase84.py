"""Phase 84 (report 116): lane-term states of Phases 80b / 82 / 83 are cleared with the older ones; telemetry bits."""
import types

import pytest

from phase_tests.harness_noncontrol import FakeParams  # noqa: F401 (stub install side effect)

import openpilot.selfdrive.controls.controlsd as cd
from openpilot.cereal import log
from phase_tests.test_noncontrol_controlsd_state import mk_controls, run
from phase_tests.test_phase73 import model, term

LCS = log.LaneChangeState
V = 20.0                                                              # 72 km/h: plan, fit and avoidance parts all active


@pytest.fixture
def keep():
  names = ("LANE_STATE_RESET", "LANE_AVOID_RELEASE", "LANE_FIT_GAIN", "LANE_CENTER_KI")
  saved = {k: getattr(cd, k) for k in names}
  yield
  for k, v in saved.items():
    setattr(cd, k, v)


def solo(active=True):
  s = mk_controls(); s._steer_legacy = False
  s.get_lat_active = lambda sm: s._on
  s._on = active
  return s


def test_shipped_values():
  assert cd.LANE_STATE_RESET is True
  assert cd.STEER_FLAG_LANE_FIT == 1 << 14 and cd.STEER_FLAG_LANE_AVOID == 1 << 15
  bits = [v for k, v in vars(cd).items() if k.startswith("STEER_FLAG_")]
  assert len(bits) == len(set(bits)) and max(bits) < 1 << 16          # steerFlags is a UInt16


def test_lateral_off_clears_the_fit_and_avoidance_states(keep):
  s = solo(); run(s, 600, model(centre=0.30, plan_shift=-0.25))
  assert abs(s._lc_f) > 2e-5 and s._lc_usual == pytest.approx(-0.55, abs=0.03) and s._lc_usual_n > 300
  s._conf_blend_lc_frames = 80
  s._on = False; run(s, 1, model())
  assert (s._lc_p, s._lc_i, s._lc_e, s._lc_f) == (0.0, 0.0, 0.0, 0.0)
  assert s._lc_usual is None and s._lc_wa == 1.0 and s._lc_usual_n == 0 and s._lc_avoid_gap == 0
  assert s._conf_blend_lc_frames == 0


def test_re_engaging_starts_like_a_fresh_controller(keep):
  road_b = model(plan_shift=0.20)                                     # another road: plan 20 cm right of the centre
  s = solo(); run(s, 600, model(centre=0.30, plan_shift=-0.25))
  s._on = False; run(s, 100, model())
  s._on = True
  fresh = solo()
  for n in (1, 49, 250):
    a = run(s, n, road_b)[0]; b = run(fresh, n, road_b)[0]
    assert a == pytest.approx(b, abs=1e-9), n
  assert s._lc_wa == pytest.approx(1.0)


def test_kill_switch_keeps_the_old_states(keep):
  cd.LANE_STATE_RESET = False
  s = solo(); run(s, 600, model(centre=0.30, plan_shift=-0.25))
  f = s._lc_f
  s._conf_blend_lc_frames = 80
  s._on = False; run(s, 100, model())
  assert s._lc_f == f and s._lc_usual is not None and s._conf_blend_lc_frames == 80
  s._on = True; run(s, 1, model(plan_shift=0.20))
  assert s._lc_wa < 0.5                                               # the old road's usual spot releases the terms


def _after_lane_change(n_lc):
  s = solo(); run(s, 600, model(plan_shift=-0.30))                    # usual spot 30 cm left in the old lane
  lc = model(plan_shift=0.40); lc.meta.laneChangeState = LCS.laneChangeStarting
  run(s, n_lc, lc)
  run(s, 100, model(plan_shift=0.30))                                 # new lane: the model sits 30 cm right
  return s


def test_lane_change_always_forgets_the_old_lanes_usual_spot(keep):
  for n in (450, 550):                                                # either side of LANE_AVOID_RESET_S
    s = _after_lane_change(n)
    assert s._lc_wa == pytest.approx(1.0, abs=0.02), n
    assert s._lc_usual == pytest.approx(0.30, abs=0.02) and s._lc_usual_n == 100, n
  cd.LANE_STATE_RESET = False
  assert _after_lane_change(450)._lc_wa < 0.6 and _after_lane_change(550)._lc_wa == pytest.approx(1.0, abs=0.02)


def test_nan_lane_probability_does_not_wipe_the_states(keep):
  m = model(centre=0.30)
  _, s = term(m, v=V, n=600)
  p, i, f = s._lc_p, s._lc_i, s._lc_f
  assert p > 1e-4 and i > 2e-5 and f > 2e-5
  bad = model(centre=0.30); bad.laneLineProbs = [0.95, float('nan'), 0.95, 0.95]
  out, s = term(bad, v=V, n=1, s=s)
  assert s._lc_p == pytest.approx(p, rel=0.05) and s._lc_i == pytest.approx(i, rel=0.01) and s._lc_f == pytest.approx(f, rel=0.05)
  assert out == pytest.approx(s._lc_p + s._lc_i + s._lc_f)
  cd.LANE_STATE_RESET = False
  _, s2 = term(m, v=V, n=600)
  _, s2 = term(bad, v=V, n=1, s=s2)
  assert (s2._lc_p, s2._lc_i, s2._lc_f) == (0.0, 0.0, 0.0)


def test_telemetry_bits(keep):
  s = solo(); run(s, 600, model(centre=0.30))
  assert s.steer_flags & cd.STEER_FLAG_LANE_FIT and s.steer_flags & cd.STEER_FLAG_LANE_CENTER
  assert not s.steer_flags & cd.STEER_FLAG_LANE_AVOID
  run(s, 20, model(centre=0.30, plan_shift=-0.60))                    # plan dodges 60 cm
  assert s.steer_flags & cd.STEER_FLAG_LANE_AVOID
  cd.LANE_FIT_GAIN = 0.0; cd.LANE_AVOID_RELEASE = False
  s = solo(); run(s, 600, model(centre=0.30)); run(s, 20, model(centre=0.30, plan_shift=-0.60))
  assert not s.steer_flags & (cd.STEER_FLAG_LANE_FIT | cd.STEER_FLAG_LANE_AVOID)
  legacy = mk_controls(); legacy._steer_legacy = True; run(legacy, 600, model(centre=0.30))
  assert not legacy.steer_flags & (cd.STEER_FLAG_LANE_FIT | cd.STEER_FLAG_LANE_AVOID | cd.STEER_FLAG_LANE_CENTER)
