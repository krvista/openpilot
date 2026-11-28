"""Bench tool check: tools/i6nv3_bench/sim/roadsim.py (closed-loop lateral simulator, report 119 item 6).
 - its controller constants are the ones of controlsd.py (read from the source at import: no second copy to fall behind),
 - its port of the direct-path lane terms gives the same curvature as controlsd's own _lane_center + _entry_lead,
 - with those defaults it stays within a stated distance of two logged facts (steady-curve position at 60-80 km/h: HEAD
   builds +3.5 cm inside, 468 s / 97 curves, report 119; Phase 79 +13.1 cm, 23 s, report 118). The tolerances are the
   simulator's known error, not an accuracy claim: nothing was tuned to these numbers (sim/README.md)."""
import importlib.util
import math
import os
import types

import numpy as np
import pytest

from phase_tests.harness_noncontrol import FakeParams  # noqa: F401 (stub install side effect)
import openpilot.selfdrive.controls.controlsd as cd

_spec = importlib.util.spec_from_file_location(
  "roadsim", os.path.join(os.path.dirname(__file__), "..", "tools", "i6nv3_bench", "sim", "roadsim.py"))
rs = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(rs)

LOG_HEAD_CM, LOG_P79_CM = 3.5, 13.1        # steady curve 60-80 km/h, cm inside of the straight-line position
SPEEDS = (60, 70, 80)


def as_tuple(x):
  return tuple(x) if isinstance(x, (list, tuple)) else x


def test_every_controller_constant_is_read_from_controlsd():
  assert rs.HEAD_FROM_SOURCE == rs.HEAD_NAMES and os.path.samefile(rs.CONTROLSD, cd.__file__)
  for k in rs.HEAD_NAMES:
    assert rs.HEAD[k] == pytest.approx(as_tuple(getattr(cd, k))), k
    assert rs.DEFAULTS[k] == rs.HEAD[k] and rs.params()[k] == rs.HEAD[k]
  assert set(rs.FALLBACK) == set(rs.HEAD_NAMES)


def test_constants_follow_an_edit_of_the_source(tmp_path):
  src = open(rs.CONTROLSD, encoding="utf-8").read()
  assert "\nLANE_FIT_GAIN = 0.35\n" in src
  p = tmp_path / "controlsd.py"
  p.write_text(src.replace("\nLANE_FIT_GAIN = 0.35\n", "\nLANE_FIT_GAIN = 0.2 + 0.05\n"), encoding="utf-8")
  got = rs.read_controlsd(str(p))
  assert got["LANE_FIT_GAIN"] == pytest.approx(0.25) and got["LANE_FIT_SPEED_MS"] == pytest.approx((40 / 3.6, 55 / 3.6))
  assert rs.read_controlsd(str(tmp_path / "missing.py")) == {}       # outside the repository: FALLBACK is used
  p.write_text("LANE_FIT_GAIN = __import__('os').getcwd()\nLANE_FIT_T_S = 1.5\n", encoding="utf-8")
  assert rs.read_controlsd(str(p)) == {"LANE_FIT_T_S": 1.5}           # only literal arithmetic is evaluated


def test_unknown_parameter_is_refused():
  with pytest.raises(KeyError):
    rs.params(LANE_FIT_GAINS=0.3)


class FakeFrame:
  """what controlsd's _ModelFrame gives the lane terms, built from the simulator's observation"""

  def __init__(self, o, psi):
    self.o = o; self.psi_plan = psi; self.lane_min_p = o.lane_prob

  def lane_preview(self, dist):
    assert dist == pytest.approx(self.o.L)
    cl = 0.3                                   # centre(L) is arbitrary: only centre - plan and the offset at the car count
    return self.o.c0, cl, cl + self.o.plan_off, 3.4, 3.4

  def lane_fit(self, xf):
    k_l = self.o.k_lane_ahead[0]
    return self.o.c0, self.o.psi_lane + cd.LANE_FIT_ARM_M * k_l, k_l


@pytest.mark.parametrize("ke", [None, (0.0, 0.3e-3, 0.3e-3, 0.15e-3)])
def test_port_of_the_lane_terms_equals_controlsd(ke):
  """4000 frames of random speed (25-110 km/h), lane probability, offsets, plan deviations large enough to release for an
  'avoidance', unusable gaps: the port's added curvature = controlsd's, frame by frame. ke: Phase 79's offset part on."""
  saved = cd.LANE_CENTER_KE
  try:
    P = rs.params()
    if ke is not None:
      cd.LANE_CENTER_KE = list(ke); P = rs.params(LANE_CENTER_KE=ke)
    rng = np.random.default_rng(3); n = 4000; ctrl = rs.head_controller(P); me = types.SimpleNamespace(); o = rs.Obs()
    v = 7.0 + 24.0 * (0.5 + 0.5 * np.sin(np.arange(n) / 900.0 + 1.0)); t_idx = np.array(cd._PLAN_T_IDXS)
    worst = 0.0; released = 0; fit_on = 0; lead_on = 0; integ = []
    dev = 0.0
    for i in range(n):
      if i % 5 == 0:
        if i % 700 == 350:
          dev = rng.choice([0.3, -0.5, 0.25])        # the plan leaves its usual spot ...
        elif i % 700 == 600:
          dev = 0.0                                    # ... and comes back
        o.k_action = 2.2e-3 * math.sin(i / 260.0) + rng.normal(0, 5e-5)
        o.c0 = 0.15 * math.sin(i / 410.0) + rng.normal(0, 0.03); o.plan_off = -0.1 + dev + rng.normal(0, 0.02)
        o.cL_minus_plan = -o.plan_off; o.psi_lane = rng.normal(0, 4e-3)
        kl = o.k_action + rng.normal(0, 2e-4); o.k_lane_ahead = (kl, kl, kl)
        o.lane_prob = float(np.clip(0.75 + 0.3 * math.sin(i / 150.0), 0.0, 1.0))
        psi = 0.5 * v[i] * t_idx * (o.k_action + 1.5e-3 * math.sin(i / 90.0) * t_idx)
      o.v = float(v[i]); o.usable = not (1500 <= i < 1560 or 2600 <= i < 3200); o.new_frame = i % 5 == 0
      o.L = float(np.clip(o.v * cd.LANE_CENTER_T_S, *cd.LANE_CENTER_L_M))
      t1 = cd.LOOKAHEAD_DELAY_BASE_S + cd.ENTRY_LEAD_FRAME_S; t2 = t1 + cd.ENTRY_LEAD_S
      o.k_lead = (2.0 / o.v) * (float(np.interp(t2, t_idx, psi)) / t2 - float(np.interp(t1, t_idx, psi)) / t1)
      mf = FakeFrame(o, psi); CS = types.SimpleNamespace(vEgo=o.v)
      want = cd.Controls._lane_center(me, mf, CS, o.k_action, o.usable) + cd.Controls._entry_lead(me, mf, CS, o.k_action, o.usable)
      got, parts = ctrl(o)
      worst = max(worst, abs(got - o.k_action - want))
      assert parts[1] == pytest.approx(me._lc_i, abs=1e-15) and parts[5] == pytest.approx(me._lc_wa, abs=1e-12)
      released += parts[5] < 0.5; fit_on += abs(parts[4]) > 1e-6; lead_on += abs(parts[3]) > 1e-6; integ.append(parts[1])
    assert worst < 1e-12
    # the sequence really went through every branch
    assert released > 50 and fit_on > 1000 and lead_on > 100 and np.ptp(integ) > 2e-6
  finally:
    cd.LANE_CENTER_KE = saved


def band_mean(P, **kw):
  return float(np.mean([rs.steady_curve(v, P, **kw)["inside"] for v in SPEEDS]))


@pytest.fixture(scope="module")
def steady():
  return dict(head=band_mean(rs.params()), p79=band_mean(rs.params(**rs.P_P79)), act=band_mean(rs.params(**rs.ACTUATOR_ERROR)))


def test_head_defaults_steady_curve_position_is_near_the_logged_one(steady):
  """Logged +3.5 cm; the simulator gives about +5.1 with a perfect actuator and about +0.4 with the actuator error model -
  the log lies between the two. Tolerance 3 cm for the default (perfect actuator), 4 cm for the option."""
  assert steady["head"] == pytest.approx(LOG_HEAD_CM, abs=3.0)
  assert steady["act"] == pytest.approx(LOG_HEAD_CM, abs=4.0)
  assert steady["act"] < LOG_HEAD_CM < steady["head"]


def test_phase79_parameter_set_cuts_further_inside_but_less_than_logged(steady):
  """Logged +13.1 cm (23 s only); the simulator gives about +8.2: the direction and roughly a third of the logged gain of
  Phase 82 (+3.1 of +9.6 cm). Tolerance 6 cm - a known gap, kept visible here rather than fitted away."""
  assert steady["p79"] == pytest.approx(LOG_P79_CM, abs=6.0)
  assert steady["p79"] - steady["head"] >= 2.0
  assert steady["p79"] < LOG_P79_CM                      # if this ever flips, re-read the README fidelity section


def test_phase79_and_head_sets_differ_only_where_the_code_did():
  P = rs.params(**rs.P_P79)
  assert P["LANE_FIT_GAIN"] == 0.0 and not P["LANE_AVOID_RELEASE"] and P["LANE_CENTER_KE"] == (0.0, 0.3e-3, 0.3e-3, 0.15e-3)
  assert {k for k in P if P[k] != rs.DEFAULTS[k]} == {"LANE_FIT_GAIN", "LANE_AVOID_RELEASE", "LANE_CENTER_KE"}
  r = rs.simulate(rs.synthetic("curve", v_kph=70, k=1.8e-3), P=P, noise=False)
  assert not r["parts"][:, 4].any() and (r["parts"][:, 5] == 1.0).all()      # no fit part, never released
  assert np.abs(r["parts"][:, 2]).max() > 1e-5                                # the offset part works instead


def test_lane_fit_speed_gate_is_the_phase88_one():
  """Below the gate the fit part is zero, above it pushes outward in a curve; 50 km/h differs between Phase 84 and HEAD."""
  f = lambda v, P: rs.steady_curve(v, P, k=2.5e-3, drives=("none",))  # noqa: E731
  assert f(38, rs.params())["fit"] == 0.0
  a, b = f(50, rs.params()), f(50, rs.params(**rs.P_P84))
  assert a["fit"] < b["fit"] < 0.0 and a["inside"] < b["inside"]
  assert f(70, rs.params())["fit"] == pytest.approx(f(70, rs.params(**rs.P_P84))["fit"])
  assert rs.fit_lead_s(20.0, rs.params()) == pytest.approx(0.5) and rs.fit_lead_s(10.0, rs.params()) == pytest.approx(0.75)


def test_actuator_error_model_reproduces_the_logged_delivery_asymmetry():
  """Report 121.2 / 121.4: with a +0.30 deg offset the sensor delivers 0.89 of the command in right curves and 1.01 in left
  ones (logged b4: 0.86 / 1.00), and the curve position does not depend on the offset (the integral absorbs it)."""
  P = rs.params(**rs.ACTUATOR_ERROR, LINEAR=True); out = {}
  for off in (0.0, 0.30):
    for sg in (1, -1):
      scn = rs.synthetic("curve", v_kph=80, k=sg * 1.4e-3, hold_s=20.0, pre_s=200.0)
      r = rs.simulate(scn, P=P, drive=rs.offset_drive(off, 80), noise=False)
      ev = scn["events"]; i0 = int(ev["entry_t"] / rs.DT); h = slice(int((ev["entry_t"] + ev["ramp_s"] + 4.0) / rs.DT), int(ev["exit_t"] / rs.DT))
      out[off, sg] = ((r["e"][h].mean() - r["e"][i0 - 300:i0].mean()) * sg * 100, r["k_car"][h].mean() / r["k_cmd"][h].mean())
  assert out[0.30, 1][1] == pytest.approx(0.891, abs=0.01) and out[0.30, -1][1] == pytest.approx(1.009, abs=0.01)
  assert out[0.0, 1][1] == pytest.approx(out[0.0, -1][1], abs=1e-3) and out[0.0, 1][1] == pytest.approx(0.946, abs=0.01)
  for sg in (1, -1):
    assert out[0.30, sg][0] == pytest.approx(out[0.0, sg][0], abs=0.5)
  assert rs.DEFAULTS["COMPLIANCE"] == 0.0                # the option is off by default


def test_avoidance_release_is_modelled_and_quiet_in_a_clean_curve():
  r = rs.simulate(rs.synthetic("curve", v_kph=70, k=1.8e-3), P=rs.params(), drive="a7", noise=False)
  assert (r["parts"][:, 5] == 1.0).all()
  # the lane jumps 0.6 m (a step scenario): the plan leaves its usual spot and the lane terms let go for a while
  s = rs.simulate(rs.synthetic("step", v_kph=70, step=0.6), P=rs.params(), noise=False)
  i0 = int(s["events"]["step_t"] / rs.DT)
  assert s["parts"][i0:i0 + 300, 5].min() < 0.5 and s["parts"][-1, 5] > 0.99
  off = rs.simulate(rs.synthetic("step", v_kph=70, step=0.6), P=rs.params(LANE_AVOID_RELEASE=False), noise=False)
  assert (off["parts"][:, 5] == 1.0).all()


def test_log_scenarios_need_the_data_file(tmp_path, monkeypatch):
  monkeypatch.setattr(rs, "SCEN_LOGS", str(tmp_path / "none.npz")); monkeypatch.setattr(rs, "_LOGS", None)
  with pytest.raises(FileNotFoundError):
    rs.log_scenarios("a7")
