"""Phase 73 (report §101, first drive in the solo profile, route a1).
73a  a stop on the road is not a parking lot: once the car has been above the parking latch's exit speed the lead-less
     creep signature no longer arms the latch (it did at every traffic light as first car, and held the request passive
     until 33 km/h for 2 s); a real parking signature clears that.
73b  engage taper: for 1.5 s after the request was anchored to the wheel its rate is capped (4 -> 30 deg/s) - the MDPS
     now follows a relayed request at once.
73c  lane-centre term on the direct path: the plan is steered through the lane centre at the preview distance
     (proportional) and a slow integral removes a steady offset.
All three in the solo profile only."""
import math
import types

import numpy as np
import pytest

from phase_tests.harness import Sim
from phase_tests.harness_noncontrol import FakeParams  # noqa: F401 (stub install side effect)
from phase_tests import test_noncontrol_controlsd_state as H
from opendbc.car.hyundai.values import CarControllerParams as P
import opendbc.car.hyundai.carcontroller as ccmod
import openpilot.selfdrive.controls.controlsd as cd
from openpilot.cereal import log

KPH = 1 / 3.6


@pytest.fixture
def old_parking():
  """the pre-76 parking rules (boot / creep / door signatures, 33 km/h exit): these tests describe them"""
  keep = P.PARKING_V2
  P.PARKING_V2 = False
  yield
  P.PARKING_V2 = keep


@pytest.fixture
def params():
  saved = {k: getattr(P, k) for k in ("ROAD_STOP_NO_PARKING", "ENGAGE_TAPER_FRAMES", "ENGAGE_TAPER_RATE_DPS")}
  yield P
  for k, v in saved.items():
    setattr(P, k, v)


@pytest.fixture
def cdk():
  names = ("LANE_CENTER_WEIGHT", "LANE_CENTER_KI")
  saved = {k: getattr(cd, k) for k in names}
  yield cd
  for k, v in saved.items():
    setattr(cd, k, v)


# ------------------------------------------------------------------ 73a
def drive(sim, n, v_kph, **kw):
  for _ in range(n):
    sim.step(v=v_kph * KPH, lfa_state=0, **kw)


def road_then_stop(solo, stop_frames=1500):
  """leave the lot, drive on the road at 50 km/h, then stop at a light as first car (no lead, no lane lines)"""
  sim = Sim(solo=solo)
  drive(sim, 600, 50.0)
  assert not sim.cc.parking_mode_active
  drive(sim, 300, 20.0); drive(sim, stop_frames, 0.0)
  return sim


def test_shipped_values():
  assert P.ROAD_STOP_NO_PARKING is True and P.ENGAGE_TAPER_FRAMES == 150 and P.ENGAGE_TAPER_RATE_DPS == [4.0, 30.0]
  assert cd.LANE_CENTER_WEIGHT == 0.5 and cd.LANE_CENTER_T_S == 2.0 and cd.LANE_CENTER_KI == 0.03e-3
  assert cd.LANE_CENTER_I_CAP == 0.15e-3 and cd.LANE_CENTER_ACC == 0.4 and cd.LANE_CENTER_BIAS_M == -0.034


def test_traffic_light_stop_does_not_arm_the_parking_latch_in_solo():
  solo, leg = road_then_stop(True), road_then_stop(False)
  assert solo.cc.road_context and not solo.cc.parking_mode_active
  assert leg.cc.parking_mode_active                                  # the pre-73 behaviour: latched at the light
  # launch through the junction (no lane lines): solo steers again from the low-speed exit, legacy only after 33 km/h + 2 s
  for sim in (solo, leg):
    for i in range(500):
      sim.step(v=min(i * 0.03, 30.0 * KPH), cmd=2.0, wheel=0.0, lfa_state=0)
  assert solo.effective_lat_active() and not leg.effective_lat_active()


@pytest.mark.usefixtures("old_parking")
def test_a_real_parking_signature_clears_the_road_context():
  sim = road_then_stop(True, stop_frames=100)
  assert sim.cc.road_context
  drive(sim, 400, 8.0, wheel=300.0)                                  # a parking turn
  assert not sim.cc.road_context and sim.cc.parking_mode_active
  sim2 = road_then_stop(True, stop_frames=100)
  drive(sim2, 400, 3.0, gear='reverse')
  assert not sim2.cc.road_context and sim2.cc.parking_mode_active


@pytest.mark.usefixtures("old_parking")
def test_from_boot_the_lot_crawl_still_arms():
  sim = Sim(solo=True)
  drive(sim, 1500, 6.0)                                              # never been on the road: lot crawl as before
  assert not sim.cc.road_context and sim.cc.parking_mode_active


@pytest.mark.usefixtures("old_parking")
def test_road_stop_kill_switch(params):
  params.ROAD_STOP_NO_PARKING = False
  assert road_then_stop(True).cc.parking_mode_active


# ------------------------------------------------------------------ 73b
def release(solo, v_kph=42.0, cmd=8.0):
  """the request is passive (driver holding the wheel in a big turn) and then resumes with the command 8 deg from the wheel"""
  sim = Sim(solo=solo); w = 0.0; tx = []
  for _ in range(300):
    sim.step(v=v_kph * KPH, cmd=cmd, wheel=0.0, lat_active=False, lfa_state=0)
  for i in range(300):
    sim.step(v=v_kph * KPH, cmd=cmd, wheel=w, lat_active=True, lfa_state=0)
    t = sim.tx_angle() if sim.effective_lat_active() else w
    tx.append(t); w += (t - w) * 0.07
  return np.array(tx)


def test_engage_taper_caps_the_first_moves():
  solo, leg = release(True), release(False)
  assert np.abs(solo[20] - solo[0]) <= 1.3                           # 0.2 s after resuming: about 1 deg
  assert np.abs(leg[20] - leg[0]) >= 2.5                             # pre-73: the VM rate limit only
  assert np.max(np.abs(np.diff(solo[:150]))) <= 0.31                 # never above 30 deg/s inside the taper
  assert solo[-1] == pytest.approx(8.0, abs=0.3) and np.argmax(solo >= 7.0) <= 160   # and it does get there, within ~1.6 s


def test_engage_taper_kill_switch(params):
  params.ENGAGE_TAPER_FRAMES = 0
  assert release(True) == pytest.approx(release(False), abs=1e-9)


# ------------------------------------------------------------------ 73c
XS = [float(x) for x in np.linspace(0.0, 190.0, 33)]


def model(k_plan=0.0, centre=0.0, k_lane=None, width=3.4, probs=0.95, plan_shift=0.0, width_far=None):
  """plan: parabola of curvature k_plan from the car; lane centre: `centre` m to the RIGHT of the car, curvature k_lane"""
  kl = k_plan if k_lane is None else k_lane
  wf = width if width_far is None else width_far
  ws = [width + (wf - width) * min(x / 60.0, 1.0) for x in XS]
  yc = [centre + 0.5 * kl * x * x for x in XS]
  ln = lambda Y: types.SimpleNamespace(x=list(XS), y=Y)  # noqa: E731
  return types.SimpleNamespace(
    action=types.SimpleNamespace(desiredCurvature=k_plan),
    position=types.SimpleNamespace(x=list(XS), y=[plan_shift + 0.5 * k_plan * x * x for x in XS], yStd=[0.01] * 33),
    laneLines=[ln([c - 1.5 * w for c, w in zip(yc, ws)]), ln([c - 0.5 * w for c, w in zip(yc, ws)]),
               ln([c + 0.5 * w for c, w in zip(yc, ws)]), ln([c + 1.5 * w for c, w in zip(yc, ws)])],
    laneLineProbs=[probs] * 4,
    meta=types.SimpleNamespace(laneChangeState=log.LaneChangeState.off, laneChangeDirection=log.LaneChangeDirection.none))


def term(m, v=21.0, n=1, pressed=False, usable=True, s=None, k=None):
  s = s if s is not None else types.SimpleNamespace()
  mf = cd._ModelFrame(m, 0)
  CS = types.SimpleNamespace(vEgo=v, steeringPressed=pressed)
  out = 0.0
  for _ in range(n):
    out = cd.Controls._lane_center(s, mf, CS, m.action.desiredCurvature if k is None else k, usable)
  return out, s


def test_lane_preview_geometry():
  mf = cd._ModelFrame(model(k_plan=1e-3, centre=0.25, k_lane=2e-3), 0)
  c0, cL, pL, w0, wL = mf.lane_preview(40.0)
  assert c0 == pytest.approx(0.25) and w0 == pytest.approx(3.4) and wL == pytest.approx(3.4)
  assert cL == pytest.approx(0.25 + 0.5 * 2e-3 * 1600, rel=0.03) and pL == pytest.approx(0.5 * 1e-3 * 1600, rel=0.03)
  assert mf.lane_preview(500.0) is None                               # beyond the plan
  bad = model(); bad.laneLines[1].y[3] = float('nan')
  assert cd._ModelFrame(bad, 0).lane_preview(40.0) is None


def test_car_left_of_centre_is_steered_right_and_the_integral_takes_over():
  m = model(centre=0.25)                                              # lane centre 25 cm to the right, plan straight ahead
  v = 21.0; L = v * cd.LANE_CENTER_T_S
  p_inf = cd.LANE_CENTER_WEIGHT * 2.0 * (0.25 + cd.LANE_CENTER_BIAS_M) / L ** 2
  out, s = term(m, v=v, n=300)                                        # 3 s: the low-pass has settled
  assert s._lc_p == pytest.approx(p_inf, rel=0.02) and p_inf > 0.0
  assert s._lc_i == pytest.approx(cd.LANE_CENTER_KI * (0.25 + cd.LANE_CENTER_BIAS_M) * 3.0, rel=0.02)
  out, s = term(m, v=v, n=6000, s=s)
  assert s._lc_i == pytest.approx(cd.LANE_CENTER_I_CAP) and out == pytest.approx(p_inf + cd.LANE_CENTER_I_CAP, rel=0.02)
  out_r, _ = term(model(centre=-0.25), v=v, n=300)                    # mirror image
  assert out_r < 0.0


def test_centred_car_gets_nothing_but_the_bias():
  out, s = term(model(centre=-cd.LANE_CENTER_BIAS_M), n=500)          # truly centred (the model's centre is 3.4 cm right)
  assert abs(out) < 1e-7


def test_plan_cutting_inside_a_curve_is_pushed_outward_and_the_integral_holds():
  k = 2.0e-3                                                          # right curve; the plan runs 0.2 m to the inside (right)
  m = model(k_plan=k, centre=-0.20, k_lane=k)
  out, s = term(m, n=300)
  assert out < 0.0 and s._lc_i == 0.0                                 # less curvature = outward; no integration in a curve
  m2 = model(k_plan=-k, centre=0.20, k_lane=-k)                       # left curve, plan inside (left)
  assert term(m2, n=300)[0] > 0.0


@pytest.mark.parametrize("kw", [dict(probs=0.45), dict(width=2.2), dict(width=4.9), dict(width_far=5.2)])   # weak lines / too narrow / too wide / opening (split)
def test_unusable_lane_lines_switch_it_off(kw):
  out, s = term(model(centre=0.25, **kw), n=300)
  assert out == 0.0 and s._lc_i == 0.0


def test_gates_fade_and_bleed():
  m = model(centre=0.25)
  out, s = term(m, n=2000)
  i0 = s._lc_i; assert out > 0.0 and i0 > 0.0
  out2, s = term(m, n=100, usable=False, s=s)                         # blinker / lane change / press / dropout hold
  assert s._lc_p < 0.05 * (out - i0) and 0.9 * i0 < s._lc_i < i0      # proportional gone in 1 s, integral decays slowly
  assert term(m, v=25.0 * KPH, n=300)[0] == 0.0                       # below 30 km/h: off
  half, _ = term(m, v=37.5 * KPH, n=300); full, _ = term(m, v=45.0 * KPH, n=300)
  assert 0.0 < half and half != full
  lo, _ = term(model(centre=0.25, probs=0.65), n=300); hi, _ = term(model(centre=0.25, probs=0.8), n=300)
  assert lo == pytest.approx(0.5 * hi, rel=0.05)


def test_proportional_part_is_capped():
  out, s = term(model(centre=3.0), v=12.5, n=300)                     # absurd error
  assert s._lc_p == pytest.approx(cd.LANE_CENTER_ACC / 12.5 ** 2, rel=0.02)


def test_state_control_adds_it_on_the_direct_path_only(cdk):
  def run(legacy):
    s = H.mk_controls(); s._steer_legacy = legacy
    s.sm['carState'].vEgo = 21.0
    s.sm['modelV2'] = model(centre=0.25)
    for i in range(300):
      s.sm.logMonoTime['modelV2'] = i
      CC, _ = cd.Controls.state_control(s)
    return s, float(CC.actuators.curvature)
  solo, k_s = run(False); leg, k_l = run(True)
  assert k_s > 5e-5 and solo.steer_flags & cd.STEER_FLAG_LANE_CENTER
  assert abs(k_l) < 1e-6 and not leg.steer_flags & cd.STEER_FLAG_LANE_CENTER
  solo.sm['carState'].steeringPressed = True                          # a driver press: the term fades
  for i in range(300, 500):
    solo.sm.logMonoTime['modelV2'] = i
    cd.Controls.state_control(solo)
  assert abs(solo._lc_p) < 1e-6
  cdk.LANE_CENTER_WEIGHT = 0.0; cdk.LANE_CENTER_KI = 0.0              # kill
  s2, k2 = run(False)
  assert abs(k2) < 1e-9 and not s2.steer_flags & cd.STEER_FLAG_LANE_CENTER


def test_not_finite_inputs_never_poison_the_state():
  out, s = term(model(centre=0.25), n=50)
  out, s = term(model(centre=0.25), v=float('nan'), n=5, s=s)
  assert math.isfinite(s._lc_p) and math.isfinite(s._lc_i)
