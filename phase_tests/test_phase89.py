"""Phase 89: the steering profile that ran together with the factory LFA ('legacy') and every layer only it used were
removed; openpilot always steers alone (the Phase 72 'solo' / direct path).
  (i)   the removed names are gone (constants, state, functions, the profile switch, the param key);
  (ii)  no telemetry bit moved: controlsState.steerFlags, carStateSP.steerFlags and gatewayFlags keep their positions,
        the bits only the removed layers set stay reserved;
  (iii) a short drive through the car controller and through controlsd.state_control still gives the trace recorded
        from the commit before the removal (07980724, solo profile) - GOLDEN_* below.
The full bit-identity record (every CAN byte and every state attribute over scripted, random and logged drives) is
tools/i6nv3_bench/probes/qa/e184_solo_identity.py; (iii) is its short stand-in for the suite. Its inputs use plain
arithmetic only and what is compared exactly is integer (CAN units, flags), so the golden holds on any host; floats are
compared at 1e-9."""
import hashlib
import inspect
import os

import pytest

from phase_tests.harness import Sim
from phase_tests.harness_noncontrol import FakeParams  # noqa: F401 (stub install side effect)
from phase_tests import test_noncontrol_controlsd_state as H
from opendbc.car.hyundai.values import CarControllerParams as P
import opendbc.car.hyundai.carcontroller as ccmod
import openpilot.selfdrive.controls.controlsd as cd
import openpilot.selfdrive.controls.lib.latcontrol_angle as lca
import openpilot.selfdrive.locationd.lagd as lagd

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
KPH = 1 / 3.6


# ------------------------------------------------------------------ (i) removed names
def test_removed_names_are_gone():
  assert not os.path.exists(os.path.join(ROOT, "openpilot/common/steer_profile.py"))
  for prefix in ("LAT_CMD_SMOOTH_", "LOOKAHEAD_JERK", "LOOKAHEAD_T_AHEAD", "LOOKAHEAD_ORIENT_", "LOOKAHEAD_BUDGET_",
                 "LOOKAHEAD_DELAY_SPEEDS", "LOOKAHEAD_DELAY_EXTRA", "LOOKAHEAD_DELAY_JERK", "LOOKAHEAD_DELAY_FOLLOW",
                 "LOOKAHEAD_DELAY_EFF", "LOOKAHEAD_DELAY_PIN", "ENTRY_ASSIST_", "ProfileLatch"):
    assert not [n for n in dir(cd) if n.startswith(prefix)], prefix
  assert cd.LOOKAHEAD_DELAY_BASE_S == 0.3 and cd.LOOKAHEAD_DELAY_PARAM_FRAMES == 300    # these two are live (Phase 64b / 78)
  for name in ("fit12", "orient_plan", "klane_raw"):
    assert not hasattr(cd._ModelFrame, name), name
  assert list(inspect.signature(cd.Controls._lookahead_curvature).parameters) == ["self", "model_v2"]
  for prefix in ("CURVE_TRIM_", "ENTRY_BOOST_", "EXIT_LEAD_", "MDPS_COMP_", "STALL_KICK_"):
    assert not [n for n in dir(P) if n.startswith(prefix)], prefix
  assert not hasattr(P, "LFA_HOLD_OFF") and P.LFA_HOLD_OFF_CONFIRM_FRAMES == 30          # the switch, not the hold-off
  for name in ("_mdps_comp", "_entry_boost", "_exit_lead", "_stall_kick"):
    assert not hasattr(ccmod.CarController, name), name
  assert "lfa_button_mirror" not in inspect.signature(ccmod.hyundaicanfd.create_steering_messages).parameters
  cc = Sim().cc
  gone = ("curve_trim", "trim_resid_lp", "eb_", "xl_", "mdps_comp_", "kick_", "stall_kick_deg", "prev_mads_enabled", "solo_mode")
  assert not [n for n in vars(cc) if n.startswith(gone)]
  assert not [n for n in dir(lca) if n.startswith("LAT_FB_")]
  lac = lca.LatControlAngle(type("CP", (), dict(brand="hyundai", steerLimitTimer=0.4)), None, None, 0.01)
  assert sorted(vars(lac)) == sorted(["dt", "sat_limit", "sat_time", "sat_check_min_speed", "steer_max",
                                      "use_steer_limited_by_safety", "_roll_lp", "_roll_lp_init", "request_paused"])
  for name in ("LAGD_SEED_S", "LAGD_MAX_LAG_S", "LAGD_PUBLISH_MAX_S", "LAGD_PROFILE_PARAM_FRAMES", "CCNC_SOLO_PROFILE_KW",
               "legacy_requested"):
    assert not hasattr(lagd, name), name
  assert list(inspect.signature(lagd.lagd_profile).parameters) == ["CP"]
  assert "LatFbTrim7a6" not in open(os.path.join(ROOT, "openpilot/common/params_keys.h")).read()
  assert list(inspect.signature(Sim.__init__).parameters) == ["self"]


def test_the_car_controller_is_solo_whatever_the_old_profile_field_says():
  sim = Sim()
  sim.cc._cc_sp.stockLfaWithOp = True      # the schema field stays (ordinal 7, old logs) - nothing reads it
  for i in range(300):
    sim.cc._cc_sp.mads.enabled = i >= 100
    sim.step(v=20.0, lfa_state=0, cam_lfa_button=int(i == 200))
    assert sim.cc.gw_flags & ccmod.GW_SOLO and sim.lkas_alt()["LFA_BUTTON"] == 0


# ------------------------------------------------------------------ (ii) telemetry bit positions
def test_flag_bit_positions_are_unchanged():
  steer = dict(LOWCONF_GATE=0, LOWCONF_CAPPED=1, LOWCONF_RELEASE=2, LOOKAHEAD_CAP=3, CONF_BLEND=4, LANE_DROPOUT=5,
               DEPARTURE_HOLD=6, BSM_HOLD=7, ENTRY_ASSIST=8, LOOKAHEAD_ORIENT=9, LOOKAHEAD_BUDGET=10, DIRECT=11,
               LANE_CENTER=12, ENTRY_LEAD=13, LANE_FIT=14, LANE_AVOID=15)
  assert {n[len("STEER_FLAG_"):]: getattr(cd, n).bit_length() - 1 for n in dir(cd) if n.startswith("STEER_FLAG_")} == steer
  sf = ("EFF_ACTIVE WIRE_ACTIVE PASSTHROUGH LOWSPEED_LATCH SCENARIO_BLOCK ANGLE_PASSIVE PARKING PARKING_RELEASING "
        "DRIVER_PRESSED EPS_PRESSED HEAVY_GRIP_ANCHOR ANCHOR_HOLD FAST_FLOOR TQ_ARM_FLOOR SHOVE ACCEL_YIELD "
        "BOOST_SUPPRESSED BLIND_CAUTION CITY_RELEASE ANCHORED_RECOVERY BLINKER_CONCESSION POST_GRIP VM_REJECT REVERSE "
        "CAM_FAULT WHEEL_OUTRUN TX_REJECTED TX_SATURATED HOD_TOUCH HAND_GAP MDPS_COMP ENTRY_BOOST").split()
  assert {n[3:]: getattr(ccmod, n).bit_length() - 1 for n in dir(ccmod) if n.startswith("SF_")} == {n: k for k, n in enumerate(sf)}
  gw = [ccmod.GW_VALID, ccmod.GW_LFA_ON, ccmod.GW_LFA_ACTIVE, ccmod.GW_RELAY, ccmod.GW_SOLO, ccmod.GW_PULSE, ccmod.GW_FOLLOW,
        ccmod.GW_GAVE_UP]
  assert gw == [1 << k for k in range(8)]
  from openpilot.cereal import custom
  assert custom.CarControlSP.schema.fields["stockLfaWithOp"].proto.ordinal.explicit == 7
  from openpilot.cereal import log
  assert log.ControlsState.schema.fields["angleFbInteg"].proto.ordinal.explicit == 68


def test_the_reserved_bits_are_never_set():
  flags_cc, flags_ctl = cc_trace(Sim())[2], ctl_trace(H.mk_controls())[2]
  assert not flags_cc & (ccmod.SF_MDPS_COMP | ccmod.SF_ENTRY_BOOST)
  assert not flags_ctl & (cd.STEER_FLAG_LOOKAHEAD_CAP | cd.STEER_FLAG_ENTRY_ASSIST | cd.STEER_FLAG_LOOKAHEAD_ORIENT
                          | cd.STEER_FLAG_LOOKAHEAD_BUDGET)
  assert flags_ctl & cd.STEER_FLAG_DIRECT


# ------------------------------------------------------------------ (iii) golden traces
def tri(i, period, amp):
  """triangle wave, plain arithmetic (no libm): 0 -> amp -> 0 -> -amp -> 0 over `period` frames"""
  x = (i % period) / period
  return amp * (4 * x if x < 0.25 else (2 - 4 * x if x < 0.75 else 4 * x - 4))


def cc_trace(sim):
  """3600 frames through the car controller, the wheel following the transmitted angle: curves at three speeds, a
  resting hand, a press, a blinker, lateral off / on, the factory LFA active / releasing, a cruise edge, a parking
  manoeuvre, exact zeros of both signs. Returns (digest of the integer trace, float samples, union of the flags)."""
  h = hashlib.sha256(); samples = []; flags = 0; w = 0.0; hist = [0.0] * 4
  for i in range(3600):
    ph = i // 300
    v = [20.0, 20.0, 12.0, 30.0, 20.0, 20.0, 22.0, 22.0, 2.0, 2.0, 15.0, 15.0][ph]
    cmd = tri(i, 400, [6.0, 6.0, 12.0, 2.0, 5.0, 5.0, 4.0, 4.0, 150.0, 150.0, 0.0, 0.0][ph])
    if ph == 10:
      cmd = -0.0
    tq = (30.0 if i % 40 < 20 else -30.0) if ph != 4 else (120.0 if i % 300 < 150 else 450.0)
    lfa = 2 if (ph == 6 and i % 300 < 150) else (3 if ph == 6 and i % 300 < 220 else 0)
    relay = hist[0] if lfa == 0 else w + 1.5
    sim.cc._cc_sp.mads.enabled = ph >= 1
    sim.step(v=v, cmd=cmd, wheel=(-0.0 if ph == 11 else w), tq=tq, lat_active=not (ph == 5 and i % 300 < 60),
             blinker=(ph == 3 and i % 300 >= 200), lfa_state=lfa, gw_req_angle=relay, cruise_enabled=ph >= 7,
             lane_prob_min=0.9, gear=("reverse" if (ph == 9 and i % 300 < 100) else "drive"), gps_fix_seen=i > 500)
    m = sim.lkas_alt(); cc = sim.cc
    tx = float(m["ADAS_StrAnglReqVal"]); act = int(m["LKAS_ANGLE_ACTIVE"])
    row = (int(round(tx * 10)), int(round(float(m["ADAS_ACIAnglTqRedcGainVal"]) * 100)), act, int(m["LFA_BUTTON"]),
           int(cc.steer_debug_flags), int(cc.gw_flags), int(cc.lat_passive_reason), int(cc.parking_mode_active))
    h.update(repr(row).encode())
    flags |= int(cc.steer_debug_flags)
    if i % 150 == 149:
      samples.append((round(float(cc.tx_angle_last), 9), round(float(cc.apply_angle_last), 9), round(float(cc.aci_gain_last), 9)))
    hist = hist[1:] + [tx]
    if act == 2:
      w += (0.95 * tx - w) * 0.05
  return h.hexdigest(), samples, flags


def ctl_model(k, centre=0.0, prob=0.95):
  m = H.mk_model(fallback=k)
  yl = [-1.8 + centre + 0.5 * k * x * x for x in H.XS]; yr = [1.8 + centre + 0.5 * k * x * x for x in H.XS]
  ln = lambda Y: type("L", (), dict(x=list(H.XS), y=Y))  # noqa: E731
  m.position.y = [0.5 * k * x * x for x in H.XS]
  m.laneLines = [ln(yl), ln(yl), ln(yr), ln(yr)]
  m.laneLineProbs = [0.9, prob, prob, 0.9]
  return m


def ctl_trace(s):
  """2400 frames through controlsd.state_control (the stub of test_noncontrol_controlsd_state, lane terms as shipped):
  a curve building up and unwinding, the car off the lane centre, weak lane lines / a dropout, a non-finite model
  action, a driver press, a blinker, lateral off / on, low speed. Returns (digest of flags, float samples, union)."""
  h = hashlib.sha256(); samples = []; flags = 0; cs = s.sm['carState']; on = True
  s.get_lat_active = lambda sm: on
  for i in range(2400):
    ph = i // 200
    k = tri(i, 800, 0.004) if ph < 8 else 0.0005
    cs.vEgo = 21.0 if ph != 10 else 9.0
    cs.steeringPressed = ph == 6 and i % 200 < 60
    cs.leftBlinker = ph == 7 and i % 200 < 80
    on = not (ph == 9 and i % 200 < 50)
    prob = 0.95 if ph not in (3, 4) else (0.25 if ph == 3 else 0.08)
    if i % 5 == 0:
      m = ctl_model(k, centre=(0.25 if ph in (1, 2, 11) else 0.0), prob=prob)
      if ph == 5 and i % 200 < 40:
        m.action.desiredCurvature = float("nan")
      s.sm.logMonoTime['modelV2'] = i
    s.sm['modelV2'] = m
    CC, _ = cd.Controls.state_control(s)
    h.update(repr((int(s.steer_flags), bool(s.lane_dropout), bool(CC.latActive))).encode())
    flags |= int(s.steer_flags)
    if i % 80 == 79:
      samples.append((round(float(CC.actuators.curvature) * 1e3, 9), round(float(CC.actuators.steeringAngleDeg), 9)))
  return h.hexdigest(), samples, flags


GOLDEN_CC_DIGEST = "c1207e511bd229e0bcfa2dd4ca5131d508d0e4b0d540bd036a16b12b20afb6ec"
GOLDEN_CC_FLAGS = 0x38bd3f57
GOLDEN_CC_SAMPLES = [
  (3.2, 3.209999943, 1.0), (-5.8, -5.790000057, 1.0), (2.8, 2.790000057, 1.0),
  (0.2, 0.209999999, 1.0), (-6.4, -6.35999989, 1.0), (11.6, 11.640000168, 1.0),
  (-0.8, -0.830000019, 1.0), (-0.2, -0.17, 0.6), (2.7, 2.699999952, 1.0),
  (2.2, 2.190335274, 0.032), (2.3, 2.300000048, 1.0), (0.2, 0.200000001, 1.0),
  (-2.2, -2.189999962, 0.0), (3.8, 3.810000038, 1.0), (-1.8, -1.810000038, 1.0),
  (-0.2, -0.189999999, 1.0), (16.28097127, 16.280971527, 0.4), (16.28097127, 16.280971527, 0.0),
  (16.28097127, 16.280971527, 0.0), (16.28097127, 16.280971527, 0.0), (16.28097127, 16.280971527, 0.0),
  (16.2, 16.239238194, 0.004), (0.2, 0.15, 0.848), (0.2, 0.15, 0.784),
]   # every 1.5 s: transmitted angle, internal request, internal ACI gain
GOLDEN_CTL_DIGEST = "c2e2dbbef90acfd623b6857a830a3762c564b61a084cfd1939af061650e2f2dc"
GOLDEN_CTL_FLAGS = 0x5837
GOLDEN_CTL_SAMPLES = [
  (1.432557125, -1.231192231), (2.978679491, -2.559986353), (3.342466429, -2.872638464), (1.851390582, -1.591153026),
  (0.314184639, -0.270021796), (-1.226144261, 1.05379343), (-2.773226937, 2.383412838), (-3.196818754, 2.747463465),
  (-1.682839706, 1.446294188), (-0.090603215, 0.077867731), (-0.095825235, 0.082355723), (-0.095825235, 0.082355723),
  (-0.095825235, 0.082355723), (1.613855828, -1.387006879), (0.05112062, -0.043934938), (-1.491853385, 1.282153487),
  (-3.04408255, 2.616196156), (-3.272178583, 2.812230349), (-1.669966732, 1.435230613), (-0.10894375, 0.093630254),
  (0.454894267, -0.390952826), (0.450756401, -0.387396574), (-0.0, 0.0), (0.450930354, -0.387546092),
  (0.444349018, -0.38188985), (0.477290654, -0.410201102), (0.479439099, -0.412047535), (0.654032803, -0.562099755),
  (0.722330005, -0.620796919), (0.736902002, -0.63332057),
]   # every 0.8 s: commanded curvature (1e-3 / m), commanded angle (deg)


def test_car_controller_trace_matches_the_commit_before_the_removal():
  digest, samples, flags = cc_trace(Sim())
  assert flags == GOLDEN_CC_FLAGS
  assert len(samples) == len(GOLDEN_CC_SAMPLES)
  for got, want in zip(samples, GOLDEN_CC_SAMPLES):
    assert got == pytest.approx(want, abs=1e-9)
  assert digest == GOLDEN_CC_DIGEST                                  # every frame: CAN units, gain, bits, flags


def test_controlsd_trace_matches_the_commit_before_the_removal():
  digest, samples, flags = ctl_trace(H.mk_controls())
  assert flags == GOLDEN_CTL_FLAGS
  assert len(samples) == len(GOLDEN_CTL_SAMPLES)
  for got, want in zip(samples, GOLDEN_CTL_SAMPLES):
    assert got == pytest.approx(want, abs=1e-9)
  assert digest == GOLDEN_CTL_DIGEST                                 # every frame: steer flags, dropout latch, lateral on


def test_the_trace_is_sensitive_to_the_solo_constants():
  keep = cd.LANE_FIT_GAIN, P.ENGAGE_TAPER_FRAMES
  try:
    cd.LANE_FIT_GAIN = 0.30
    assert ctl_trace(H.mk_controls())[1] != GOLDEN_CTL_SAMPLES
    P.ENGAGE_TAPER_FRAMES = 0
    assert cc_trace(Sim())[0] != GOLDEN_CC_DIGEST
  finally:
    cd.LANE_FIT_GAIN, P.ENGAGE_TAPER_FRAMES = keep
