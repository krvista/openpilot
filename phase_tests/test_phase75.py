"""Phase 75 (report §103: who commands the MDPS - audit of the solo profile), car-controller side.
75a  TX governor mirrors the panda's own per-frame limit one CAN unit inside (was two; 10 deg/s from 67 km/h), the
     wheel-outrun threshold stays on the panda's allowance.
75b  hold-off: the retry wait is cleared once the factory LFA is really seen off (bounded per rate window).
75c  a parking signature that never armed the latch is dropped once the car is on the road.
75d  the controller's first frame anchors the request to the wheel.
75e  the internal gain stays 0 while nobody receives it (factory LFA active, a run of rejected frames, lateral not
     granted; a single rejected frame leaves it alone);
     CarControlSP.latGranted False -> passive frames, request on the wheel, engage taper afterwards.
75f  hold-off confirm 0.1 s within 1.5 s after a cruise-engagement edge (one pulse, not in the first 0.5 s).
75g  follow band around the MDPS angle; the engage taper restarts once when a binding band ends.
75h  low speed: 0.3 s re-engage dwell when the scenario gate opens because the creep zone was left; hands-off gain
     ceiling at least 0.45 (0-10 km/h) / 0.60 (20 km/h).
75i  camera stale / LFA fault are indicated; carStateSP.lateralPausedReason carries the cause.
Every behaviour change has a kill switch in CarControllerParams. (Written as 'solo profile only': the comparisons
against the legacy profile went with that profile in Phase 89.)"""
import math
import os
import shutil
import subprocess
import tempfile
import types

import numpy as np
import pytest

from phase_tests.harness import Sim
from phase_tests.test_phase72 import FactoryLfa, run
from opendbc.car import structs
from opendbc.car.lateral import get_max_angle_delta_vm
from opendbc.car.hyundai.values import CarControllerParams as P
import opendbc.car.hyundai.carcontroller as ccmod

KPH = 1 / 3.6
SWITCHES = ("TX_GOVERNOR_MIRROR", "TX_GOVERNOR_MIRROR_MIN_KPH", "WHEEL_OUTRUN_MARGIN_MIRROR_CAN", "LFA_HOLD_OFF_CLEAR_FRAMES",
            "LFA_HOLD_OFF_CLEAR_MAX", "PARKING_STALE_SIGNATURE_CLEAR", "FIRST_FRAME_ANCHOR", "ACI_HOLD_ZERO_UNRELAYED",
            "ACI_HOLD_ZERO_REJECT_FRAMES", "LAT_GRANT_GATE", "LFA_HOLD_OFF_FAST_CONFIRM_FRAMES", "LFA_HOLD_OFF_FAST_WINDOW_FRAMES",
            "LFA_HOLD_OFF_FAST_GUARD_FRAMES", "GW_FOLLOW_MDPS_REF", "GW_FOLLOW_EXIT_TAPER", "LOW_SPEED_SCEN_CREEP_EXIT_FRAMES",
            "ACIGAIN_LOWSPEED_RAISE_V", "PASSIVE_INDICATE_CAM_FAULT", "TX_GOVERNOR_RESYNC_FRAMES", "ENGAGE_TAPER_FRAMES")


@pytest.fixture
def old_parking():
  """the pre-76 parking rules (boot / creep / door signatures, 33 km/h exit): these tests describe them"""
  keep = P.PARKING_V2
  P.PARKING_V2 = False
  yield
  P.PARKING_V2 = keep


@pytest.fixture
def params():
  saved = {k: getattr(P, k) for k in SWITCHES}
  yield P
  for k, v in saved.items():
    setattr(P, k, v)


def test_shipped_values():
  assert P.TX_GOVERNOR_MIRROR is True and P.TX_GOVERNOR_MIRROR_MIN_KPH == 50.0 and P.WHEEL_OUTRUN_MARGIN_MIRROR_CAN == 1
  assert P.WHEEL_OUTRUN_MARGIN_CAN == 2                                # the old pair is the kill-switch path
  assert P.LFA_HOLD_OFF_CLEAR_FRAMES == 20 and P.LFA_HOLD_OFF_CLEAR_MAX == 2
  assert P.PARKING_STALE_SIGNATURE_CLEAR is True and P.FIRST_FRAME_ANCHOR is True
  assert P.ACI_HOLD_ZERO_UNRELAYED is True and P.LAT_GRANT_GATE is True and P.ACI_HOLD_ZERO_REJECT_FRAMES == 3
  assert P.LFA_HOLD_OFF_CONFIRM_FRAMES == 30                           # unchanged outside the cruise-edge window
  assert (P.LFA_HOLD_OFF_FAST_CONFIRM_FRAMES, P.LFA_HOLD_OFF_FAST_WINDOW_FRAMES, P.LFA_HOLD_OFF_FAST_GUARD_FRAMES) == (10, 150, 50)
  assert P.GW_FOLLOW_MDPS_REF is True and P.GW_FOLLOW_EXIT_TAPER is True
  assert P.LOW_SPEED_SCEN_CREEP_EXIT_FRAMES == 30 and P.LOW_SPEED_SCEN_TO_ACTIVE_FRAMES == 100
  assert P.ACIGAIN_LOWSPEED_RAISE_SPEEDS_KPH == [0.0, 10.0, 20.0] and P.ACIGAIN_LOWSPEED_RAISE_V == [0.45, 0.45, 0.60]
  assert P.PASSIVE_INDICATE_CAM_FAULT is True


# ------------------------------------------------------------------ 75a TX governor
def panda_units(sim, v):
  """the panda's per-frame allowance in CAN units (opendbc/safety/lateral.h steer_angle_cmd_checks_vm): the VM delta at
  the speed fudged down by 1 m/s, truncated, plus one"""
  d = get_max_angle_delta_vm(max(v - 1.0, 1.0), sim.cc.BASELINE_VM, sim.cc.params)
  return int(np.float32(d) * np.float32(10.0) + np.float32(1.0))


def wire_steps(v_kph, amp=12.0, n=500):
  """hands off, the wheel following: a command step up and back; returns the wire angle per frame (CAN units)"""
  sim = Sim(); v = v_kph * KPH; w = 0.0; tx = []
  for i in range(-500, n):
    cmd = amp if 0 <= i < n // 2 else 0.0
    sim.step(v=max(v, 50.0 * KPH) if i < -250 else v, cmd=cmd, wheel=w, lfa_state=0, lead_dist=6.0)   # (on the road first)
    assert sim.effective_lat_active() or i < -200
    t = sim.tx_angle(); w += (t - w) * 0.2
    if i >= -1:
      tx.append(int(round(t * 10.0)))
  return sim, np.array(tx)


def test_governor_is_one_unit_inside_the_panda_from_50_kph():
  rates = {}
  for kph in range(50, 131, 5):
    sim, tx = wire_steps(kph)
    step = int(np.max(np.abs(np.diff(tx))))
    assert step <= panda_units(sim, kph * KPH) - 1, kph             # never the panda's last unit
    rates[kph] = step
  assert [rates[k] for k in (60, 70, 80, 90, 100, 120)] == [4, 3, 2, 2, 1, 1]   # x 10 deg/s (was 2 / 1 / 1 / 1 / 1 / 1)


def test_governor_keeps_the_one_unit_floor_at_high_speed(params):
  # where the panda's allowance is down to its last units the wire still moves one unit per frame, never more than the panda
  for kph in range(140, 201, 10):
    sim, tx = wire_steps(kph, amp=4.0)
    step = int(np.max(np.abs(np.diff(tx))))
    assert 1 <= step <= panda_units(sim, kph * KPH), kph
    assert abs(tx[240] - 40) <= 3, kph                                # and the command is reached
    if panda_units(sim, kph * KPH) == 1:                              # both formulas on the floor: the old pair, bit for bit
      params.TX_GOVERNOR_MIRROR = False
      assert np.array_equal(tx, wire_steps(kph, amp=4.0)[1]), kph
      params.TX_GOVERNOR_MIRROR = True


def test_governor_kill_switch_keeps_the_old_allowance(params):
  params.TX_GOVERNOR_RESYNC_FRAMES = 10**9                             # (the 1 s saturation resync would add its own jump)
  _, new = wire_steps(80)
  params.TX_GOVERNOR_MIRROR = False
  for kph, old_units in ((60, 2), (70, 1), (80, 1), (90, 1)):
    _, tx = wire_steps(kph)
    assert int(np.max(np.abs(np.diff(tx)))) == old_units, kph
  _, killed = wire_steps(80)
  assert np.argmax(new >= 110) < np.argmax(killed >= 110) - 30        # 11 of 12 deg: 0.6 s instead of 1.1 s


def test_governor_is_unchanged_below_50_kph(params):
  for kph in (10, 25, 40, 49):
    _, new = wire_steps(kph, amp=20.0)
    params.TX_GOVERNOR_MIRROR = False
    _, old = wire_steps(kph, amp=20.0)
    params.TX_GOVERNOR_MIRROR = True
    assert np.array_equal(new, old), kph


def test_a1_ramp_is_followed_instead_of_falling_12_deg_behind(params):
  # route a1, 990.8 s: 72 km/h, hands off, the command went +9 -> -19.5 deg at 17 deg/s; the wire moved 10 deg/s
  def gap(mirror):
    params.TX_GOVERNOR_MIRROR = mirror
    sim = Sim(); w = 9.0; worst = 0.0
    for i in range(-250, 400):
      cmd = float(np.clip(9.0 - 17.0 * max(i - 50, 0) * 0.01, -19.5, 9.0))
      sim.step(v=20.0, cmd=cmd, wheel=w, lfa_state=0, lead_dist=6.0)
      w += (sim.tx_angle() - w) * 0.2
      if i >= 0:
        worst = max(worst, abs(cmd - sim.tx_angle()))
    return worst
  assert gap(True) < 0.5 and gap(False) > 10.0


def outrun(v_kph, step_can, n=40):
  sim = Sim(); v = v_kph * KPH
  for _ in range(250):
    sim.step(v=v, lfa_state=0, lead_dist=6.0)
  for _ in range(n):
    sim.step(v=v, lfa_state=0, lead_dist=6.0, mdps_step_can=step_can, tq=200.0)   # Phase 85: a wheel the driver moves
  return sim.cc.wheel_outrun_passive


def test_wheel_outrun_threshold_sits_on_the_pandas_allowance(params):
  for kph in (60, 80, 100):
    allow = panda_units(Sim(), kph * KPH)
    assert not outrun(kph, allow), kph                                # an active frame can still follow this step
    assert outrun(kph, allow + 1), kph
  assert panda_units(Sim(), 60 * KPH) == 5
  params.TX_GOVERNOR_MIRROR = False                                   # old pair: allowance 2 + margin 2 at 60 km/h
  assert outrun(60, 5) and not outrun(60, 4)


# the real panda safety code (opendbc/safety), built on the fly; skipped where there is no C compiler
@pytest.fixture(scope="module")
def panda():
  cc_bin = shutil.which("cc")
  if cc_bin is None:
    pytest.skip("no C compiler")
  from opendbc.safety.tests.libsafety import libsafety_py
  from opendbc.safety.tests.common import CANPackerSafety
  from opendbc.safety import ALTERNATIVE_EXPERIENCE
  from opendbc.car.hyundai.values import HyundaiSafetyFlags
  from opendbc.sunnypilot.car.hyundai.values import ANGLE_STEERING_MODEL_BY_CAR, encode_angle_model_id
  root = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "opendbc_repo")
  tmp = tempfile.mkdtemp()
  try:
    obj, lib = os.path.join(tmp, "safety.os"), os.path.join(tmp, "libsafety.so")
    try:
      subprocess.check_call([cc_bin, "-fPIC", "-nostdlib", "-fno-builtin", "-std=gnu11", "-Wno-pointer-to-int-cast", "-O0", "-I", root,
                             "-c", os.path.join(root, "opendbc/safety/tests/libsafety/safety.c"), "-o", obj],
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
      subprocess.check_call([cc_bin, "-shared", obj, "-o", lib], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except (subprocess.CalledProcessError, OSError):
      pytest.skip("the safety library does not build here")
    libsafety_py.load(lib)
    S = libsafety_py.libsafety
    packer = CANPackerSafety("hyundai_canfd_generated")
    param = (HyundaiSafetyFlags.EV_GAS | HyundaiSafetyFlags.CANFD_LKA_STEER_MSG | HyundaiSafetyFlags.CANFD_LKA_STEER_MSG_ALT |
             HyundaiSafetyFlags.CANFD_ALT_BUTTONS | HyundaiSafetyFlags.CCNC)
    state = {"t": 0}

    def init():
      S.set_current_safety_param_sp(encode_angle_model_id(ANGLE_STEERING_MODEL_BY_CAR["HYUNDAI_IONIQ_6_N"]))
      S.set_safety_hooks(structs.CarParams.SafetyModel.hyundaiCanfd, int(param))
      S.init_tests()
      S.set_alternative_experience(ALTERNATIVE_EXPERIENCE.ENABLE_MADS)
      S.mads_apply_alternative_experience(ALTERNATIVE_EXPERIENCE.ENABLE_MADS)

    def rx(v, wheel):
      S.set_heartbeat_engaged_mads(True)
      S.safety_rx_hook(packer.make_can_msg_safety("SCC_CONTROL", 1, {"MainMode_ACC": 1, "ACCMode": 0}))
      S.safety_rx_hook(packer.make_can_msg_safety("WHEEL_SPEEDS", 1, {f"WHL_Spd{p}Val": v * 3.6 for p in ("FL", "FR", "RL", "RR")}))
      S.safety_rx_hook(packer.make_can_msg_safety("MDPS", 1, {"STEERING_ANGLE_2": wheel}))

    def tx(vals):
      state["t"] += 10000
      S.set_timer(state["t"])
      return bool(S.safety_tx_hook(packer.make_can_msg_safety("LKAS_ALT", 0, vals)))

    yield init, rx, tx
  finally:
    shutil.rmtree(tmp, ignore_errors=True)


def panda_run(panda, vfun, cmdfun, n, lag=3):
  """the real car controller's frames into the real panda checks; the wheel follows the accepted wire `lag` frames later.
  Returns (rejected frames, active frames) after the warm-up."""
  init, rx, tx = panda
  sim = Sim(); init()
  wheel = cmdfun(0); hist = [wheel] * (lag + 1); rej = act = 0; rej_last = False
  road = 250 if vfun(0) < 34.0 * KPH else 0                            # on the road first, so that the low speeds are steered
  for i in range(-200 - road, n):
    v, cmd = (50.0 * KPH if i < -200 else vfun(max(i, 0))), cmdfun(max(i, 0))
    rx(v, wheel)
    sim.step(v=v, cmd=cmd, wheel=wheel, lfa_state=0, lead_dist=6.0, tx_rejected=rej_last)
    ok = tx(sim.lkas_alt()); rej_last = not ok
    if i >= 0:
      rej += not ok; act += sim.effective_lat_active()
    hist.append(sim.tx_angle() if ok else hist[-1]); wheel = hist[-1 - lag]
  return rej, act


def test_real_panda_accepts_every_frame_5_to_130_kph(params, panda):
  # the 1 s saturation watchdog (38-2, unchanged) deliberately sends one frame the panda drops; it is taken out here so
  # that every remaining frame is the governor's
  params.TX_GOVERNOR_RESYNC_FRAMES = 10**9
  rej = act = 0
  for kph in range(5, 131, 5):
    v = kph * KPH; amp = 12.0 if kph >= 40 else 30.0
    shapes = [lambda i, a=amp: 0.0 if (i < 30 or i >= 250) else a]                                       # step up and back
    shapes += [lambda i, a=amp, r=r: float(np.clip(r * (i - 30) * 0.01, 0, a) - np.clip(r * (i - 250) * 0.01, 0, 2 * a))
               for r in ((17.0, 40.0) if kph % 10 == 0 else (17.0,))]                                     # ramps
    for f in shapes:
      r, a = panda_run(panda, lambda i, v=v: v, f, 450)
      assert r == 0 and a == 450, (kph, r, a)
      rej += r; act += a
  for vfun in (lambda i: min(40 * KPH + 3.0 * i * 0.01, 130 * KPH), lambda i: max(130 * KPH - 6.0 * i * 0.01, 20 * KPH),
               lambda i: (50.0 + math.sin(2 * math.pi * i * 0.01)) * KPH):                               # accelerating, braking, around 50
    r, a = panda_run(panda, vfun, lambda i: 8.0 * math.sin(2 * math.pi * 0.6 * i * 0.01), 900)
    rej += r; act += a
  assert rej == 0 and act > 30000


def test_real_panda_rejects_what_exceeds_its_limit(panda):
  # the check is live: a jump of the request is dropped
  init, rx, tx = panda
  sim = Sim(); init(); oks = []
  for i in range(300):
    rx(20.0, 0.0)
    sim.step(v=20.0, cmd=0.0, wheel=0.0, lfa_state=0, lead_dist=6.0)
    frame = dict(sim.lkas_alt())
    if i == 299:
      frame["ADAS_StrAnglReqVal"] = 5.0
    oks.append(tx(frame))
  assert all(oks[200:299]) and not oks[299]


# ------------------------------------------------------------------ 75b hold-off retry wait
class Car(FactoryLfa):
  """as test_phase72.FactoryLfa, with the standby state switching off at once (no 'releasing' second), as logged"""

  def step(self, bit):
    before = self.state
    super().step(bit)
    if before == 1 and self.state == 3:
      self.state = 0
    return self.state


def answer_time(lost, gap, to=2):
  """standby LFA switched off after `lost` ignored pulses; `gap` frames after it reads off the car switches it on
  again. Returns (wait left when it was seen off, frames from the switch-on to our pulse)."""
  sim, car = Sim(), Car(state=1, ignore=lost > 0)
  while len(car.pulses) < lost:
    run(sim, car, 1)
  car.ignore = False
  while car.state != 0:
    run(sim, car, 1)
  left = sim.cc.lfa_hold_wait
  run(sim, car, gap)
  car.state = to; n = len(car.pulses); t0 = car.n
  for _ in range(1200):
    run(sim, car, 1)
    if len(car.pulses) > n:
      break
  return left, car.pulses[-1] - t0


def test_switch_on_after_a_seen_off_is_answered_after_the_confirm_time(params):
  for lost, left in ((0, 141), (1, 291), (2, 591), (3, 991)):
    w, dt = answer_time(lost, 50)
    assert w == left and dt == 30, lost                               # 0.3 s, whatever was left of the wait
  params.LFA_HOLD_OFF_CLEAR_FRAMES = 0                                # kill switch: the left-over wait runs out first
  assert [answer_time(lost, 50)[1] for lost in (0, 1, 2, 3)] == [91, 241, 541, 941]


def test_a_short_off_or_an_unknown_icon_does_not_clear_the_wait():
  assert answer_time(0, 10)[1] == 131                                 # off for 0.1 s only: the wait stays (141 - 10)
  sim = Sim()
  sim.cc.lfa_hold_wait = 500
  for _ in range(100):
    sim.step(lfa_state=-1)                                            # stale / not received: nothing is known to be off
  assert sim.cc.lfa_hold_wait == 400
  for _ in range(100):
    sim.step(lfa_state=3)
  assert sim.cc.lfa_hold_wait == 300
  for i in range(60):
    sim.step(lfa_state=0 if i % 20 < 19 else 1)                       # 'off' never lasts 0.2 s
  assert sim.cc.lfa_hold_wait == 240


def dense(delay, frames=2000):
  """a car that switches the LFA back on (standby) `delay` frames after every switch-off"""
  sim, car = Sim(), Car(state=1); off = 0; pause_at = None
  for i in range(frames):
    run(sim, car, 1)
    off = off + 1 if car.state == 0 else 0
    if off >= delay:
      car.state = 1; off = 0
    if pause_at is None and sim.cc.lfa_hold_give_up > 0:
      pause_at = i
  return pause_at, np.diff(car.pulses)


def test_clearing_cannot_pulse_storm_and_the_pause_is_not_brought_forward(params):
  for delay in (21, 30, 60):
    pause_new, gaps = dense(delay)
    assert int(np.sum(gaps[:7] < 150)) == P.LFA_HOLD_OFF_CLEAR_MAX and np.all(gaps[2:7] >= 150), delay   # two short waits, then the old spacing
    assert gaps.min() >= 10 + P.LFA_HOLD_OFF_CLEAR_FRAMES + P.LFA_HOLD_OFF_CONFIRM_FRAMES                 # toggle + seen off + confirm
    params.LFA_HOLD_OFF_CLEAR_FRAMES = 0
    pause_old, gaps_old = dense(delay)
    params.LFA_HOLD_OFF_CLEAR_FRAMES = 20
    assert np.all(gaps_old[:7] >= 150)
    assert 0 <= pause_old - pause_new <= 2 * 150, delay               # same 8 pulses, at most two waits sooner
  for delay in (1, 10):                                               # back on before it was seen off for 0.2 s: nothing changes
    params.LFA_HOLD_OFF_CLEAR_FRAMES = 20
    a = dense(delay)
    params.LFA_HOLD_OFF_CLEAR_FRAMES = 0
    b = dense(delay)
    params.LFA_HOLD_OFF_CLEAR_FRAMES = 20
    assert a[0] == b[0] and np.array_equal(a[1], b[1])


def test_cleared_wait_never_puts_a_pulse_on_an_lfa_that_is_off():
  # random switch-ons, 10 % of our pulses lost: no pulse may ever arrive while the car's LFA is off or releasing
  rng = np.random.default_rng(75)
  for seed in range(3):
    sim, car = Sim(), Car(state=0); nxt = 100; wrong = 0; prev = 0
    for i in range(9000):
      if i >= nxt and car.state == 0:
        car.state = int(rng.choice([1, 2])); nxt = i + int(rng.choice([40, 120, 400, 2500]))
      car.ignore = bool(rng.random() < 0.1)
      sim.step(lfa_state=car.state)
      bit = int(sim.lkas_alt()["LFA_BUTTON"])
      wrong += bool(bit and not prev and car.state in (0, 3))
      prev = bit
      car.step(bit)
    assert wrong == 0 and len(car.pulses) > 5, seed


# ------------------------------------------------------------------ 75c stale parking signature
def boot_through_30():
  """the controller starts while the car accelerates through 30 km/h (signature set at frame 50, never armed), 20 s on
  the road, then a stop behind a lead and the launch"""
  sim = Sim()
  for i in range(150):
    sim.step(v=(26.0 + 8.0 * i / 149.0) * KPH, lfa_state=0)
  for _ in range(2000):
    sim.step(v=50.0 * KPH, lfa_state=0)
  on_road = sim.cc.parking_signature_seen
  for i in range(400):
    sim.step(v=50.0 * KPH * (1 - i / 399.0), lfa_state=0, lead_dist=5.0)
  for _ in range(500):
    sim.step(v=0.0, lfa_state=0, lead_dist=5.0, lat_active=False)
  return sim, on_road


@pytest.mark.usefixtures("old_parking")
def test_signature_that_never_armed_the_latch_is_dropped_on_the_road(params):
  sim, on_road = boot_through_30()
  assert on_road is False and not sim.cc.parking_mode_active and sim.cc.road_context
  for i in range(300):                                                # the launch behind the lead is steered
    sim.step(v=min(i * 0.03, 30.0 * KPH), cmd=1.0, lfa_state=0, lead_dist=5.0)
  assert sim.effective_lat_active()
  params.PARKING_STALE_SIGNATURE_CLEAR = False
  sim, on_road = boot_through_30()
  assert on_road is True and sim.cc.parking_mode_active               # the old behaviour: latched at a later stop


@pytest.mark.usefixtures("old_parking")
def test_real_signatures_still_arm_and_release_as_before():
  sim = Sim()
  for _ in range(400):
    sim.step(v=8.0 * KPH, wheel=300.0, lfa_state=0)                   # a parking turn
  assert sim.cc.parking_mode_active
  for _ in range(250):
    sim.step(v=40.0 * KPH, lfa_state=0)
  assert not sim.cc.parking_mode_active and not sim.cc.parking_signature_seen
  for _ in range(400):
    sim.step(v=3.0 * KPH, gear='reverse', lfa_state=0)                # and a new one arms again
  assert sim.cc.parking_mode_active


# ------------------------------------------------------------------ 75d first control frame
def test_first_frame_with_lateral_on_starts_at_the_wheel(params):
  def first():
    sim = Sim(); tx = []
    for _ in range(60):
      sim.step(v=80.0 * KPH, wheel=5.0, mdps_angle_2=5.2, cmd=5.0, lfa_state=0)
      tx.append(sim.tx_angle())
    return sim, tx
  sim, tx = first()
  assert tx[0] == pytest.approx(5.0, abs=0.06) and min(tx) > 4.7      # (the 0.15 deg command band)
  params.FIRST_FRAME_ANCHOR = False
  assert first()[1][0] == pytest.approx(0.2, abs=0.11)                # the old behaviour: from 0 deg


def test_first_frame_starts_the_engage_taper_and_the_wire_reference():
  sim = Sim(); tx = []
  sim.step(v=80.0 * KPH, wheel=5.0, mdps_angle_2=5.2, cmd=9.0, lfa_state=0)
  assert sim.cc.frames_since_apply_anchor == 0 and sim.cc.apply_angle_last == pytest.approx(5.04, abs=0.011)
  tx.append(sim.tx_angle())
  for _ in range(150):
    sim.step(v=80.0 * KPH, wheel=5.0, mdps_angle_2=5.2, cmd=9.0, lfa_state=0)
    tx.append(sim.tx_angle())
  d = np.diff(tx)
  assert 4.9 <= tx[0] <= 5.3                                          # the wire starts from the MDPS angle, not from 0
  assert np.max(np.abs(d[:20])) <= 0.1 + 1e-6 and tx[25] <= 6.7       # 4 deg/s at first, rising (the VM rate would be at 9 deg)
  assert tx[-1] == pytest.approx(9.0, abs=0.3)
  sim2 = Sim()                                               # a first frame with lateral off: as before
  sim2.step(v=80.0 * KPH, wheel=5.0, cmd=9.0, lat_active=False, lfa_state=0)
  assert sim2.tx_angle() == pytest.approx(5.0) and sim2.cc.first_ctrl_frame is False


# ------------------------------------------------------------------ 75e gain while nobody receives it, lateral grant
def gain_trace(seq, **kw):
  sim = Sim(); g = []
  for st in seq:
    sim.step(v=80.0 * KPH, wheel=0.0, cmd=0.5, **st, **kw)
    g.append((sim.tx_gain(), sim.cc.aci_gain_last))
  return np.array(g)


def test_gain_is_held_at_zero_while_the_factory_lfa_is_active(params):
  seq = [dict(lfa_state=0)] * 100 + [dict(lfa_state=2)] * 100 + [dict(lfa_state=3)] * 40 + [dict(lfa_state=0)] * 100
  g = gain_trace(seq)
  assert g[99, 0] > 0.3 and np.all(g[100:200] == 0.0)                 # wire and internal state
  up = np.diff(g[199:, 1])
  assert np.all(up <= 0.04 + 1e-9) and g[200, 0] <= 0.04 and g[239, 0] < 0.3   # re-ramps under the rate cap from 0
  assert g[-1, 0] > 0.5
  params.ACI_HOLD_ZERO_UNRELAYED = False
  old = gain_trace(seq)
  assert old[199, 1] >= 0.8 and old[200, 0] >= 0.8                    # the old behaviour: it kept ramping behind the factory LFA


def test_gain_is_held_at_zero_after_rejected_frames(params):
  seq = [dict(lfa_state=0)] * 100 + [dict(lfa_state=0, tx_rejected=True)] * 57 + [dict(lfa_state=0)] * 100
  g = gain_trace(seq)
  n = P.ACI_HOLD_ZERO_REJECT_FRAMES
  assert np.all(g[100:100 + n - 1, 0] > 0.3)                          # not before the run is n frames long
  assert np.all(g[100 + n - 1:157] == 0.0) and g[157, 0] == pytest.approx(0.004) and g[170, 0] < 0.1
  params.ACI_HOLD_ZERO_REJECT_FRAMES = 0                              # 0 = rejects never zero it
  assert gain_trace(seq)[157, 0] > 0.6
  params.ACI_HOLD_ZERO_REJECT_FRAMES = n
  params.ACI_HOLD_ZERO_UNRELAYED = False
  assert gain_trace(seq)[157, 0] > 0.6                                # 0.57 s of rejects used to end at 0.63


def test_one_or_two_rejected_frames_leave_the_gain_alone(params):
  # an isolated reject while steering (the 38-2 resync frame, route a1 885.96 s / 992.30 s) is not "nobody receives it"
  def trace(nrej, v_kph, off):
    sim = Sim(); g = []; w = 0.0
    for i in range(500):
      cmd = min(5.0 * i * 0.01, 8.0)                                  # curve entry, hands off, the wheel `off` deg behind
      sim.step(v=v_kph * KPH, cmd=cmd, wheel=w, lfa_state=0, lead_dist=6.0, tx_rejected=200 <= i < 200 + nrej)
      w = max(sim.tx_angle() - off, 0.0)
      g.append(sim.tx_gain())
    return np.array(g)
  for v_kph, off in ((72, 0.3), (72, 1.0), (100, 0.2)):
    for nrej in (1, 2):
      new = trace(nrej, v_kph, off)
      params.ACI_HOLD_ZERO_UNRELAYED = False
      old = trace(nrej, v_kph, off)
      params.ACI_HOLD_ZERO_UNRELAYED = True
      assert np.array_equal(new, old) and (nrej > 1 or new[200:260].min() > 0.3), (v_kph, off, nrej)
    assert trace(3, v_kph, off)[202] == 0.0                           # a run of three does


def test_blind_resync_frame_does_not_drop_the_gain(params, panda):
  # 100 km/h, +10 -> -10 deg at 16 deg/s with the 1 s saturation watchdog ON: its one sacrificed frame is rejected by
  # the real panda; the gain on the wire goes on as with the kill switch
  def loop():
    init, rx, tx = panda
    sim = Sim(); init()
    wheel = 10.0; hist = [wheel] * 4; rej_last = False; g = []; rej = []
    for i in range(-300, 500):
      cmd = float(np.clip(10.0 - 16.0 * max(i - 100, 0) * 0.01, -10.0, 10.0))
      rx(100.0 * KPH, wheel)
      sim.step(v=100.0 * KPH, cmd=cmd, wheel=wheel, lfa_state=0, lead_dist=6.0, tx_rejected=rej_last)
      ok = tx(sim.lkas_alt()); rej_last = not ok
      hist.append(sim.tx_angle() if ok else hist[-1]); wheel = hist[-4]
      if i >= 0:
        g.append(sim.tx_gain()); rej.append(not ok)
    return np.array(g), np.array(rej)
  g, rej = loop()
  assert 1 <= rej.sum() <= 3 and not np.any(rej[1:] & rej[:-1])       # isolated resync frames only
  for k in np.flatnonzero(rej):
    assert g[k + 1] >= g[k] - 0.1 and g[k + 1:k + 40].min() > 0.5, k  # no step to 0 on the next accepted frame
  params.ACI_HOLD_ZERO_UNRELAYED = False
  g_old, rej_old = loop()
  assert np.array_equal(g, g_old) and np.array_equal(rej, rej_old)


def test_yield_to_the_driver_is_unaffected_by_the_gain_hold(params):
  # no factory LFA, no rejects: hands-off, a resting hand, a press, a blinker - the gain trace is bit-identical
  def trace():
    sim = Sim(); g = []
    for i in range(1200):
      tq = 0.0 if i < 300 else (150.0 if i < 600 else (500.0 if i < 900 else 0.0))
      sim.step(v=70.0 * KPH, cmd=2.0, wheel=2.0, tq=tq, blinker=1000 <= i < 1100, lfa_state=0)
      g.append(sim.tx_gain())
    return np.array(g)
  new = trace()
  params.ACI_HOLD_ZERO_UNRELAYED = False
  assert np.array_equal(new, trace())
  assert new[290] > 0.7 and new[890] < 0.1                            # and it does yield


def grant(gate=True):
  if not gate:
    P.LAT_GRANT_GATE = False
  sim = Sim(); out = []
  for i in range(320):
    sim.step(v=60.0 * KPH, wheel=2.0, mdps_angle_2=2.2, cmd=8.0 if i >= 100 else 2.0, lfa_state=0,
             lat_granted=not (100 <= i < 160))
    out.append((sim.effective_lat_active(), sim.tx_angle(), sim.tx_gain(), sim.cc.lat_passive_reason, sim.cc.apply_angle_last))
  return sim, out


def test_ungranted_lateral_sends_passive_frames_on_the_wheel_and_tapers_in(params):
  sim, out = grant()
  assert out[99][0] and out[99][2] > 0.3
  for act, tx, gain, reason, apply in out[100:160]:
    assert not act and tx == pytest.approx(2.2) and gain == 0.0 and reason == ccmod.PR_NOT_GRANTED
    assert apply == pytest.approx(2.0)                                # the request waits on the wheel
  assert sim.lkas_alt()["LKAS_ANGLE_ACTIVE"] == 2
  assert out[160][0] and out[160][2] <= 0.004 + 1e-9 and out[160][3] == ccmod.PR_NONE
  tx = np.array([o[1] for o in out])
  assert np.max(np.abs(np.diff(tx[160:180]))) <= 0.1 + 1e-9           # engage taper: 4 deg/s at first, not the VM rate
  assert tx[170] < 3.0 and tx[-1] == pytest.approx(8.0, abs=0.3)
  assert np.all(np.diff([o[2] for o in out[160:200]]) <= 0.04 + 1e-9)


def test_grant_kill_switch_and_absent_field(params):
  _, out = grant(gate=False)
  assert all(o[0] for o in out[100:160]) and out[159][1] > 7.0        # kill switch: active frames as before
  params.LAT_GRANT_GATE = True

  class NoField(types.SimpleNamespace):                               # a CarControlSP object without the field = granted
    def __setattr__(self, k, v):
      if k != "latGranted":
        super().__setattr__(k, v)
  sim = Sim()
  sim.cc._cc_sp = NoField(**vars(sim.cc._cc_sp))
  for _ in range(50):
    sim.step(v=60.0 * KPH, lfa_state=0, lat_granted=False)
  assert not hasattr(sim.cc._cc_sp, "latGranted") and sim.effective_lat_active() and not sim.cc.lat_ungranted


def test_lat_granted_schema():
  from openpilot.cereal import custom
  from openpilot.selfdrive.car.helpers import convert_carControlSP
  assert structs.CarControlSP().latGranted is True                    # default: granted
  assert custom.CarControlSP.new_message().latGranted is True
  assert convert_carControlSP(custom.CarControlSP.new_message().as_reader()).latGranted is True
  m = custom.CarControlSP.new_message(); m.latGranted = False
  with custom.CarControlSP.from_bytes(m.to_bytes()) as r:
    assert convert_carControlSP(r).latGranted is False
  old = custom.CarControlSP.new_message(); old.stockLfaWithOp = False  # a message written before the field existed
  assert convert_carControlSP(old.as_reader()).latGranted is True
  fields = custom.CarControlSP.schema.fields
  assert fields["latGranted"].proto.ordinal.explicit == 8 and fields["stockLfaWithOp"].proto.ordinal.explicit == 7


# ------------------------------------------------------------------ 75f fast confirm after a cruise edge
def cruise_case(edge_at, on_at, n=500, to=2):
  """cruise engaged at frame edge_at (None = never, 0 = already on at the controller's first frame); the car switches
  the LFA on at on_at. Returns the pulse times relative to the switch-on and the final LFA state."""
  sim, car = Sim(), FactoryLfa(state=0)
  for i in range(n):
    if i == on_at:
      car.state = to
    sim.step(v=20.0, lfa_state=car.state, cruise_enabled=edge_at is not None and i >= edge_at)
    car.step(int(sim.lkas_alt()["LFA_BUTTON"]))
  return [p - on_at for p in car.pulses], car.state


def test_fast_confirm_only_right_after_a_cruise_edge(params):
  assert cruise_case(100, 107) == ([10], 0)                           # the car's own switch-on 70 ms after the edge: 0.1 s
  assert cruise_case(100, 107, to=1) == ([10], 0)
  assert cruise_case(None, 107) == ([30], 0)                          # no cruise edge: 0.3 s as before
  assert cruise_case(100, 300, 700) == ([30], 0)                      # later than 1.5 s after the edge
  assert cruise_case(0, 107) == ([30], 0)                             # cruise already on when the controller started: no edge
  params.LFA_HOLD_OFF_FAST_WINDOW_FRAMES = 0                          # kill switch
  assert cruise_case(100, 107) == ([30], 0)


def test_fast_confirm_never_in_the_controllers_first_half_second():
  for edge, on in ((1, 3), (5, 12), (20, 30), (30, 38)):
    pulses, state = cruise_case(edge, on)
    # either the normal 0.3 s confirm, or the fast one once the first 0.5 s are over
    assert (pulses[0] >= 30 or pulses[0] + on >= P.LFA_HOLD_OFF_FAST_GUARD_FRAMES) and state == 0, (edge, on)
  assert cruise_case(1, 3)[0] == [30] and cruise_case(5, 12)[0] == [30]
  assert cruise_case(20, 30)[0] == [21]                               # controller frame 50, where the guard ends - not 0.1 s after the icon
  assert cruise_case(1, 45)[0] == [10]                                # the confirm completes after the guard


def test_fast_confirm_needs_two_icon_samples_and_is_used_once_per_edge():
  sim = Sim(); bits = []
  for i in range(400):                                                # one 20 Hz sample of 'on' (5 frames) is not enough
    st = 2 if 107 <= i < 112 else 0
    sim.step(v=20.0, lfa_state=st, cruise_enabled=i >= 100); bits.append(int(sim.lkas_alt()["LFA_BUTTON"]))
  assert sum(bits) == 0
  sim, car = Sim(), FactoryLfa(state=0, ignore=True)         # the car ignores the pulse: the retry is the normal one
  for i in range(400):
    if i == 107:
      car.state = 1
    sim.step(v=20.0, lfa_state=car.state, cruise_enabled=i >= 100)
    car.step(int(sim.lkas_alt()["LFA_BUTTON"]))
  assert car.pulses[0] in (117, 118) and car.pulses[1] - car.pulses[0] == 151 and sim.cc.lfa_fast_left == 0
  # the first pulse switches the LFA off, 0.2 s of icon 0 clears the wait and the car switches it on again inside the
  # 1.5 s window: the second answer takes the normal confirm time
  sim, car = Sim(), Car(state=0)
  for i in range(400):
    if i in (107, 160):
      car.state = 1
    sim.step(v=20.0, lfa_state=car.state, cruise_enabled=i >= 100)
    car.step(int(sim.lkas_alt()["LFA_BUTTON"]))
  assert car.pulses == [117, 160 + P.LFA_HOLD_OFF_CONFIRM_FRAMES]


# ------------------------------------------------------------------ 75g follow band
def test_follow_band_is_taken_around_the_mdps_angle(params):
  def band(**kw):
    sim = Sim()
    for _ in range(300):
      sim.step(v=25.0, cmd=7.0, wheel=1.0, lfa_state=2, **kw)
    return sim.tx_angle()
  assert band(mdps_angle_2=1.6) == pytest.approx(2.6, abs=0.051)      # 1 deg from the MDPS's own angle
  assert band(mdps_angle_2=0.5) == pytest.approx(1.5, abs=0.051)
  assert band() == pytest.approx(2.0, abs=0.051)                      # same sensor value: as before
  params.GW_FOLLOW_MDPS_REF = False
  assert band(mdps_angle_2=1.6) == pytest.approx(2.0, abs=0.051)      # kill switch: around steeringAngleDeg


def handover(v=15.0, cmd=30.0):
  """the factory LFA moves the wheel 10 deg/s for 2 s with our command far away, releases (1 s), is off"""
  sim = Sim(); tx = []; wh = []; w = 1.0
  for i in range(700):
    st = 2 if i < 300 else (3 if i < 400 else 0)
    if 100 <= i < 300:
      w += 0.1
    sim.step(v=v, cmd=cmd, wheel=w, lfa_state=st)
    tx.append(sim.tx_angle()); wh.append(w)
  return sim, np.array(tx), np.array(wh)


def test_engage_taper_restarts_once_when_a_binding_band_ends(params):
  sim, tx, wh = handover()
  assert np.max(np.abs(tx[50:400] - wh[50:400])) <= P.GW_FOLLOW_BAND_DEG + 0.051   # keeps up with the moving wheel while bound
  assert sim.cc.frames_since_apply_anchor == 299                      # restarted at frame 400, once
  assert tx[410] - tx[399] <= 0.6 and tx[420] - tx[399] <= 1.4        # 4 deg/s rising, not the VM rate
  assert tx[-1] - tx[399] > 6.0                                       # and then it goes to the command
  params.GW_FOLLOW_EXIT_TAPER = False
  _, old, _ = handover()
  assert old[410] - old[399] > 4.0                                    # the old behaviour: 5 deg in 0.1 s


def test_no_taper_restart_when_the_band_was_not_binding():
  sim = Sim()
  for i in range(500):
    st = 2 if i < 300 else (3 if i < 400 else 0)
    sim.step(v=25.0, cmd=1.3, wheel=1.0, lfa_state=st)                # the command is inside the band
  assert not sim.cc.gw_follow and sim.cc.frames_since_apply_anchor >= 499


# ------------------------------------------------------------------ 75h low speed: scenario gate dwell
def launch(**kw):
  """on the road, a stop as first car, then the launch at 2 m/s2; returns frames from leaving the creep zone
  (12 km/h) to the first steered frame, and the speed there"""
  sim = Sim()
  for _ in range(600):
    sim.step(v=50.0 * KPH, lfa_state=0)
  for _ in range(300):
    sim.step(v=0.0, lfa_state=0, lat_active=False)
  t12 = None
  for i in range(900):
    v = min(i * 0.02, 40.0 * KPH)
    step_kw = dict(cmd=2.0)
    step_kw.update({k: (f(i, v) if callable(f) else f) for k, f in kw.items()})
    sim.step(v=v, wheel=0.0, lfa_state=0, lat_active=v > 5.0 * KPH, **step_kw)
    if t12 is None and v > P.CREEP_GATE_EXIT_MS:
      t12 = i
    if sim.effective_lat_active():
      return i - t12, v / KPH
  return None


def test_first_car_launch_is_steered_0p3_s_after_the_creep_zone(params):
  dt, v = launch()
  assert dt == 29 and 13.5 < v < 15.0                                 # was 1.0 s / 19 km/h
  params.LOW_SPEED_SCEN_CREEP_EXIT_FRAMES = P.LOW_SPEED_SCEN_TO_ACTIVE_FRAMES   # kill switch
  dt, v = launch()
  assert dt == 99 and v > 18.5


def test_other_openings_of_the_gate_keep_the_1_s_dwell():
  assert launch(blinker=True)[0] == 99                                # blinker on while leaving the creep zone
  # a big command through the junction: the gate stays shut after the creep zone and opens on the |cmd| path
  dt, _ = launch(cmd=lambda i, v: 60.0 if i < 200 else 2.0)
  t12 = int(P.CREEP_GATE_EXIT_MS / 0.02) + 1
  assert dt == (200 - t12) + 99
  # a lead comes into the follow window inside the creep zone: that opening has its own 1.0 s running
  sim = Sim()
  for _ in range(600):
    sim.step(v=50.0 * KPH, lfa_state=0)
  for _ in range(300):
    sim.step(v=5.0 * KPH, lfa_state=0)
  assert not sim.cc.low_speed_scen_ok
  n = 0
  while not sim.cc.low_speed_scen_ok:
    sim.step(v=(8.0 if n < 50 else 14.0) * KPH, lfa_state=0, lead_dist=6.0); n += 1
  assert n == 100


def test_going_passive_still_takes_0p3_s():
  sim = Sim()
  for _ in range(600):
    sim.step(v=50.0 * KPH, lfa_state=0)
  for _ in range(200):
    sim.step(v=15.0 * KPH, cmd=2.0, lfa_state=0)
  assert sim.effective_lat_active()
  n = 0
  while sim.cc.low_speed_scen_ok:
    sim.step(v=15.0 * KPH, cmd=60.0, lfa_state=0); n += 1
  assert n == P.LOW_SPEED_SCEN_TO_PASSIVE_FRAMES


# ------------------------------------------------------------------ 75h low speed: hands-off gain ceiling
def settled_gain(v_kph, **kw):
  sim = Sim()
  for _ in range(600):
    sim.step(v=50.0 * KPH, cmd=1.0, wheel=1.0, lfa_state=0, lead_dist=6.0)
  for _ in range(800):
    sim.step(v=v_kph * KPH, cmd=1.0, wheel=1.0, lfa_state=0, lead_dist=6.0, **kw)
  assert sim.effective_lat_active()
  return sim.tx_gain()


def test_low_speed_hands_off_ceiling_is_one_rung_up(params):
  want = {6: 0.45, 10: 0.45, 15: 0.525, 20: 0.60, 25: 0.60, 30: 0.60}
  for kph, g in want.items():
    assert settled_gain(kph) == pytest.approx(g, abs=0.0041), kph
  for kph in (35, 40, 60, 100):                                       # blends into the existing table: unchanged above ~33 km/h
    assert settled_gain(kph) == pytest.approx(float(np.interp(kph, [0, 20, 40, 120], [0.18, 0.30, 0.75, 0.95])), abs=0.0041), kph
  params.ACIGAIN_LOWSPEED_RAISE_V = [0.0, 0.0, 0.0]                   # kill switch = the old table
  for kph in (6, 10, 15, 20, 25, 30):
    assert settled_gain(kph) == pytest.approx(float(np.interp(kph, [0, 20, 40, 120], [0.18, 0.30, 0.75, 0.95])), abs=0.0041), kph


def test_low_speed_raise_leaves_blinker_and_driver_paths_alone(params):
  f = ccmod.compute_torque_reduction_gain
  def settle(v, tq, raise_, **kw):
    g = kw.pop("start", 0.0)
    for _ in range(600):
      g = f(tq, v, True, g, 0.0, lowspeed_raise=raise_, **kw)
    return g
  for v in (0.0, 5.0, 10.0, 15.0, 20.0, 30.0):
    for tq in (0.0, 60.0, 200.0):                                     # blinker ceiling (0.45 / 0.28 by torque): identical
      assert settle(v, tq, True, blinker_on=True) == settle(v, tq, False, blinker_on=True), (v, tq)
    for tq in (140.0, 200.0, 400.0):                                  # at / beyond full yield: the same floor
      assert settle(v, tq, True) == settle(v, tq, False) == pytest.approx(0.15, abs=0.0041), (v, tq)
    for tq in (120.0, 200.0):                                         # debounced press (caller's grip band): the same floor
      assert settle(v, tq, True, grip_full=120.0, grip_floor=0.05) == settle(v, tq, False, grip_full=120.0, grip_floor=0.05)
    # the fall when the driver takes the wheel is rate-limited exactly as before (same step per frame from the same gain)
    assert f(300.0, v, True, 0.6, 0.0, lowspeed_raise=True) == f(300.0, v, True, 0.6, 0.0, lowspeed_raise=False)
    assert f(0.0, v, False, 0.6, 0.0, lowspeed_raise=True) == f(0.0, v, False, 0.6, 0.0, lowspeed_raise=False)   # inactive
    assert settle(v, 0.0, True, ceiling_scale=0.5) == pytest.approx(0.5 * settle(v, 0.0, True), abs=0.0041)     # accel yield scales it
  for v in (34.0, 40.0, 80.0, 120.0):                                 # above the blend: identical for every torque and error
    for tq in (0.0, 50.0, 100.0, 200.0):
      for err in (0.0, 1.0, 3.0):
        g1 = g2 = 0.0
        for _ in range(300):
          g1 = f(tq, v, True, g1, err, lowspeed_raise=True); g2 = f(tq, v, True, g2, err, lowspeed_raise=False)
          assert g1 == g2, (v, tq, err)
  # the raise is a floor under the ceiling: the error boost is not multiplied on top of it
  assert settle(20.0, 0.0, True) == settle(20.0, 0.0, True, start=0.0) == pytest.approx(0.60, abs=0.0041)
  g = 0.0
  for _ in range(600):
    g = f(0.0, 20.0, True, g, 3.0, lowspeed_raise=True)
  assert g == pytest.approx(0.60, abs=0.0041)
  assert f(0.0, 20.0, True, 0.0, 0.0) == f(0.0, 20.0, True, 0.0, 0.0, lowspeed_raise=False)   # default argument = off


def test_low_speed_raise_keeps_the_yield_in_the_harness(params):
  # driver torque at 15 km/h in a queue: the gain still drops to the same floor at the same rate; blinker as before
  def trace(**kw):
    sim = Sim(); g = []
    for _ in range(600):
      sim.step(v=50.0 * KPH, cmd=1.0, wheel=1.0, lfa_state=0, lead_dist=6.0)
    for i in range(900):
      sim.step(v=15.0 * KPH, cmd=1.0, wheel=1.0, lfa_state=0, lead_dist=6.0, **{k: f(i) for k, f in kw.items()})
      g.append(sim.tx_gain())
    return np.array(g)
  press = dict(tq=lambda i: 320.0 if i >= 400 else 0.0)
  new = trace(**press)
  blink_new = trace(blinker=lambda i: i >= 400)
  params.ACIGAIN_LOWSPEED_RAISE_V = [0.0, 0.0, 0.0]
  old = trace(**press)
  blink_old = trace(blinker=lambda i: i >= 400)
  assert new[399] == pytest.approx(0.524, abs=0.0041) and old[399] == pytest.approx(0.268, abs=0.0041)
  assert new[-1] == old[-1] and new[-1] <= 0.16                       # the same yielded level
  assert np.min(np.diff(new[400:])) == np.min(np.diff(old[400:]))     # the same fall rate
  assert blink_new[-1] == blink_old[-1]                               # blinker ceiling as before


# ------------------------------------------------------------------ 75i indication
def test_camera_stale_and_lfa_fault_are_indicated(params):
  def frozen():
    sim = Sim()
    for _ in range(300):
      sim.step(v=50.0 * KPH, lfa_state=0)
    assert not sim.cc.lat_passive_indicated and sim.cc.lat_passive_reason == ccmod.PR_NONE
    for _ in range(120):
      sim.cam_counter = (sim.cam_counter - 1) % 256                   # the camera's counter stops
      sim.step(v=50.0 * KPH, lfa_state=0)
    return sim
  sim = frozen()
  assert not sim.effective_lat_active() and sim.cc.lat_passive_indicated and sim.cc.lat_passive_reason == ccmod.PR_CAM_STALE
  assert sim.lkas_alt()["LKA_ASSIST"] == 0                            # the frame to the car is as before
  for _ in range(60):
    sim.step(v=50.0 * KPH, lfa_state=0)
  assert sim.effective_lat_active() and not sim.cc.lat_passive_indicated and sim.cc.lat_passive_reason == ccmod.PR_NONE
  sim = Sim()
  for _ in range(100):
    sim.step(v=50.0 * KPH, lfa_state=0, fault_lfa=1)
  assert sim.cc.lat_passive_indicated and sim.cc.lat_passive_reason == ccmod.PR_LFA_FAULT
  params.PASSIVE_INDICATE_CAM_FAULT = False                           # kill switch: not shown (the reason code stays)
  sim = frozen()
  assert not sim.cc.lat_passive_indicated and sim.cc.lat_passive_reason == ccmod.PR_CAM_STALE


def test_reason_codes():
  codes = [ccmod.PR_NONE, ccmod.PR_CAM_STALE, ccmod.PR_LFA_FAULT, ccmod.PR_REVERSE, ccmod.PR_PARKING, ccmod.PR_ANGLE,
           ccmod.PR_LOWSPEED_GRIP, ccmod.PR_SCENARIO, ccmod.PR_VM_REJECT, ccmod.PR_NOT_GRANTED, ccmod.PR_WHEEL_OUTRUN]
  assert codes == list(range(11))
  def road():
    sim = Sim()
    for _ in range(600):
      sim.step(v=50.0 * KPH, lfa_state=0)
    return sim
  sim = road()
  assert sim.cc.lat_passive_reason == ccmod.PR_NONE
  for _ in range(400):
    sim.step(v=8.0 * KPH, wheel=300.0, lfa_state=0)
  assert sim.cc.lat_passive_reason == ccmod.PR_PARKING and sim.cc.lat_passive_indicated
  sim = road()
  for _ in range(200):
    sim.step(v=15.0 * KPH, cmd=80.0, lfa_state=0)
  assert sim.cc.lat_passive_reason == ccmod.PR_SCENARIO
  sim = road()
  for _ in range(100):
    sim.step(v=15.0 * KPH, tq=500.0, lfa_state=0, lead_dist=6.0)
  assert sim.cc.lat_passive_reason == ccmod.PR_LOWSPEED_GRIP
  sim = road()
  for _ in range(100):
    sim.step(v=50.0 * KPH, wheel=60.0, cmd=60.0, tq=600.0, lfa_state=0)
  assert sim.cc.lat_passive_reason == ccmod.PR_ANGLE
  sim = road()
  for _ in range(50):
    sim.step(v=3.0 * KPH, gear='reverse', lfa_state=0)
  assert sim.cc.lat_passive_reason == ccmod.PR_REVERSE
  sim = road()
  for _ in range(20):
    sim.step(v=80.0 * KPH, lfa_state=0, mdps_step_can=30, tq=200.0)   # Phase 85: a wheel the driver moves
  assert sim.cc.wheel_outrun_passive and sim.cc.lat_passive_reason == ccmod.PR_WHEEL_OUTRUN
  assert not sim.cc.lat_passive_indicated                             # not a 'paused' state for the display, as before
  sim.step(v=80.0 * KPH, lfa_state=0, lat_active=False)
  assert sim.cc.lat_passive_reason == ccmod.PR_NONE                   # lateral off: nothing to explain
  park = Sim()
  for _ in range(400):
    park.step(v=8.0 * KPH, wheel=300.0)
  assert park.cc.lat_passive_reason == ccmod.PR_PARKING


def test_factory_lfa_active_and_hold_off_pause_reach_car_state_sp():
  from openpilot.cereal import custom
  sim = Sim()
  sim.step(v=25.0, lfa_state=2)
  assert sim.cc.gw_flags & ccmod.GW_LFA_ACTIVE and not sim.cc.gw_flags & ccmod.GW_GAVE_UP
  sim.cc.lfa_hold_give_up = 500
  sim.step(v=25.0, lfa_state=0)
  assert sim.cc.gw_flags & ccmod.GW_GAVE_UP and not sim.cc.gw_flags & ccmod.GW_LFA_ACTIVE
  s = custom.CarStateSP.new_message()
  s.gatewayFlags = sim.cc.gw_flags & 0xFF; s.lateralPausedReason = ccmod.PR_WHEEL_OUTRUN
  with custom.CarStateSP.from_bytes(s.to_bytes()) as r:
    assert r.gatewayFlags & ccmod.GW_GAVE_UP and r.lateralPausedReason == 10
  assert custom.CarStateSP.new_message().lateralPausedReason == 0
  assert custom.CarStateSP.schema.fields["lateralPausedReason"].proto.ordinal.explicit == 4
  root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
  card = open(os.path.join(root, "openpilot/selfdrive/car/card.py")).read()
  assert "CS_SP.gatewayFlags = int(getattr(self.CI.CC, 'gw_flags', 0)) & 0xFF" in card
  assert "CS_SP.lateralPausedReason = int(getattr(self.CI.CC, 'lat_passive_reason', 0)) & 0xFF" in card


def test_card_mirrors_the_reason_code_into_the_message():
  # card.state_update's own lines, run against a real CarStateSP builder and the real controller
  from openpilot.cereal import custom
  from openpilot.selfdrive.car.helpers import convert_to_capnp
  import types
  sim = Sim()
  for _ in range(400):
    sim.step(v=8.0 * KPH, wheel=300.0, lfa_state=2)
  root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
  lines = [ln.strip() for ln in open(os.path.join(root, "openpilot/selfdrive/car/card.py")).read().splitlines()
           if ln.strip().startswith("CS_SP.") and "getattr(self.CI.CC" in ln]
  assert len(lines) == 4
  CS_SP = convert_to_capnp(structs.CarStateSP())
  me = types.SimpleNamespace(CI=types.SimpleNamespace(CC=sim.cc))
  for ln in lines:
    exec(ln, {"CS_SP": CS_SP, "self": me})
  assert isinstance(CS_SP, type(custom.CarStateSP.new_message()))
  assert CS_SP.lateralControlPaused is True and CS_SP.lateralPausedReason == ccmod.PR_PARKING
  assert CS_SP.gatewayFlags & ccmod.GW_LFA_ACTIVE and CS_SP.gatewayFlags & ccmod.GW_SOLO
