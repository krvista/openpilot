"""Phase 59 (report §65-66): MDPS inverse-model compensator. Closed loop against the identified plant
  d wheel/dt = Kf * du/dt + Kp * (u - wheel) - Kc * wheel,  u = TX delayed by d
(55-70 km/h all-data fit). Checks: the wheel reaches a steady curve command (the uncompensated path stays short), the request
stays within GMAX of the reference, a hand stops the feedback and returns the request to the command, the trim / stall kick are
off while active, the compensator is off below MIN_KPH and with the kill switch."""
import numpy as np
import pytest

from phase_tests.harness import Sim
from opendbc.car.hyundai.values import CarControllerParams as P
import opendbc.car.hyundai.carcontroller as ccmod

USE_MDPS_COMP = True


@pytest.fixture(autouse=True)
def _compensator_on():
  # Phase 61 ships the compensator off (MDPS_COMP_LAMBDA = 0); these tests cover it in its enabled configuration
  old = P.MDPS_COMP_LAMBDA
  P.MDPS_COMP_LAMBDA = 2.0
  yield
  P.MDPS_COMP_LAMBDA = old
D, KF, KP, KC = 0.32, 0.58, 0.18, 0.16


def run(cmd_fn, n, v=60 / 3.6, tq_fn=None, lam=None, w0=0.0):
  old = P.MDPS_COMP_LAMBDA
  try:
    if lam is not None:
      P.MDPS_COMP_LAMBDA = lam
    sim = Sim(); w = w0; hist = []
    W, TX, FL, REF = [], [], [], []
    for i in range(n):
      tq = tq_fn(i) if tq_fn else 0.0
      sim.step(v=v, tq=tq, wheel=w, cmd=cmd_fn(i))
      tx = sim.tx_angle() if sim.effective_lat_active() else w
      hist.append(tx)
      nd = int(round(D / 0.01))
      u1 = hist[-1 - nd] if len(hist) > nd else hist[0]; u2 = hist[-2 - nd] if len(hist) > nd + 1 else hist[0]
      w = w + 0.01 * (KF * (u1 - u2) / 0.01 + KP * (u1 - w) - KC * w)
      W.append(w); TX.append(tx); FL.append(sim.cc.steer_debug_flags); REF.append(cmd_fn(i))
    return np.array(W), np.array(TX), FL, sim
  finally:
    P.MDPS_COMP_LAMBDA = old


def ramp_to(a, t0=100, rate=10.0):
  return lambda i: float(np.clip((i - t0) * 0.01 * rate, 0.0, a))


class TestMdpsCompensator:
  def test_wheel_reaches_a_steady_curve_command(self):
    # 6 deg: the steady lead it needs (6 * Kc / Kp ~ 5.3 deg) fits inside GMAX
    w_on, tx_on, fl, sim = run(ramp_to(6.0), 600)
    w_off, _, _, _ = run(ramp_to(6.0), 600, lam=0.0)
    # Phase 59b: the leak / per-speed GMAX trade some steady delivery (report §69: closed-loop curve delivery 0.99 -> 0.89)
    assert w_on[-1] >= 0.85 * 6.0, w_on[-1]
    t85 = lambda w: int(np.argmax(w >= 0.85 * 6.0)) if (w >= 0.85 * 6.0).any() else 10 ** 6
    assert t85(w_on) + 50 <= t85(w_off), (t85(w_on), t85(w_off))   # gets there >= 0.5 s sooner than the trim / kick
    assert any(f & ccmod.SF_MDPS_COMP for f in fl[200:])
    assert sim.cc.curve_trim == 0.0                         # the trim is replaced, not stacked

  def test_request_within_gmax_of_the_reference(self):
    _, tx, _, _ = run(ramp_to(15.0), 600)
    ref = np.array([ramp_to(15.0)(i) for i in range(600)])
    # the clamp is around the previewed reference cmd + d * cmd' (10 deg/s ramp: + ~3 deg while ramping)
    assert np.max(np.abs(tx - ref)) <= P.MDPS_COMP_GMAX_DEG + 0.4 * 10.0 + 0.5
    assert np.max(np.abs(tx[400:] - ref[400:])) <= P.MDPS_COMP_GMAX_DEG + 0.2   # steady: GMAX itself

  def test_gmax_bounds_delivery_in_a_sharp_steady_curve(self):
    # 15 deg at 60 km/h needs ~13 deg of lead on this plant; GMAX trades the rest for a smaller stored gap (report §66)
    w, tx, _, _ = run(ramp_to(15.0), 900)
    assert 15.0 * 0.5 <= w[-1] < 15.0                     # 59b: GMAX_V ~4 deg at 60 km/h
    assert abs(tx[-1] - 15.0) <= float(np.interp(60.0, P.MDPS_COMP_SPEEDS_KPH, P.MDPS_COMP_GMAX_V)) + 0.2

  def test_hand_stops_the_feedback_and_returns_to_the_command(self):
    # a resting hand (raw 330 in a 10 deg curve at 60 km/h = ~130 driver-domain over op's own ~200 hold torque)
    hand = lambda i: (330.0 if i >= 400 else 0.0)
    w, tx, fl, sim = run(ramp_to(10.0), 700, tq_fn=hand)
    assert not any(f & ccmod.SF_MDPS_COMP for f in fl[460:])      # feedback off while the hand is detected
    assert abs(tx[-1] - 10.0) <= abs(tx[399] - 10.0) + 0.3         # request back at (not beyond) the command

  def test_off_below_min_speed_and_with_kill(self):
    _, _, fl, _ = run(ramp_to(5.0), 300, v=25 / 3.6)
    assert not any(f & ccmod.SF_MDPS_COMP for f in fl)
    _, _, fl, sim = run(ramp_to(5.0), 300, lam=0.0)
    assert not any(f & ccmod.SF_MDPS_COMP for f in fl) and not sim.cc.mdps_comp_active

  def test_straight_command_keeps_the_request_quiet(self):
    w, tx, _, _ = run(lambda i: 0.0, 500, w0=0.0)
    assert np.max(np.abs(tx[100:])) < 0.2
