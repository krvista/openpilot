"""Phase 58 (report §62-63): hand-on gap cap. While a hand rests on the wheel (hold-compensated driver torque >= HAND_GAP_ON_NM)
the request stays within HAND_GAP_CAP_DEG of the measured wheel; the cap starts at the current gap and tightens at
HAND_GAP_DOWN_DPS, widens at HAND_GAP_UP_DPS once the hand is gone and switches off at HAND_GAP_RELEASE_DEG."""
import numpy as np
import pytest

from phase_tests.harness import Sim
from opendbc.car.hyundai.values import CarControllerParams as P
import opendbc.car.hyundai.carcontroller as ccmod

V = 15.0           # m/s, 54 km/h
WHEEL = 2.0        # the wheel lags behind
CMD = 14.0         # op's request (12 deg ahead)


def settle(sim, n=400):
  for _ in range(n):
    sim.step(v=V, tq=0.0, wheel=WHEEL, cmd=CMD)


def hand_tq(sim):
  # raw torque that gives ~100 driver-domain Nm on top of op's own hold estimate
  return sim.cc.hold_comp_last + 100.0


def run(hand_frames, after=500, cap=None, v=V):
  old = P.HAND_GAP_CAP_DEG
  try:
    if cap is not None:
      P.HAND_GAP_CAP_DEG = cap
    sim = Sim(); settle(sim)
    tx, fl = [], []
    for i in range(hand_frames + after):
      tq = hand_tq(sim) if i < hand_frames else 0.0
      sim.step(v=v, tq=tq, wheel=WHEEL, cmd=CMD)
      tx.append(sim.tx_angle()); fl.append(sim.cc.steer_debug_flags)
    return np.array(tx), fl, sim
  finally:
    P.HAND_GAP_CAP_DEG = old


def ref_tx():
  # the request with Phase 58 off (includes the curve trim / stall kick that ride on the stuck wheel)
  tx, _, _ = run(0, after=200, cap=0.0)
  return tx[-1]


class TestHandGapCap:
  def test_hands_off_request_is_untouched(self):
    tx, fl, sim = run(0, after=200)
    tk, _, _ = run(0, after=200, cap=0.0)
    np.testing.assert_allclose(tx, tk, atol=1e-9)
    assert not any(f & ccmod.SF_HAND_GAP for f in fl)
    assert sim.cc.hand_gap_cap is None

  def test_resting_hand_caps_the_gap_without_a_step(self):
    tx, fl, _ = run(300, after=0)
    assert tx[0] == pytest.approx(ref_tx(), abs=0.3)                         # starts from the current gap
    assert np.max(np.abs(np.diff(tx))) <= P.HAND_GAP_DOWN_DPS * 0.01 + 0.05  # tightens as a ramp
    assert tx[-1] == pytest.approx(WHEEL + P.HAND_GAP_CAP_DEG, abs=0.2)
    assert fl[-1] & ccmod.SF_HAND_GAP

  def test_release_widens_as_a_ramp_then_switches_off(self):
    tx, fl, sim = run(300, after=600)
    rel = tx[300:]
    assert np.max(np.diff(rel)) <= P.HAND_GAP_UP_DPS * 0.01 + 0.05          # no step back to the plan
    assert rel[49] == pytest.approx(WHEEL + P.HAND_GAP_CAP_DEG, abs=0.2)    # held through the 0.5 s off debounce
    assert rel[-1] == pytest.approx(ref_tx(), abs=0.3)
    assert sim.cc.hand_gap_cap is None

  def test_kill_switch(self):
    tx, fl, sim = run(300, after=0, cap=0.0)
    assert not any(f & ccmod.SF_HAND_GAP for f in fl)
    assert sim.cc.hand_gap_cap is None

  def test_inactive_below_min_speed(self):
    tx, fl, sim = run(300, after=0, v=10.0 / 3.6)
    assert not any(f & ccmod.SF_HAND_GAP for f in fl)
