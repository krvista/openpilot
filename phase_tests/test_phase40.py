"""Phase 40b: stall kick as a ramp. Full CarController via the harness: a stuck wheel (measured angle fixed) with the
request quiet and >2.5 deg away, hands off, gain >= 0.9, 40 km/h -> the request ramps away from the wheel at 10 deg/s
until the wheel moves or 3 deg is reached, holds 0.2 s, decays at 0.5 deg/s (values.py STALL_KICK_*)."""
import math

import numpy as np

from phase_tests.harness import Sim, run_signal
from opendbc.car.hyundai.values import CarControllerParams as P

import pytest

V40 = 40.0 / 3.6
DT = 0.01
AMP_ON = 3.0   # the 40b on-road value; the shipped default is 0.0 (off) after routes 15/16
RAMP = P.STALL_KICK_RAMP_DPS * DT
DECAY = P.STALL_KICK_DECAY_DPS * DT


def settle(sim):
  run_signal(sim, 120, v=15.0, wheel=0.0, cmd=0.0, tq=0.0)
  run_signal(sim, 200, v=V40, wheel=0.0, cmd=0.0, tq=0.0)


def stall(sim, n=2000, cmd=5.0, wheel=1.0, v=V40, tq=0.0):
  return run_signal(sim, n, v=v, wheel=wheel, cmd=cmd, tq=tq)


def ramps(kick):
  k = np.abs(np.array(kick))
  d = np.diff(k)
  up = d > 0.9 * RAMP
  return int(np.sum(up[1:] & ~up[:-1]) + (1 if up[0] else 0))


@pytest.fixture(autouse=True)
def _kick_on(monkeypatch):
  monkeypatch.setattr(P, 'STALL_KICK_AMPLITUDE_DEG', AMP_ON)


class TestPhase40bStallKick:
  def test_shipped_default_is_off(self, monkeypatch):
    monkeypatch.undo()
    assert P.STALL_KICK_AMPLITUDE_DEG == 0.0
    sim = Sim()
    settle(sim)
    assert not any(stall(sim, n=400)['kick'])

  def test_ramp_hold_decay_on_a_stuck_wheel(self):
    sim = Sim()
    settle(sim)
    tr = stall(sim)
    k = np.array(tr['kick'])
    assert tr['gain'][-1] >= P.STALL_KICK_MIN_GAIN
    assert ramps(k) == P.STALL_KICK_MAX_PULSES, ramps(k)
    assert k.min() >= 0.0                                          # away from the wheel = positive (gap +4)
    assert k.max() <= P.STALL_KICK_ENVELOPE_DEG + 1e-6
    assert k.max() >= P.STALL_KICK_AMPLITUDE_DEG - 1e-6              # wheel never moves: full amplitude
    d = np.diff(k)
    rising = d[d > 1e-9]
    falling = d[d < -1e-9]
    assert np.allclose(rising, RAMP, atol=1e-9)                     # 10 deg/s
    assert np.all(falling >= -DECAY - 1e-9)                        # 0.5 deg/s decay (never faster)
    # first ramp: 30 frames up, 20 hold, then decay
    first = int(np.argmax(k > 0))
    seg = k[first:first + 30 + P.STALL_KICK_HOLD_FRAMES + 5]
    assert np.isclose(seg[29], P.STALL_KICK_AMPLITUDE_DEG, atol=1e-9)
    assert np.allclose(seg[30:30 + P.STALL_KICK_HOLD_FRAMES - 1], P.STALL_KICK_AMPLITUDE_DEG, atol=1e-9)
    assert seg[30 + P.STALL_KICK_HOLD_FRAMES + 2] < P.STALL_KICK_AMPLITUDE_DEG
    # after the budget: everything decays to zero, nothing stored
    tr2 = stall(sim, n=1000)
    assert not ramps(tr2['kick']) and tr2['kick'][-1] == 0.0

  def test_ramp_stops_as_soon_as_the_wheel_moves(self):
    sim = Sim()
    settle(sim)
    # the wheel starts to follow once the ramp has reached 1.4 deg: the ramp stops there, well under 3 deg
    tr = run_signal(sim, 400, v=V40, wheel=lambda i: 1.0 + (0.6 if sim.s.kick_off >= 1.4 else 0.0), cmd=5.0, tq=0.0)
    k = np.array(tr['kick'])
    assert 1.3 < k.max() < P.STALL_KICK_AMPLITUDE_DEG - 0.5, k.max()

  def test_sign_follows_the_gap(self):
    sim = Sim()
    settle(sim)
    tr = stall(sim, n=400, cmd=-5.0, wheel=-1.0)
    k = np.array(tr['kick'])
    assert ramps(k) >= 1 and k.min() <= -P.STALL_KICK_AMPLITUDE_DEG + 1e-6 and k.max() <= 0.0

  def test_kill_switch(self, monkeypatch):
    monkeypatch.setattr(P, 'STALL_KICK_AMPLITUDE_DEG', 0.0)
    sim = Sim()
    settle(sim)
    assert not any(stall(sim, n=400)['kick'])

  def test_no_kick_when_gap_small_or_wheel_moving_or_speed_out(self):
    for kw in ({'cmd': 3.0, 'wheel': 1.0},                                          # gap 2.0 < 2.5
               {'cmd': 5.0, 'wheel': lambda i: 1.0 + 0.5 * math.sin(i / 3.0)},       # wheel not still
               {'cmd': 5.0, 'wheel': 1.0, 'v': 18.0 / 3.6},                          # below 25 km/h
               {'cmd': 5.0, 'wheel': 1.0, 'v': 65.0 / 3.6}):                         # above 60 km/h
      sim = Sim()
      settle(sim)
      assert not any(stall(sim, n=400, **kw)['kick']), kw

  def test_hand_aborts_fast(self):
    sim = Sim()
    settle(sim)
    t1 = stall(sim, n=60)
    assert any(t1['kick']) and sim.s.kick_off > 0.0
    tr2 = stall(sim, n=120, tq=460.0)                              # pressed grip
    k2 = np.array(tr2['kick'])
    assert np.all(np.diff(k2[:30]) <= 1e-9)                        # only decays, no new ramp
    assert np.all(k2[25:] == 0.0), k2[:30]                          # gone within ~0.2 s

  def test_budget_returns_after_the_stall_clears(self):
    sim = Sim()
    settle(sim)
    tr = stall(sim, n=2000)
    assert ramps(tr['kick']) == P.STALL_KICK_MAX_PULSES
    run_signal(sim, 900, v=V40, wheel=1.0, cmd=1.0, tq=0.0)        # plan back at the wheel: episode over, offset decays
    assert sim.s.kick_off == 0.0
    tr3 = stall(sim, n=2000)
    assert ramps(tr3['kick']) == P.STALL_KICK_MAX_PULSES

  def test_state_clears_when_lat_inactive(self):
    sim = Sim()
    settle(sim)
    stall(sim, n=60)
    run_signal(sim, 50, v=V40, wheel=1.0, cmd=1.0, tq=0.0, lat_active=False)
    assert sim.s.kick_off == 0.0 and sim.s.kick_count == 0 and sim.s.stall_kick_deg == 0.0
