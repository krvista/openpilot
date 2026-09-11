"""Phase 40: stall kick. Full CarController via the harness: a stuck wheel (measured angle fixed) with the
request quiet and >2.5 deg away, hands off, gain >= 0.9, 40 km/h -> fast request steps away from the wheel
with a slow decay back (the CCNC MDPS follows request rate, see values.py STALL_KICK_*)."""
import math

import numpy as np

from phase_tests.harness import Sim, run_signal
from opendbc.car.hyundai.values import CarControllerParams as P

V40 = 40.0 / 3.6
UP = P.STALL_KICK_AMPLITUDE_DEG / P.STALL_KICK_UP_FRAMES
DOWN = P.STALL_KICK_AMPLITUDE_DEG / P.STALL_KICK_DOWN_FRAMES


def settle(sim):
  run_signal(sim, 120, v=15.0, wheel=0.0, cmd=0.0, tq=0.0)
  run_signal(sim, 200, v=V40, wheel=0.0, cmd=0.0, tq=0.0)


def stall(sim, n=800, cmd=5.0, wheel=1.0, v=V40, tq=0.0):
  return run_signal(sim, n, v=v, wheel=wheel, cmd=cmd, tq=tq)


def steps(kick):
  k = np.abs(np.array(kick))
  d = np.diff(k)
  up = d > 0.9 * UP
  return int(np.sum(up[1:] & ~up[:-1]) + (1 if up[0] else 0))


class TestPhase40StallKick:
  def test_steps_on_a_stuck_wheel(self):
    sim = Sim()
    settle(sim)
    tr = stall(sim)
    k = np.array(tr['kick'])
    g = np.array(tr['gain'])
    assert g[-1] >= P.STALL_KICK_MIN_GAIN, g[-1]
    assert steps(k) == P.STALL_KICK_MAX_PULSES, steps(k)
    assert k.min() >= 0.0                                              # away from the wheel = positive (gap +4)
    assert k.max() <= P.STALL_KICK_RETRIGGER_DEG + P.STALL_KICK_AMPLITUDE_DEG + 1e-6
    assert k.max() >= P.STALL_KICK_AMPLITUDE_DEG - 1e-6
    assert k[-1] == 0.0                                                # budget spent, offset decayed: nothing stored
    d = np.diff(k)
    rising = d[d > 1e-9]
    falling = d[d < -1e-9]
    assert np.allclose(rising, UP, atol=1e-9)                          # 15 deg/s step
    assert np.allclose(falling, -DOWN, atol=1e-9) or np.all(falling >= -UP)   # 1 deg/s decay (last frame may clip to 0)
    first = int(np.argmax(k > 0))
    nxt = first + P.STALL_KICK_UP_FRAMES + int(np.argmax(d[first + P.STALL_KICK_UP_FRAMES:] > 0.9 * UP))
    assert nxt - first >= P.STALL_KICK_MIN_GAP_FRAMES

  def test_sign_follows_the_gap(self):
    sim = Sim()
    settle(sim)
    tr = stall(sim, n=300, cmd=-5.0, wheel=-1.0)
    k = np.array(tr['kick'])
    assert steps(k) >= 1 and k.min() <= -P.STALL_KICK_AMPLITUDE_DEG + 1e-6 and k.max() <= 0.0

  def test_kill_switch(self, monkeypatch):
    monkeypatch.setattr(P, 'STALL_KICK_AMPLITUDE_DEG', 0.0)
    sim = Sim()
    settle(sim)
    tr = stall(sim, n=300)
    assert not any(tr['kick'])

  def test_no_kick_when_gap_small_or_wheel_moving_or_speed_out(self):
    for kw in ({'cmd': 3.0, 'wheel': 1.0},                                          # gap 2.0 < 2.5
               {'cmd': 5.0, 'wheel': lambda i: 1.0 + 0.5 * math.sin(i / 3.0)},       # wheel not still
               {'cmd': 5.0, 'wheel': 1.0, 'v': 18.0 / 3.6},                          # below 25 km/h
               {'cmd': 5.0, 'wheel': 1.0, 'v': 65.0 / 3.6}):                         # above 60 km/h
      sim = Sim()
      settle(sim)
      tr = stall(sim, n=300, **kw)
      assert not any(tr['kick']), kw

  def test_hand_aborts_fast(self):
    sim = Sim()
    settle(sim)
    t1 = stall(sim, n=60)
    assert any(t1['kick']) and sim.s.kick_off > 0.0
    tr2 = stall(sim, n=120, tq=460.0)                                    # pressed grip
    k2 = np.array(tr2['kick'])
    assert np.all(k2[P.STALL_KICK_ABORT_DOWN_FRAMES + 1:] == 0.0), k2[:30]
    assert np.all(np.diff(k2[:P.STALL_KICK_ABORT_DOWN_FRAMES]) <= 1e-9)     # only decays, no new step

  def test_budget_returns_after_the_stall_clears(self):
    sim = Sim()
    settle(sim)
    tr = stall(sim, n=800)
    assert steps(tr['kick']) == P.STALL_KICK_MAX_PULSES
    tr2 = stall(sim, n=300)                                              # still stuck: budget spent
    assert not any(tr2['kick'])
    run_signal(sim, 150, v=V40, wheel=1.0, cmd=1.0, tq=0.0)              # plan back at the wheel: episode over
    tr3 = stall(sim, n=800)
    assert steps(tr3['kick']) == P.STALL_KICK_MAX_PULSES

  def test_unstuck_wheel_stops_stepping(self):
    sim = Sim()
    settle(sim)
    t1 = stall(sim, n=60)
    assert any(t1['kick'])
    # the wheel starts moving toward the plan: no further steps, the offset just decays
    tr = run_signal(sim, 400, v=V40, wheel=lambda i: min(1.0 + 0.02 * i, 5.0), cmd=5.0, tq=0.0)
    k = np.array(tr['kick'])
    assert steps(k) == 0 and k[-1] == 0.0

  def test_state_clears_when_lat_inactive(self):
    sim = Sim()
    settle(sim)
    stall(sim, n=60)
    run_signal(sim, 50, v=V40, wheel=1.0, cmd=1.0, tq=0.0, lat_active=False)
    assert sim.s.kick_off == 0.0 and sim.s.kick_count == 0 and sim.s.stall_kick_deg == 0.0
