"""Phase 63 (report §74-75).
63a: curve-trim arm / hold gates and residual deadband scheduled on speed (4° / 3° / 0.7° up to 55 km/h -> 1.5° / 1° /
0.35° from 70 km/h) so the trim also works on highway curves that need only a few degrees of wheel.
63b: corner-entry boost — while the clean command grows away from centre faster than its slow reference, add
ENTRY_BOOST_GAIN x the excess over ENTRY_BOOST_DEADBAND_DEG (capped per speed); bleed it at ENTRY_BOOST_BLEED_DPS once
the command settles; never on unwinding; hands off / no blinker / >= 45 km/h only."""
import numpy as np
import pytest

from phase_tests.harness import Sim
from opendbc.car.hyundai.values import CarControllerParams as P
import opendbc.car.hyundai.carcontroller as ccmod

KILL_63A = dict(CURVE_TRIM_GATE_V=[4.0, 4.0], CURVE_TRIM_HOLD_V=[3.0, 3.0], CURVE_TRIM_DEADBAND_V=[0.7, 0.7])


class params:
  def __init__(self, **kw):
    self.kw = kw

  def __enter__(self):
    self.old = {k: getattr(P, k) for k in self.kw}
    for k, v in self.kw.items():
      setattr(P, k, v)

  def __exit__(self, *a):
    for k, v in self.old.items():
      setattr(P, k, v)


def run(cmds, v, wheel_fn=None, tq=0.0, blinker=False, **kw):
  """cmds: per-frame commands; wheel_fn(i, tx_prev) -> wheel. Returns TX, entry offsets, flags."""
  with params(**kw):
    sim = Sim(); tx_prev = 0.0
    w0 = wheel_fn(0, 0.0) if wheel_fn else 0.0
    for _ in range(300):
      sim.step(v=v, tq=0.0, wheel=w0, cmd=float(cmds[0]))
    tx, eb, fl = [], [], []
    for i, c in enumerate(cmds):
      w = wheel_fn(i, tx_prev) if wheel_fn else 0.0
      sim.step(v=v, tq=tq, wheel=w, cmd=float(c), blinker=blinker)
      tx_prev = sim.tx_angle()
      tx.append(tx_prev); eb.append(sim.cc.eb_off); fl.append(sim.cc.steer_debug_flags)
    return np.array(tx), np.array(eb), fl


def steady(cmd_deg, frames=600):
  return np.full(frames, cmd_deg)


def ramp(to_deg, rate_dps, hold_frames=800):
  n = int(abs(to_deg) / rate_dps * 100)
  return np.concatenate([np.linspace(0.0, to_deg, n), np.full(hold_frames, to_deg)])


class TestCurveTrimGate:
  # a stuck wheel at 60 % of a small highway curve command: residual 1.0° > the 0.35° deadband
  def stuck(self, frac):
    return lambda i, tx: frac * 2.5

  def test_highway_small_curve_arms_the_trim(self):
    tx, _, _ = run(steady(2.5), 80 / 3.6, self.stuck(0.6), ENTRY_BOOST_GAIN=0.0)
    assert tx[-1] > 2.5 + 0.5          # trim added on top of the 2.5° command

  def test_kill_switch_restores_the_4_deg_gate(self):
    tx, _, _ = run(steady(2.5), 80 / 3.6, self.stuck(0.6), ENTRY_BOOST_GAIN=0.0, **KILL_63A)
    assert tx[-1] == pytest.approx(2.5, abs=0.25)

  def test_city_speed_keeps_the_4_deg_gate(self):
    tx, _, _ = run(steady(2.5), 40 / 3.6, self.stuck(0.6), ENTRY_BOOST_GAIN=0.0)
    assert tx[-1] == pytest.approx(2.5, abs=0.25)
    tx4, _, _ = run(steady(5.0), 40 / 3.6, lambda i, t: 3.0, ENTRY_BOOST_GAIN=0.0)
    assert tx4[-1] > 5.0 + 0.5         # a 4°+ curve still arms in town as before

  def test_deadband_scales_with_speed(self):
    # residual ~0.5°: inside the city deadband (0.7°), outside the highway one (0.35°)
    tx_hw, _, _ = run(steady(2.5), 80 / 3.6, lambda i, t: 1.9, ENTRY_BOOST_GAIN=0.0)
    assert tx_hw[-1] > 2.5 + 0.05
    tx_c, _, _ = run(steady(5.0), 40 / 3.6, lambda i, t: 4.5, ENTRY_BOOST_GAIN=0.0)
    assert tx_c[-1] == pytest.approx(5.0, abs=0.25)


class TestEntryBoost:
  V = 80 / 3.6

  def follow(self, k=0.6):
    return lambda i, tx: k * tx

  def test_entry_ramp_is_boosted_within_the_cap(self):
    cmds = ramp(6.0, 10.0)
    tx, eb, fl = run(cmds, self.V, self.follow(), **KILL_63A)
    tx0, _, _ = run(cmds, self.V, self.follow(), ENTRY_BOOST_GAIN=0.0, **KILL_63A)
    cap = float(np.interp(80.0, P.ENTRY_BOOST_SPEEDS_KPH, P.ENTRY_BOOST_CAP_DEG))
    assert eb.max() > 0.5
    assert eb.max() <= cap + 1e-6
    assert any(f & ccmod.SF_ENTRY_BOOST for f in fl)
    assert np.max(tx - tx0) > 0.5                     # the request leads the un-boosted one during the entry
    assert np.max(tx - tx0) <= cap + 0.3

  def test_settled_curve_bleeds_under_the_rate_floor(self):
    tx, eb, _ = run(ramp(6.0, 10.0, hold_frames=1500), self.V, self.follow(), **KILL_63A)
    i = int(np.argmax(eb)) + 150
    d = np.diff(eb[i:])
    assert np.all(d <= 1e-9)
    assert -d.min() <= P.ENTRY_BOOST_BLEED_DPS * 0.01 + 1e-6
    assert eb[-1] < eb.max()

  def test_unwinding_is_never_boosted(self):
    cmds = np.concatenate([np.full(1500, 6.0), np.linspace(6.0, 0.0, 60), np.zeros(300)])
    _, eb, _ = run(cmds, self.V, self.follow(), **KILL_63A)
    # after the 1.5 s settle the offset can only decay; unwinding adds nothing
    assert np.all(np.diff(eb[1400:]) <= 1e-9)

  def test_straight_line_noise_stays_in_the_deadband(self):
    t = np.arange(1500) * 0.01
    _, eb, _ = run(0.6 * np.sin(2 * np.pi * 0.8 * t), self.V, self.follow(), **KILL_63A)
    assert np.abs(eb).max() == 0.0

  def test_hand_on_blinker_low_speed_and_kill_switch(self):
    cmds = ramp(6.0, 10.0)
    sim = Sim()
    hand = sim.cc.hold_comp_last + 400.0   # raw; well above the hold-compensated 60 Nm at 80 km/h
    for kw, extra in ((dict(tq=hand), {}), (dict(blinker=True), {}), ({}, dict(ENTRY_BOOST_GAIN=0.0))):
      _, eb, fl = run(cmds, self.V, self.follow(), **kw, **extra, **KILL_63A)
      assert np.abs(eb).max() == 0.0
      assert not any(f & ccmod.SF_ENTRY_BOOST for f in fl)
    _, eb, _ = run(cmds, 40 / 3.6, self.follow(), **KILL_63A)
    assert np.abs(eb).max() == 0.0

  def test_s_curve_flip_collapses_the_offset(self):
    cmds = np.concatenate([ramp(6.0, 10.0, hold_frames=50), np.linspace(6.0, -6.0, 120), np.full(300, -6.0)])
    _, eb, _ = run(cmds, self.V, self.follow(), **KILL_63A)
    j = 60 + 50 + 60                     # command crosses zero
    assert eb[:j].min() >= -1e-9         # nothing toward the new side before the flip
    assert eb[j + 60] <= 0.05            # the old offset is gone within 0.6 s (tau 0.15 s)
    assert eb[j:].max() <= eb[j] + 1e-9  # and never grows again toward the old side
