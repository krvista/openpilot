"""Invariant (a): latch flap resistance under torque noise oscillating around each
threshold, and (d): sign symmetry of every latch/trim/gate."""
import math
import random

import numpy as np

from phase_tests.harness import Sim, count_transitions, run_signal

DT = 0.01


def band_noise(seed, lo=100.0, hi=300.0, freqs=(1.0, 2.0, 3.5, 5.0)):
  """Band-limited 1-5 Hz noise spanning [lo, hi] Nm (column-torque offset regime)."""
  rng = random.Random(seed)
  phases = [rng.uniform(0, 2 * math.pi) for _ in freqs]
  amps = [rng.uniform(0.5, 1.0) for _ in freqs]
  norm = sum(amps)
  mid, half = (lo + hi) / 2.0, (hi - lo) / 2.0

  def f(i):
    t = i * DT
    x = sum(a * math.sin(2 * math.pi * fr * t + p) for a, fr, p in zip(amps, freqs, phases)) / norm
    return mid + half * x
  return f


def rate_per_s(trace):
  return count_transitions(trace) / (len(trace) * DT)


class TestFlapResistance:
  def test_low_speed_cam_latch_no_flap_under_offset_noise(self):
    # 100-300 Nm hands-off band at 4 m/s: pressed (350x5) never fires -> latch never enters
    for seed in range(3):
      sim = Sim()
      tr = run_signal(sim, 3000, tq_fn=band_noise(seed), v=4.0, cmd=5.0)
      assert rate_per_s(tr['low_speed_cam_latched']) < 0.2, seed
      assert not any(tr['low_speed_cam_latched'])

  def test_low_speed_cam_latch_release_band(self):
    # enter via real grip, then oscillate around the 260 Nm release threshold
    sim = Sim()
    run_signal(sim, 30, tq_fn=lambda i: 420.0, v=4.0, cmd=5.0)
    assert sim.s.low_speed_cam_latched
    noise = band_noise(7, lo=180.0, hi=340.0)
    tr = run_signal(sim, 6000, tq_fn=noise, v=4.0, cmd=5.0)
    assert rate_per_s(tr['low_speed_cam_latched']) < 0.2

  def test_scenario_gate_no_flap_around_cmd_threshold(self):
    # |cmd| oscillating across the 35/45 deg hysteresis at 1 Hz
    sim = Sim()
    tr = run_signal(sim, 6000, v=4.0,
                    cmd=lambda i: 40.0 + 10.0 * math.sin(2 * math.pi * 1.0 * i * DT))
    assert rate_per_s(tr['low_speed_scen_ok']) < 0.2

  def test_speed_zone_latch_no_flap(self):
    # vEgo dithering around the 20/22 km/h hysteresis (+-0.2 m/s sensor noise)
    sim = Sim()
    tr = run_signal(sim, 6000, tq_fn=band_noise(9),
                    v=lambda i: 5.8 + 0.2 * math.sin(2 * math.pi * 2.0 * i * DT), cmd=5.0)
    assert rate_per_s(tr['in_low_speed_zone']) < 0.2

  def test_angle_passive_no_flap_under_offset_noise(self):
    # noise band never sustains 260 Nm for 0.3 s -> latch quiet
    for seed in range(3):
      sim = Sim()
      tr = run_signal(sim, 3000, tq_fn=band_noise(seed + 20), v=5.0, cmd=0.0, wheel=10.0)
      assert rate_per_s(tr['angle_passive_active']) < 0.2, seed

  def test_blinker_anchor_no_flap_under_offset_noise(self):
    # Phase 14-2 flap case: blinker on, |tq| noise 100-300 Nm crossing the
    # 220/180 fire/release band at 1-5 Hz.
    rates = []
    for seed in range(5):
      sim = Sim()
      tr = run_signal(sim, 6000, tq_fn=band_noise(seed + 40), v=5.0, cmd=0.0, blinker=True)
      rates.append(rate_per_s(tr['blinker_anchor_on']))
    assert max(rates) < 0.2, rates

  def test_traffic_follow_no_flap_radar_noise(self):
    sim = Sim()
    tr = run_signal(sim, 6000, v=4.0, cmd=5.0,
                    lead_dist=lambda i: 10.0 + 0.6 * math.sin(2 * math.pi * 3.0 * i * DT))
    assert rate_per_s(tr['traffic_following']) < 0.2

  def test_parking_mode_no_flap_speed_noise(self):
    # activate parking, then hover around the 30/33 km/h band with noise
    sim = Sim()
    run_signal(sim, 320, v=5.0, cmd=0.0, wheel=280.0)
    assert sim.s.parking_mode_active
    tr = run_signal(sim, 6000, cmd=0.0, wheel=0.0,
                    v=lambda i: 8.75 + 0.5 * math.sin(2 * math.pi * 1.5 * i * DT))
    assert rate_per_s(tr['parking_mode_active']) < 0.2


class TestSignSymmetry:
  def _run_pair(self, n, **kw):
    simp, simn = Sim(), Sim()
    for i in range(n):
      k = {key: (v(i) if callable(v) else v) for key, v in kw.items()}
      simp.step(**k)
      k2 = dict(k)
      for key in ('tq', 'wheel', 'cmd'):
        if key in k2:
          k2[key] = -k2[key]
      simn.step(**k2)
    return simp, simn

  def _assert_mirror(self, simp, simn):
    assert simp.s.apply_angle_last == -simn.s.apply_angle_last
    assert simp.s.curve_trim == -simn.s.curve_trim
    assert simp.s.cmd_hyst == -simn.s.cmd_hyst
    assert simp.s.trim_resid_lp == -simn.s.trim_resid_lp
    assert simp.s.low_speed_cam_latched == simn.s.low_speed_cam_latched
    assert simp.s.low_speed_scen_ok == simn.s.low_speed_scen_ok
    assert simp.s.angle_passive_active == simn.s.angle_passive_active
    assert simp.s.blinker_anchor_on == simn.s.blinker_anchor_on
    assert simp.s.parking_mode_active == simn.s.parking_mode_active
    assert simp.s.aci_gain_last == simn.s.aci_gain_last

  def test_curve_trim_and_hyst_symmetry(self):
    kw = dict(v=15.0, cmd=lambda i: 8.0 + 0.5 * math.sin(0.05 * i),
              wheel=lambda i: 5.0 + 0.3 * math.sin(0.04 * i), tq=lambda i: 50.0 * math.sin(0.03 * i))
    self._assert_mirror(*self._run_pair(2000, **kw))

  def test_latches_symmetry_low_speed(self):
    kw = dict(v=4.0, cmd=lambda i: 30.0 * math.sin(0.02 * i),
              wheel=lambda i: 50.0 * math.sin(0.015 * i),
              tq=lambda i: 380.0 * math.sin(2 * math.pi * 0.3 * i * DT), blinker=True)
    self._assert_mirror(*self._run_pair(3000, **kw))

  def test_intent_disagree_symmetry(self):
    kw = dict(v=5.0, cmd=20.0, wheel=0.0, tq=-280.0)
    simp, simn = self._run_pair(200, **kw)
    self._assert_mirror(simp, simn)


class TestFunctionalAndKillSwitches:
  def test_blinker_anchor_still_fires_and_releases(self):
    # the 14-2b sustained release must not break the intended behavior:
    # fire within ~3 frames of sustained 240 Nm + blinker, release 0.5 s
    # after a real let-go
    sim = Sim()
    run_signal(sim, 10, v=5.0, cmd=0.0, tq=240.0, blinker=True)
    assert sim.s.blinker_anchor_on
    run_signal(sim, 40, v=5.0, cmd=0.0, tq=240.0, blinker=True)  # past min hold
    tr = run_signal(sim, 60, v=5.0, cmd=0.0, tq=20.0, blinker=True)
    assert not sim.s.blinker_anchor_on
    assert sum(1 for x in tr['blinker_anchor_on'] if x) >= 49  # held ~0.5 s

  def test_kill_switch_blinker_anchor_off(self, monkeypatch):
    from opendbc.car.hyundai.values import CarControllerParams
    monkeypatch.setattr(CarControllerParams, 'BLINKER_ANCHOR_TORQUE_NM', 1e9)
    sim = Sim()
    tr = run_signal(sim, 500, v=5.0, cmd=0.0, tq=340.0, blinker=True)
    assert not any(tr['blinker_anchor_on'])

  def test_kill_switch_curve_trim_off(self, monkeypatch):
    from opendbc.car.hyundai.values import CarControllerParams
    monkeypatch.setattr(CarControllerParams, 'CURVE_TRIM_RATE_DPS', 0.0)
    sim = Sim()
    tr = run_signal(sim, 2000, v=15.0, cmd=10.0, wheel=5.0)
    assert all(t == 0.0 for t in tr['trim'])

  def test_kill_switch_scenario_gate_open(self, monkeypatch):
    from opendbc.car.hyundai.values import CarControllerParams
    monkeypatch.setattr(CarControllerParams, 'LOW_SPEED_CMD_PASSIVE_DEG', 1e9)
    monkeypatch.setattr(CarControllerParams, 'LOW_SPEED_CMD_ACTIVE_DEG', 1e9)
    sim = Sim()
    tr = run_signal(sim, 500, v=4.0, cmd=120.0, wheel=0.0)
    assert all(tr['low_speed_scen_ok'])
