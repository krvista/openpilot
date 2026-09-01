"""Invariant (b): latActive dropping mid-latch resets counters; no stale state on
re-engage. Invariant (c): NaN/inf robustness of every stateful path."""
import math

import numpy as np

from phase_tests.harness import Sim, run_signal

DT = 0.01
FINITE_STATE = ('apply_angle_last', 'curve_trim', 'trim_resid_lp', 'cmd_hyst',
                'aci_gain_last', 'creep_sum', 'blinker_anchor_hold')


class TestResetOnDisengage:
  def _build_state(self, sim):
    # build curve trim + hysteresis + angle_passive on a sustained curve
    run_signal(sim, 400, v=15.0, cmd=10.0, wheel=6.0, tq=0.0)

  def test_angle_passive_counters_reset(self):
    sim = Sim()
    run_signal(sim, 100, v=5.0, cmd=0.0, wheel=45.0, tq=400.0)
    assert sim.s.angle_passive_active
    sim.step(v=5.0, cmd=0.0, wheel=45.0, tq=400.0, lat_active=False)
    assert not sim.s.angle_passive_active
    assert sim.s.angle_passive_enter_frames == 0
    assert sim.s.intent_disagree_frames == 0
    assert sim.s.angle_passive_release_frames == 0

  def test_no_stale_trim_after_disengage(self):
    sim = Sim()
    self._build_state(sim)
    assert abs(sim.s.curve_trim) > 0.5
    # 2 s disengaged: trim must bleed to ~0 (tau 0.5 s)
    run_signal(sim, 200, v=15.0, cmd=10.0, wheel=6.0, lat_active=False)
    assert abs(sim.s.curve_trim) < 0.1
    # re-engage: sustain counter must re-arm from zero (no instant integration)
    assert sim.s.curve_trim_sustain == 0
    sim.step(v=15.0, cmd=10.0, wheel=6.0)
    assert abs(sim.s.curve_trim) < 0.1

  def test_cmd_hyst_recenters_while_inactive(self):
    sim = Sim()
    self._build_state(sim)
    sim.step(v=15.0, cmd=-20.0, wheel=6.0, lat_active=False)
    # while inactive the hysteresis state must track the raw desired
    assert sim.s.cmd_hyst == -20.0

  def test_trim_resid_lp_resets_when_gate_drops(self):
    sim = Sim()
    self._build_state(sim)
    run_signal(sim, 5, v=15.0, cmd=0.0, wheel=0.0)  # straight -> gate drops
    assert sim.s.trim_resid_lp == 0.0

  def test_apply_angle_snaps_to_wheel_when_inactive(self):
    sim = Sim()
    self._build_state(sim)
    sim.step(v=15.0, cmd=10.0, wheel=-30.0, lat_active=False)
    assert sim.s.apply_angle_last == -30.0

  def test_creep_window_reset_by_lead_and_speed(self):
    sim = Sim()
    run_signal(sim, 900, v=5.0, cmd=0.0)
    assert sim.s.creep_frames == 900
    sim.step(v=5.0, cmd=0.0, lead_dist=10.0)      # lead near -> reset
    assert sim.s.creep_frames == 0
    assert sim.s.creep_min == float('inf')
    assert sim.s.creep_sum == 0.0
    run_signal(sim, 900, v=5.0, cmd=0.0)
    sim.step(v=7.2, cmd=0.0)                       # >= 25 km/h -> reset
    assert sim.s.creep_frames == 0


class TestNaNRobustness:
  def _assert_finite(self, sim):
    for name in FINITE_STATE:
      v = getattr(sim.s, name)
      assert np.isfinite(v), (name, v)
    m = sim.lkas_alt()
    for k in ("ADAS_StrAnglReqVal", "ADAS_ACIAnglTqRedcGainVal"):
      assert np.isfinite(m[k]), (k, m[k])

  def _burst(self, sim, bad, n=50, **base):
    for key in ('tq', 'wheel', 'cmd', 'v_raw'):
      kw = dict(base)
      kw[key] = bad
      for _ in range(n):
        sim.step(**kw)
        self._assert_finite(sim)
      # recovery frames
      for _ in range(50):
        sim.step(**base)
        self._assert_finite(sim)

  def test_nan_burst_no_propagation(self):
    sim = Sim()
    run_signal(sim, 300, v=15.0, cmd=8.0, wheel=5.0)
    self._burst(sim, float('nan'), v=15.0, cmd=8.0, wheel=5.0, tq=50.0)

  def test_inf_burst_no_propagation(self):
    for bad in (float('inf'), float('-inf')):
      sim = Sim()
      run_signal(sim, 300, v=15.0, cmd=8.0, wheel=5.0)
      self._burst(sim, bad, v=15.0, cmd=8.0, wheel=5.0, tq=50.0)

  def test_nan_vego_low_speed_latches_fail_safe(self):
    # a NaN speed burst while latched low must not freeze the latch machinery
    # into a nonsensical state or emit non-finite TX
    sim = Sim()
    run_signal(sim, 200, v=4.0, cmd=5.0)
    for _ in range(100):
      sim.step(v=4.0, v_raw=float('nan'), cmd=5.0)
      self._assert_finite(sim)
    for _ in range(100):
      sim.step(v=4.0, cmd=5.0)
      self._assert_finite(sim)

  def test_all_nan_simultaneously(self):
    sim = Sim()
    run_signal(sim, 100, v=15.0, cmd=8.0, wheel=5.0)
    nan = float('nan')
    for _ in range(100):
      sim.step(v=15.0, v_raw=nan, cmd=nan, wheel=nan, tq=nan)
      self._assert_finite(sim)
