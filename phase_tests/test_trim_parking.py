"""Invariant (e): steady-curve trim; (f): parking-mode latch; (g): interactions."""
import math

import numpy as np

from phase_tests.harness import Sim, run_signal
from opendbc.car.hyundai.values import CarControllerParams

DT = 0.01


def trim_cap(v):
  return float(np.interp(v, CarControllerParams.CURVE_TRIM_CAP_SPEEDS_MS,
                         CarControllerParams.CURVE_TRIM_CAP_DEG))


class TestTrimInvariants:
  def _build_trim(self, sim, v=15.0, cmd=10.0, wheel=5.0, n=3000):
    tr = run_signal(sim, n, v=v, cmd=cmd, wheel=wheel)
    return tr

  def test_trim_never_exceeds_speed_cap(self):
    for v in (5.0, 10.0, 20.0, 27.8):
      sim = Sim()
      tr = self._build_trim(sim, v=v, cmd=12.0, wheel=2.0)  # large persistent deficit
      # 1e-6 tolerance: capnp stores vEgoRaw as float32, so the in-code
      # np.interp cap evaluates at v +- half a float32 ULP
      assert max(abs(t) for t in tr['trim']) <= trim_cap(v) + 1e-6, v

  def test_trim_bleeds_within_1s_of_gate_drop(self):
    sim = Sim()
    self._build_trim(sim)
    t0 = abs(sim.s.curve_trim)
    assert t0 > 1.0
    run_signal(sim, 100, v=15.0, cmd=0.0, wheel=0.0)  # straight: gate drops
    assert abs(sim.s.curve_trim) < 0.15 * t0 + 0.05

  def test_s_curve_flip_fast_decay(self):
    sim = Sim()
    self._build_trim(sim)
    t0 = abs(sim.s.curve_trim)
    # flip the curve: opposing trim must decay with tau 0.15 (~87% gone in 0.3 s)
    run_signal(sim, 30, v=15.0, cmd=-10.0, wheel=-5.0)
    assert abs(sim.s.curve_trim) < 0.25 * t0

  def test_trim_never_grows_while_pressed(self):
    sim = Sim()
    self._build_trim(sim)
    t0 = abs(sim.s.curve_trim)
    tr = run_signal(sim, 200, v=15.0, cmd=10.0, wheel=5.0, tq=420.0)  # pressed
    assert all(abs(t) <= t0 + 1e-9 for t in tr['trim'])
    assert abs(sim.s.curve_trim) < t0

  def test_trim_bleeds_when_passthrough_flips_mid_curve(self):
    # (g) build trim in low-speed traffic-follow, then lose the scenario.
    # Fast boot first so the S1b cold-start-at-low-speed parking signature
    # does not arm and hold the trim gate closed.
    sim = Sim()
    run_signal(sim, 60, v=20.0, cmd=0.0)
    run_signal(sim, 3000, v=5.0, cmd=10.0, wheel=5.0, lead_dist=6.0)
    t0 = abs(sim.s.curve_trim)
    assert t0 > 0.5
    tr = run_signal(sim, 200, v=5.0, cmd=50.0, wheel=5.0)  # lead gone, sharp cmd
    assert not sim.s.low_speed_scen_ok
    assert abs(sim.s.curve_trim) < 0.2 * t0
    assert not sim.effective_lat_active()


class TestParkingMode:
  def test_enter_and_exit_only_sustained_above_33(self):
    sim = Sim()
    run_signal(sim, 320, v=5.0, cmd=0.0, wheel=280.0)
    assert sim.s.parking_mode_active
    # 32.4 km/h forever: never exits
    tr = run_signal(sim, 3000, v=9.0, cmd=0.0, wheel=0.0)
    assert all(tr['parking_mode_active'])
    # 34.2 km/h for 1.9 s then a dip: not exited
    run_signal(sim, 190, v=9.5, cmd=0.0, wheel=0.0)
    sim.step(v=9.0, cmd=0.0)
    assert sim.s.parking_mode_active
    # sustained 2 s above 33: exits and clears the signature fully
    run_signal(sim, 210, v=9.5, cmd=0.0, wheel=0.0)
    assert not sim.s.parking_mode_active
    assert not sim.s.parking_signature_seen
    assert sim.s.parking_low_speed_frames == 0
    # a later plain low-speed stretch must NOT re-trip on stale state
    run_signal(sim, 400, v=7.5, cmd=0.0, wheel=0.0)  # 27 km/h, no signature
    assert not sim.s.parking_mode_active

  def test_s1b_fires_exactly_once_per_boot(self):
    sim = Sim()
    # cold start at low speed: S1b decides at frame 50
    run_signal(sim, 49, v=5.0, cmd=0.0)
    assert not sim.s.parking_signature_seen or sim.s.boot_parking_pending
    run_signal(sim, 2, v=5.0, cmd=0.0)
    assert sim.s.parking_signature_seen
    assert not sim.s.boot_parking_pending
    # ride the mode out via >33 km/h
    run_signal(sim, 260, v=5.0, cmd=0.0)
    assert sim.s.parking_mode_active
    run_signal(sim, 210, v=9.5, cmd=0.0)
    assert not sim.s.parking_mode_active
    # S1b must not re-fire on the next low-speed stretch
    run_signal(sim, 400, v=7.5, cmd=0.0)
    assert not sim.s.parking_signature_seen
    assert not sim.s.parking_mode_active

  def test_s1b_does_not_fire_on_fast_boot(self):
    sim = Sim()
    run_signal(sim, 60, v=20.0, cmd=0.0)
    assert not sim.s.parking_signature_seen

  def test_creep_signature_arms_parking(self):
    sim = Sim()
    # skip the boot window at speed so S1b does not fire
    run_signal(sim, 60, v=20.0, cmd=0.0)
    # 10 s lot crawl with a dip below 8 km/h (2.22 m/s), no lead
    run_signal(sim, 1100, v=lambda i: 3.5 + 1.7 * math.sin(2 * math.pi * i * DT / 10.0), cmd=0.0)
    assert sim.s.parking_signature_seen
    assert sim.s.parking_mode_active

  def test_steady_school_zone_does_not_arm(self):
    sim = Sim()
    run_signal(sim, 60, v=20.0, cmd=0.0)
    # steady 23 km/h (6.4 m/s): no dip, mean above 12 km/h -> no signature
    run_signal(sim, 3000, v=6.4, cmd=0.0)
    assert not sim.s.parking_signature_seen
    assert not sim.s.parking_mode_active


class TestInteractions:
  def test_passthrough_composition_forces_passive(self):
    sim = Sim()
    # parking + scenario gate + cam latch all at once
    run_signal(sim, 320, v=5.0, cmd=0.0, wheel=280.0)
    run_signal(sim, 30, v=4.0, cmd=50.0, wheel=280.0, tq=420.0)
    assert sim.s.parking_mode_active and sim.s.low_speed_cam_latched
    assert not sim.effective_lat_active()
    # releasing one layer at a time keeps passive until all clear
    run_signal(sim, 100, v=4.0, cmd=50.0, wheel=0.0)   # grip released (0.5 s < needed)
    assert not sim.effective_lat_active()

  def test_heavy_grip_anchor_while_angle_passive(self):
    sim = Sim()
    run_signal(sim, 100, v=5.0, cmd=0.0, wheel=45.0, tq=400.0)
    assert sim.s.angle_passive_active
    # both anchors demand apply_angle_last == wheel
    sim.step(v=5.0, cmd=0.0, wheel=51.0, tq=400.0)
    assert sim.s.apply_angle_last == 51.0

  def test_passthrough_release_resumes_from_wheel(self):
    # scenario gate passthrough while the driver turns; on re-engage the very
    # first active TX must start from the wheel, not a stale op trajectory
    sim = Sim()
    # fast boot (no S1b), lead at 10 m keeps the S3' creep window reset
    # without entering traffic-follow (needs < 8 m)
    run_signal(sim, 60, v=20.0, cmd=0.0, lead_dist=10.0)
    run_signal(sim, 200, v=4.0, cmd=5.0, wheel=5.0, lead_dist=10.0)
    assert sim.effective_lat_active()
    # sharp command -> gate yields (0.3 s); driver meanwhile holds wheel at 60
    run_signal(sim, 100, v=4.0, cmd=60.0, wheel=60.0, lead_dist=10.0)
    assert not sim.effective_lat_active()
    run_signal(sim, 300, v=4.0, cmd=60.0, wheel=60.0, lead_dist=10.0)
    # command returns gentle; 1.0 s dwell to re-engage
    tr = run_signal(sim, 200, v=4.0, cmd=20.0, wheel=60.0, lead_dist=10.0)
    idx = tr['eff_active'].index(True)
    first_tx = tr['apply'][idx]
    assert abs(first_tx - 60.0) < 6.0, f"re-engage TX {first_tx} deg jumped away from wheel 60 deg"
