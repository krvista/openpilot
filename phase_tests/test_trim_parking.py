"""Invariant (f): parking-mode latch; (g): interactions. (Invariant (e), the steady-curve trim, was removed with the
trim in Phase 89; the file keeps its name.)"""
import math

from phase_tests.harness import Sim, run_signal

from phase_tests.harness import pre73_rules  # noqa: F401,E402  (autouse: this module is written against the pre-73 car-controller rules)

DT = 0.01


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
    # parking + scenario gate + cam latch all at once.
    # Phase 26b: parking mode is already passive here, so hold compensation
    # is gated off (op contributes nothing to the bar) — raw 420 is all
    # driver and correctly reads as driver_tq 420 >= 250 (raw threshold in
    # passive frames, 31b) -> pressed.
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
