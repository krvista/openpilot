"""Phase 45 (i6nv3 0x2e-0x38, report §31): release the parking-mode latch once the car is plainly on a marked road.

Signal: CarControlSP.laneLineProbMin (controlsd_ext: min of the model's inner lane-line probabilities, 0.0 when unknown).
Rule: latched AND laneLineProbMin >= 0.8 AND v >= 15 km/h AND |wheel| < 30 deg for 3 s continuous -> release, clearing
the parking signature and the creep window. A new signature re-arms the latch exactly as before.
"""
import pytest

from phase_tests.harness import Sim
import opendbc.car.hyundai.carcontroller as ccmod

KPH = 1 / 3.6


def boot_latched(v_kph=10.0, frames=500):
  """Cold start at low speed -> boot signature -> latch after the 3 s low-speed window."""
  sim = Sim()
  for _ in range(frames):
    sim.step(v=v_kph * KPH, tq=0.0, wheel=0.0, cmd=0.0)
  assert sim.cc.parking_mode_active
  return sim


def drive(sim, n, v_kph=20.0, lane=0.9, wheel=0.0, **kw):
  for _ in range(n):
    sim.step(v=v_kph * KPH, tq=0.0, wheel=wheel, cmd=0.0, lane_prob_min=lane, **kw)


class TestPhase45Release:
  def test_marked_road_releases_after_3s(self):
    sim = boot_latched()
    drive(sim, ccmod.PARKING_RELEASE_FRAMES - 1)
    assert sim.cc.parking_mode_active                      # 2.99 s: not yet
    drive(sim, 2)
    assert not sim.cc.parking_mode_active
    assert not sim.cc.parking_signature_seen
    assert sim.cc.parking_low_speed_frames >= ccmod.PARKING_MODE_ENTER_SUSTAIN_FRAMES   # kept for an immediate re-arm

  def test_op_steers_again_after_release(self):
    sim = boot_latched()
    drive(sim, ccmod.PARKING_RELEASE_FRAMES + 50)
    assert sim.effective_lat_active()

  @pytest.mark.parametrize("kw", [dict(lane=0.7), dict(v_kph=12.0), dict(wheel=40.0), dict(lane=0.0)])
  def test_each_condition_is_required(self, kw):
    sim = boot_latched()
    args = dict(v_kph=20.0, lane=0.9, wheel=0.0); args.update(kw)
    drive(sim, 2 * ccmod.PARKING_RELEASE_FRAMES, **args)
    assert sim.cc.parking_mode_active

  def test_unknown_signal_never_releases(self):
    """Old logs / missing modelV2: the field defaults to 0.0 -> the pre-45 behaviour."""
    sim = boot_latched()
    for _ in range(2 * ccmod.PARKING_RELEASE_FRAMES):
      sim.step(v=20.0 * KPH, tq=0.0, wheel=0.0, cmd=0.0)    # harness default lane_prob_min = 0.0
    assert sim.cc.parking_mode_active

  def test_interruption_resets_the_count(self):
    sim = boot_latched()
    half = ccmod.PARKING_RELEASE_FRAMES - 10
    drive(sim, half)
    drive(sim, 5, lane=0.5)                                 # lines drop for 50 ms
    drive(sim, half)
    assert sim.cc.parking_mode_active

  def test_new_signature_rearms_after_release(self):
    sim = boot_latched()
    drive(sim, ccmod.PARKING_RELEASE_FRAMES + 5)
    assert not sim.cc.parking_mode_active
    drive(sim, 50, v_kph=10.0, lane=0.0, wheel=300.0)      # tight turn in a lot
    drive(sim, 400, v_kph=10.0, lane=0.0, wheel=0.0)       # 4 s low-speed window
    assert sim.cc.parking_mode_active

  def test_release_does_not_rearm_from_the_old_creep_window(self):
    sim = boot_latched()
    drive(sim, ccmod.PARKING_RELEASE_FRAMES + 5)
    drive(sim, 1200, v_kph=18.0, lane=0.0)                  # 12 s steady 18 km/h, no lead, no new signature
    assert not sim.cc.parking_mode_active

  def test_junction_stop_after_release_does_not_rearm(self):
    """Review: release, 2 s on the road, a 1 s dip to 6 km/h at the lot-exit junction (no lead), then 22 km/h —
    before the hold-off the creep signature re-armed the latch ~10 s later while moving."""
    sim = boot_latched()
    drive(sim, ccmod.PARKING_RELEASE_FRAMES + 5, v_kph=18.0)
    assert not sim.cc.parking_mode_active
    drive(sim, 200, v_kph=18.0, lane=0.9)
    drive(sim, 100, v_kph=6.0, lane=0.9)
    seen_passive = False
    for _ in range(1500):
      sim.step(v=22.0 * KPH, tq=0.0, wheel=5.0, cmd=5.0, lane_prob_min=0.9)
      seen_passive |= sim.cc.parking_mode_active
    assert not seen_passive

  def test_creep_rearms_again_after_holdoff(self):
    sim = boot_latched()
    drive(sim, ccmod.PARKING_RELEASE_FRAMES + 5, v_kph=18.0)
    drive(sim, ccmod.PARKING_RELEASE_CREEP_HOLDOFF_FRAMES + 50, v_kph=40.0, lane=0.9)   # hold-off expires on the road
    drive(sim, 1400, v_kph=6.0, lane=0.0)                   # a real lot crawl afterwards
    assert sim.cc.parking_mode_active

  def test_request_far_from_wheel_blocks_release(self):
    sim = boot_latched()
    for _ in range(2 * ccmod.PARKING_RELEASE_FRAMES):
      sim.step(v=25.0 * KPH, tq=0.0, wheel=2.0, cmd=60.0, lane_prob_min=0.9)
    assert sim.cc.parking_mode_active

  def test_nan_wheel_does_not_count_as_straight(self):
    sim = boot_latched()
    for _ in range(2 * ccmod.PARKING_RELEASE_FRAMES):
      sim.step(v=20.0 * KPH, tq=0.0, wheel=float("nan"), cmd=0.0, lane_prob_min=0.9)
    assert sim.cc.parking_mode_active

  def test_leadless_stop_and_go_on_marked_road_does_not_flap(self):
    """Stress test: 5-24 km/h cycle every 12 s, no lead, clear lanes — before the lane gate the creep window re-armed
    the latch 10 s after every release (11 toggles in 90 s)."""
    sim = boot_latched()
    drive(sim, ccmod.PARKING_RELEASE_FRAMES + 5, v_kph=18.0)
    assert not sim.cc.parking_mode_active
    import math as _m
    toggles = 0; prev = sim.cc.parking_mode_active
    for k in range(9000):
      v = 14.5 + 9.5 * _m.sin(2 * _m.pi * k / 1200)
      sim.step(v=v * KPH, tq=0.0, wheel=0.0, cmd=0.0, lane_prob_min=0.9)
      toggles += sim.cc.parking_mode_active != prev; prev = sim.cc.parking_mode_active
    assert toggles == 0

  def test_leadless_crawl_without_lanes_still_arms(self):
    """Same crawl in a lot (no clear lane lines): the creep signature works as before."""
    sim = Sim()
    for _ in range(300):
      sim.step(v=40.0 * KPH, tq=0.0, wheel=0.0, cmd=0.0)        # past the boot window, latch clear
    assert not sim.cc.parking_mode_active
    drive(sim, 1400, v_kph=6.0, lane=0.0)
    assert sim.cc.parking_mode_active

  def test_hard_signature_right_after_release_rearms_at_once(self):
    sim = boot_latched()
    drive(sim, ccmod.PARKING_RELEASE_FRAMES + 5, v_kph=18.0)
    assert not sim.cc.parking_mode_active
    for k in range(60):
      sim.step(v=12.0 * KPH, tq=0.0, wheel=min(300.0, 10.0 * k), cmd=0.0, lane_prob_min=0.2)
    assert sim.cc.parking_mode_active                         # within 0.6 s of the tight turn, not 3 s

  def test_highspeed_exit_unchanged(self):
    sim = boot_latched()
    drive(sim, ccmod.PARKING_MODE_EXIT_SUSTAIN_FRAMES + 5, v_kph=40.0, lane=0.0)
    assert not sim.cc.parking_mode_active

  def test_kill(self):
    old = ccmod.PARKING_RELEASE_FRAMES
    try:
      ccmod.PARKING_RELEASE_FRAMES = 0
      sim = boot_latched()
      drive(sim, 1000)
      assert sim.cc.parking_mode_active
    finally:
      ccmod.PARKING_RELEASE_FRAMES = old

  def test_shipped_defaults(self):
    assert ccmod.PARKING_RELEASE_LANE_PROB == 0.8 and ccmod.PARKING_RELEASE_FRAMES == 300
    assert abs(ccmod.PARKING_RELEASE_MIN_MS - 15.0 / 3.6) < 1e-9 and ccmod.PARKING_RELEASE_WHEEL_DEG == 30.0
    assert ccmod.PARKING_RELEASE_CMD_GAP_DEG == 10.0 and ccmod.PARKING_RELEASE_CREEP_HOLDOFF_FRAMES == 2000


class TestPhase45Plumbing:
  def test_struct_default_is_zero(self):
    from opendbc.car import structs
    assert structs.CarControlSP().laneLineProbMin == 0.0

  def test_capnp_roundtrip(self):
    try:
      from openpilot.cereal import custom
      from openpilot.selfdrive.car.helpers import convert_carControlSP
    except Exception as e:  # noqa: BLE001 - environments without the native build
      pytest.skip(f"capnp/helpers unavailable: {e}")
    m = custom.CarControlSP.new_message(); m.laneLineProbMin = 0.87
    out = convert_carControlSP(m.as_reader())
    assert abs(out.laneLineProbMin - 0.87) < 1e-6
    assert convert_carControlSP(custom.CarControlSP.new_message().as_reader()).laneLineProbMin == 0.0
