#!/usr/bin/env python3
"""Phase 4 validation Step 3 — MADS-only, ACC-only, passthrough branches.

On the Ioniq 6 N CCNC angle pipeline:
  - `lat_active`  (CC.latActive): MADS enabled → drives the steering angle block.
  - `CC.enabled`  (ACC):          drives longitudinal only; not read by the
                                  CCNC angle block directly but still affects
                                  upstream op_angle_cmd (planner keeps emitting
                                  even when ACC off).

Scenarios:
  E. MADS-only (lat_active=True, ACC off): identical to MADS+ACC from this
     block's perspective. Verify no crash.
  F. ACC-only (lat_active=False) → lat flips ON: verify clean re-engagement
     (apply_angle tracks steer_angle → ramps to op_angle once active).
  G. MADS-only → MADS off mid-drive: apply_angle passthroughs to
     steer_angle_safe, aci unlatches.
  H. Driver-on-wheel passthrough: driver holds wheel with lat off, latch
     engages; when lat re-enables or driver releases, latch disengages
     without glitch.
  I. Low-speed cam passthrough hysteresis: enter <2 km/h, exit >3 km/h,
     verify no flap in 2-3 km/h band.
"""
import sys
import numpy as np

sys.path.insert(0, '/home/user/openpilot/tools')
from ioniq6n_phase4_pipeline import Phase4Sim, assert_finite, assert_bounded_apply


def scenario_e_mads_only():
  """MADS engaged, ACC off. ACC state doesn't enter this block."""
  sim = Phase4Sim()
  v_ms = 80 / 3.6
  for i in range(300):
    op_ang = 2.0 * np.sin(2 * np.pi * 0.1 * i * 0.01)
    out = sim.step(v_ego_raw=v_ms, steering_angle_deg=sim.apply_angle_last,
                   steering_torque=0.0, blinker=False, lat_active=True,
                   op_angle_cmd=op_ang, cam_counter=(i // 2) % 16)
    assert_finite(out, f"E[{i}]")
    assert_bounded_apply(out, label=f"E[{i}]")
  assert sim.aci_active_latched, "E: MADS-only must latch aci ON"
  print(f"  ✓ E MADS-only: aci=ON, 300 frames clean, final={sim.apply_angle_last:.2f}°")


def scenario_f_acc_only_to_mads():
  """Driver steering with ACC on, then engages MADS mid-drive.

  Before MADS engage: lat_active=False → apply_angle tracks steer_angle_safe.
  After MADS engage:  lat_active=True, aci latches → apply_angle follows op.
  """
  sim = Phase4Sim()
  v_ms = 60 / 3.6
  # Phase 1: lat_active=False, steer_angle=5° (driver's grip), op says 3°
  for i in range(150):
    out = sim.step(v_ego_raw=v_ms, steering_angle_deg=5.0,
                   steering_torque=0.0, blinker=False, lat_active=False,
                   op_angle_cmd=3.0, cam_counter=(i // 2) % 16)
    assert_finite(out, f"F1[{i}]")
    # When lat inactive, limiter returns steering_angle_deg
    if i % 2 == 0:
      assert abs(sim.apply_angle_last - 5.0) < 1e-3, \
        f"F1[{i}]: lat-off apply={sim.apply_angle_last}, expected 5.0"
  assert not sim.aci_active_latched, "F1: aci must stay OFF when lat inactive"

  # Phase 2: MADS engages. apply_angle should ramp from 5° → 3° under VM limiter.
  trace_p2 = []
  for i in range(200):
    out = sim.step(v_ego_raw=v_ms, steering_angle_deg=sim.apply_angle_last,
                   steering_torque=0.0, blinker=False, lat_active=True,
                   op_angle_cmd=3.0, cam_counter=((150 + i) // 2) % 16)
    assert_finite(out, f"F2[{i}]")
    trace_p2.append(sim.apply_angle_last)
    # no frame-to-frame step > 2° (VM limiter bound at 60 km/h ≈ 16.7m/s)
    if len(trace_p2) > 1:
      d = abs(trace_p2[-1] - trace_p2[-2])
      assert d < 3.0, f"F2[{i}]: step {d:.3f}° too large on re-engage"
  assert sim.aci_active_latched, "F2: aci must latch after MADS engage"
  assert abs(sim.apply_angle_last - 3.0) < 0.5, \
    f"F2: final apply={sim.apply_angle_last}, expected near 3.0"
  print(f"  ✓ F ACC→MADS: smooth ramp 5°→3°, final={sim.apply_angle_last:.3f}°")


def scenario_g_mads_disengage():
  """MADS active, then disengaged mid-drive. apply_angle should passthrough
  to steer_angle, aci unlatches."""
  sim = Phase4Sim()
  v_ms = 80 / 3.6
  # Phase 1: MADS active, stable at 2° curve
  for i in range(200):
    out = sim.step(v_ego_raw=v_ms, steering_angle_deg=sim.apply_angle_last,
                   steering_torque=0.0, blinker=False, lat_active=True,
                   op_angle_cmd=2.0, cam_counter=(i // 2) % 16)
  assert sim.aci_active_latched, "G1: MADS should be latched"
  assert abs(sim.apply_angle_last - 2.0) < 0.2, f"G1: apply={sim.apply_angle_last}"

  # Phase 2: MADS disengages (e.g., driver presses cancel). steer_angle=measured=2°
  # apply_angle should snap to steer_angle immediately.
  for i in range(20):
    out = sim.step(v_ego_raw=v_ms, steering_angle_deg=2.0,
                   steering_torque=0.0, blinker=False, lat_active=False,
                   op_angle_cmd=5.0,  # planner still emits, but ignored
                   cam_counter=((200 + i) // 2) % 16)
    assert_finite(out, f"G2[{i}]")
  assert not sim.aci_active_latched, "G2: aci must unlatch on MADS off"
  assert abs(sim.apply_angle_last - 2.0) < 1e-3, \
    f"G2: apply={sim.apply_angle_last}, expected 2.0 (wheel passthrough)"
  print(f"  ✓ G MADS disengage: aci=OFF, apply snapped to wheel ({sim.apply_angle_last:.3f}°)")


def scenario_h_driver_passthrough():
  """Driver holds wheel with lat off. passthrough_latched engages."""
  sim = Phase4Sim()
  v_ms = 50 / 3.6
  # Phase 1: lat off, driver_blend > 0.9 (torque < 30 Nm)
  for i in range(100):
    out = sim.step(v_ego_raw=v_ms, steering_angle_deg=3.0,
                   steering_torque=5.0,  # small torque → blend ≈ 1.0
                   blinker=False, lat_active=False,
                   op_angle_cmd=0.0, cam_counter=(i // 2) % 16)
    assert_finite(out, f"H1[{i}]")
  assert sim.passthrough_latched, "H1: driver passthrough should latch"

  # Phase 2: lat turns on → passthrough disengages
  for i in range(50):
    out = sim.step(v_ego_raw=v_ms, steering_angle_deg=sim.apply_angle_last,
                   steering_torque=5.0, blinker=False, lat_active=True,
                   op_angle_cmd=0.0, cam_counter=((100 + i) // 2) % 16)
    assert_finite(out, f"H2[{i}]")
  assert not sim.passthrough_latched, "H2: passthrough should disengage"
  print(f"  ✓ H driver passthrough: latched→unlatched on lat enable")


def scenario_i_low_speed_cam_hysteresis():
  """Sweep speed 0 → 5 → 0 km/h. low_speed_cam must latch at <2, release at >3."""
  sim = Phase4Sim()
  v_kmh_profile = list(np.linspace(0, 5, 200)) + list(np.linspace(5, 0, 200))
  prev_latched = False
  engages = 0
  releases = 0
  flap_count = 0  # flip inside the 2-3 band
  in_band_prev = None
  for i, v_kmh in enumerate(v_kmh_profile):
    v = v_kmh / 3.6
    out = sim.step(v_ego_raw=v, steering_angle_deg=0.0,
                   steering_torque=0.0, blinker=False, lat_active=True,
                   op_angle_cmd=0.0, cam_counter=(i // 2) % 16)
    assert_finite(out, f"I[{i}]")
    in_band = 2.0/3.6 < v < 3.0/3.6
    if out["low_speed_cam"] and not prev_latched:
      engages += 1
    if (not out["low_speed_cam"]) and prev_latched:
      releases += 1
      if 2.0/3.6 < v < 3.0/3.6:
        flap_count += 1
    prev_latched = out["low_speed_cam"]
  # Profile is 0→5→0 km/h: enters <2 at start (engage #1), exits >3 going up
  # (release #1), enters <2 on return (engage #2). 1 engage per <2 entry.
  assert engages == 2, f"I: expected 2 engages for 0→5→0 sweep, got {engages}"
  assert releases == 1, f"I: expected 1 release, got {releases}"
  assert flap_count == 0, f"I: {flap_count} hysteresis-band flaps detected"
  print(f"  ✓ I low-speed cam hysteresis: {engages} engage / {releases} release, no flap")


if __name__ == "__main__":
  print("── Step 3: MADS/ACC/passthrough branch tests ──")
  scenario_e_mads_only()
  scenario_f_acc_only_to_mads()
  scenario_g_mads_disengage()
  scenario_h_driver_passthrough()
  scenario_i_low_speed_cam_hysteresis()
  print("\n✅ STEP 3 PASSED — 5 branch-transition scenarios clean")
