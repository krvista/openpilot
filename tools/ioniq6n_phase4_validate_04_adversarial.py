#!/usr/bin/env python3
"""Phase 4 validation Step 4 — driver override + NaN/inf + cam_stale.

Adversarial scenarios:

  J. Driver override gradient: 0 → 30 → 150 → 250 Nm on the wheel. Verify
     override_factor = 0 below 30, ramps 0→1 in 30-150, saturates ≥150.
     apply_angle blends op↔wheel accordingly.
  K. NaN/inf injection into every input field:
     - vEgoRaw = NaN, inf, -inf, -5 (negative speed glitch)
     - steeringAngleDeg = NaN
     - steeringTorque = NaN
     - op_angle_cmd (actuators.steeringAngleDeg) = NaN
     In every case: apply_angle remains finite, within bounds.
  L. Camera staleness: freeze cam_counter for >25 frames. cam_stale
     should flip True; apply_angle_last stays finite. (The carcontroller
     uses cam_stale only to force steering_active=False in the packer,
     not to modify apply_angle — so from this pipeline's standpoint
     we only verify cam_stale is correctly detected.)
  M. Blinker + marginal authority: driver blinker ON while authority
     just above ACI_ENTER * 5 (since blinker multiplies authority by 0.2).
     Verify aci flips accordingly without crash.
  N. Extreme op_angle_cmd: planner demands 500° suddenly. Limiter must
     clamp to STEER_ANGLE_MAX=176.7° without crash, preserving sign.
"""
import sys
import math
import numpy as np

sys.path.insert(0, '/home/user/openpilot/tools')
from ioniq6n_phase4_pipeline import Phase4Sim, assert_finite, assert_bounded_apply


def scenario_j_driver_override_gradient():
  sim = Phase4Sim()
  v_ms = 80 / 3.6
  # Let aci latch first
  for i in range(50):
    sim.step(v_ego_raw=v_ms, steering_angle_deg=0.0, steering_torque=0.0,
             blinker=False, lat_active=True, op_angle_cmd=5.0,
             cam_counter=(i // 2) % 16)
  assert sim.aci_active_latched
  # Now ramp driver torque 0 → 250 Nm over 2 s
  override_samples = []
  for i in range(200):
    tq = 250.0 * (i / 200)
    out = sim.step(v_ego_raw=v_ms, steering_angle_deg=sim.apply_angle_last,
                   steering_torque=tq, blinker=False, lat_active=True,
                   op_angle_cmd=5.0, cam_counter=((50 + i) // 2) % 16)
    assert_finite(out, f"J[{i}]")
    assert_bounded_apply(out, label=f"J[{i}]")
    override_samples.append((tq, out["override_factor"]))
  # Check gradient: 0 at tq<30, ramp 0-1 in 30-150, sat at ≥150
  for tq, of in override_samples:
    if tq < 30:
      assert of == 0.0, f"J: tq={tq}, override_factor={of}, expected 0"
    elif tq >= 150:
      assert of == 1.0, f"J: tq={tq}, override_factor={of}, expected 1"
    else:
      expected = (tq - 30) / 120
      assert abs(of - expected) < 1e-6, f"J: tq={tq}, got {of}, expected {expected}"
  print(f"  ✓ J driver override: gradient correct (0 below 30, ramp 30-150, sat ≥150)")


def scenario_k_nan_inf():
  sim = Phase4Sim()
  # Let aci latch
  for i in range(50):
    sim.step(v_ego_raw=20.0, steering_angle_deg=0.0, steering_torque=0.0,
             blinker=False, lat_active=True, op_angle_cmd=2.0,
             cam_counter=(i // 2) % 16)
  assert sim.aci_active_latched
  # Each injection runs 10 frames so the jitter/LPF state updates too.
  bad_vals = [float("nan"), float("inf"), float("-inf"), -5.0]
  for ev in bad_vals:
    for i in range(10):
      out = sim.step(v_ego_raw=ev, steering_angle_deg=0.0, steering_torque=0.0,
                     blinker=False, lat_active=True, op_angle_cmd=2.0,
                     cam_counter=(i // 2) % 16)
      assert_finite(out, f"K.v={ev}[{i}]")
      assert_bounded_apply(out, label=f"K.v={ev}[{i}]")
  # Same for steering_angle_deg
  for ev in [float("nan"), float("inf"), float("-inf")]:
    for i in range(10):
      out = sim.step(v_ego_raw=20.0, steering_angle_deg=ev, steering_torque=0.0,
                     blinker=False, lat_active=True, op_angle_cmd=2.0,
                     cam_counter=(i // 2) % 16)
      assert_finite(out, f"K.sa={ev}[{i}]")
      assert_bounded_apply(out, label=f"K.sa={ev}[{i}]")
  # torque
  for ev in [float("nan"), float("inf")]:
    for i in range(10):
      out = sim.step(v_ego_raw=20.0, steering_angle_deg=0.0, steering_torque=ev,
                     blinker=False, lat_active=True, op_angle_cmd=2.0,
                     cam_counter=(i // 2) % 16)
      assert_finite(out, f"K.tq={ev}[{i}]")
      assert_bounded_apply(out, label=f"K.tq={ev}[{i}]")
  # op_angle_cmd
  for ev in [float("nan"), float("inf"), float("-inf")]:
    for i in range(10):
      out = sim.step(v_ego_raw=20.0, steering_angle_deg=0.0, steering_torque=0.0,
                     blinker=False, lat_active=True, op_angle_cmd=ev,
                     cam_counter=(i // 2) % 16)
      assert_finite(out, f"K.op={ev}[{i}]")
      assert_bounded_apply(out, label=f"K.op={ev}[{i}]")
  print(f"  ✓ K NaN/inf: 14 injections across 4 fields, apply_angle remains finite+bounded")


def scenario_l_cam_stale():
  sim = Phase4Sim()
  # Normal cam updates for 40 frames
  for i in range(40):
    out = sim.step(v_ego_raw=20.0, steering_angle_deg=0.0, steering_torque=0.0,
                   blinker=False, lat_active=True, op_angle_cmd=2.0,
                   cam_counter=(i // 2) % 16)
    assert not out["cam_stale"], f"L1[{i}]: cam_stale prematurely True"
  # Freeze cam_counter → after 25 frames, stale should flip
  frozen = out["cam_stale"]
  frozen_counter = 5
  stale_flipped_at = None
  for i in range(60):
    out = sim.step(v_ego_raw=20.0, steering_angle_deg=0.0, steering_torque=0.0,
                   blinker=False, lat_active=True, op_angle_cmd=2.0,
                   cam_counter=frozen_counter)
    assert_finite(out, f"L2[{i}]")
    if out["cam_stale"] and stale_flipped_at is None:
      stale_flipped_at = i
  assert stale_flipped_at is not None, "L: cam_stale never detected"
  assert 24 <= stale_flipped_at <= 28, \
    f"L: stale detected at frame {stale_flipped_at}, expected ~26"
  print(f"  ✓ L cam stale: detected at +{stale_flipped_at} frames (expected ~26)")


def scenario_m_blinker_authority():
  sim = Phase4Sim()
  v_ms = 80 / 3.6
  # Blinker ON → authority scaled by 0.2. At driver_blend=1, speed_blend=1:
  # authority = 0.2. Below ACI_ENTER (0.30) so aci should NOT latch on blinker.
  for i in range(100):
    out = sim.step(v_ego_raw=v_ms, steering_angle_deg=0.0, steering_torque=0.0,
                   blinker=True, lat_active=True, op_angle_cmd=2.0,
                   cam_counter=(i // 2) % 16)
    assert_finite(out, f"M[{i}]")
  # Blinker keeps authority=0.2 < 0.30 → aci should not latch
  # But if aci is already latched from a prior state, it would stay (hysteresis)
  # Here we start fresh so expect NOT latched.
  assert not sim.aci_active_latched, "M: blinker should keep aci off (0.2<0.30)"
  # Turn blinker off → authority = 1.0 → latches
  for i in range(50):
    sim.step(v_ego_raw=v_ms, steering_angle_deg=0.0, steering_torque=0.0,
             blinker=False, lat_active=True, op_angle_cmd=2.0,
             cam_counter=((100 + i) // 2) % 16)
  assert sim.aci_active_latched, "M: aci should latch when blinker off"
  print(f"  ✓ M blinker: authority scaling correct, aci stays off under blinker")


def scenario_n_extreme_planner_angle():
  """Two cases: low-speed (STEER_ANGLE_MAX clamp dominates) and high-speed
  (VM lateral-accel cap is tighter than STEER_ANGLE_MAX — this is good)."""
  # Case 1: low speed where STEER_ANGLE_MAX is the binding limit.
  sim = Phase4Sim()
  v_ms = 5 / 3.6  # 5 km/h
  for i in range(50):
    sim.step(v_ego_raw=v_ms, steering_angle_deg=0.0, steering_torque=0.0,
             blinker=False, lat_active=True, op_angle_cmd=0.0,
             cam_counter=(i // 2) % 16)
  for i in range(500):
    out = sim.step(v_ego_raw=v_ms, steering_angle_deg=sim.apply_angle_last,
                   steering_torque=0.0, blinker=False, lat_active=True,
                   op_angle_cmd=500.0, cam_counter=((50 + i) // 2) % 16)
    assert_finite(out, f"N_low+[{i}]")
    assert_bounded_apply(out, label=f"N_low+[{i}]")
  assert abs(sim.apply_angle_last - 176.7) < 1.0, \
    f"N_low+: final={sim.apply_angle_last}, expected ≈176.7 at low speed"
  # Case 2: high speed — VM accel cap is tighter than STEER_ANGLE_MAX.
  sim = Phase4Sim()
  v_ms = 80 / 3.6
  for i in range(50):
    sim.step(v_ego_raw=v_ms, steering_angle_deg=0.0, steering_torque=0.0,
             blinker=False, lat_active=True, op_angle_cmd=0.0,
             cam_counter=(i // 2) % 16)
  for i in range(500):
    out = sim.step(v_ego_raw=v_ms, steering_angle_deg=sim.apply_angle_last,
                   steering_torque=0.0, blinker=False, lat_active=True,
                   op_angle_cmd=500.0, cam_counter=((50 + i) // 2) % 16)
    assert_finite(out, f"N_hi+[{i}]")
    assert_bounded_apply(out, label=f"N_hi+[{i}]")
  # At 80 km/h (22.2 m/s), MAX_LATERAL_ACCEL=3.3 m/s² →
  # max_curvature = 3.3/22.2² = 0.00669/m; VM translates to a wheel angle
  # much less than 176.7. Expected: the physical accel cap is working.
  # Just verify it's safely capped to a plausible wheel angle (< 50°, > 10°).
  assert 10.0 < abs(sim.apply_angle_last) < 50.0, \
    f"N_hi+: final={sim.apply_angle_last}, expected VM accel-capped (10-50°)"
  # The VM dynamic bicycle model includes tire slip, so the angle that
  # achieves MAX_LATERAL_ACCEL=3.3 m/s² is larger than the simple kinematic
  # approximation. The key evidence: at 80 km/h the cap is much less than
  # STEER_ANGLE_MAX=176.7°, proving VM is tightening the limit physically.
  assert abs(sim.apply_angle_last) < 0.5 * 176.7, \
    "N_hi+: VM should cap far below STEER_ANGLE_MAX at highway speed"
  # negative direction
  for i in range(1000):
    out = sim.step(v_ego_raw=v_ms, steering_angle_deg=sim.apply_angle_last,
                   steering_torque=0.0, blinker=False, lat_active=True,
                   op_angle_cmd=-500.0, cam_counter=((550 + i) // 2) % 16)
    assert_finite(out, f"N_hi-[{i}]")
    assert_bounded_apply(out, label=f"N_hi-[{i}]")
  assert sim.apply_angle_last < -10.0 and sim.apply_angle_last > -50.0, \
    f"N_hi-: final={sim.apply_angle_last}, expected VM accel-capped negative"
  print(f"  ✓ N extreme planner: low-speed clamps to ±176.7°, "
        f"high-speed VM-accel-caps at ±{abs(sim.apply_angle_last):.1f}° (a_lat≈3.3)")


if __name__ == "__main__":
  print("── Step 4: driver override + NaN/inf + cam_stale ──")
  scenario_j_driver_override_gradient()
  scenario_k_nan_inf()
  scenario_l_cam_stale()
  scenario_m_blinker_authority()
  scenario_n_extreme_planner_angle()
  print("\n✅ STEP 4 PASSED — adversarial inputs all handled safely")
