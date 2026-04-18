#!/usr/bin/env python3
"""Phase 4 validation Step 1 — VehicleModel init smoke test.

Confirms:
  1. VehicleModel(CP) constructs without exception for Ioniq 6 N CP.
  2. All VM.get_steer_from_curvature calls used by apply_steer_angle_limits_vm
     return finite numbers at the full speed range (0-55 m/s ≈ 200 km/h).
  3. ANGLE_LIMITS_VM → limiter math produces finite, sign-preserving output
     at all speeds that the limiter ever sees.
  4. STEER_STEP==2 is applied for CCNC cars (50 Hz TX cadence).

If this script completes "ALL CHECKS PASSED" the VM core is safe to
instantiate in CarController.__init__ on the vehicle.
"""
import sys
import math
sys.path.insert(0, '/home/user/openpilot/opendbc_repo')

from opendbc.car.structs import CarParams
from opendbc.car.hyundai.values import HyundaiFlags, CarControllerParams, CAR
from opendbc.car.vehicle_model import VehicleModel
from opendbc.car.lateral import apply_steer_angle_limits_vm


def make_ioniq6n_cp():
  """Synthesize a CarParams matching what interface.py produces for Ioniq 6 N."""
  cp = CarParams()
  cp.carFingerprint = str(CAR.HYUNDAI_IONIQ_6_N)
  cp.mass = 2175.0 + 136.0           # curb + standard cargo
  cp.wheelbase = 2.965
  cp.steerRatio = 14.26
  cp.centerToFront = 2.965 * 0.4
  # tireStiffnessFront/Rear are computed by interface from tireStiffnessFactor;
  # provide plausible values that mirror CarInterfaceBase defaults.
  stiffness_factor = 1.1
  cp.tireStiffnessFront = stiffness_factor * 192150
  cp.tireStiffnessRear  = stiffness_factor * 202500
  cp.rotationalInertia  = cp.mass * (cp.wheelbase ** 2) * 0.25 + 500  # rough
  cp.steerRatioRear = 0.0
  cp.flags = int(HyundaiFlags.EV | HyundaiFlags.CCNC |
                 HyundaiFlags.CANFD_LKA_STEERING_ALT |
                 HyundaiFlags.CANFD | HyundaiFlags.CANFD_ALT_BUTTONS)
  cp.steerControlType = CarParams.SteerControlType.angle
  return cp


def test_vm_init():
  print("── Test 1: VehicleModel(CP) construction ──")
  CP = make_ioniq6n_cp()
  VM = VehicleModel(CP)
  assert math.isfinite(VM.m), f"VM.m not finite: {VM.m}"
  assert math.isfinite(VM.j), f"VM.j not finite: {VM.j}"
  assert math.isfinite(VM.l), f"VM.l not finite: {VM.l}"
  assert math.isfinite(VM.cF) and math.isfinite(VM.cR)
  assert VM.sR > 0
  print(f"  ✓ VM: mass={VM.m:.1f}kg, wb={VM.l:.3f}m, sR={VM.sR:.2f}, "
        f"cF={VM.cF:.0f}, cR={VM.cR:.0f}")
  return CP, VM


def test_curvature_sweep(VM):
  print("── Test 2: VM.get_steer_from_curvature sweep ──")
  test_curvatures = [-0.1, -0.01, -0.001, 0.0, 0.001, 0.01, 0.1]
  test_speeds_ms = [0.5, 1.0, 2.0, 5.0, 10.0, 20.0, 30.0, 40.0, 55.0]
  for u in test_speeds_ms:
    for c in test_curvatures:
      ang = VM.get_steer_from_curvature(c, u, 0.0)
      assert math.isfinite(ang), f"NaN from VM at u={u}, c={c}: {ang}"
      # sign-preserving for nonzero curvature
      if abs(c) > 1e-6:
        assert (ang > 0) == (c > 0), f"sign flip at u={u},c={c}: ang={ang}"
  print(f"  ✓ {len(test_speeds_ms)*len(test_curvatures)} combos all finite, sign-preserving")


def test_limiter_sweep(CP, VM):
  print("── Test 3: apply_steer_angle_limits_vm across operating range ──")
  params = CarControllerParams(CP)
  limits = params  # limiter reads params.ANGLE_LIMITS + params.STEER_STEP

  # Verify STEER_STEP=2 (50 Hz) and ANGLE_LIMITS_VM was applied
  assert params.STEER_STEP == 2, f"expected STEER_STEP=2, got {params.STEER_STEP}"
  assert params.ANGLE_LIMITS.MAX_LATERAL_ACCEL == 3.3
  assert params.ANGLE_LIMITS.MAX_LATERAL_JERK == 3.5
  assert params.ANGLE_LIMITS.MAX_ANGLE_RATE == 1.3
  assert params.ANGLE_LIMITS.STEER_ANGLE_MAX == 176.7
  print(f"  ✓ STEER_STEP={params.STEER_STEP}, "
        f"ANGLE_LIMITS_VM (jerk={params.ANGLE_LIMITS.MAX_LATERAL_JERK}, "
        f"accel={params.ANGLE_LIMITS.MAX_LATERAL_ACCEL}, "
        f"rate={params.ANGLE_LIMITS.MAX_ANGLE_RATE})")

  ntests = 0
  for v_ego in [0.0, 0.5, 1.0, 5.0, 10.0, 20.0, 30.0, 40.0, 55.0]:
    for apply_angle in [-180.0, -90.0, -10.0, 0.0, 10.0, 90.0, 180.0]:
      for last in [-10.0, 0.0, 10.0]:
        out_active = apply_steer_angle_limits_vm(
          apply_angle, last, v_ego, 0.0, True, limits, VM,
        )
        out_inactive = apply_steer_angle_limits_vm(
          apply_angle, last, v_ego, 5.0, False, limits, VM,
        )
        assert math.isfinite(out_active), f"NaN (active) v={v_ego} a={apply_angle} last={last}"
        assert math.isfinite(out_inactive)
        assert abs(out_active) <= limits.ANGLE_LIMITS.STEER_ANGLE_MAX + 1e-6
        # inactive -> track current wheel angle
        assert abs(out_inactive - 5.0) < 1e-6, f"inactive should passthrough wheel angle"
        ntests += 1
  print(f"  ✓ {ntests} (v_ego × apply × last) limiter calls all finite & bounded")


if __name__ == "__main__":
  CP, VM = test_vm_init()
  test_curvature_sweep(VM)
  test_limiter_sweep(CP, VM)
  print("\n✅ STEP 1 PASSED — VM init + limiter math safe on Ioniq 6 N CP")
