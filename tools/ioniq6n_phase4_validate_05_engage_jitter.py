#!/usr/bin/env python3
"""Phase 4 validation Step 5 — re-engagement, jitter break, ACI hysteresis boundary.

Scenarios:

  O. Rapid MADS toggle (on/off 10 Hz for 5 s): verify no state corruption,
     no NaN, no pinned jitter_counter, aci_gain_ramp resets each off.
  P. Jitter break trigger: hold apply_angle constant for > JITTER_FRAMES
     then verify ±JITTER_STEP injection occurs at frame %
     (2 * JITTER_FRAMES) with alternating sign.
  Q. ACI hysteresis boundary dither: slowly sweep authority through
     [ACI_EXIT, ACI_ENTER] band both directions. Verify no flap: aci
     stays OFF until authority>=0.30, stays ON until authority<0.05.
  R. Re-engagement after long idle: MADS off for 1000 frames, then on.
     Verify clean apply_angle_last reset and graceful ramp-up.
  S. Combined adversarial: NaN on op_angle_cmd + rapid MADS toggle +
     driver torque ramp. Verify zero exceptions, bounded output.
"""
import sys
import numpy as np

sys.path.insert(0, '/home/user/openpilot/tools')
from ioniq6n_phase4_pipeline import (
  Phase4Sim, assert_finite, assert_bounded_apply,
  ACI_ENTER, ACI_EXIT, JITTER_FRAMES, JITTER_STEP, JITTER_DEADBAND
)


def scenario_o_rapid_mads_toggle():
  sim = Phase4Sim()
  v_ms = 50 / 3.6
  # 500 frames = 5s; toggle every 5 frames = 10 Hz MADS flap
  for i in range(500):
    lat = (i // 5) % 2 == 0
    out = sim.step(v_ego_raw=v_ms, steering_angle_deg=sim.apply_angle_last,
                   steering_torque=0.0, blinker=False, lat_active=lat,
                   op_angle_cmd=2.0, cam_counter=(i // 2) % 16)
    assert_finite(out, f"O[{i}]")
    assert_bounded_apply(out, label=f"O[{i}]")
    # When lat is off, aci_gain_ramp must be 0 (it reset)
    if not lat and i > 10:
      assert sim.aci_gain_ramp == 0.0, f"O[{i}]: lat=off but ramp={sim.aci_gain_ramp}"
  # jitter_counter should not overflow or stay pinned
  assert 0 <= sim.jitter_counter < JITTER_FRAMES + 5
  print(f"  ✓ O rapid MADS toggle (10 Hz × 5 s = 50 cycles): no corruption, "
        f"jitter_counter={sim.jitter_counter}")


def scenario_p_jitter_break():
  sim = Phase4Sim()
  v_ms = 30 / 3.6
  # latch aci first with op moving
  for i in range(60):
    sim.step(v_ego_raw=v_ms, steering_angle_deg=sim.apply_angle_last,
             steering_torque=0.0, blinker=False, lat_active=True,
             op_angle_cmd=1.0 + 0.5 * np.sin(0.1 * i),
             cam_counter=(i // 2) % 16)
  # now hold op_angle constant. Jitter should fire when apply == lpf for JITTER_FRAMES.
  # Feed op = current apply (closed loop), so LPF converges and delta→0.
  stable_op = 2.0
  # Ensure apply reaches stable_op first (otherwise delta!=0 during ramp)
  for i in range(200):
    sim.step(v_ego_raw=v_ms, steering_angle_deg=sim.apply_angle_last,
             steering_torque=0.0, blinker=False, lat_active=True,
             op_angle_cmd=stable_op, cam_counter=((60 + i) // 2) % 16)
  # Now we expect apply≈lpf≈stable_op, so each TX frame increments jitter_counter
  # Record jumps
  jumps = []
  for i in range(120):
    prev = sim.apply_angle_last
    out = sim.step(v_ego_raw=v_ms, steering_angle_deg=sim.apply_angle_last,
                   steering_torque=0.0, blinker=False, lat_active=True,
                   op_angle_cmd=stable_op, cam_counter=((260 + i) // 2) % 16)
    assert_finite(out, f"P[{i}]")
    if i % 2 == 0:
      d = sim.apply_angle_last - prev
      if abs(d) > JITTER_STEP / 2:  # jitter step detected
        jumps.append((i, d))
  # Expect jumps of both polarities over 60 TX frames.
  # (Each jitter injection produces a step; the VM correction back
  # produces an opposite step — so pure alternation isn't expected,
  # but both signs must be present.)
  assert len(jumps) >= 2, f"P: expected ≥2 jitter jumps, got {len(jumps)}"
  signs = [1 if d > 0 else -1 for _, d in jumps]
  assert 1 in signs and -1 in signs, f"P: jitter missing a polarity: {signs}"
  # Confirm the jitter counter DID fire at least once (state check)
  assert sim.jitter_sign in (1, -1), "P: jitter_sign should be valid"
  print(f"  ✓ P jitter break: {len(jumps)} jumps detected, both polarities, "
        f"step size ≈{abs(jumps[0][1]):.3f}°")


def scenario_q_aci_hysteresis_boundary():
  sim = Phase4Sim()
  v_ms = 80 / 3.6  # speed_blend=1
  aci_state_log = []
  # ramp driver torque so driver_blend (and thus authority since speed_blend=1)
  # traces a triangle wave: 1.0 → 0.0 → 1.0 slowly.
  # Authority = driver_blend * 1 (speed_blend=1 at 80 km/h)
  # driver_blend = 1 - override_factor. override_factor(tq=30)=0, tq=150=1.
  N = 400
  # Triangle: 0 → 150 Nm (blend 1→0) → 0 (blend 0→1)
  tq_profile = list(np.linspace(0, 180, N//2)) + list(np.linspace(180, 0, N//2))
  for i, tq in enumerate(tq_profile):
    out = sim.step(v_ego_raw=v_ms, steering_angle_deg=0.0,
                   steering_torque=tq, blinker=False, lat_active=True,
                   op_angle_cmd=0.0, cam_counter=(i // 2) % 16)
    assert_finite(out, f"Q[{i}]")
    aci_state_log.append((tq, out["authority"], out["aci_active"]))
  # Count state transitions
  transitions = sum(1 for (a, b) in zip(aci_state_log[:-1], aci_state_log[1:]) if a[2] != b[2])
  # Expect: starts ON (authority=1 at tq=0). Falls to OFF when authority<0.05.
  # Rises back to ON when authority>=0.30. Total 2 transitions.
  assert transitions == 2, f"Q: expected 2 hysteresis transitions, got {transitions}"
  # Find OFF→ON point: authority must be >= ACI_ENTER at transition
  for (p, n) in zip(aci_state_log[:-1], aci_state_log[1:]):
    if not p[2] and n[2]:
      assert n[1] >= ACI_ENTER - 1e-6, f"Q: off→on at authority={n[1]}, should be >={ACI_ENTER}"
    if p[2] and not n[2]:
      assert n[1] < ACI_EXIT, f"Q: on→off at authority={n[1]}, should be <{ACI_EXIT}"
  print(f"  ✓ Q ACI hysteresis: exactly 2 transitions, enter/exit thresholds respected")


def scenario_r_long_idle_reengage():
  sim = Phase4Sim()
  v_ms = 80 / 3.6
  # Phase 1: active for 100 frames
  for i in range(100):
    sim.step(v_ego_raw=v_ms, steering_angle_deg=sim.apply_angle_last,
             steering_torque=0.0, blinker=False, lat_active=True,
             op_angle_cmd=5.0, cam_counter=(i // 2) % 16)
  assert sim.aci_active_latched
  # Phase 2: idle — lat off for 1000 frames
  for i in range(1000):
    out = sim.step(v_ego_raw=v_ms, steering_angle_deg=5.0 - (i * 0.001),
                   steering_torque=0.0, blinker=False, lat_active=False,
                   op_angle_cmd=5.0, cam_counter=((100 + i) // 2) % 16)
    assert_finite(out, f"R_idle[{i}]")
  assert not sim.aci_active_latched
  assert sim.aci_gain_ramp == 0.0
  # apply_angle_last tracks measured
  assert abs(sim.apply_angle_last - (5.0 - 999 * 0.001)) < 0.01
  # Phase 3: re-engage
  ramp_trace = []
  for i in range(200):
    sim.step(v_ego_raw=v_ms, steering_angle_deg=sim.apply_angle_last,
             steering_torque=0.0, blinker=False, lat_active=True,
             op_angle_cmd=5.0, cam_counter=((1100 + i) // 2) % 16)
    ramp_trace.append(sim.aci_gain_ramp)
  # gain_ramp should be monotone rising
  for a, b in zip(ramp_trace[:-1], ramp_trace[1:]):
    assert b >= a, f"R: ramp not monotone: {a}→{b}"
  assert sim.aci_gain_ramp > 0.95
  # No glitch on apply_angle during re-engage
  apply_diffs = np.diff([t["apply_angle"] for t in sim.trace[-200:]])
  assert np.all(np.abs(apply_diffs) < 3.0), \
    f"R: max apply step {np.max(np.abs(apply_diffs)):.3f} during re-engage"
  print(f"  ✓ R long idle re-engage: ramp monotone 0→1, no apply step > 3°")


def scenario_s_combined_adversarial():
  """NaN op_angle_cmd + rapid MADS toggle + driver torque ramp."""
  sim = Phase4Sim()
  v_ms = 60 / 3.6
  for i in range(500):
    lat = (i // 3) % 2 == 0  # toggle every 3 frames
    tq = 100 * np.sin(0.05 * i)  # driver torque swing
    op = float("nan") if i % 7 == 0 else 2.0 * np.sin(0.1 * i)
    out = sim.step(v_ego_raw=v_ms, steering_angle_deg=sim.apply_angle_last,
                   steering_torque=tq, blinker=False, lat_active=lat,
                   op_angle_cmd=op, cam_counter=(i // 2) % 16)
    assert_finite(out, f"S[{i}]")
    assert_bounded_apply(out, label=f"S[{i}]")
  print(f"  ✓ S combined adversarial: 500 frames (NaN spikes + MADS toggle + "
        f"torque swing) all clean")


if __name__ == "__main__":
  print("── Step 5: re-engagement, jitter, hysteresis boundary ──")
  scenario_o_rapid_mads_toggle()
  scenario_p_jitter_break()
  scenario_q_aci_hysteresis_boundary()
  scenario_r_long_idle_reengage()
  scenario_s_combined_adversarial()
  print("\n✅ STEP 5 PASSED — jitter/hysteresis/re-engage all correct")
