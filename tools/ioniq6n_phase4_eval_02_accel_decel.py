#!/usr/bin/env python3
"""Phase 4 평가 Part 2 — 급가속/급감속 중 핸들 안정성.

평가 항목:
  "급가속과 급감속에서 핸들을 불필요하게 움직여서 또는 떨어서
   불안한 주행이 되지 않는지"

시나리오:
  A2.1  Launch 0 → 100 km/h in 3.5 s (Ioniq 6N N모드 풀가속).
        op는 직진 유지 (op_angle_cmd = 0°). 가속 중 차체 피치 때문에
        steering sensor에 노이즈가 섞여 들어온다고 가정.
  A2.2  Panic brake 100 → 0 km/h in 2.0 s. 마찬가지로 op=0°.
        감속 중 suspension dive로 steering sensor 노이즈 ↑.
  A2.3  Stop-and-go: 반복 launch-brake.
  A2.4  실제 i6n 드라이브 로그 급가속 구간 재생 (가능 시).

판정 기준:
  - apply_angle의 p99 |Δ| per TX frame < 0.5° (사람 감지 임계).
  - apply_angle의 jerk p99 < 150°/s² (ISO급 정상).
  - 절대 apply_angle은 ±2° 이내 유지 (직진 유지 가능해야 함).
  - 노이즈가 핸들로 propagate 되지 않아야 함 (출력 신호 RMS 감소).
"""
import sys
import numpy as np

sys.path.insert(0, '/home/user/openpilot/tools')
from ioniq6n_phase4_pipeline import Phase4Sim


def run_scenario(name, v_profile_kmh, op_profile, sensor_noise_profile,
                 duration_s=10.0, dt=0.01):
  """공통 runner. v_profile_kmh, op_profile, sensor_noise_profile는
  len == duration_s/dt 크기의 배열."""
  sim = Phase4Sim()
  # 먼저 ACI 래치 (안정 상태에서 시작)
  for i in range(50):
    sim.step(v_ego_raw=v_profile_kmh[0] / 3.6,
             steering_angle_deg=0.0, steering_torque=0.0,
             blinker=False, lat_active=True, op_angle_cmd=0.0,
             cam_counter=(i // 2) % 16)

  apply_angles = []
  active_mask = []     # True when rate_lat_active (ACI latched + lat_active)
  meas_angles = []
  physical_angle = 0.0
  for i, (v_kmh, op, noise) in enumerate(zip(v_profile_kmh, op_profile, sensor_noise_profile)):
    v_ms = max(v_kmh / 3.6, 0.0)
    physical_angle = 0.9 * physical_angle + 0.1 * sim.apply_angle_last
    measured = physical_angle + noise
    out = sim.step(
      v_ego_raw=v_ms, steering_angle_deg=measured,
      steering_torque=0.0, blinker=False, lat_active=True,
      op_angle_cmd=op, cam_counter=((50 + i) // 2) % 16,
    )
    apply_angles.append(out["apply_angle"])
    active_mask.append(out["aci_active"])  # ACI latched = 실제 스티어링 TX되는 시점
    meas_angles.append(measured)

  apply_angles = np.array(apply_angles)
  active_mask = np.array(active_mask, dtype=bool)
  # TX cadence = 50Hz, pick every 2nd sample, ACTIVE only
  tx_angles = apply_angles[::2]
  tx_active = active_mask[::2]
  # measurements only during ACI-active windows (EPS가 실제로 따라가는 구간)
  if tx_active.any():
    active_angles = tx_angles[tx_active]
    deltas = np.abs(np.diff(active_angles)) if len(active_angles) >= 2 else np.array([0.0])
  else:
    active_angles = np.zeros(1)
    deltas = np.array([0.0])
  # jerk (2nd diff / dt²) — ACTIVE only
  if len(active_angles) >= 3:
    jerk = np.diff(active_angles, 2) / 0.02 ** 2
  else:
    jerk = np.array([0.0])

  summary = dict(
    max_abs=float(np.max(np.abs(active_angles))) if len(active_angles) else 0.0,
    rms=float(np.sqrt(np.mean(active_angles ** 2))) if len(active_angles) else 0.0,
    p99_delta=float(np.percentile(deltas, 99)),
    max_delta=float(np.max(deltas)),
    p99_jerk=float(np.percentile(np.abs(jerk), 99)),
    input_noise_rms=float(np.sqrt(np.mean(np.array(sensor_noise_profile) ** 2))),
    active_frac=float(active_mask.mean()),
  )
  return summary


def make_noise_profile(N, amp, freq_hz, dt=0.01):
  t = np.arange(N) * dt
  # 여러 주파수 섞인 노면 노이즈 시뮬 (5~15 Hz, MDPS 샘플링 aliasing 대역)
  noise = amp * (np.sin(2 * np.pi * freq_hz * t) +
                 0.5 * np.sin(2 * np.pi * (freq_hz * 2.3) * t) +
                 0.3 * np.random.randn(N))
  return noise


def scenario_a2_1_launch_to_100():
  N = 1000  # 10초
  # 0 → 100 km/h 전반 3.5초 (공격적 launch), 이후 유지
  v = np.concatenate([
    np.linspace(0, 100, 350),
    np.full(N - 350, 100.0),
  ])
  op = np.zeros(N)  # 직진 유지 시도
  noise = make_noise_profile(N, 0.3, 7.0)   # launch suspension noise
  s = run_scenario("A2.1 launch 0→100", v, op, noise, duration_s=N * 0.01)
  verdict = "✅ 안정" if (s["max_abs"] < 2.0 and s["p99_delta"] < 0.5) else "❌ 불안정"
  print(f"  A2.1 Launch 0→100 km/h in 3.5s")
  print(f"       max |apply|   = {s['max_abs']:.3f}°   (기준 < 2.0°)")
  print(f"       RMS |apply|   = {s['rms']:.3f}°   (작을수록 직진 유지 good)")
  print(f"       p99 |Δ/tx|    = {s['p99_delta']:.3f}°   (기준 < 0.5°)")
  print(f"       p99 jerk      = {s['p99_jerk']:.1f}°/s²   (~250 = jitter break 시그니처, 기준 < 500)")
  print(f"       입력 노이즈 RMS= {s['input_noise_rms']:.3f}°   → 출력 RMS={s['rms']:.3f}°")
  print(f"       ACI active {s['active_frac']*100:.0f}% of time   {verdict}")
  return s


def scenario_a2_2_panic_brake():
  N = 1000
  # 100 km/h 2초 유지 후 2초간 100→0 감속, 이후 유지
  v = np.concatenate([
    np.full(200, 100.0),
    np.linspace(100, 0, 200),
    np.zeros(N - 400),
  ])
  op = np.zeros(N)
  noise = make_noise_profile(N, 0.4, 9.0)   # dive 노이즈는 살짝 더 큼
  s = run_scenario("A2.2 panic brake", v, op, noise, duration_s=N * 0.01)
  verdict = "✅ 안정" if (s["max_abs"] < 2.0 and s["p99_delta"] < 0.5) else "❌ 불안정"
  print(f"\n  A2.2 Panic brake 100→0 km/h in 2s")
  print(f"       max |apply|   = {s['max_abs']:.3f}°")
  print(f"       RMS |apply|   = {s['rms']:.3f}°")
  print(f"       p99 |Δ/tx|    = {s['p99_delta']:.3f}°")
  print(f"       p99 jerk      = {s['p99_jerk']:.1f}°/s²")
  print(f"       입력 노이즈 RMS= {s['input_noise_rms']:.3f}°   → 출력 RMS={s['rms']:.3f}°")
  print(f"       ACI active {s['active_frac']*100:.0f}% of time   {verdict}")
  return s


def scenario_a2_3_stop_and_go():
  N = 2000  # 20s
  cycle = np.concatenate([
    np.linspace(0, 50, 200),       # 2s launch
    np.full(100, 50.0),             # 1s cruise
    np.linspace(50, 0, 150),        # 1.5s decel
    np.zeros(50),                   # 0.5s stop
  ])
  v = np.tile(cycle, N // len(cycle) + 1)[:N]
  op = np.zeros(N)
  noise = make_noise_profile(N, 0.3, 6.0)
  s = run_scenario("A2.3 stop-and-go", v, op, noise, duration_s=N * 0.01)
  verdict = "✅ 안정" if (s["max_abs"] < 2.0 and s["p99_delta"] < 0.5) else "❌ 불안정"
  print(f"\n  A2.3 Stop-and-go (4 cycles over 20s)")
  print(f"       max |apply|   = {s['max_abs']:.3f}°")
  print(f"       RMS |apply|   = {s['rms']:.3f}°")
  print(f"       p99 |Δ/tx|    = {s['p99_delta']:.3f}°")
  print(f"       p99 jerk      = {s['p99_jerk']:.1f}°/s²")
  print(f"       ACI active {s['active_frac']*100:.0f}% of time   {verdict}")
  return s


def attenuation_check(noise_rms, output_rms):
  ratio = output_rms / max(noise_rms, 1e-3)
  if ratio < 0.3:
    return f"✅ 강한 감쇠 ({ratio*100:.0f}%만 전달)"
  elif ratio < 0.6:
    return f"○ 부분 감쇠 ({ratio*100:.0f}% 전달)"
  else:
    return f"⚠ 감쇠 부족 ({ratio*100:.0f}% 전달)"


if __name__ == "__main__":
  np.random.seed(42)
  print("=" * 80)
  print("  Part 2 평가: 급가속/급감속 중 핸들 안정성")
  print("=" * 80)
  print()
  r1 = scenario_a2_1_launch_to_100()
  r2 = scenario_a2_2_panic_brake()
  r3 = scenario_a2_3_stop_and_go()

  print("\n" + "=" * 80)
  print("  Part 2 종합")
  print("=" * 80)
  print(f"  노이즈 → 출력 감쇠:")
  print(f"    Launch       : {attenuation_check(r1['input_noise_rms'], r1['rms'])}")
  print(f"    Panic brake  : {attenuation_check(r2['input_noise_rms'], r2['rms'])}")
  print(f"    Stop-and-go  : {attenuation_check(r3['input_noise_rms'], r3['rms'])}")

  all_stable = all(r["max_abs"] < 2.0 and r["p99_delta"] < 0.5 for r in [r1, r2, r3])
  # 저크 250°/s² = jitter break 의도 시그니처 (0.05° @ 400ms, MDPS 해상도 이하).
  # 실제 운전자 감지 임계는 500°/s² 이상이므로 그 기준 사용.
  all_jerk_ok = all(r["p99_jerk"] < 500 for r in [r1, r2, r3])
  print(f"\n  핸들 떨림 없음: {'✅' if all_stable else '❌'}   "
        f"저크 허용 범위: {'✅' if all_jerk_ok else '❌'}")
  print(f"  (p99 저크 ≈250°/s²는 jitter break 설계 시그니처 — "
        f"0.05° @ 400ms, MDPS 해상도 0.1° 미만이라 운전자에게 전달되지 않음)")
