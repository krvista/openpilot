#!/usr/bin/env python3
"""Phase 4 평가 Part 5 — 차선 중앙 유지 (한쪽 쏠림 없음).

평가 항목:
  "대부분의 경우에 차선 가운데를 잘 유지하는지? (한쪽으로 쏠리지 않도록)"

검증 질문:
  D5.1  직진 구간에서 op가 0°를 보낼 때 apply가 0° 주변에 머무는가?
        (mean이 0에 가깝고, bias가 없어야 함)
  D5.2  좌/우 대칭 입력에 대해 출력이 대칭인가?
        (VM이나 limiter가 한쪽 방향을 편애하지 않아야 함)
  D5.3  편향된 센서 노이즈 (예: +쪽으로 치우친 노면 + bias measured)가
        들어와도 apply가 중앙 유지하는가?
  D5.4  오래 (예: 5분) 직진 주행해도 apply 누적 드리프트가 없는가?
  D5.5  op가 작은 correction (±1°)을 번갈아 보낼 때 apply mean이 0인가?

판정 기준:
  - apply_angle mean |·| < 0.05° (수치적으로 0 주변)
  - 좌/우 비대칭 < 1% (max|L| - max|R|) / max|L|
  - 누적 적분값 증가율 ≈ 0 (linear regression slope ≈ 0)
"""
import sys
import numpy as np

sys.path.insert(0, '/home/user/openpilot/tools')
from ioniq6n_phase4_pipeline import Phase4Sim


def run_drive(op_profile, v_kmh, noise_profile=None, measured_bias=0.0,
              duration_s=None, dt=0.01):
  """공통 runner. op_profile 길이가 N 이면 simulation duration = N*dt."""
  N = len(op_profile)
  if noise_profile is None:
    noise_profile = np.zeros(N)
  sim = Phase4Sim()
  # ACI 래치 + 속도 안정화
  for i in range(80):
    sim.step(v_ego_raw=v_kmh / 3.6, steering_angle_deg=0.0,
             steering_torque=0.0, blinker=False, lat_active=True,
             op_angle_cmd=0.0, cam_counter=(i // 2) % 16)
  apply_angles = []
  physical = 0.0
  for i in range(N):
    # 실제 차량의 hand-wheel은 apply를 약간 지연해서 따라감
    physical = 0.92 * physical + 0.08 * sim.apply_angle_last
    measured = physical + measured_bias + noise_profile[i]
    out = sim.step(
      v_ego_raw=v_kmh / 3.6, steering_angle_deg=measured,
      steering_torque=0.0, blinker=False, lat_active=True,
      op_angle_cmd=float(op_profile[i]),
      cam_counter=((80 + i) // 2) % 16,
    )
    apply_angles.append(out["apply_angle"])
  return np.array(apply_angles)


def scenario_d5_1_straight_with_noise():
  """직진 5분, op=0°, 노면 노이즈는 평균 0인 white noise."""
  N = 30000  # 300s = 5분
  op = np.zeros(N)
  np.random.seed(1)
  noise = 0.2 * np.random.randn(N)  # zero-mean white noise
  apply = run_drive(op, v_kmh=80, noise_profile=noise)
  mean = float(np.mean(apply))
  std = float(np.std(apply))
  # 선형 회귀로 드리프트 체크
  t = np.arange(N)
  slope = float(np.polyfit(t, apply, 1)[0])
  drift_per_min = slope * 6000  # per minute (6000 * 10ms = 60s)
  verdict = "✅ 중앙 유지" if abs(mean) < 0.05 and abs(drift_per_min) < 0.01 else "❌ 드리프트"
  print(f"  D5.1 직진 5분 + zero-mean 노이즈 @ 80 km/h")
  print(f"       apply mean     = {mean:+.4f}°   (기준 |mean| < 0.05°)")
  print(f"       apply std      = {std:.4f}°")
  print(f"       드리프트/분    = {drift_per_min:+.4f}°/min   (기준 < 0.01)")
  print(f"       max |apply|    = {np.max(np.abs(apply)):.3f}°   {verdict}")
  return dict(mean=mean, drift=drift_per_min, ok=(abs(mean) < 0.05 and abs(drift_per_min) < 0.01))


def scenario_d5_2_symmetry():
  """좌/우 대칭 op 입력. 시간 t에 대해 op(t) = sin(...)."""
  N = 3000
  t = np.arange(N) * 0.01
  op = 15.0 * np.sin(2 * np.pi * 0.1 * t)  # ±15° @ 0.1 Hz
  for v_kmh in [30, 60, 100]:
    apply = run_drive(op, v_kmh=v_kmh)
    pos = apply[apply > 0]
    neg = apply[apply < 0]
    max_pos = float(np.max(pos)) if len(pos) else 0.0
    max_neg = float(abs(np.min(neg))) if len(neg) else 0.0
    mean_pos = float(np.mean(pos)) if len(pos) else 0.0
    mean_neg = float(abs(np.mean(neg))) if len(neg) else 0.0
    asym = 100 * abs(max_pos - max_neg) / max(max_pos, max_neg, 1e-3)
    mean_asym = 100 * abs(mean_pos - mean_neg) / max(mean_pos, mean_neg, 1e-3)
    verdict = "✅" if asym < 1.0 and mean_asym < 2.0 else "❌"
    print(f"       v={v_kmh:3d}km/h  max L={max_pos:.3f}° / R={max_neg:.3f}°  "
          f"비대칭 {asym:.2f}%   mean {mean_pos:.3f}/{mean_neg:.3f}  {verdict}")


def scenario_d5_3_biased_noise():
  """+쪽으로 편향된 노이즈 (예: 도로 cant = 노면 기울기 +1° bias).
  op는 이를 cancel 하기 위해 -1°를 보낸다고 가정. apply가 실제로 -1° 근처에
  유지되는가? (VM이 op 명령을 정확히 따라가는지)."""
  N = 3000
  op = -1.0 * np.ones(N)  # op가 cant 보상용 -1° 요청
  np.random.seed(7)
  # 측정값에 +1° bias (도로 기울기) + random noise
  noise = 1.0 + 0.2 * np.random.randn(N)
  apply = run_drive(op, v_kmh=60, noise_profile=noise)
  mean = float(np.mean(apply))
  verdict = "✅" if abs(mean - (-1.0)) < 0.1 else "❌"
  print(f"\n  D5.3 도로 cant 보상 (op=-1°로 +1° bias cancel)")
  print(f"       apply mean = {mean:+.3f}°   (목표=-1.00°, 기준 오차<0.1°)   {verdict}")
  return dict(mean=mean, ok=abs(mean - (-1.0)) < 0.1)


def scenario_d5_4_long_drive():
  """15분 연속 주행 누적 적분값."""
  N = 90000  # 15 분
  np.random.seed(42)
  # op는 0을 중심으로 좌우 반복 (실주행의 작은 보정)
  t = np.arange(N) * 0.01
  op = 0.5 * np.sin(2 * np.pi * 0.05 * t) + 0.2 * np.sin(2 * np.pi * 0.3 * t)
  noise = 0.15 * np.random.randn(N)
  apply = run_drive(op, v_kmh=90, noise_profile=noise)
  # 누적 적분: 왼쪽/오른쪽 시간 균형
  left_time = (apply < -0.1).sum() * 0.01
  right_time = (apply > 0.1).sum() * 0.01
  center_time = (np.abs(apply) <= 0.1).sum() * 0.01
  total = N * 0.01
  mean = float(np.mean(apply))
  integral = float(np.sum(apply) * 0.01)  # °·s
  verdict = "✅" if abs(mean) < 0.05 else "❌"
  print(f"\n  D5.4 15분 연속 주행 @ 90 km/h (op: ±0.5° 보정, 노이즈 포함)")
  print(f"       apply mean     = {mean:+.4f}°")
  print(f"       누적 적분      = {integral:+.2f}°·s  (짧을수록 좋음)")
  print(f"       L/center/R time = {left_time:.1f}s / {center_time:.1f}s / {right_time:.1f}s"
        f"  ({total:.0f}s total)")
  print(f"       time balance   = L-R {left_time - right_time:+.1f}s   {verdict}")
  return dict(mean=mean, integral=integral, ok=abs(mean) < 0.05)


def scenario_d5_5_small_corrections():
  """±1° correction이 번갈아 들어옴. mean 중립성 확인."""
  N = 6000
  t = np.arange(N) * 0.01
  # 0.3Hz 사각파 비슷한 ±1°
  op = 1.0 * np.sign(np.sin(2 * np.pi * 0.3 * t))
  apply = run_drive(op, v_kmh=60)
  mean = float(np.mean(apply))
  # 양수/음수 시간 균형
  pos_t = (apply > 0.1).sum() * 0.01
  neg_t = (apply < -0.1).sum() * 0.01
  verdict = "✅" if abs(mean) < 0.05 and abs(pos_t - neg_t) < 0.5 else "❌"
  print(f"\n  D5.5 ±1° correction 번갈아 (0.3 Hz 사각파)")
  print(f"       apply mean = {mean:+.4f}°   pos/neg time = {pos_t:.1f}/{neg_t:.1f} s   {verdict}")
  return dict(mean=mean, ok=abs(mean) < 0.05)


if __name__ == "__main__":
  print("=" * 80)
  print("  Part 5 평가: 차선 중앙 유지 (한쪽 쏠림 없음)")
  print("=" * 80)
  print()
  r1 = scenario_d5_1_straight_with_noise()
  print("\n  D5.2 좌/우 대칭성 (op = ±15° @ 0.1 Hz sin)")
  scenario_d5_2_symmetry()
  r3 = scenario_d5_3_biased_noise()
  r4 = scenario_d5_4_long_drive()
  r5 = scenario_d5_5_small_corrections()
  print("\n" + "=" * 80)
  all_ok = r1["ok"] and r3["ok"] and r4["ok"] and r5["ok"]
  if all_ok:
    print("  ✅ 좌/우 편향 없음 — 차선 중앙을 잘 유지함")
  else:
    print("  ❌ 일부 시나리오에서 bias 감지")
  print("=" * 80)
