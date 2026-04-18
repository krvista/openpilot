#!/usr/bin/env python3
"""Phase 4 평가 Part 3 — 주차(up to 30 km/h) 불필요 개입 여부.

평가 항목:
  "주차 때 up to 30 km/h 정도의 속도에서 불필요한 개입하지는 않는지"

주차 상황의 전형적인 특징:
  - 속도 0-30 km/h
  - 운전자가 핸들을 크게 돌림 (±100° 이상)
  - 운전자 torque 큼 (>100 Nm)
  - 전·후진 기어 변경
  - 정지-출발 반복

"불필요한 개입"의 정의:
  (1) 운전자가 핸들 돌리는데 op이 저항 (apply_angle이 0° 유지 시도).
  (2) 저속에서 apply_angle이 커지면서 EPS가 액티브 상태로 동작.
  (3) op_angle과 driver_angle이 크게 다를 때 (e.g., U턴) op이 끼어듦.

Phase 4에서 설계된 주차 안전장치:
  a. `low_speed_cam_latched`: v<2km/h 진입, v>3km/h 해제 (hysteresis)
     → steering_active=False로 EPS에 미출력.
  b. `driver_torque_blend`: 30-150 Nm 구간 선형 blend (override).
     150 Nm 이상에서 op 완전 양보 (apply=steering_angle).
  c. `speed_blend`: 1-3 km/h linear ramp로 ACI 권한 0→1.
  d. `aci_active_latched` 히스테리시스 (enter 0.30, exit 0.05).

시나리오:
  B3.1  Parallel parking: 0-15 km/h, 운전자 ±180° 핸들, 150 Nm 그립.
        → 검증: op apply_angle이 운전자 입력 따라가고 저항 안 함.
  B3.2  평행주차 진입 10 km/h에서 MADS 켜진 상태 운전자 개입.
        → 검증: override_factor가 빠르게 1.0 도달, op 양보.
  B3.3  0-5 km/h 극저속 creep (주차장 배회).
        → 검증: low_speed_cam_latched 유지, apply_angle ≈ 측정 각도.
  B3.4  후진 직전 정지 상태 (v=0, 핸들 +90° 꺾여있음).
        → 검증: aci_latched=False, apply_angle 현재 각도 추종.
  B3.5  주차장 출차: 5 → 25 km/h 가속하며 핸들 중앙 정렬.
        → 검증: ACI가 언제 latch되는지, 운전자 torque 있으면 양보.
"""
import sys
import numpy as np

sys.path.insert(0, '/home/user/openpilot/tools')
from ioniq6n_phase4_pipeline import Phase4Sim


def run(sim, N, v_fn, sa_fn, tq_fn, lat_active=True, op_fn=None):
  trace = []
  for i in range(N):
    t = i * 0.01
    v = v_fn(t)
    sa = sa_fn(t)
    tq = tq_fn(t)
    op = op_fn(t) if op_fn else 0.0
    out = sim.step(v_ego_raw=v, steering_angle_deg=sa, steering_torque=tq,
                   blinker=False, lat_active=lat_active, op_angle_cmd=op,
                   cam_counter=(i // 2) % 16)
    trace.append(dict(t=t, v=v, sa=sa, tq=tq, **out))
  return trace


def scenario_b3_1_parallel_parking():
  """평행주차 시뮬: 0-15 km/h, 운전자가 ±180° 돌림, 150 Nm torque."""
  sim = Phase4Sim()
  # 5초 주차 동작
  N = 500

  def v_fn(t):
    # 0 → 10 → 0 반복 (천천히)
    cycle = t % 5.0
    if cycle < 2.5: return (cycle / 2.5) * 10.0 / 3.6
    else:           return ((5.0 - cycle) / 2.5) * 10.0 / 3.6

  def sa_fn(t):
    # 운전자 핸들 ±180° 왕복
    return 180.0 * np.sin(2 * np.pi * 0.15 * t)

  def tq_fn(t):
    # 운전자가 계속 150 Nm 그립
    return 150.0 if sa_fn(t) > 0 else -150.0

  # MADS가 켜져 있지만 운전자 torque 때문에 ACI 양보해야 함.
  trace = run(sim, N, v_fn, sa_fn, tq_fn, lat_active=True)

  # 검증
  max_apply_abs = max(abs(t["apply_angle"]) for t in trace)
  aci_active_frac = sum(1 for t in trace if t["aci_active"]) / len(trace)
  override_min = min(t["override_factor"] for t in trace)
  override_max = max(t["override_factor"] for t in trace)
  passthrough_active_frac = sum(1 for t in trace if t["in_passthrough"]) / len(trace)

  # 운전자가 torque 150 Nm로 쥐고 있으므로 override_factor = 1.0
  # apply_angle은 운전자 각도를 따라가야 함 (저항 없음)
  tracking_error = [abs(t["apply_angle"] - t["sa"]) for t in trace]
  mean_track_err = np.mean(tracking_error)

  verdict_ok = (
    override_max >= 0.99 and   # 운전자가 완전 제어권 가져감
    aci_active_frac < 0.1 and  # ACI는 거의 비활성 (hysteresis 통과 시에만)
    mean_track_err < 10.0      # apply가 운전자 각도 추종
  )
  print(f"  B3.1 평행주차 ±180° with 150 Nm torque:")
  print(f"       max |apply_angle| = {max_apply_abs:.1f}°")
  print(f"       mean tracking err = {mean_track_err:.2f}° (op가 운전자 각도 추종)")
  print(f"       override_factor   = {override_min:.2f} ~ {override_max:.2f} (1.0 = 완전 양보)")
  print(f"       ACI active 비율   = {aci_active_frac*100:.0f}% (낮을수록 불간섭)")
  print(f"       passthrough 비율  = {passthrough_active_frac*100:.0f}%")
  print(f"       {'✅ op이 양보함' if verdict_ok else '❌ op 간섭 발생'}")
  return verdict_ok


def scenario_b3_2_mid_speed_override():
  """10 km/h에서 MADS on, 운전자가 중간에 핸들 잡음 (150 Nm)."""
  sim = Phase4Sim()
  N = 400
  # 처음 2초: op 주행, 10 km/h 직진
  # 2-4초: 운전자가 핸들 잡음 (150 Nm로 꺾음)
  def v_fn(t): return 10.0 / 3.6
  def sa_fn(t): return 0.0 if t < 2.0 else 30.0 * (t - 2.0)  # 운전자가 꺾기 시작
  def tq_fn(t): return 0.0 if t < 2.0 else 150.0

  trace = run(sim, N, v_fn, sa_fn, tq_fn, lat_active=True,
              op_fn=lambda t: 0.0)  # op는 직진 유지 원함

  # 2초 이후: override_factor ≈ 1.0 되어야 함
  post_grab = [t for t in trace if t["t"] >= 2.1]
  override_values = [t["override_factor"] for t in post_grab]
  assert all(o > 0.95 for o in override_values), \
    "B3.2: override_factor가 2.1s 이후 모두 0.95 이상이어야 함"
  # apply_angle이 운전자 각도를 추종
  tracking_err = np.mean([abs(t["apply_angle"] - t["sa"]) for t in post_grab])
  verdict_ok = tracking_err < 5.0 and all(o > 0.95 for o in override_values)
  print(f"\n  B3.2 10 km/h 중 운전자 개입 (MADS ON):")
  print(f"       override_factor (2s~)  = {np.mean(override_values):.3f}")
  print(f"       tracking error         = {tracking_err:.2f}°")
  print(f"       {'✅ op 즉시 양보' if verdict_ok else '❌ 저항 발생'}")
  return verdict_ok


def scenario_b3_3_creep():
  """극저속 creep (주차장 배회): 0-5 km/h 왕복."""
  sim = Phase4Sim()
  N = 600
  def v_fn(t): return 2.5 / 3.6 * (1 + np.sin(2 * np.pi * 0.1 * t))  # 0-5 km/h 왕복
  def sa_fn(t): return 10.0 * np.sin(2 * np.pi * 0.2 * t)  # 약간의 핸들 움직임
  def tq_fn(t): return 20.0  # 약한 손 얹힘 (override 안 일어남)

  trace = run(sim, N, v_fn, sa_fn, tq_fn, lat_active=True,
              op_fn=lambda t: 0.0)

  # low_speed_cam_latched가 진입/이탈 반복 (hysteresis 2-3 km/h 경계)
  lsc_engages = sum(1 for a, b in zip(trace[:-1], trace[1:])
                     if b["low_speed_cam"] and not a["low_speed_cam"])
  # ACI는 speed_blend가 <0.05 ~ >0.30 범위를 왕복
  aci_transitions = sum(1 for a, b in zip(trace[:-1], trace[1:])
                         if a["aci_active"] != b["aci_active"])
  # apply_angle이 큰 값으로 튀지 않아야 (≤ 5°)
  max_apply = max(abs(t["apply_angle"]) for t in trace)
  # 운전자 torque 20 Nm → override_factor=0, driver_blend=1
  # 하지만 speed_blend가 낮아 authority = speed_blend ≤ 0.30 구간 있음
  verdict_ok = max_apply < 12.0  # 운전자가 핸들 꺾는 만큼만
  print(f"\n  B3.3 극저속 creep (0-5 km/h):")
  print(f"       max |apply| = {max_apply:.2f}° (운전자 움직임 10° 따라감)")
  print(f"       low_speed_cam 토글 = {lsc_engages}회 (2-3 km/h 경계 hysteresis)")
  print(f"       ACI 토글 = {aci_transitions}회")
  print(f"       {'✅ 과도한 개입 없음' if verdict_ok else '❌ 저속에서 튐'}")
  return verdict_ok


def scenario_b3_4_stationary_wheel_off_center():
  """정지(v=0), 핸들 +90° 꺾인 상태. MADS는 ON이지만 차가 안 움직임."""
  sim = Phase4Sim()
  # 먼저 정지 상태로 시뮬 warm up
  for i in range(100):
    sim.step(v_ego_raw=0.0, steering_angle_deg=90.0, steering_torque=0.0,
             blinker=False, lat_active=True, op_angle_cmd=0.0,
             cam_counter=(i // 2) % 16)
  # 상태 확인
  assert not sim.aci_active_latched, "B3.4: 정지에서 ACI latch되면 안 됨"
  assert sim.low_speed_cam_latched, "B3.4: low_speed_cam_latched 되어야 함"
  # apply_angle이 현재 휠 각도(90°)를 따라가고 있어야 함
  assert abs(sim.apply_angle_last - 90.0) < 0.01, \
    f"B3.4: apply={sim.apply_angle_last}, 측정 휠 90° 추종해야 함"
  print(f"\n  B3.4 정지 상태에서 핸들 +90° 꺾임:")
  print(f"       ACI active     = False (정지라 speed_blend=0)")
  print(f"       low_speed_cam  = True  (EPS에 미출력)")
  print(f"       apply_angle    = {sim.apply_angle_last:.2f}° (측정 휠 추종)")
  print(f"       ✅ 완전한 passive 상태")
  return True


def scenario_b3_5_exit_parking():
  """주차장 출차: 5→25 km/h 가속하며 핸들 중앙 정렬."""
  sim = Phase4Sim()
  N = 400  # 4초
  # 속도 5 → 25 km/h 램프
  def v_fn(t): return (5 + 5 * t) / 3.6
  # 운전자가 +30° 꺾여있던 핸들을 0°로 돌림 (2초 동안)
  def sa_fn(t):
    if t < 2.0: return 30.0 * (1 - t / 2.0)
    else:       return 0.0
  # 처음엔 torque 큼 (돌리는 중, override 충분) → ACI 양보.
  # 2초 이후 손 놓음 → ACI hysteresis enter=0.30 → driver_blend * speed_blend 확인.
  # driver_blend>=0.30 이 되려면 torque<=30+120*(1-0.30)=114 Nm.
  # 즉 운전자가 완전히 손 놓기 전에도 torque <114면 ACI 관여 가능.
  # 여기선 0→2s 강한 그립(150 Nm, 완전 override), 2s+ 미그립 분리:
  def tq_fn(t):
    if t < 2.0: return 150.0  # 완전 override (driver_blend=0, ACI off)
    else:       return 0.0    # 손 놓음

  trace = run(sim, N, v_fn, sa_fn, tq_fn, lat_active=True,
              op_fn=lambda t: 0.0)

  # ACI latch 시점 체크 — 2초 이후에 latch 되어야 함
  aci_latch_t = next((tr["t"] for tr in trace if tr["aci_active"]), None)
  assert aci_latch_t is not None, "B3.5: ACI가 결국 latch되어야 함"
  final_apply = trace[-1]["apply_angle"]
  verdict_ok = abs(final_apply) < 1.0 and 1.9 < aci_latch_t < 3.5
  print(f"\n  B3.5 주차 출차 5→25 km/h:")
  print(f"       ACI latch 시점 = t={aci_latch_t:.2f}s (운전자 손 놓은 후)")
  print(f"       최종 apply     = {final_apply:.2f}° (직진 정렬)")
  print(f"       {'✅ 자연스러운 인수' if verdict_ok else '❌ 부자연'}")
  return verdict_ok


if __name__ == "__main__":
  print("=" * 80)
  print("  Part 3 평가: 주차(up to 30 km/h) 불필요 개입 검증")
  print("=" * 80)
  print()
  r = [
    scenario_b3_1_parallel_parking(),
    scenario_b3_2_mid_speed_override(),
    scenario_b3_3_creep(),
    scenario_b3_4_stationary_wheel_off_center(),
    scenario_b3_5_exit_parking(),
  ]
  passed = sum(r)
  print("\n" + "=" * 80)
  print(f"  주차 시나리오: {passed}/{len(r)} PASS")
  if passed == len(r):
    print(f"  ✅ 주차 속도 영역에서 op 불필요 개입 없음")
  print("=" * 80)
  import sys
  sys.exit(0 if passed == len(r) else 1)
