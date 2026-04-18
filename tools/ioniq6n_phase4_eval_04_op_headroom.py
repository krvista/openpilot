#!/usr/bin/env python3
"""Phase 4 평가 Part 4 — op가 해결 가능한 각도에 경고 뜨는지 검증.

평가 항목:
  "충분히 op가 해결할 수 있는 스티어링 각도에도 경고가 뜨는 경우는 없는지"

이 시스템에서 "경고/제한" 발생 경로:
  (a) apply_steer_angle_limits_vm: 목적 각도가 limits.STEER_ANGLE_MAX(±176.7°) 초과
      → 하드 clip. op 실력 밖이면 적절, 실력 안쪽이면 false positive.
  (b) VM의 물리적 lateral accel 캡 (MAX_LATERAL_ACCEL=3.3 m/s²).
      → 고속에서 op가 "충분히 안전한" 각도라고 생각해도 물리적으로 큰 lateral g가
         발생하는 각도는 clip.
  (c) MAX_ANGLE=85° + MAX_ANGLE_FRAMES=89 프레임 유지 시 EPS fault 방지 cut
      (common_fault_avoidance) → op가 급커브를 지속하면 발동.
  (d) aci_active_latched가 false인 상태에서는 apply가 사용자 휠 각도 tracking
      → op가 열심히 계산해도 TX 안 됨. 운전자 관점에선 "경고 없이 무시됨".

이 스크립트는 op가 "실현 가능한" 수준의 각도를 요청했을 때 어디서 cut/clip
이 발생하는지 속도별로 측정한다.

테스트 시나리오:
  C4.1  속도별 op 명령 vs 실제 출력 각도 스윕.
        op = 0°, 5°, 10°, 20°, 40°, 80°, 160° 각각에 대해,
        속도 10/30/60/100 km/h에서 실제 apply 크기 비교.
        ▶ 'op의 요청이 잘려서 경고(ISO 초과 감속/경보) 트리거되는 경계'를 찾음.

  C4.2  op가 실제 도로에서 만들 수 있는 최대 각도 (합리적 커브) 테스트.
        시립 내 최소회전반경 = 11m (i6n 기준), 120km/h 고속 최소 R=100m 가정.
        → 해당 각도가 VM cap에 걸리는지 체크.

  C4.3  MAX_ANGLE=85° EPS fault 방지 cut 트리거 조건 검증.
        → op가 85° 이상 유지 시 `apply_steer_req`가 false로 빠지는 프레임 수 측정.

  C4.4  op가 급격하게 각도를 늘릴 때 (lane change 시작):
        4° → 30° 1.5초 램프를 속도 60/80/100/120 km/h에서 테스트.
        VM rate limit이 op의 자연스러운 ramp를 방해하는지 확인.
"""
import sys
import math
import numpy as np

sys.path.insert(0, '/home/user/openpilot/tools')
from ioniq6n_phase4_pipeline import Phase4Sim, make_ioniq6n_cp
from opendbc.car.vehicle_model import VehicleModel
from opendbc.car.hyundai.values import CarControllerParams
from opendbc.car.lateral import apply_steer_angle_limits_vm


CP = make_ioniq6n_cp()
VM = VehicleModel(CP)
PARAMS = CarControllerParams(CP)


def scenario_c4_1_sweep():
  """속도 × op 명령 스윕: 실제 출력 각도와 clip 여부 측정."""
  speeds_kmh = [10, 30, 60, 100]
  op_angles = [0, 5, 10, 20, 40, 80, 160]

  print("  C4.1 속도별 op 요청 → 실제 출력 (충분 시간 후 stable 값):")
  print(f"  {'op 각도 (°)':<14}", end="")
  for v in speeds_kmh:
    print(f"{v:>4d} km/h  ", end="")
    print(f"{'clip?':>7}", end="")
  print()
  print("  " + "─" * 80)

  any_false_positive = False

  for op_ang in op_angles:
    row = f"  {op_ang:<14.0f}"
    for v_kmh in speeds_kmh:
      sim = Phase4Sim()
      v_ms = v_kmh / 3.6
      # ACI 래치
      for i in range(50):
        sim.step(v_ego_raw=v_ms, steering_angle_deg=0.0,
                 steering_torque=0.0, blinker=False, lat_active=True,
                 op_angle_cmd=0.0, cam_counter=(i // 2) % 16)
      # op가 op_ang° 지속 요청
      for i in range(1000):
        sim.step(v_ego_raw=v_ms, steering_angle_deg=sim.apply_angle_last,
                 steering_torque=0.0, blinker=False, lat_active=True,
                 op_angle_cmd=float(op_ang), cam_counter=((50 + i) // 2) % 16)
      actual = sim.apply_angle_last
      clipped = abs(abs(actual) - op_ang) > 0.5
      # ISO 11270 기준 합리적 각도: a_lat ≤ 3.0 m/s² 이어야 함.
      # 해당 op_ang에서의 실제 lateral accel 추산 (작은각 근사):
      curv = (op_ang * math.pi / 180) / 14.26 / 2.965  # rad/m
      a_lat = v_ms ** 2 * curv
      reasonable = a_lat <= 3.0
      flag = "❌" if (clipped and reasonable) else ("✓" if clipped else " ")
      if clipped and reasonable:
        any_false_positive = True
      row += f"{actual:>10.2f}°"
      row += f"{flag:>6}"
    print(row)

  print("\n  판독:")
  print("   - 표시 없음: op 요청대로 출력 (clip 없음, 정상).")
  print("   - ✓: clip 됨, ISO 3.0 m/s² 초과 요청 (정당한 안전 clip).")
  print("   - ❌: clip 됨, ISO 3.0 m/s² 이내 요청 (false positive).")
  return not any_false_positive


def scenario_c4_2_realistic_turns():
  """실제 도로 커브 시나리오에서 VM cap에 걸리는지."""
  print("\n  C4.2 실제 주행 커브 기준 체크:")

  # 시나리오 데이터: (상황, 속도 km/h, 커브 반경 m)
  scenarios = [
    ("주차장 U턴",        5,   5.5),    # i6n min radius ~5.5m
    ("교차로 우회전",     20,  15.0),
    ("도시 우회전",       30,  20.0),
    ("고속도로 램프",     50,  60.0),
    ("고속도로 완만 커브", 80,  400.0),
    ("고속 S코너(일반)",  100, 400.0),  # 일반적 고속 S (ISO 이내)
    ("lane change",       100, 500.0),
  ]
  false_positives = 0
  for name, v_kmh, radius_m in scenarios:
    v_ms = v_kmh / 3.6
    curvature = 1.0 / radius_m
    sw_angle = math.degrees(VM.get_steer_from_curvature(curvature, v_ms, 0.0))
    max_curv_cap = PARAMS.ANGLE_LIMITS.MAX_LATERAL_ACCEL / max(v_ms ** 2, 1.0)
    max_angle_cap = math.degrees(VM.get_steer_from_curvature(max_curv_cap, v_ms, 0.0))
    max_angle_cap = min(max_angle_cap, PARAMS.ANGLE_LIMITS.STEER_ANGLE_MAX)
    headroom_pct = (max_angle_cap - abs(sw_angle)) / max_angle_cap * 100
    fits_vm = abs(sw_angle) <= max_angle_cap
    a_lat = v_ms ** 2 / radius_m
    within_iso = a_lat <= 3.0
    # STEER_ANGLE_MAX(176.7°)는 DBC `ADAS_StrAnglReqVal` 시그널 한계이므로
    # 이 자체를 초과하는 요청은 플랫폼 한계 (op이 어느 구현이든 불가) → FP 제외.
    within_dbc = abs(sw_angle) <= 176.7
    false_positive = (not fits_vm) and within_iso and within_dbc
    if false_positive:
      false_positives += 1
    if false_positive:
      flag = "❌FP"
    elif not within_dbc:
      flag = "DBC+"  # DBC 신호 한계 초과 (플랫폼 한계)
    elif not within_iso:
      flag = "ISO+"  # ISO 승차감 한계 초과 (정당 clip)
    else:
      flag = "✅"
    print(f"       {name:<20} v={v_kmh:>3}km/h R={radius_m:>5.1f}m  "
          f"필요={sw_angle:>6.2f}°  VM cap={max_angle_cap:>6.2f}°  "
          f"headroom={headroom_pct:>5.1f}%  a_lat={a_lat:.1f}m/s²  {flag}")
  return false_positives == 0


def scenario_c4_3_eps_fault_cut():
  """MAX_ANGLE=85°, MAX_ANGLE_FRAMES=89 EPS fault cut 조건 트리거 체크."""
  MAX_ANGLE = 85
  MAX_ANGLE_FRAMES = 89  # ~0.89s at 100Hz
  print("\n  C4.3 EPS fault 방지 cut 조건 (> 85° 지속):")

  # 85° 이상 각도를 지속할 수 있는 실제 시나리오?
  # i6n 스티어링 휠 lock-to-lock ≈ 475° (실측), max 명령 ±176.7° (DBC cap)
  # 85°는 whether angle — 고속에선 거의 나오지 않음 (VM이 먼저 잘라냄).

  # 저속에서만 의미 있음.
  for v_kmh in [5, 20, 40, 80]:
    v_ms = v_kmh / 3.6
    # op가 100°를 지속 요구
    sim = Phase4Sim()
    for i in range(50):
      sim.step(v_ego_raw=v_ms, steering_angle_deg=0.0, steering_torque=0.0,
               blinker=False, lat_active=True, op_angle_cmd=0.0,
               cam_counter=(i // 2) % 16)
    # 300 프레임 지속 request
    over_85_frames = 0
    for i in range(300):
      sim.step(v_ego_raw=v_ms, steering_angle_deg=sim.apply_angle_last,
               steering_torque=0.0, blinker=False, lat_active=True,
               op_angle_cmd=100.0, cam_counter=((50 + i) // 2) % 16)
      if abs(sim.apply_angle_last) > MAX_ANGLE:
        over_85_frames += 1
    max_reached = sim.apply_angle_last
    # v >= 40 km/h에선 VM accel cap이 85° 훨씬 아래로 끊음
    would_trigger_cut = over_85_frames > MAX_ANGLE_FRAMES
    print(f"       v={v_kmh:>3} km/h  op=100°  실제 출력={max_reached:>6.2f}°  "
          f">85° 프레임 수 = {over_85_frames}  "
          f"{'⚠ EPS cut 트리거 예상' if would_trigger_cut else '✓ cut 불필요 (VM이 이미 clip)'}")
  return True  # informational only


def scenario_c4_4_lane_change_rate():
  """lane change 각도 ramp가 VM rate limit 때문에 느려지는지."""
  print("\n  C4.4 Lane change (4° → 30° 1.5s ramp) VM 추종 여부:")
  for v_kmh in [60, 80, 100, 120]:
    v_ms = v_kmh / 3.6
    sim = Phase4Sim()
    for i in range(50):
      sim.step(v_ego_raw=v_ms, steering_angle_deg=0.0, steering_torque=0.0,
               blinker=False, lat_active=True, op_angle_cmd=0.0,
               cam_counter=(i // 2) % 16)
    # op: 0 → 30° 1.5초 램프 (typical lane change planner signature)
    N = 150
    apply_trace = []
    op_trace = []
    for i in range(N):
      op = 30.0 * min(i / 150, 1.0)
      sim.step(v_ego_raw=v_ms, steering_angle_deg=sim.apply_angle_last,
               steering_torque=0.0, blinker=False, lat_active=True,
               op_angle_cmd=op, cam_counter=((50 + i) // 2) % 16)
      apply_trace.append(sim.apply_angle_last)
      op_trace.append(op)
    # 최종 apply 값
    final_apply = apply_trace[-1]
    final_op = op_trace[-1]
    # VM cap이 ≥ 30°이면 통과, 아니면 cap 값
    max_curv_cap = PARAMS.ANGLE_LIMITS.MAX_LATERAL_ACCEL / v_ms**2
    vm_cap = math.degrees(VM.get_steer_from_curvature(max_curv_cap, v_ms, 0))
    vm_cap = min(vm_cap, PARAMS.ANGLE_LIMITS.STEER_ANGLE_MAX)
    # 평균 lateral accel 계산 (실측 apply 기준)
    curv_actual = final_apply * math.pi / 180 / 14.26 / 2.965 if final_apply else 0
    a_lat_actual = v_ms ** 2 * curv_actual
    # lag = op와 apply의 시간 차 (ramp 중 얼마나 뒤처지는지)
    apply_trace_np = np.array(apply_trace)
    op_trace_np = np.array(op_trace)
    # op=15°에 도달하는 시점 대비 apply=15° 도달 시점
    try:
      op_15_idx = next(i for i, o in enumerate(op_trace) if o >= 15)
      apply_15_idx = next(i for i, a in enumerate(apply_trace) if a >= 15)
      lag_ms = (apply_15_idx - op_15_idx) * 10
    except StopIteration:
      lag_ms = -1  # apply didn't reach 15°
    print(f"       v={v_kmh:>3} km/h  op 요청 {final_op:.1f}° → apply {final_apply:>6.2f}° "
          f"(VM cap {vm_cap:.1f}°, a_lat ≈ {abs(a_lat_actual):.2f} m/s²)  "
          f"lag={lag_ms}ms")
  return True


if __name__ == "__main__":
  print("=" * 80)
  print("  Part 4 평가: op가 해결 가능한 각도인데 경고/제한 발생 여부")
  print("=" * 80)
  print()
  r1 = scenario_c4_1_sweep()
  r2 = scenario_c4_2_realistic_turns()
  r3 = scenario_c4_3_eps_fault_cut()
  r4 = scenario_c4_4_lane_change_rate()
  print("\n" + "=" * 80)
  all_ok = r1 and r2 and r3 and r4
  if all_ok:
    print("  ✅ op 가능 각도에 대한 false-positive 경고/제한 없음")
  else:
    print("  ❌ 일부 합리적 각도에서 cut/clip 발생 (상세 로그 참조)")
  print("=" * 80)
  sys.exit(0 if all_ok else 1)
