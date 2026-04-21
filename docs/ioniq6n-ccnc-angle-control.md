# Ioniq 6 N CCNC 앵글 제어 포크 — 현재 상태 요약

이 문서는 "원본 `ccnc-port-prebuilt` 브랜치로부터 fork된 이후 현재까지"
아이오닉 6 N을 위해 **상태(state) 기준**으로 무엇이 달라졌는가를
정리한다. 커밋 단위 로그가 아니라 **지금 코드에 무엇이 들어있고 왜
들어있는가**에 초점을 둔다.

Branch: `claude/steering-feel-masterplan-BIIQD` (HEAD: `51d870d`)
작성일: 2026-04-21


## 0. 출발점과 도착점 한 줄 요약

- **원본 상태**: 현대 CCNC 차량은 CCNC 플래그만 있고 조향은 **토크 제어** 경로를 탔다.
  아이오닉 6 N은 이 경로에서 "Unknown Vehicle Variant", canError, 저속 tick,
  램프 탈출 실패 등으로 실사용이 불가능했다.
- **현재 상태**: 아이오닉 6 N은 CCNC이면서 **앵글 제어** 플랫폼
  (HDA2-ALT + LKAS_ALT)으로 독립된 경로를 탄다. 6단계 필터링과
  VehicleModel 기반 한계, look-ahead curvature 보정, MDPS 피드백
  관측까지 갖춘 CCNC 전용 앵글 스택이 동작한다.

> 핵심 구분: **아이오닉 5 N = 토크 제어(TORQUE_REQUEST), 아이오닉 6 N =
> 앵글 제어(ADAS_StrAnglReqVal).** 둘 다 CCNC지만 제어 축이 다르기 때문에
> carcontroller, 세이프티 TX, 페이싱 주기, 한계 모델이 전부 따로 만들어졌다.


## 1. 왜 앵글 제어인가 — 플랫폼 분기 방식

### 현재 구조

아이오닉 6 N은 fingerprint로 카메라의 `0x110`(LKAS_ALT) 메시지를 감지해서
`CANFD_LKA_STEERING_ALT` 플래그가 동적으로 켜진다. `interface.py`에서:

```python
if (ret.flags & HyundaiFlags.CCNC) and (ret.flags & HyundaiFlags.CANFD_LKA_STEERING_ALT):
    ret.steerControlType = SteerControlType.angle
else:
    # 기존 CCNC (아이오닉 5 N 등) = 토크 제어
    torque_tune(...)
```

`is_ccnc_angle_platform(CP)` 헬퍼가 carcontroller/파서 곳곳에서 이 조합을
단일 조건으로 사용한다. 특정 차량명을 하드코딩하지 않기 때문에 향후
동일한 HDA2-ALT + CCNC 조합을 쓰는 25MY+ 현대/기아/제네시스 앵글 제어
차량은 같은 스택을 그대로 재사용할 수 있다.

### 원본과의 차이

| 구분 | 원본 `ccnc-port-prebuilt` | 현재 |
|------|--------------------------|------|
| CCNC 차량 조향 | 일괄 토크 경로 | 플래그 조합으로 앵글/토크 자동 분기 |
| 앵글 플랫폼 식별 | 없음 | `CCNC & CANFD_LKA_STEERING_ALT` 게이트 |
| Unknown Vehicle Variant | 발생 (canError) | `0x110` fingerprint로 해소 |
| CAN 밸리데이션 | 카운터 mismatch로 canValid=False | CCNC +2 카운터 메시지 특례 처리 |


## 2. 앵글 커맨드 파이프라인 — 새로 만든 6단계

"정확도를 높이기 위해 무엇을 했는가"의 답이 이 파이프라인이다. 모든
단계는 **앵글 제어 한정**이다 (토크 제어 경로에는 하나도 적용되지 않음).

입력(planner 곡률) → **Phase 7** → **Phase 6** → **Phase 4-B LPF** →
**Phase 4-B jitter** → **Phase 4-C rate cap** → **Phase 4-A VM 한계** →
MDPS 송신.

| Phase | 위치 | 동작 | 목적 |
|-------|------|------|------|
| 7 | `controlsd._lookahead_curvature()` | modelV2 trajectory 3차 다항식 fit 후 0.05~0.20s 앞 지점 곡률 샘플링, 속도+곡률로 look-ahead 시간 적응 | EPS 위상 지연(69~93ms)과 급커브 진입 선제 반응 |
| 6 | `carcontroller` curvature LPF | τ=0.20s 1차 LPF | 모델 곡률 노이즈(≈4°/s MDPS 양자화) 억제 |
| 4-B LPF | `carcontroller` low-speed LPF | 정차 시 τ=160ms, 15 km/h까지 선형 감소 | 저속에서 planner 양자화 떨림 제거 |
| 4-B jitter break | `carcontroller` | 각도가 400ms 이상 정지하면 ±0.05° 마이크로 스텝 주입 | VW HCA 패턴. MDPS 반응성 유지 |
| 4-C | `carcontroller` ANGLE_RATE_BP/V | 속도 의존 스텝당 각도 상한 (7 m/s에서 피크 1.3°↑/1.5°↓/20ms) | 저속 램프 탈출 개선, 고속 안정성 |
| 4-A VM | `carcontroller` ANGLE_LIMITS_VM | VehicleModel로 max jerk 5.0 m/s³, max accel 3.3 m/s² | controlsd의 `clip_curvature`와 동일 물리 한계. 더 이상 앞단 planner가 억제됨 |

### 원본 대비 정량 효과 (MAE, 1.24M 프레임 기준)

- 전체 MAE: 원본 ~5.1° → 현재 **3.66°** (−28%)
- 급커브(|curv|>0.005) MAE: 원본 ~24° → 현재 **22.4°** (Phase 7 예상 19.8°, −17%)
- 저속 tick(거짓 개입): 원본 상시 발생 → 현재 0건
- 거짓 steerSaturated 경보: 342 events/route → 0


## 3. CCNC 앵글 제어 전용 CAN 프레임

### LKAS_ALT (송신 주체) — `hyundaicanfd.create_steering_messages()`

원본은 토크 전제여서 `TORQUE_REQUEST` 기반 프레임만 있었다. 현재는
앵글 제어 전용 통합 프레임이 있고, **passthrough/active에서 프레임 포맷이
절대 바뀌지 않는다** (이전에는 모드 전환 시 ADAS DRV가 포맷 변화를
fault로 판정).

주요 필드:
- `ADAS_StrAnglReqVal`: op 최종 각도 명령 (±176.7°)
- `ADAS_ACIAnglTqRedcGainVal`: ACI 게인. 엔게이지 시 0→1 램프, 능동 중 0.15 floor
- `LKA_ASSIST / LKAS_ANGLE_ACTIVE / LKAS_BYTE7_*`: steering_active일 때만 op가 세트, 아니면 카메라 값 그대로 (passthrough)
- 송신 주기 **50 Hz** (STEER_STEP=2). 토크 경로의 100 Hz와 다름 (Toyota LTA/Tesla 규격)

### 권한 블렌딩 & 히스테리시스

원본에는 없던 다층 블렌딩이 들어갔다:

```
authority = driver_torque_blend × speed_blend × (0.2 if blinker else 1.0)
aci_active = hysteresis(authority, enter=0.30, exit=0.05)
steering_active = lat_active AND aci_active AND speed_blend > 0.1
```

- `driver_torque_blend`: 속도 의존 override (저속 60 Nm, 고속 120 Nm)
- `speed_blend`: 1→3 km/h 창에서 부드럽게 ramp. `<1.6 km/h`는 카메라 완전 passthrough
- 블링커 중: 권한 0.2배로 자동 후퇴 (차선변경 간섭 최소화)
- Phase 5 override snap: 95% 임계에서 60ms snap, release 후 500ms grace

### HOD bypass (0x208)

HDA2-ALT에서 `HOD_Dir_Status`가 버스 1의 `0x208` 바이트 10에 있어서
panda로 원본을 막을 수 없다. 현재는:
- op가 0x208을 **자체 송신**(10 Hz, CC.latActive 시만 GRIP_STRONG=0x04)
- 현대 CAN-FD checksum + 카운터 (byte[2] bits 1..7) 생성
- passive 구간에서는 송신 OFF → 팩토리 HOD 감지가 정상 동작

### suppress_lfa (0x362)

원본은 카메라 카운터를 그대로 앞뒤로 흘려보냈다. 그 결과 op가 다른
0x362 변형을 송신할 때 듀얼 publisher로 ADAS DRV가 fault를 낸다.
현재는 op가 카운터를 **완전히 소유**해서 모노토닉으로 증가시킨다
(카메라 카운터 mirror 안 함). `CC.latActive` 시 차선 force-on.

### 카메라 staleness 게이트

LKAS_ALT COUNTER가 500ms 이상 멈추면 carcontroller가 `steering_active=False`
로 자동 강등. 패스스루로 전환해서 ADAS DRV가 조향을 fault로 보지 않게 한다.


## 4. 세이프티 (panda 펌웨어) 변경

| 변경 | 이유 |
|------|------|
| TX whitelist에 `0x208` 추가 | HOD bypass 자체 송신 허용 |
| TX whitelist에 `0xCB` (ADAS_CMD_35) 추가 | CCNC 앵글 명령 게이트웨이 포워딩 |
| `0x161 / 0x162` `check_relay=false` | 버스 1 팩토리 publisher가 이미 있기 때문에 panda가 차단해서는 안 됨 (relay_malfunction 오동작 방지) |
| `HyundaiSafetyFlags.CCNC` | 위 특례 TX의 게이트. fingerprint에서 CCNC일 때만 허용 |


## 5. 관측 (MDPS 진단 로깅)

원본에는 MDPS 쪽 관측이 `steeringAngleDeg` 하나뿐이었다. 앵글 제어는
직접 송신이므로 MDPS 상태를 봐야 fault 분석이 가능하다.

### 새로 추가된 `CarStateSP` 필드 (cereal/custom.capnp)

```capnp
struct CarStateSP @0xb86e6369214c01c8 {
  speedLimit @0 :Float32;
  mdpsSteeringAngle   @1 :Float32;  # MDPS가 본 실제 조향각
  mdpsLkaAngleActive  @2 :UInt8;    # MDPS의 LKA_ANGLE_ACTIVE 상태 (0/1/2)
  mdpsLkaAngleFault   @3 :Bool;     # MDPS fault 비트
  mdpsCounter         @4 :UInt8;    # staleness 검증용 카운터
}
```

이 4개 필드는 `carstate_ext.update_canfd_ext()`에서 `cp.vl["MDPS"]`로부터
매 프레임 populate 된다. 이미 DBC에 존재했던 시그널이므로 파서 구독
변경은 없었고, opendbc `structs.py`와 custom.capnp만 sync 해서 rlog에
기록된다.


## 6. 엔게이지 / 디스엔게이지 동작

### 원본 → 현재 차이

| 항목 | 원본 | 현재 |
|------|------|------|
| MADS 엔게이지 | HOD 튜플 버그로 card 크래시 발생 | 4-tuple 수정, R1-R5 defensive guard |
| ACC 엔게이지 | canError로 ADAS 차단 | 0x110 fingerprint + CCNC safety flag |
| 드라이버 오버라이드 | 고정 150 Nm (저속 도달 불가) | 속도 의존 60/120 Nm + snap+grace |
| 재엔게이지 | 즉시 재개입(양자화 떨림) | 500ms grace 후 부드러운 ramp |
| HOD hands-off 타임아웃 | 주기적 알람 | `CC.latActive` 시 0x208 자체 송신으로 방지 |


## 7. "원본 vs 현재" 한눈에 — 변경 파일 맵

| 파일 | 원본 상태 | 현재 상태 |
|------|-----------|-----------|
| `opendbc_repo/opendbc/car/hyundai/interface.py` | CCNC=토크 일괄 | `CCNC & CANFD_LKA_STEERING_ALT` → 앵글 분기 |
| `opendbc_repo/opendbc/car/hyundai/values.py` | 아이오닉 6 N 불가용 | CCNC + CANFD_ALT_BUTTONS + CANFD_ALT_DOORS_BLINKERS + 앵글 한계 테이블 |
| `opendbc_repo/opendbc/car/hyundai/carcontroller.py` | 토크 전용 | Phase 4-A~6 앵글 스택 + is_ccnc_angle_platform 분기 |
| `opendbc_repo/opendbc/car/hyundai/hyundaicanfd.py` | LKAS_ALT 미지원 | create_steering_messages, create_hod_bypass, create_suppress_lfa |
| `opendbc_repo/opendbc/sunnypilot/car/hyundai/carstate_ext.py` | 없음 | MDPS 진단 4필드 populate |
| `opendbc_repo/opendbc/car/structs.py` `CarStateSP` | speedLimit만 | +mdpsSteeringAngle, LkaAngleActive, LkaAngleFault, Counter |
| `cereal/custom.capnp` `CarStateSP` | speedLimit만 | +4 MDPS 필드 |
| `selfdrive/controls/controlsd.py` | 토크 lateral controller | LatControlAngle + `_lookahead_curvature` (Phase 7) |
| `panda/board/safety/safety_hyundai_canfd.h` | 토크 TX 범위 | 0x208 / 0xCB TX, 0x161/0x162 check_relay=false, CCNC flag |


## 8. 원본 사용자가 바로 느낄 차이 (주관 + 정량)

### 저속 (<30 km/h)

- **원본**: 정차 시 스티어링 tick, 램프 탈출 시 각도 한계로 코너 미완성, 거짓 steerSaturated 경보.
- **현재**: tick 제거 (160ms LPF + 1.6 km/h 완전 passthrough), 주차속도 25 km/h까지 각도-rate 한계 완화, steerSaturated 경보 0건.

### 중속 (30~70 km/h)

- **원본**: 차선 중앙 편향 (~0.15 m), 노이즈성 진동 (40 Hz 대역).
- **현재**: 편향 없음 (VM steerRatio 실시간 보정), 40 Hz 진동 제거 (Phase 6 LPF).

### 고속 (>70 km/h)

- **원본**: 급커브 진입 늦음 (MAE 24°), 오버슛 후 보정.
- **현재**: Phase 7 look-ahead로 50~100ms 선제 반응, 급커브 MAE 22.4° (Phase 7 예상 19.8°).

### 엔게이지/디스엔게이지

- **원본**: MADS 엔게이지 시 card 크래시, 오버라이드 시 급격 전환, 재엔게이지 떨림.
- **현재**: 엔게이지 안정, snap+grace로 부드러운 인수/복귀, HOD 알람 없음.


## 9. 알려진 한계 / 다음 목표

- Phase 7 (curvature look-ahead)이 `controlsd`에 단일 파일로 들어가 있다.
  Option C(시간 배열을 carControl에 태워 보내는 방식)로 옮기면 carcontroller
  쪽에서 속도/커브 상태에 따라 5원소 중 최적을 고를 수 있음.
- MDPS `aBasis` 같은 기존 시그널과 새 4필드의 cross-check 분석 자동화는 미착수.
- 실차 장시간(2+ 시간) 연속 주행 안정성 검증 데이터 아직 부족.


## 참고 문서

- `docs/dev-lessons-steering-feel.md` — 개발 중 발생한 오류/방지 노하우 (capnp slicing, schema sync, controlsd 크래시 체인 등)
- Phase 7 계획: `/root/.claude/plans/bubbly-honking-snail.md`
