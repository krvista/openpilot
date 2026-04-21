# Route 0x49 (433dad5bb2) 증상 분석 보고서

Route: `99b215d21bbf8735_00000049--433dad5bb2` (35 segments, ~10분)
빌드: Phase 7 + MDPS 진단 필드 추가 이후
분석일: 2026-04-22


## 사용자 보고 증상

- **A**: 일시적 ADAS 오류 + LFA 아이콘이 계기판에서 꺼짐 (여러 번)
- **B**: ACC ON + LFA ON 인데도 LFA 녹색이 아니라 흰색, 스티어링 어시스트 미동작
- **C**: 차선 변경 시 자연스러운 핸들 넘김 실패, 다른 차선으로 넘어감


## 핵심 발견: 드라이버 오버라이드 임계값 오보정

### 증거

앵글 제어 MDPS의 `STEERING_COL_TORQUE` 분포 (latActive=True 구간):

| 조건 | p50 | p75 | p90 | max |
|------|-----|-----|-----|-----|
| steeringPressed=False (가볍게 잡고 있음) | **36** | **92** | **184** | 601 |
| steeringPressed=True (드라이버가 돌리는 중) | 381 | 488 | 619 | 895 |
| 전체 latActive | **67** | **264** | **428** | 895 |

현재 오버라이드 설정 (`values.py`):

| 파라미터 | 값 | 문제 |
|----------|-----|------|
| `DRIVER_TORQUE_DEADZONE` | 25 | 가볍게 잡기만 해도 p50=36 > 25 → 오버라이드 시작 |
| `FULL_OVERRIDE_LOW_V` (≤29 km/h) | 60 | **불잡기 p75=92 > 60** → 드라이버 손만 얹어도 full override |
| `FULL_OVERRIDE_HIGH_V` (≥54 km/h) | 120 | 가볍게 잡기 p90=184 > 120 → 고속에서도 full override |

### 결과

```
driver_torque_blend (DTB) 분포 (latActive=True):
  DTB = 0.0  (완전 override):  47.0%  ← 거의 절반!
  DTB < 0.30 (authority 부족): 50.7%
  DTB = 1.0  (override 없음):  28.5%
```

**latActive=True이지만 steering_active=False인 프레임: 50,170개 (23.9%)**

| 원인 | 건수 | 비율 |
|------|------|------|
| `driver_override` (DTB 붕괴로 authority=0) | 37,629 | **75.0%** |
| `blinker_authority_trap` (블링커 0.2배 × 낮은 DTB) | 12,193 | 24.3% |
| `low_speed_blend` (1-3 km/h 구간) | 348 | 0.7% |

### 왜 이런 일이 발생하는가

토크 제어 차량(예: 아이오닉 5 N)에서 `STEERING_COL_TORQUE`는 **순수
드라이버 입력**이다 — EPS가 어시스트를 추가하기 전의 토션바 토크. 드라이버가
핸들에 손만 올리면 0-10 Nm 수준.

앵글 제어 차량(아이오닉 6 N)에서 `STEERING_COL_TORQUE`에는 **MDPS가 앵글
명령을 실행하면서 생기는 반력**이 포함된다. MDPS가 목표 각도로 스티어링을
돌릴 때 토션바에 힘이 걸리고, 이것이 컬럼 토크로 잡힌다. 드라이버가
핸들에 가볍게 손을 올리기만 해도 중앙값 36 Nm, 커브 구간에서는 92+ Nm이
보고된다.

기존 임계값(DZ=25, Full=60/120)은 토크 제어 기준으로 설정되어 있어서,
앵글 제어 플랫폼에서는 **드라이버가 오버라이드하지 않는데 오버라이드로
판정**한다.

### MDPS 피드백 교차 검증

```
mdpsLkaAngleActive (MDPS가 보는 상태):
  latActive=True & mdpsLkaAngleActive=2 (능동): 46,015 (71.2%)
  latActive=True & mdpsLkaAngleActive=1 (수동): 18,660 (28.8%)  ← op가 steering하고 싶은데 MDPS는 수동
```

이는 정확히 steering_active=False 비율(23.9%)과 일치한다. op가
`LKAS_ANGLE_ACTIVE=1`(수동)을 보내면 MDPS도 수동으로 전환한다.


## 증상별 근본 원인

### Symptom A: ADAS 오류 일시 발생

- **onroadEvents 분석**: `commIssue` 이벤트 17건, 전부 **t=11~20초 (부팅 직후)**
- **mid-drive ADAS 이벤트**: 0건
- **판정**: 부팅 시 selfdrived 초기화 과정에서의 일시적 commIssue.
  사용자가 mid-drive에서 느낀 "ADAS 오류"는 Symptom B (흰색 LFA 아이콘 +
  스티어링 무반응)를 ADAS 오류로 인식한 것일 가능성 높음.

### Symptom B: 흰색 LFA 아이콘 + 스티어링 어시스트 미동작

- **근본 원인 확정**: `DRIVER_TORQUE` 임계값 오보정.
  - 드라이버가 핸들에 손만 올려도 DTB가 0.0으로 붕괴 (47%)
  - authority=0.0 → aci_active_latched=False → steering_active=False
  - LKAS_ANGLE_ACTIVE=1 (수동), ACIGain=카메라 값(≈0) 송신
  - MDPS 수동 모드 → 스티어링 어시스트 없음 → 흰색 아이콘
- **카메라 staleness**: 0건 (원인 아님)
- **저속 passthrough latch**: 54회 전환, 348 프레임 (미미)

### Symptom C: 차선 변경 핸들 넘김 실패

- **근본 원인 확정**: Symptom B의 합성 효과.
  - 24개 블링커 이벤트 중 **21개는 블링커 시작 전에 이미 aci_active_latched=False**
  - 이유: 드라이버가 손을 잡고 있어서 DTB 붕괴 → 블링커와 무관하게 이미 수동 상태
  - 나머지 3개만 "blinker trap" (블링커 중에 aci_active에서 탈락)
  - 블링커 0.2배 감쇠는 **부차적 문제** — 메인 이슈는 DTB 붕괴
- **Lane change state 분포**: `laneChangeStarting`: 996프레임,
  `laneChangeFinishing`: 216, `preLaneChange`: 358. LC 자체는 planner가
  시도하지만 steering_active=False라 MDPS가 실행하지 않음.


## 권장 수정 (사용자 승인 필요)

### 수정 1 (MUST): 앵글 제어 플랫폼 오버라이드 임계값 재보정

파일: `opendbc_repo/opendbc/car/hyundai/values.py:117-121`

```python
# 현재 (토크 제어 기준)
DRIVER_TORQUE_DEADZONE = 25.0
DRIVER_TORQUE_FULL_OVERRIDE_LOW_V  = 60.0
DRIVER_TORQUE_FULL_OVERRIDE_HIGH_V = 120.0

# 제안 (앵글 제어 전용 — 별도 상수 or 조건 분기)
DRIVER_TORQUE_DEADZONE_ANGLE = 100.0        # 손 올리기 p50=36 < 100 → 오버라이드 0
DRIVER_TORQUE_FULL_OVERRIDE_LOW_V_ANGLE  = 300.0   # 실제 잡기 p25≈250 → 시작
DRIVER_TORQUE_FULL_OVERRIDE_HIGH_V_ANGLE = 500.0   # 실제 잡기 p50≈381 → 완료
```

예상 효과 (동일 route 시뮬레이션):
- DTB=0.0 비율: 47% → ~15%
- DTB=1.0 비율: 28.5% → ~65%
- steering_active=False (latActive=True 중): 23.9% → ~5%

### 수정 2 (SHOULD): 블링커 authority 감쇠 op-LC 구분

파일: `opendbc_repo/opendbc/car/hyundai/carcontroller.py:382-383`

수정 1로 DTB 붕괴가 해소되면 3개 blinker trap도 대부분 해결되지만,
안전망으로 추가:

```python
# 현재
if blinker_on:
    authority *= 0.2

# 제안: 드라이버가 실제로 override 의사를 보일 때만 감쇠
if blinker_on and driver_torque_blend < 0.7:
    authority *= 0.3
```

### 수정 3 (SKIP): 카메라 staleness / 저속 passthrough

이번 route에서 각각 0건, 348프레임으로 증상 기여도 미미. 다음 분석까지 보류.


## 참고

- 분석 도구: `tools/ioniq6n_route49_symptom_correlation.py`
- EPS 출력 토크 (`STEERING_OUT_TORQUE`): p50=3, p90=12 — 이 값은 매우
  낮아서 override 감지에 부적합 (드라이버 vs MDPS 구분 불가)
- `steeringPressed` threshold = 250 (CAN-FD 기본) → 이미 column torque가
  높은 환경을 어느 정도 인지하고 있으나, override 쪽에는 반영 안 됨
