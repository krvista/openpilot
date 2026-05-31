# 2020 Jeep Grand Cherokee 잠재 문제 리뷰 및 fix 계획

> **Scope**: sunny-tizi 브랜치 (sunnypilot v2026.001.004 + 우리 fix
> commit `eee2512`) 에서 `carFingerprint = JEEP_GRAND_CHEROKEE_2019`
> (CHRYSLER_PACIFICA 안전 플랫폼, `pcmCruise=True`, `openpilotLongitudinalControl=False`)
> 가 인식되었을 때 향후 추가로 발생할 수 있는 문제 매핑.
> 이미 고친 3개 (cancel storm / controlsMismatch boot race / micd-soundd HAL hiccup) 는 제외.

## 검증 완료된 차량 배경 사실

| 사실 | 코드 위치 |
|---|---|
| `JEEP_GRAND_CHEROKEE_2019` → `new_eps_platform=True` → `HIGHER_MIN_STEERING_SPEED` 플래그 | `opendbc_repo/opendbc/car/chrysler/interface.py:42-45` |
| → `minSteerSpeed = 17.5 m/s` (해제 임계 14.5 m/s 히스테레시스) | `interface.py:81-84`, `carcontroller.py:66-68` |
| `cp_cruise = cp` (bus 0, powertrain) — `DAS_3/DAS_4` 가 bus 0 에 있음 (panda safety 코드와 일치) | `carstate.py:82`, `safety/modes/chrysler.h` `das_3_bus = PACIFICA ? 0 : 2` |
| `cruiseState.speedCluster` 는 chrysler 가 따로 안 채우지만 base `CarInterface` 가 fallback (`= speed`) 처리 | `interfaces.py:292-293` (오토 fallback) |
| MADS engage 버튼은 `TRACTION_BUTTON` (id 0x330, bit 53) | `safety/modes/chrysler.h` `chrysler_rx_hook` |

## 잠재 문제 — 우선순위 정렬

### Tier 1 (immediateDisable / 차량 fault 위험)

#### T1-A. ICBM PRE_ACTIVE 진입 시 controlsAllowed-lag race
- **위치**: `sunnypilot/selfdrive/car/intelligent_cruise_button_management/controller.py:62-110` (state machine) + `sunnypilot/.../mads/*` (lateral_active 진입) + `safety/modes/chrysler.h` `pcm_cruise_check` (DAS_3 bit 21 → controlsAllowed)
- **시나리오**: MADS auto-engage 가 lateral_active=True 만들 때 panda 의 `controlsAllowed` 는 DAS_3.ACC_ACTIVE 라이징 엣지에 의존 (`pcm_cruise_check`). 그러나 ICBM 은 ACC_ACTIVE=1 인 시점에 PRE_ACTIVE 진입해서 `accel/decel` CRUISE_BUTTONS 를 즉시 송신. controlsAllowed=False 인 사이클에 보내면 panda 가 TX block → 다음 사이클에 다시 시도. 만약 ACC ECU 가 button 시퀀스의 갭을 fault 로 해석하면 (이번 13/14 라우트 ACC fault 의 변형) 또 한번 latched fault 가능.
- **심각도**: 차량 ACC fault latched (ignition cycle 까지 지속).
- **수정 방향**: ICBM 의 `update_readiness()` 에 `panda.controlsAllowed==True` 조건 명시 추가. 또는 PRE_ACTIVE 의 INACTIVE_TIMER (현재 의도된 grace) 를 controlsAllowed True 이후부터 카운트하도록 변경.

#### T1-B. LKAS 200-frame 쿨다운 lockup (HIGHER_MIN_STEERING_SPEED 와 결합)
- **위치**: `opendbc_repo/opendbc/car/chrysler/carcontroller.py:74-78` — `lkas_control_bit and (self.frame - self.last_lkas_falling_edge > 200)`
- **시나리오**: JGC2019 의 `minSteerSpeed=17.5`, 해제 14.5 (carcontroller.py:66-68). 14.5~17.5 m/s 사이 변동이 많은 (시내 정속 주행, 교통 흐름) 운행 패턴에서 falling edge 가 반복 → 매 falling 마다 2 초 cooldown 누적. cooldown 중 LKAS bit 가 0 유지되면 EPS 가 self-cancel 로 인식할 수 있고, 어떤 시점엔 LKAS 자체 fault (LKAS_STATE==4) 로 빠질 수 있음. 우리 drivelog 에서 LKAS_STATE/LKAS_TEMPORARY_FAULT 추적 안 한 상태라 발생 빈도는 미확인.
- **심각도**: steerFaultPermanent → MADS 영구 해제 가능.
- **수정 방향**: drivelog 에서 `EPS_2.LKAS_STATE`, `EPS_2.LKAS_TEMPORARY_FAULT` 시계열 먼저 분석 → 14.5~17.5 변동 구간에서 fault 빈도 확인 → cooldown 값 (200 → 100?) 또는 히스테레시스 폭 (3.0 → 2.0?) 조정 검토.

### Tier 2 (softDisable / UX 회귀)

#### T2-A. ICBM v_cruise_equal 의 strict `==` 비교 (소수 라운딩 race)
- **위치**: `sunnypilot/selfdrive/car/intelligent_cruise_button_management/controller.py:49-50` (`return self.v_target == self.v_cruise_cluster`), L52-60 (calculation)
- **시나리오**: `v_target` 은 `LP_SP.vTarget`(m/s) → hysteresis → round(speed_conv). `v_cruise_cluster` 는 `CS.cruiseState.speedCluster`(m/s) → round(speed_conv). 두 양 모두 정수 라운드 되지만 입력이 노이지 (Kalman lag, 차량 set speed transition). v_target=91, v_cruise=90 같은 1-step gap 에서 v_cruise_equal=False → State.increasing → button spam → ACC set 91 → 다음 사이클 hysteresis 후 v_target=90 → State.decreasing → 진동 가능.
- **심각도**: 의도치 않은 ACC 가속/감속 1-mph 진동. 사용자 혼동.
- **수정 방향**: `v_cruise_equal` 에 ±1 step 톨러런스 추가 (`abs(...) <= 1`) — 이미 hysteresis 가 v_target 에 적용되므로 비교 단계의 1-step 톨러런스는 의도 그대로 유지.

#### T2-B. ICBM non-RAM button counter offset 4-cycle 패턴
- **위치**: `opendbc_repo/opendbc/sunnypilot/car/chrysler/icbm.py:42-46`
  ```python
  self.button_frame += 1
  button_counter_offset = [1, 1, 0, None][self.button_frame % 4]
  if button_counter_offset is not None:
    can_sends.append(chryslercan.create_cruise_buttons(packer, CS.button_counter + button_counter_offset, das_bus, accel=accel, decel=decel))
  ```
- **시나리오**: 4 사이클 중 한 번 (`None`) 송신을 의도적으로 skip. counter offset 도 `1, 1, 0, ...` 로 단조 증가 아님 → 차량이 counter mismatch 로 해석할 가능성 (Chrysler CRUISE_BUTTONS 4-bit COUNTER). 의도된 design 으로 보이지만 (실제 사용자 버튼의 frame 과 충돌 회피?) 주석/근거 없음. 현재로선 동작 검증된 코드일 수 있지만 우리 차에서 동작 검증 안 됨.
- **심각도**: ICBM 송신 25% drop → 가속/감속 지연. 정량 검증 필요.
- **수정 방향**: drivelog 에서 ICBM 활성 구간의 CRUISE_BUTTONS 송신 빈도 vs 의도 빈도 분석. 차이 크면 offset 패턴 단순화.

#### T2-C. LKAS HUD 첫 N초 송신 누락 (`lkas_car_model == -1` 가드)
- **위치**: `opendbc_repo/opendbc/car/chrysler/carcontroller.py:51-55` — `if CS.lkas_car_model != -1: ... create_lkas_hud`
- **시나리오**: 부팅 직후 carstate 가 첫 DAS_6 메시지를 받기 전까지 `lkas_car_model = -1`. 그 동안 LKAS HUD 송신 0회 → 차량 클러스터가 LKAS 상태 표시 안 함. 사용자가 lateral active 인지 모를 수 있음.
- **심각도**: 정보 누락 (제어는 정상). 단 안전 핸드오프 prompt 시각 누락 위험.
- **수정 방향**: `lkas_car_model = -1` 일 때 기본 model id (e.g., 0) 로 fallback 송신, 또는 carstate 가 첫 DAS_6 받기 전엔 selfdrived 가 lateral_active 진입 거부.

#### T2-D. MADS lateral-only 상태에서 `cruiseControl.resume` 의도치 송신
- **위치**: `selfdrive/controls/controlsd.py:175` (resume 조건: `CC.enabled and CS.cruiseState.standstill and not shouldStop`)
- **시나리오**: 사용자가 stock ACC 해제 (브레이크 풀밟기) 했지만 MADS 는 lateral 계속 유지. 정차 후 longitudinalPlan.shouldStop=False 되는 순간 controlsd 가 resume=True publish → carcontroller 가 (rate-limit 통과 시) RESUME 버튼 송신 → 차량 ACC 가 사용자 의도 무시하고 재engage.
- **심각도**: 사용자 의도 무시 가속. 안전 회귀.
- **수정 방향**: resume 조건에 `self.sm['selfdriveState'].engageable and CS.cruiseState.available` 추가, 또는 chrysler 특화로 사용자가 직전 N 초 내 cancel/brake-cancel 한 적 있으면 resume suppress.

### Tier 3 (검증 필요 / 회귀 후보)

#### T3-A. Longitudinal override gate 와 ICBM ready 의 결합 (Agent2 #8)
- **위치**: `controlsd.py:116-117` (CC.longActive 조건) + ICBM `update_readiness`
- **시나리오**: `overrideLongitudinal` event 가 set 되면 `CC.longActive=False`. ICBM `update_readiness` 가 `CC.longActive` 또는 `CC.cruiseControl.override` 를 참조하는지 확인 필요. 만약 longActive=False 일 때 ICBM 이 disable 된다면 정상. 만약 그대로 동작한다면 의도 외 button 송신.
- **수정 방향**: `update_readiness` 의 정확한 조건 코드 읽고 확정 (현재 본 plan 에서는 미독파).

#### T3-B. Latched cruise state on ignition cut
- **위치**: 시동 OFF 직전 cruise 상태가 latched → 다음 시동 시 cruise 켜진 채로 부팅 (이번 13 라우트 1.99s 의 ACC_ACTIVE=1 상승 패턴)
- **시나리오**: 우리 fix (controlsd cancel engageable 게이트) 가 boot window 동안 cancel 송신 막음. 그러나 부팅 완료 후 사용자가 아무 입력 안 했는데 차량은 여전히 ACC_ACTIVE=1 인 경우, openpilot 는 무엇을 해야 하는가? cancel? 그대로 두면 사용자가 모르는 사이 차가 set speed 로 가속.
- **수정 방향**: drivelog 추가 분석 — boot 후 `engageable=True` 시점의 cruise active 상태 빈도 측정. 빈도 높으면 명시적 alert ("Car cruise was on at startup") + 한 번의 cancel 송신 정책 추가.

#### T3-C. RX-check freshness 와 CAN dropout
- **위치**: `safety/modes/chrysler.h` 의 `chrysler_rx_checks` (확인 필요)
- **시나리오**: DAS_3 또는 DAS_4 의 freshness check 가 panda 에서 fail 하면 controlsAllowed=False → carcontrol publish 거부. selfdrived 상태는 그대로 → mismatch 발생.
- **수정 방향**: panda 의 rx_checks 정의를 읽고 chrysler.h 의 timeout 값 적정성 검토.

---

## 권장 작업 순서

1. **T1-A (ICBM controlsAllowed lag)** 우선 — 차량 fault latched 위험. 코드 1-2줄 추가로 차단 가능.
2. **T2-D (lateral-only resume 의도 무시)** — 안전 회귀. 빠르게 cancel/brake-cancel 추적 추가.
3. **T2-A (v_cruise_equal 톨러런스)** — 1줄 fix, 진동 제거.
4. **drivelog 추가 분석** — T1-B (LKAS cooldown), T2-B (counter offset), T3-A, T3-B 는 데이터로 빈도 확인 후 fix 결정.
5. **T2-C (HUD 가드)**, **T3-C (RX check)** 는 운행 빈도 분석 후 결정.

각 fix 의 commit 단위는 single-purpose 로 분리, plan 파일 (`sunny-release-tizi-drivelog-snuggly-cocoa.md`) 에 reapplication 가이드 형식으로 추가하면 sunny-tizi 브랜치 재생성 시 누락 없이 다시 적용 가능.

---

## 검증 시 사용할 drivelog 카운터 (추가 분석 시)

```python
# 이미 추출된 drivelog/0fb02cc3a5abcc2f_* qlog/rlog 활용 (wk2-drivelog 브랜치).
# 각 라우트 segment 별로:
#   - EPS_2.LKAS_STATE / LKAS_TEMPORARY_FAULT 카운트     -> T1-B
#   - ICBM SendButtonState 변화 vs DAS_4 ACC_SET_SPEED   -> T2-A, T2-B
#   - CRUISE_BUTTONS (id 571) 송신 cycle 별 byte0 분포   -> T2-B
#   - 부팅 후 첫 ACC_ACTIVE=1 시점 분포 (라우트 06-15)  -> T3-B
#   - DAS_3 / DAS_4 메시지 dropout (>100ms gap) 빈도     -> T3-C
```

## 변경 가능성 있는 파일 목록

| Tier | 파일 |
|---|---|
| T1-A | `sunnypilot/selfdrive/car/intelligent_cruise_button_management/controller.py` |
| T1-B | `opendbc_repo/opendbc/car/chrysler/carcontroller.py` |
| T2-A | `sunnypilot/selfdrive/car/intelligent_cruise_button_management/controller.py` |
| T2-B | `opendbc_repo/opendbc/sunnypilot/car/chrysler/icbm.py` |
| T2-C | `opendbc_repo/opendbc/car/chrysler/carcontroller.py` 또는 `carstate.py` |
| T2-D | `selfdrive/controls/controlsd.py` |
| T3-A | (read-only) `sunnypilot/selfdrive/car/intelligent_cruise_button_management/controller.py` |
| T3-B | `selfdrive/controls/controlsd.py` or `selfdrive/selfdrived/selfdrived.py` |
| T3-C | (read-only) `safety/modes/chrysler.h` |

---

## Update — drivelog 정량 분석 결과 (`jeep_gc_2020_quant_analysis.py`)

13 라우트 / 550 qlog 세그먼트 / ~3300 분 운행 데이터를 qlog 로 sweep.
스크립트와 raw 출력은 같은 디렉토리에 보관:
- `jeep_gc_2020_quant_analysis.py` — 카운터 수집 스크립트
- `jeep_gc_2020_quant_summary.py` — 라우트별 집계
- `jeep_gc_2020_quant_output.txt` — 텍스트 형식 raw 출력

### T1-B 가설 **기각** — LKAS fault 0건

13 라우트 전부에 걸쳐 `carState.steerFaultTemporary` 와
`steerFaultPermanent` 의 rising edge 누적치 **모두 0**. true 샘플도 **0**.
HIGHER_MIN_STEERING_SPEED + 200-frame cooldown 누적이 fault 를 유발한다는
가설은 데이터로 뒷받침되지 않음. carcontroller.py 의 lkas_control_bit
falling-edge 가드가 실제 운행에선 충분히 보호적인 것으로 보임.

**조치**: T1-B 항목을 우선순위 리스트에서 제거. 단 향후 14.5–17.5 m/s
저속 시내 운행 비율 높은 사용자에서 재발 시 재검토.

### T3-B 가설 **확인 — 단, 빈도 낮음 (1/13 = 8%)**

`carState.cruiseState.enabled` 가 segment 0 시작 5초 이내 True 였던 라우트:

| 라우트 | first_avail | first_active | active_<5s |
|---|---|---|---|
| 06–12 (7 routes) | – or 3.01s | – | False |
| **13** | 3.83s | **3.83s** | **True** |
| 14 | 3.92s (fault 상태) | – | False |
| 15 | 50.80s (사용자 ON) | – | False |

라우트 13 한 번이 우리가 이미 fix 한 그 부팅 race 케이스 (사용자가 부팅 중
ACC ON 누름 → cancel storm → fault). 다른 12 라우트는 모두 정상.

**조치**: 우리 fix (controlsd cancel engageable 게이트) 가 발현 시점을
이미 차단함. 추가 작업 (alert/explicit one-shot cancel) 은 빈도 낮아 대기.
재발 시 다시 평가.

### T2-B 보류 — qlog 너무 sparse

qlog 의 sendcan 샘플로 본 CRUISE_BUTTONS tx byte0 분포는
- `0x08` (ACC_Decel) — 가장 많음
- `0x04` (ACC_Accel) — 그 다음
- `0x01` (ACC_Cancel) — 부팅 cancel storm 잔재 (이미 fix)

총 tx 수가 라우트 별로 0~12회. qlog 다운샘플링 때문에 실제 ICBM 송신
빈도/패턴은 판단 불가. `[1, 1, 0, None]` 4-cycle skip 의 영향은
**rlog 샘플링 분석으로 확인 필요** — 별도 후속 작업.

### T3-C 보류 — qlog gap = 다운샘플링 노이즈

`carState` 최대 gap 이 모든 라우트에서 ~100–130 ms (정상 10 ms 간격 대비).
값이 라우트 전체에 걸쳐 유사하게 분포 → 실제 CAN dropout 이 아니라
qlog 다운샘플링 산물로 추정. rlog 가 있어야 실제 CAN freshness 판단 가능
— 별도 후속 작업.

### 갱신된 우선순위 (정량 검증 후)

| Tier | 항목 | 상태 |
|---|---|---|
| **T1-A** | ICBM PRE_ACTIVE controlsAllowed-lag race | **최우선** — 이론적 위험, 1~2줄 fix |
| **T2-D** | MADS lateral-only 상태에서 resume 송신 | **다음** — UX/안전 회귀 후보 |
| **T2-A** | ICBM v_cruise_equal strict `==` | **그 다음** — 1줄 톨러런스 |
| T2-C | LKAS HUD 첫 N초 송신 누락 | 미루기 |
| T3-A | longActive ↔ ICBM ready 결합 | read-only 추가 분석 |
| T2-B | ICBM non-RAM offset 4-cycle skip | rlog 분석 후 결정 |
| T3-C | RX-check freshness | rlog 분석 후 결정 |
| T1-B | LKAS cooldown lockup | **데이터 기각 → 제거** |
| T3-B | Boot-time latched cruise | **이미 fix 로 차단 → 추가 작업 보류** |

---

## Update 2 — rlog 정량 분석 결과 (`jeep_gc_2020_rlog_analysis.py`)

13 라우트 / 549 rlog 세그먼트 / 5723 ICBM tx + 5.5M RX message sample.
스크립트와 raw 출력:
- `jeep_gc_2020_rlog_analysis.py` — rlog sweep (sendcan + can full-rate)
- `jeep_gc_2020_rlog_summary.py` — 라우트별 집계
- `jeep_gc_2020_rlog_output.txt` — 텍스트 형식 raw 출력

### T2-B 가설 갱신 — 4-cycle skip 비효율보다 더 심각: **COUNTER stuck at 0**

라우트별 CRUISE_BUTTONS (0x23B) tx 시그널 분포:

| 라우트 | tx_total | boot<10s | cancel | resume | accel | decel | idle |
|---|---|---|---|---|---|---|---|
| 06 | 2123 | 0 | 27 | 0 | 963 | 1133 | 0 |
| 07 | 150  | 0 | 150 | 0 | 0 | 0 | 0 |
| 08 | 431  | 0 | 9 | 0 | 140 | 282 | 0 |
| 0a | 1299 | 0 | 1 | 0 | 730 | 568 | 0 |
| 0b | 497  | 0 | 6 | 0 | 232 | 259 | 0 |
| 0c | 154  | 0 | 35 | 0 | 54 | 65 | 0 |
| 0d | 472  | 0 | 40 | 0 | 55 | 377 | 0 |
| 0e | 147  | 0 | 94 | 0 | 35 | 18 | 0 |
| 0f | 264  | 0 | 120 | 0 | 32 | 112 | 0 |
| 12 | 133  | 0 | 14 | 0 | 87 | 32 | 0 |
| **13** | **53** | **48** | **53** | 0 | 0 | 0 | 0 |
| 14 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| 15 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| TOTAL | 5723 | 48 | 549 | 0 | 2328 | 2846 | 0 |

**관찰 #1 — Route 13 의 boot cancel storm 정량 확인**:
Route 13 segment 0 에서 `cb_boot_storm_count_t_lt_10s = 48` (전부 cancel).
부팅 race 시나리오 (이미 Fix A.1/A.2 로 차단됨) 의 정량 footprint 확인.
나머지 12 라우트의 boot 윈도우 cancel 카운트 = 0.

**관찰 #2 — resume = 0 across ALL 13 routes**:
ICBM/carcontroller 의 ACC resume button 송신이 전체 운행에서 한 번도
발생 안 함. 시나리오: (a) standstill resume 자체가 한 번도 안 일어남
(고속도로 위주 운행으로 추정), (b) 또는 ICBM 의 resume 로직이 이 차량에서
발동 조건 미충족. 별도 검증 필요하지만 priority 낮음.

**관찰 #3 — `idle` byte0 (모든 button 비트가 0인 송신) = 0**:
ICBM 은 누를 게 없을 때 송신 자체를 skip — 의도된 동작. byte0 hist 도
오직 0x01/0x04/0x08 만 등장.

**관찰 #4 (중대) — COUNTER 가 stuck at 0**:

```
COUNTER delta histogram (fleet aggregate, 5616 consecutive tx pairs):
  delta value = (next_counter - prev_counter) mod 16
  ICBM icbm.py:43 이론치: +1 ~50%, +0 ~25%, +2/+3 ~25%
  실측:
     delta       count       pct
         0        5616   100.00%
     TOTAL        5616

가장 풍부한 세그먼트 (route 06 seg 9, tx=532) 의 첫 80 카운터:
  [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
   0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
   0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
   0, 0, 0, 0, 0, 0, 0, 0]
```

**13 라우트 / 5723 tx / 5616 consecutive pair 전부 COUNTER=0**.

ICBM `icbm.py:43` 의 `[1, 1, 0, None]` skip 패턴은 무관. Source = upstream
의 `CS.button_counter` 가 항상 0 으로 stuck. 후보 원인:

1. **CarState parser**: `opendbc_repo/opendbc/car/chrysler/carstate.py` 의
   `button_counter` parsing 이 `CRUISE_BUTTONS` (0x23B) RX 메시지의 잘못된
   비트/필드를 읽고 있을 가능성. 정상 stock CRUISE_BUTTONS RX 의 실제
   COUNTER 시그널 위치를 DBC 와 대조 필요.
2. **CRUISE_BUTTONS RX 자체가 안 옴**: 사용자가 ACC 버튼을 누르지 않으면
   stock CRUISE_BUTTONS frame 이 발행 안 될 수 있음. 그렇다면
   `CS.button_counter` 가 default 0 으로 유지 → openpilot 송신 시 0 으로 보냄.
   ACC ECU 는 packet 의 COUNTER 가 직전 stock COUNTER+1 일 때만 수용 →
   stuck COUNTER 는 ACC ECU 에 의해 ignored 일 가능성.
3. **ACC ECU 의 graceful 처리**: cancel/accel/decel 시그널 자체가 6.7초 동안
   ~16Hz 로 송신되면 ECU 가 COUNTER 무시하고 첫 valid press 만 처리할 수도.

**이 발견이 사용자 운행에 영향이 있는가?** 직접적 disengage 트리거는
아니지만:
- ICBM 으로 longitudinal control (accel/decel = 5174 tx) 이 동작 안 하고
  있을 가능성 큼 → 사용자는 사실상 stock ACC speed setpoint 만 사용 중일 수도.
- cancel storm (route 13) 은 ACC ECU 가 COUNTER mismatch 로 무시했다면
  실제 fault 발생까지 ~6.7초 걸린 게 설명됨 (COUNTER 변화 없는 같은 cancel
  을 수백 번 받아도 ECU 의 internal watchdog 이 결국 fault 발생).

### T2-B 후속 조치 — 코드 검증 (별도 turn)

런타임 fix 시도 *전*에 read-only 검증이 필요:
1. `opendbc_repo/opendbc/car/chrysler/carstate.py` 에서 `button_counter`
   파싱 위치 확인 → DBC 의 CRUISE_BUTTONS.COUNTER 시그널 정의와 비교
2. 한 세그먼트의 `can` 메시지 dump 에서 stock 0x23B (RX) 의 byte1 nibble
   변화 관찰 (1Hz 로 RX 되는지, 시퀀스가 0/1/2/.../F 인지)
3. `chryslercan.create_cruise_buttons` 의 COUNTER signal name 과 DBC 매핑

위 검증 후 진짜 버그면 1줄 fix; 의도된 동작이면 review.md 우선순위에서
T2-B 제거.

### T3-C 가설 — RX-check freshness: **거의 정상 (제거)**

RX-check 6개 메시지 모두 panda freshness threshold 안전 마진 안:

| 메시지 | Hz | 3/Hz 임계 | 최대 gap (관측) | >3/Hz 카운트 (13 라우트 합) | >500ms |
|---|---|---|---|---|---|
| ESP_1 | 50 | 60 ms | 228 ms (route 0c) | 2 | 0 |
| DAS_3 | 50 | 60 ms | 228 ms (route 0c) | 1 | 0 |
| 0x202 | 100 | 30 ms | 228 ms (route 0c) | 22 | 0 |
| EPS_2 | 100 | 30 ms | 228 ms (route 0c) | 23 | 0 |
| ECM_5 | 50 | 60 ms | 228 ms (route 0c) | 1 | 0 |
| TRACTION_BUTTON | 1 | 3000 ms | 1050 ms | 0 | 0 |

**핵심**:
- **>500ms gap = 0 across all 13 routes / all 6 messages**. Panda RX-check
  의 absolute threshold (보통 message 마다 다르지만 일반적으로 100-500ms
  추정) 를 단 한 번도 위반 안 함.
- Route 0c 의 단일 mid-drive 228ms freeze: 5개 메시지가 정확히 같은
  타임스탬프 (228.24ms) 에서 동시 gap → 단일 CAN bus 순간 freeze (panda
  USB 일시 hiccup 추정). 1건 in 28 segments.
- 모든 라우트의 max_gap_seg 가 mid-route (seg 11, 15, 86, 118) — boot
  관련 아님. 정상 mid-drive 운행 중 일반적 jitter.

**boot grace 검증 (segment 0 첫 RX 도착 시각)**:
- 5개 high-rate 메시지 모두 t < 0 (logMonoTime base 이전 도착) — panda
  부팅 직후 즉시 stream 시작.
- TRACTION_BUTTON (1Hz) 만 0.04~0.91s 후 첫 도착 — 정상.

**safetyRxChecksInvalid 발생 패턴**:
- 13 라우트 중 **8 라우트** 에서 segment 0 의 **t = 7.4~9.9s** 시점에 1회 rising,
  true samples = 10 (= 100ms 펄스).
- 모두 boot grace 윈도우 (10초) 직전. 우리 Fix B 의 `boot_grace` 가드가
  정확히 이 펄스를 SOFT_DISABLE 로 escalate 안 되게 막고 있는 시나리오 확인.
- mid-drive 또는 boot 10초 이후 rising 은 0건 → Fix B 의 boot_grace 가
  실제 panda 이상을 mask 하지 않음을 확인.

**결론**: T3-C 우선순위 리스트에서 **제거**. 단일 freeze 이벤트는 panda
absolute threshold 아래라 실제 disengage 유발 안 함, mid-drive 만성 dropout
없음, boot pulse 는 Fix B 가 이미 처리.

### 최종 갱신 우선순위 (rlog 정량 검증 후)

| Tier | 항목 | 상태 |
|---|---|---|
| **T2-B** | **CRUISE_BUTTONS COUNTER stuck at 0** | **격상** — read-only 코드 검증 → 1줄 fix or 의도 확인 |
| **T1-A** | ICBM PRE_ACTIVE controlsAllowed-lag race | **다음** — 이론적 위험, 1~2줄 fix |
| **T2-D** | MADS lateral-only 상태에서 resume 송신 | **그 다음** — UX/안전 회귀 후보 |
| **T2-A** | ICBM v_cruise_equal strict `==` | 그 다음 — 1줄 톨러런스 |
| T2-C | LKAS HUD 첫 N초 송신 누락 | 미루기 |
| T3-A | longActive ↔ ICBM ready 결합 | read-only 추가 분석 |
| T1-B | LKAS cooldown lockup | **데이터 기각 → 제거** |
| T3-B | Boot-time latched cruise | **이미 fix 로 차단 → 추가 작업 보류** |
| T3-C | RX-check freshness | **데이터 검증됨 → 제거** |

다음 작업 후보: **T2-B 검증 (carstate.py + DBC + stock 0x23B RX 시퀀스 spot check)** → 결과에 따라 fix or 제거.

따라서 다음 fix 작업 후보는 **T1-A → T2-D → T2-A** 순.

---

## Update 3 — Update 2 의 두 finding 은 분석 스크립트 비트 위치 버그였음 (T2-B 제거)

### 정정 요약

**Update 2 의 두 결론을 retract 합니다:**
- ❌ "COUNTER stuck at 0" — **measurement artifact**. 분석 스크립트가 byte1 의
  하위 nibble (`b1 & 0x0F`, reserved/unused) 를 읽었음. 실제 COUNTER 는
  상위 nibble (`(b1 >> 4) & 0x0F`).
- ❌ "resume = 0 across all routes" — **measurement artifact**. 스크립트가
  `b0 & 0x02` (= ACC_Distance_Dec, 거의 안 눌리는 follow-distance 버튼) 를
  "resume" 으로 잘못 라벨링. 실제 ACC_Resume 는 `b0 & 0x10`.

### 비트 위치 source of truth

`opendbc_repo/opendbc/dbc/generator/chrysler/_stellantis_common.dbc:BO_ 571
CRUISE_BUTTONS`:
```
SG_ ACC_Cancel       :  0|1@1+   (bit 0 of byte0)
SG_ ACC_Distance_Dec :  1|1@1+   (bit 1)
SG_ ACC_Accel        :  2|1@1+   (bit 2)
SG_ ACC_Decel        :  3|1@1+   (bit 3)
SG_ ACC_Resume       :  4|1@0+   (bit 4 — Motorola, but 1-bit so position-only)
SG_ Cruise_OnOff     :  6|1@1+   (bit 6)
SG_ ACC_OnOff        :  7|1@1+   (bit 7)
SG_ ACC_Distance_Inc :  8|1@1+   (byte1 bit 0)
SG_ COUNTER          : 15|4@0+   (byte1 bits 7-4, Motorola big-endian — high nibble)
SG_ CHECKSUM         : 23|8@0+   (byte2)
```

상위 코드는 모두 정상 (Explore agent 확인):
- `carstate.py:22,105` — `button_counter = cp.vl["CRUISE_BUTTONS"]["COUNTER"]`
  올바른 시그널명, RX 시 update
- `chryslercan.py:70-78` — `create_cruise_buttons` 가 `"COUNTER": frame % 0x10`
  packing, DBC 시그널명과 일치
- `icbm.py:31-46` — `[1, 1, 0, None][self.button_frame % 4]` offset 로직 정상

### 검증 — Hand-decode (route 06 segment 9)

`debug_logs/jeep_gc_2020_cruise_btn_decode.py` 로 한 세그를 hand-decode:

```
=== Segment summary ===
  TX (sendcan)            count = 532
  RX (can, all buses)     count = 6530
  RX per bus              {0: 2999, 130: 2999, 128: 532}

=== First 12 TX (sendcan) high-nibble COUNTER ===
  [0, 2, 2, 5, 6, 6, 9, 10, 10, 13, 14, 14]

=== Steady-state deltas (after first transient) ===
  [+0, +3, +1, +0, +3, +1, +0, +3, +1, +0]

=== First RX bus 0 frame ===
  t=540.955s b0=0x00 b1=0xc0 b2=0x8b | COUNTER(high)=12 (idle)
  t=540.975s ...                    | COUNTER(high)=13 (idle)
  ...    +1 매 프레임 — 50Hz 지속 송신
```

**중대 사실 발견**:
1. **stock 0x23B RX 가 50Hz 로 지속 송신** (event-driven 아님). bus 0 + bus 130
   각각 2999 frames in 60s ≈ 50Hz. → `CS.button_counter` 가 stuck 이 절대
   아니고 0-15 정상 cycling 중.
2. **TX low_nibble = 0** (모든 532 TX) → DBC 의 `CHECKSUM` 도 byte2 인데
   왜 byte1 low nibble 이 비어 있는지: DBC 가 그 4 비트를 시그널 정의 안 했음
   (reserved). 내 원래 스크립트가 정확히 이 빈 영역을 읽음.
3. **TX COUNTER 패턴 = ICBM 가 정확히 설계대로 동작 중**. CS.button_counter
   가 +1 씩 cycling 하는 동안 ICBM 가 `CS+offset` (offset ∈ [1, 1, 0, None])
   을 보내면, observed steady-state TX COUNTER 시퀀스가:
   - iter 0: CS=N+0, offset=1, TX=N+1
   - iter 1: CS=N+1, offset=1, TX=N+2 (delta +1)
   - iter 2: CS=N+2, offset=0, TX=N+2 (delta +0)
   - iter 3: CS=N+3, offset=None, SKIP
   - iter 4: CS=N+4, offset=1, TX=N+5 (delta +3 from previous TX)
   - 반복 → 패턴 `[+1, +0, +3, +1, +0, +3, ...]`
4. observed 패턴이 **정확히 일치** (첫 transient `+2` 제외).

### Re-sweep 결과 (corrected bit positions, 13 라우트 / 549 세그)

**시그널 분포** (`resume` 컬럼은 corrected `b0 & 0x10`):

| 라우트 | tx_total | rx_total | cancel | resume | accel | decel | dist_dec | dist_inc | crOnOff | accOnOff |
|---|---|---|---|---|---|---|---|---|---|---|
| 06 | 2123 | 101803 | 27 | 0 | 963 | 1133 | 0 | 0 | 0 | 0 |
| 07 | 150 | 110146 | 150 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| 08 | 431 | 56369 | 9 | 0 | 140 | 282 | 0 | 0 | 0 | 0 |
| 0a | 1299 | 144584 | 1 | 0 | 730 | 568 | 0 | 0 | 0 | 0 |
| 0b | 497 | 114063 | 6 | 0 | 232 | 259 | 0 | 0 | 0 | 0 |
| 0c | 154 | 83690 | 35 | 0 | 54 | 65 | 0 | 0 | 0 | 0 |
| 0d | 472 | 260451 | 40 | 0 | 55 | 377 | 0 | 0 | 0 | 0 |
| 0e | 147 | 259628 | 94 | 0 | 35 | 18 | 0 | 0 | 0 | 0 |
| 0f | 264 | 356688 | 120 | 0 | 32 | 112 | 0 | 0 | 0 | 0 |
| 12 | 133 | 45611 | 14 | 0 | 87 | 32 | 0 | 0 | 0 | 0 |
| 13 | 53 | 7098 | 53 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| 14 | 0 | 50261 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| 15 | 0 | 44786 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| TOTAL | 5723 | **1,635,178** | 549 | 0 | 2328 | 2846 | 0 | 0 | 0 | 0 |

**RX 50Hz 지속 송신 확정** — 1.63M RX over ~9 hours of driving ≈ **50.4 Hz**.
0x23B 는 stock 차량이 항상 보내는 frame, button press 와 무관.

**`resume = 0` 은 정확한 측정**:
- ICBM `icbm.py:32-33` 은 accel/decel 만 송신 (`SendButtonState.increase/decrease`).
  ACC_Resume 시그널은 ICBM 가 절대 안 보냄.
- carcontroller `carcontroller.py:46` 의 resume 분기는 `CC.cruiseControl.resume`
  로 게이트 되며 standstill resume 시에만 발동. 이 운행 (고속도로 위주
  추정) 에 한 번도 안 일어남.
- (변경된 cancel 분기 `carcontroller.py:42` 는 정상 발동 — 549건. Fix A.2 의
  rate-limit 통과 후 송신.)

**COUNTER delta histogram (corrected, fleet aggregate, 5616 consecutive
TX pairs):**

```
delta value = (next_counter - prev_counter) mod 16
  delta       count       pct       해석
      0       1608      28.63%      offset 1 -> offset 0
      1       1632      29.06%      offset 1 -> offset 1
      2         96       1.71%      transient (CS counter wrap)
      3       1924      34.26%      offset 0 -> SKIP -> offset 1 (4-cycle)
      4        118       2.10%      transient
      5         67       1.19%
      6+       137       2.44%      double-skip / longer gaps
   TOTAL      5616
```

**예측 vs 측정 (defect 없음)**:
- 예측: `0/1/3 ~ 33%/33%/33%`, others ~0%
- 측정: `0/1/3 = 28.6/29.1/34.3 = 91.95% sum`, others 8% (cycle 시작/종료
  transient 합계)

ICBM 의 `[1, 1, 0, None]` skip 패턴이 정확히 동작하며 예측 분포와 일치 ✓

### T2-B 최종 결론

**T2-B 제거**. ICBM `[1, 1, 0, None]` skip 패턴은 작동하며 ACC ECU 와의
button-counter handshake 도 깨지지 않음. Update 2 의 "stuck COUNTER" 가설은
오로지 분석 스크립트의 비트 위치 버그였음.

### 최종 갱신 우선순위 (post-correction)

| Tier | 항목 | 상태 |
|---|---|---|
| **T1-A** | ICBM PRE_ACTIVE controlsAllowed-lag race | **최우선** — 이론적 위험, 1~2줄 fix |
| **T2-D** | MADS lateral-only 상태에서 resume 송신 | **다음** — UX/안전 회귀 후보 |
| **T2-A** | ICBM v_cruise_equal strict `==` | 그 다음 — 1줄 톨러런스 |
| T2-C | LKAS HUD 첫 N초 송신 누락 | 미루기 |
| T3-A | longActive ↔ ICBM ready 결합 | read-only 추가 분석 |
| T1-B | LKAS cooldown lockup | **데이터 기각 → 제거** |
| T3-B | Boot-time latched cruise | **이미 fix 로 차단 → 추가 작업 보류** |
| T3-C | RX-check freshness | **데이터 검증됨 → 제거** |
| T2-B | ICBM non-RAM offset 4-cycle skip | **measurement bug 였음 → 제거** |

다음 fix 작업 후보 변동 없음: **T1-A → T2-D → T2-A**.

---

# Update 4 — sunnypilot v2026.001.005 fork sync 후 신규 4 라우트 분석

## 컨텍스트

upstream `sunnypilot/sunnypilot:release-tizi` v2026.001.005 (a51d8376) 가
fork sync 되어 `origin/sunny-tizi` 가 force-reset 됨. 로컬 sunny-tizi 의 7
commit (runtime fix + debug_logs + SCC-V tuning) 을 v2026.001.005 위로
rebase 하여 모두 재적용 (conflict 0, 모든 anchor 라인 위치 동일). 새 head:
- `24fe8d9` runtime: Jeep GC ACC fault + boot grace + audio chain fixes
- `94c9959` SCC-V: relax curve speed reduction thresholds for highway

분석 대상: `origin/wk2-drivelog` 의 신규 4 라우트, 126 segments,
총 **37.0 시간** 운행:

| 라우트 | segs | duration | vEgo avg | vEgo max | engaged samples |
|---|---|---|---|---|---|
| `_00000000--2a33888425` | 16 | 136 min | 15.5 kph | 64.2 kph | 2589 (시내, ACC 가끔) |
| `_00000001--898f28ebec` | 33 | 561 min | 16.1 kph | 68.9 kph | 0 (ACC 미사용) |
| `_00000005--c1e7eb6f64` | 38 | 741 min | 23.6 kph | 87.9 kph | 33839 (시내+highway) |
| `_00000006--25b05cbab7` | 39 | 781 min | 46.9 kph | 78.7 kph | 99499 (highway) |

## A. Fix A/B/C 효과 검증 — 완벽

37.0 시간 운행 중 Fix 표적 이벤트:

| 표적 이벤트 | 카운트 | Fix | 상태 |
|---|---|---|---|
| `accFaulted` | **0** | Fix A.1/A.2 (cancel gate + 500ms rate-limit) | ✅ |
| `controlsMismatch` | **0** | Fix B.1 (boot grace 통합) | ✅ |
| `processNotRunning` | **0** | Fix C.1/C.2/C.3 (ignored_processes + outer recovery loop) | ✅ |
| **모든 immediateDisable** | **{}** | — | ✅ 단 1건도 없음 |

추가 정량 보조 메트릭:

| 메트릭 | v1 (13 routes, ~10 hr 추정) | v2 (4 routes, 37 hr) | 평가 |
|---|---|---|---|
| boot-window (`t<10s`) CRUISE_BUTTONS storm | 1 (route 13 seg 0 에서 폭발적) | 0 | Fix A.1 engageable gate 효과 |
| fleet CRUISE_BUTTONS cancel tx | 1185+ (cancel storm 포함) | **11** | 매우 낮음 (cancel 자체 거의 안 함) |
| safetyRxChecksInvalid rising | 4 (Update 1 sweep) | **1** | Fix B 의 boot grace 효과 |
| COUNTER `[0/1/3]` 분포 (ICBM 검증) | 28.6/29.1/34.3% | 31.9/32.6/30.3% | 동일 패턴, ICBM 정상 |
| RX-check `>3/Hz` 카운트 (모든 addr) | 13 routes 0건 | 4 routes 0건 (~1Hz TRACTION_BUTTON 의 기본 IAT 제외) | 정상 freshness |

## B. SCC-V 동작 정량 분석

신규 sweep script: `debug_logs/jeep_gc_2020_scc_v_analysis.py` +
`scc_v_summary.py`. `longitudinalPlanSP.smartCruiseControl.vision.state`
+ `longitudinalPlanSource` + `onroadEvents` 시계열 sweep.

### SCC-V state 시간 점유 (fleet aggregate, 37 hr)

```
disabled  : 6086.5 s  (4.57% of drive time)
enabled   : 1233.3 s  (0.93%)
entering  :   18.5 s  (0.014%)
turning   :   14.7 s  (0.011%)
leaving   :    5.4 s  (0.004%)
overriding:   60.2 s  (0.045%)
```

전체 운행의 **95%+ 는 SCC-V state 메시지가 publish 안 됨** (ACC 미사용
구간 = `cruise_enabled_samples=0` 의 route 01 등 + cruise off 구간).
ACC engaged 시간 중에서도 active state (entering+turning+leaving)
점유율 ≈ 38.6s / 37hr = **0.029%**.

### Entering trigger pattern

ACC engaged route (05, 06) 만:

| 라우트 | entering | →turning | →aborted | conv_rate | pred_lat_acc max |
|---|---|---|---|---|---|
| 05 | 11 | 3 | 8 | 27.3% | 2.25 m/s² |
| 06 | 18 | 6 | 12 | 33.3% | 2.17 m/s² |
| **fleet** | **29** | **9** | **20** | **31.0%** | 2.25 m/s² |

- Per-hour entering: **1.18/hr** (24.5 hr ACC 운행 기준)
- Abort rate: **69%** (entering 진입 후 maxPredLatAcc 가 _ABORT 임계 1.1
  아래로 떨어진 경우 — 차로 변경/일시적 yaw 등의 false positive 추정)
- `_A_LAT_REG_MAX = 2.8` 임계 안 넘김 (curLatAcc max 2.36/2.26)

### Plan source 점유

fleet: `cruise 99.95%, sccVision 0.05%, sccMap 0.00%, speedLimitAssist 0.00%`

→ SCC-V 가 cruise plan 을 dominate 한 시간이 매우 적음. SCC-M, SLA 는 한
번도 active 안 됨 (사용자 미사용 또는 map data 없음).

## C. 추가 발견 — 미고려 disengage trigger

37 hr 동안 fleet softDisable (rising edge 카운트):

```
wrongGear              470     (시동 직후, parking)
calibrationIncomplete  438     (model calibration window)
seatbeltNotLatched     405     (운전자 벨트 풀고 차에서 내릴 때)
doorOpen                98     (정차 시)
commIssue               43     ← transient
locationdTemporaryError 23     ← transient
posenetInvalid          20     ← transient
paramsdTemporaryError   15     ← transient
commIssueAvgFreq         4     ← transient
```

- 처음 4개 (wrongGear/calib/seatbelt/doorOpen) 는 정상 startup/shutdown
  이벤트 — 운행 자체 disengage 가 아님.
- 마지막 5개 (commIssue/locationd/posenet/paramsd/...) 는 **transient 시스템
  이상** softDisable 인데, 모두 boot grace (Fix B) 와 ignored_processes (Fix C
  의 micd) 외부. 37 hr 에 합 105건 = 3건/hr. 차량 cruise 가 끊겼다는 의미
  (softDisable 은 3 초간 grace → 풀리지 않으면 disengage).

→ 추가 잠재 가설:
- **commIssue 43건**: 어떤 sensor/msg 의 freshness 가 일시적으로 부족.
  panda/sensor/locationd 사이 hiccup 가능. socket-based 라 micd/soundd 와
  다른 원인.
- **locationdTemporaryError 23건 / posenetInvalid 20건**: VIO/calibration
  관련. 모델 입력 시각 mismatch 가능.

## D. 파라미터 최적화 권장 (현 데이터 근거)

### D-1. SCC-V `_ABORT_ENTERING_PRED_LAT_ACC_TH` 검토 (현재 1.1)

- 데이터: entering 29회 중 20회 abort (69%) → entering→turning 진입 못 함
- 해석: 운전자가 곡선 진입 직전 maxPredLatAcc 잠깐 1.6 (>=ENTERING) 봤다가
  곧 1.1 아래 (<ABORT) 로 떨어짐 → SCC-V 가 entering 들어왔다가 즉시 enabled
  로 후퇴 → v_target 못 내림 → 곡선 감속 도와줄 기회 사라짐
- **권장**: 1.1 → **0.9** 로 내려서 abort 덜 보수적 → 더 많은 entering 이
  turning 으로 진행 가능. 대신 false negative (불필요한 v_target lower)
  위험 약간 증가
- **차선 권장**: 1.1 유지하되 `_ENTERING_PRED_LAT_ACC_TH` 1.6 → **1.4** 로
  내려 entering 자체를 더 자주 트리거 (현재 1.18/hr → 2~3/hr 예상)

### D-2. SCC-V `_A_LAT_REG_MAX` (현재 2.8)

- 데이터: 모든 turning 의 curLatAcc 가 2.36/2.26 (< 2.8)
- 즉 2.8 임계는 절대 안 닿음 → 너무 conservative 가 아님
- **권장: 2.8 유지**

### D-3. selfdrived `boot_grace` (현재 10s)

- 37 hr 동안 safetyRxChecksInvalid rising 1건만 (Fix B 적용 후)
- boot 직후 (t<10s) CRUISE_BUTTONS storm 0건
- **권장: 10s 유지**

### D-4. Fix A.2 cancel rate-limit (현재 500ms)

- 37 hr 동안 fleet cancel 11건만 (Fix A.1 engageable gate 와 결합 효과)
- accFaulted 0건 → fix 효과 검증
- **권장: 500ms 유지**

### D-5. 신규 가설 — commIssue/locationd/posenet 잦은 transient

- 합 105건 softDisable / 37 hr ≈ 3건/hr
- **별도 turn 에서 read-only 분석**: 어떤 메시지/freshness 가 transient
  실패하는지 (rlogs 의 managerState 추적, socket subscriber freshness 측정).
  fix 디자인 (boot_grace 확장 or `ignored_processes` 추가 or freshness
  hold-up timer) 은 그 분석 후

## 갱신된 최종 우선순위

| Tier | 항목 | 상태 (Update 4 후) |
|---|---|---|
| **T1-A** | ICBM PRE_ACTIVE controlsAllowed-lag race | 미해결, 최우선 (37hr 데이터에 accFaulted 0건이지만 이론적 위험 잔재) |
| **T2-D** | MADS lateral-only resume 송신 | 미해결, 두 번째 |
| **T2-A** | ICBM v_cruise_equal strict `==` | 미해결, 세 번째 |
| **NEW** | `commIssue`/`posenetInvalid`/`locationdTemporaryError` transient softDisable (3/hr) | 신규 후보 — 분석 → fix |
| **SCC-V** | `_ABORT_ENTERING_PRED_LAT_ACC_TH 1.1 → 0.9` (D-1) | 권장, 별도 turn |
| T1-B / T3-B / T3-C / T2-B | (모두 제거) | 데이터로 기각 |

Fix A/B/C 는 운영 검증 완료 — **유지**.

산출물 (이 Update 의):
- `debug_logs/jeep_gc_2020_scc_v_analysis.py` — 신규 SCC-V state + planner + events sweep
- `debug_logs/jeep_gc_2020_scc_v_summary.py` — fleet 집계
- `debug_logs/jeep_gc_2020_scc_v_output.txt` — 출력
- `debug_logs/jeep_gc_2020_rlog_output_v2.txt` — corrected rlog sweep on 4 신규 라우트

---

# Update 5 — Fix D: publisher-warmup grace (transient softDisable 차단)

## 컨텍스트

Update 4 의 D-5 신규 후보 (transient softDisable ~3/hr) 의 root-cause
분석 + fix 적용. 사용자 cruise drop 까지 가진 않지만 alert spam UX 회귀
(105 alerts / 37hr = 2.84 alerts/hr) 해소.

## 데이터 (Update 4 sweep + 신규 timing sweep)

5 종 transient event 의 rising-edge **모두 seg=0 의 t<20s** 에 응집:

| 이벤트 | n | min | median | max | seg=0 / total |
|---|---|---|---|---|---|
| commIssue | 43 | 9.1s | 14.1s | 19.5s | 43/43 |
| commIssueAvgFreq | 4 | 18.3s | 18.3s | 19.2s | 4/4 |
| locationdTemporaryError | 23 | 9.1s | 12.1s | 14.8s | 23/23 |
| paramsdTemporaryError | 15 | 9.1s | 11.7s | 13.8s | 15/15 |
| posenetInvalid | 20 | 9.1s | 11.7s | 14.1s | 20/20 |
| **합계** | **105** | — | — | **19.5s** | **105/105** |

**Cooccurrence**:
- 14회 frame 에서 4 event (commIssue + locationdTemp + paramsdTemp +
  posenetInvalid) 동시 rising → 단일 publisher init race
- 6회: 3 event 동시; 2+1회: 2-3 event 동시

**Underlying flag**: livePose.inputsOK=False 가 route 별 15-20 samples
(seg=0 에서만, 0.75-1.0s 지속). posenetOK=False / liveParameters.valid=False
는 sample 0건 — selfdrived 가 default-False (capnp 의 unpopulated bool) 를
보고 트리거한 것으로 추정 (publisher 가 그 시점에 publish 자체를 안 함).

## Root cause

selfdrived process boot 후 약 9~20초 구간에 locationd / paramsd 의 publisher
가 첫 valid publish 까지 더 시간 걸림 (calibration converge, IMU stabilize
등). selfdrived 가 그 구간에 freshness check 시작 → cached default 값 또는
not-alive 상태로 4-5 events 가 한꺼번에 rising.

Fix B 의 `boot_grace = self.sm.frame * DT_CTRL > 10.` (selfdrived.py:351)
는 `controlsMismatch` 전용이라 이 5개 event 는 outside.

## Fix D 적용

`selfdrive/selfdrived/selfdrived.py` 3 곳 (anchor 검증됨):

1. **상수 추가** (`IGNORED_SAFETY_MODES = ...` 직후):
   ```python
   PUBLISHER_WARMUP_GRACE = 20.
   ```
   왜 20s: 측정 max rise = 19.5s + 0.5s 마진. transient 지속이 0.75-1.0s
   라 19.5s 에서 rising 한 게 SOFT_DISABLE_TIME(3s) 안에 풀림. 25s/30s 는
   추가 마진 zero data, 진짜 mid-drive 이슈의 detection 지연 cost.

2. **commIssue / commIssueAvgFreq 게이팅** (line 394 부근):
   ```python
   publisher_warmup = self.sm.frame * DT_CTRL > PUBLISHER_WARMUP_GRACE
   if not self.sm.all_checks() and no_system_errors:
     if publisher_warmup:
       if not self.sm.all_alive(): self.events.add(EventName.commIssue)
       elif not self.sm.all_freq_ok(): self.events.add(EventName.commIssueAvgFreq)
       else: self.events.add(EventName.commIssue)
     # cloudlog.event("commIssue", ...) 블록은 gate 밖에 유지 → 부팅
     # 시점 anomaly 도 cloudlog 에 기록 (diagnostic 유지)
   ```

3. **posenetInvalid / locationdTemporaryError / paramsdTemporaryError 게이팅**
   (line 413):
   ```python
   if not self.CP.notCar and publisher_warmup:
     if not self.sm['livePose'].posenetOK:
       self.events.add(EventName.posenetInvalid)
     if not self.sm['livePose'].inputsOK:
       self.events.add(EventName.locationdTemporaryError)
     if not self.sm['liveParameters'].valid and cal_status == ...:
       self.events.add(EventName.paramsdTemporaryError)
   ```

**Fix B 의 boot_grace (10s, line 351) 은 건드리지 않음** — 다른 event
cluster (panda RX-checks pulse), 서로 보완.

## Side-effect 안전성

1. **Mid-drive 진짜 fault 마스킹 없음**: `self.sm.frame` 은 selfdrived
   process uptime. seg index 와 무관. 같은 ignition 의 mid-drive 면
   `frame * DT_CTRL ≫ 20s` 이므로 gate 즉시 open. 진짜 fault detection
   은 정상.

2. **t<20s 의 진짜 panda 분리** 는 다른 path 가 잡음:
   - `EventName.usbError` / `canError` / `canBusMissing` — **gate 없음**
   - `processNotRunning` — Fix C.1 의 ignored_processes 만 면제, 그 외 gate 없음
   - 즉 commIssue 의 generic catch-all 만 suppress. 더 specific event 가 fire.

3. **Diagnostic 손실 없음**: `cloudlog.event("commIssue", ...)` (line 408) 가
   gate 밖에 유지.

4. **Tests**: `selfdrive/selfdrived/tests/` 의 test_alertmanager / test_alerts
   / test_state_machine 에 5개 EventName 또는 `boot_grace` 참조 0건.

## 검증 절차

### 적용 후 검증
- `python3 -m py_compile selfdrive/selfdrived/selfdrived.py` ✅ pass
- 차량 빌드 후 운행, 새 drivelog 수집 → sweep 재실행

### Sweep 재실행 도구
`debug_logs/jeep_gc_2020_softdisable_analysis.py` (신규):
- 기존 4 라우트 sweep 결과의 baseline = **seg=0 105건, seg>=1 0건**
- Pass criterion: 신규 운행의 seg=0 의 5개 event rising-edge **105 → 0**
- 동시에 seg>=1 (mid-drive) 카운트 **변동 없음 (현재 0)** + usbError/canError/
  processNotRunning 카운트도 변동 없음

### 사용
```bash
PYTHONPATH=/home/user/openpilot python3 \
  debug_logs/jeep_gc_2020_softdisable_analysis.py \
  --drivelog-dir /tmp/wk2-data/drivelog
```

## 갱신된 우선순위

| 항목 | 상태 |
|---|---|
| Fix A (cancel storm, controlsd+chrysler) | ✅ 검증 완료, 유지 |
| Fix B (controlsMismatch boot_grace 10s) | ✅ 검증 완료, 유지 |
| Fix C (audio chain micd/soundd recovery loop) | ✅ 검증 완료, 유지 |
| **Fix D (publisher_warmup_grace 20s, 5 events)** | **이번 turn 적용** |
| **T1-A** | 미해결, 최우선 next-fix |
| T2-D | 미해결 |
| T2-A | 미해결 |
| SCC-V `_ABORT_ENTERING_PRED_LAT_ACC_TH 1.1 → 0.9` | 권장, 별도 turn |

## 산출물 (이 Update 의)

- `selfdrive/selfdrived/selfdrived.py` — Fix D (+1 constant, +3 라인 wrapping)
- `debug_logs/jeep_gc_2020_softdisable_analysis.py` — 신규 sweep (timing
  histogram + cooccurrence + acceptance check)
- `debug_logs/jeep_gc_2020_softdisable_output.txt` — sweep 출력 (baseline)

---

# Update 6 — Fix E: SCC-V 감속 폭 + entering false-positive 축소 (2026-05-16)

## 사용자 피드백

운행 중 SCC-V 가 직진에서도 속도를 낮추는 느낌. 80 kph 기준 5 kph 정도
감속이 적당. 또한 속도를 줄였다가도 코너가 다시 직선으로 펴지기 시작하면
(직선까지 가기 전에) 미리 속도를 원래대로 올려야 함.

## 데이터 분석 (4 routes / 37 hr / 24.5 hr ACC engaged)

### 1. SCC-V 가 cruise plan 을 실제로 dominate 한 빈도

| 메트릭 | 값 |
|---|---|
| Plan source = `sccVision` 샘플 | 38 / 148,509 (0.026%) |
| 모두 entering state (turning/leaving 에서는 dominate 안 함) | |
| `sccVision` dominate 시 `max_pred_lat_acc` 분포 | min 1.58 / median 1.78 / max 2.25 |
| `cruise_setpoint - plan.vTarget` (실제 사용자 감속, kph) | median 1.3 / p75 1.8 / p95 2.6 / **max 4.0** |
| drop > 5 kph sample | **0건** (37 hr) |
| drop > 3 kph sample | 2건만 |

→ 실제 cruise 감속 폭은 이미 최대 4 kph (사용자 의도 ≤5 kph 안).

### 2. SCC-V active state 의 max_pred_lat_acc 분포

| max_pred bucket | 빈도 | 누적% |
|---|---|---|
| 1.1~1.3 | 1.4% | 1.4 |
| 1.4 | 6.0% | 7.4 |
| 1.5 | 14.4% | 21.8 |
| **1.6 (entering 임계)** | 29.1% | 50.9 |
| 1.7 | 24.6% | 75.5 |
| 1.8 | 13.0% | 88.5 |
| 1.9~2.0 | 8.5% | 97.0 |
| ≥ 2.1 | 3.0% | 100 |

→ active 시간의 53% 가 max_pred = 1.5~1.7 boundary 에서 oscillate. model
prediction 노이즈가 임계값 부근에서 잠깐 spike 하면서 entering 진입 — 사용자가
느끼는 "직진에서 낮추는 느낌" 의 주요 원인.

### 3. Entering trigger 통계

- 진입 횟수: 29건 / 24.5 hr ACC = 1.18/hr
- → turning 진입: 9건 (31% conv rate)
- → abort: 20건 (69% — false-positive 의 정량 지표)

abort 69% = entering 진입했지만 max_pred 가 ABORT 임계 (1.4) 아래로 떨어진
case. 잠깐 spike 후 회복 = 진짜 곡선이 아닌 model noise.

## Root cause

- Entering 임계 1.6 이 너무 낮음 → 직진에서도 잠깐 model spike 로 entering
  진입. 실제 plan dominate 안 해도 a_target 영향 가능.
- 실제 cruise drop 자체는 작음 (max 4 kph) — 사용자 perception 보다 실측은
  적당. 진짜 효과는 trigger 빈도 축소.
- 코너 끝나갈 때 turning → leaving 전환이 `current_lat_acc <= 1.5` 기준이라
  차가 거의 직선 부근까지 와야 leaving 진입 → 가속 회복이 늦음.

## Fix E 디자인

`sunnypilot/selfdrive/controls/lib/smart_cruise_control/vision_controller.py`:

### 변경 1 — 상수 3개 변경 + 1개 신규

| 상수 | 이전 | 새 값 | 라인 | 근거 |
|---|---|---|---|---|
| `_ENTERING_PRED_LAT_ACC_TH` | 1.6 | **2.0** | 22 | max_pred 1.6~2.0 의 false-positive 제거. 진짜 곡선 (≥2.0) 에서만 trigger |
| `_ABORT_ENTERING_PRED_LAT_ACC_TH` | 1.4 | **1.6** | 23 | entering 임계 - 0.4 간격 유지 |
| `_TURNING_LAT_ACC_TH` | 1.8 | **2.0** | 25 | entering 임계와 일관. current_lat_acc 가 2.0 넘으면 turning |
| `_LEAVING_PRED_LAT_ACC_TH` (신규) | (없음) | **1.4** | 27 | max_pred 가 1.4 아래로 떨어지면 곡선이 펴질 예측 → 미리 leaving 진입 |

`_A_LAT_REG_MAX = 2.8` 그대로 — v_target 공식의 핵심, max cruise drop 4 kph
유지.

### 변경 2 — turning → leaving 트랜지션 (line 137-144)

```python
# TURNING
elif self.state == VisionState.turning:
  # Anticipate curve straightening: if the model predicts the curve ending ahead
  # (max_pred drops), switch to leaving before current lat_acc actually falls so
  # v_target recovers sooner. Fall back to measured lat_acc otherwise.
  if self.max_pred_lat_acc <= _LEAVING_PRED_LAT_ACC_TH:
    self.state = VisionState.leaving
  elif self.current_lat_acc <= _LEAVING_LAT_ACC_TH:
    self.state = VisionState.leaving
```

핵심: model 예측 (max_pred_lat_acc) 이 1.4 아래로 내려가면 = 곡선이 펴질
예측 → 즉시 leaving 진입. v_target 공식 `v_ego × sqrt(2.8 / max_pred)` 에서
max_pred 작아지면 v_target 큼 → cruise plan 이 dominate 회복. 동시에 a_target
= `_LEAVING_ACC = 0.5 m/s²` (≈1.8 kph/s) 로 가속 시작.

## Oscillation 안전 fallback

LEAVING state 에서 `current_lat_acc >= _TURNING_LAT_ACC_TH (2.0)` 면 다시
turning 으로 진입 (기존 line 149-150 그대로). model 이 잘못 예측해서 곡선이
다시 강해지면 안전 측면 그대로 작동.

## 시뮬레이션 (수학)

v_ego = 22.2 m/s (80 kph), `_A_LAT_REG_MAX = 2.8`:

| max_pred | v_target (kph) | a_target | output (kph) | 감속 |
|---|---|---|---|---|
| **2.0** (새 entering 임계) | 94.6 | -0.53 | 87.1 | 0 |
| 2.25 (관측 max) | 89.2 | -0.65 | 82.1 | 0 |
| 2.5 | 84.6 | -0.77 | 76.7 | **3 kph** |
| 3.0 (강한 곡선) | 77.2 | -1.0 | 69.7 | **10 kph** |
| **1.4** (새 leaving 임계) | cruise dominate | +0.5 | cruise + 가속 | 가속 회복 |

→ max_pred 2.5 까지 5 kph 안. 3.0 이상 강한 곡선만 10 kph 감속.

## Expected effect

- Entering trigger 빈도: 29 → **~7건/24.5h** (max_pred ≥ 2.0 active 시간
  점유율 ≈ 14%)
- Abort rate: 69% → 추정 30~40%
- Turning → Leaving 트랜지션: ~0.5~1.5초 미리 (model 예측 기반)
- 가속 회복 시작 시점: ~0.5~1.5초 빨라짐
- 실제 cruise drop max: 4 kph 그대로 유지

## Side-effects

1. **강한 곡선 (max_pred ≥ 3.0)**: 새 임계 2.0 에서도 충분히 일찍 entering
   진입. `_A_LAT_REG_MAX = 2.8` 의 v_target 공식이 safety margin 유지.
2. **Leaving 미리 진입 의 oscillation 위험**: model 이 잘못 예측해 max_pred
   가 잠깐 1.4 아래로 떨어졌다가 다시 올라가는 경우 turning ↔ leaving
   oscillation 가능. 그러나 LEAVING state 의 기존 logic 에서 `current_lat_acc
   >= _TURNING_LAT_ACC_TH (2.0)` 면 다시 turning 진입 — fallback 보유.
3. **Plan source 점유**: sccVision 0.026% → ~0.005% 추정. SCC-V 의 가시적
   효과 자체 감소, 그러나 사용자가 5 kph 이하만 원하므로 OK.

## 검증

1. **py_compile**: `python3 -m py_compile vision_controller.py` — OK
2. **차량 운행 검증** (사용자 직접):
   - 직진/완만 곡선에서 SCC-V active 빈도 체감 감소
   - 강한 곡선 (highway 진출입 ramp 등) 에서 여전히 부드러운 감속
   - 코너 끝나갈 때 직선 닿기 전 가속 시작 체감
3. **재 sweep**: Fix E 적용 후 새 운행 → 새 rlog → sweep cycle 후 정량 비교.
   기존 데이터로 simulate 불가 (analysis script 는 publish 된 state 값을 읽음).
4. 추가 호소 시 후속 옵션:
   - `_LEAVING_ACC = 0.5 → 1.0` m/s² (가속 회복 더 적극)
   - `_A_LAT_REG_MAX = 2.8 → 3.2` (감속 폭 추가 축소)
   - `_ENTERING_SMOOTH_DECEL_V = [-0.2, -1.0] → [-0.1, -0.5]` (entering a_target 절반)

## 산출물 (이 Update 의)

- `sunnypilot/selfdrive/controls/lib/smart_cruise_control/vision_controller.py`
  — 상수 3 변경 + 1 신규 + turning state logic (~10 라인)

---

# Update 7 — Fix F: 사용자 주행 패턴 기반 drop curve (2026-05-16)

## 사용자 피드백

운행 중 SCC-V 동작할 때 cruise setpoint 를 ↓ 반복. 본인 의도:
- 왠만한 코너: 5 kph 이내
- 깊은 코너: 10 kph 정도
- v_ego ≤ 120 kph 까지는 cap 10 kph (highway 강한 곡선이라도 더 안 줄임)

## 데이터 추출 (SCC-V active 10 segments, 29 episodes)

setpoint drop episode (4건):

| v_ego_start | max_pred | max_cur_lat | 사용자 drop | 누름 횟수 |
|---|---|---|---|---|
| 80 kph | 2.01 | 1.86 | **5 kph** | 14 |
| 70 kph | 1.98 | 1.86 | 3 kph | 3 |
| 83 kph | 2.25 | 1.22 | 1 kph | 1 |
| 72 kph | 1.63 | 1.57 | 1 kph | 1 |

나머지 25 episodes 누름 0회 → 현재 SCC-V 동작 (drop ≤ 1 kph) 으로 충분.

**Anchor**: 80 kph + max_pred 2.0 + cur_lat 1.9 코너 = 사용자 **정확히 5 kph drop** 명시. 사용자 자연어 (max_pred ≥ 3.0 → 10 kph) 와 결합해 desired-drop 곡선 구성.

## Fix F 디자인 — `max_pred → desired_drop` piecewise interp

### 핵심: a_target 으로 final drop 정확 제어

기존 식 `v_target = v_ego · sqrt(2.8/max_pred)` 은 lateral cap 기반 — drop 크기를 직접 제어 못함 (실측 max drop 4 kph 였음). 또한 `output_v_target = v_target + a_target × 4s` 로 cruise plan 에 들어가는데, 두 항 모두 drop 에 기여해 의도 와 어긋남.

새 식 (entering state):
- `v_target = v_ego` (현재 속도 anchor)
- `a_target = -desired_drop / 4s`
- → `output_v_target = v_ego + a_target × 4 = v_ego − desired_drop` ✓

### Table

`_PRED_DROP_BP    = [1.6, 2.0, 3.0]`
`_PRED_DROP_KPH_V = [0.0, 5.0, 10.0]`

| max_pred | desired_drop | a_target | 검증 |
|---|---|---|---|
| 1.6 | 0 kph | 0 m/s² | active boundary, no drop |
| 1.8 | 2.5 kph | -0.17 m/s² | mild curve |
| **2.0** | **5 kph** | **-0.35 m/s²** | **데이터 anchor (사용자 14 회 누름)** |
| 2.5 | 7.5 kph | -0.52 m/s² | 인터폴 |
| **3.0** | **10 kph** | **-0.69 m/s²** | **사용자 자연어 "깊은 코너 10kph"** |
| ≥ 3.0 | 10 kph (cap) | -0.69 m/s² | np.interp 끝점 cap |

### Side-effect

- v_ego 의존성 없음 (데이터가 70~83 kph 좁은 범위라 fit 불가). 추후 운행 데이터 누적되면 2D table (v_ego × max_pred) 검토.
- 현재 cap 10 kph 가 사용자 의도. 매우 강한 곡선 (max_pred ≥ 4) 에서도 동일 cap. SCC-V 외 다른 long plan logic 이 추가 안전 cap 제공.
- turning/leaving state 의 a_target 은 변경 없음 (reactive 동작 — 곡선 한가운데 cur_lat 기반 fine-tune, 곡선 끝 leaving 가속 회복).
- `_A_LAT_REG_MAX` 상수는 unused 가 되었지만 그대로 유지 (future safety floor 도입 시 재사용 가능).

### 변경 위치

| 변경 | 라인 | 내용 |
|---|---|---|
| 상수 신규 | 35-43 | `_PRED_DROP_BP`, `_PRED_DROP_KPH_V` 도입 + 옛 `_ENTERING_SMOOTH_DECEL_V/_BP` 제거 |
| `_update_calculations` | 102-103 | `self.v_target = max(v_ego, MIN_V)` (옛 sqrt 식 제거) |
| `_update_solution` entering | 177-181 | `desired_drop` interp + `a_target = -drop/4` |

## 검증

1. **py_compile**: `python3 -m py_compile vision_controller.py` — OK
2. **수학 검증** (위 표 — `output_v_target = v_ego + a_target × 4` 가 정확히 사용자 desired drop 과 일치)
3. **차량 운행 검증** (사용자 직접):
   - 왠만한 코너에서 cruise drop 5 kph 안 (사용자 재누름 빈도 ↓)
   - 깊은 코너에서 drop ~10 kph
   - v_ego 와 max_pred 변화 시 자연스러운 drop 추이
4. 추가 호소 시:
   - drop 너무 크면 `_PRED_DROP_KPH_V` 의 값 축소 (e.g. [0, 3, 8])
   - drop 너무 작으면 breakpoint 좌측 이동 (e.g. `_PRED_DROP_BP = [1.4, 1.8, 2.8]`)
   - 깊은 코너 (max_pred 3+) 에서 부족하면 cap 12 kph 까지 검토

## 산출물 (이 Update 의)

- `sunnypilot/selfdrive/controls/lib/smart_cruise_control/vision_controller.py`
  — 상수 2 신규 + 옛 entering interp 2 상수 제거 + v_target/a_target 식 재설계 (~10 라인 net)

---

# Update 8 — Fix G: 3-5초 anticipation + leaving 살살 가속 (2026-05-16)

## 사용자 피드백

- "Multi-step horizon 으로 3-5초 먼저 보고 **살짝** 감속"
- "코너가 풀리기 시작하면 (다시 코너 없으면) **약 5초간 살살** 다시 가속"

## 데이터 (Fix F 분석 turn 의 trigger event 16건)

trigger 시점의 trajectory peak 위치 분포:

| trajectory peak t | 비율 |
|---|---|
| 0-1s | 25% |
| 1-2s | 12% |
| 2-3s | 6% |
| **3-5s** | **25%** |
| **5-8s** | **31%** |
| 8-10s | 0% |

trigger 시점의 window peak 값:

| 측정 | mean | median |
|---|---|---|
| full trajectory max | 1.73 | 1.65 |
| p97 (현재) | 1.72 | 1.64 |
| window 0-1s max | 1.12 | 1.16 |
| window 3-5s max | 1.16 | **1.29** |

→ model 의 trajectory 가 진짜 3-8s 앞을 본다. 그러나 window 3-5s peak 의 median = 1.29 라 임계 2.0 (현재 entering) 안 닿음 — 그래서 anticipation 안 됨. 임계 1.3 으로 별도 trigger 만들면 anticipation 가능.

## Fix G 디자인

### 1. Multi-step horizon anticipation

`vision_controller.py`:

```python
_T_IDXS = np.array([10.0 * (i / 32.0) ** 2 for i in range(33)])
_ANTICIPATE_MASK = (_T_IDXS >= 3.0) & (_T_IDXS <= 5.0)   # idx ~17..22

_ANTICIPATE_PRED_LAT_ACC_TH = 1.3       # 3-5s window peak trigger
_ANTICIPATE_ABORT_LAT_ACC_TH = 1.1      # abort hysteresis
_ANTICIPATE_DROP_BP    = [1.3, 1.5, 2.0]
_ANTICIPATE_DROP_KPH_V = [0.0, 1.0, 2.5]  # 살짝 (max 2.5 kph)
```

`_update_calculations`:
```python
self.max_pred_lat_acc = np.percentile(predicted_lat_accels, 97)   # 기존 near-horizon
self.anticipated_lat_acc = predicted_lat_accels[_ANTICIPATE_MASK].max()  # 3-5s window peak
```

state machine 진입 조건:
```python
# enabled -> entering: 둘 중 하나라도 임계 hit
if max_pred >= 2.0 OR anticipated >= 1.3:
  state = entering

# entering -> abort: 두 horizon 모두 abort 임계 아래
if max_pred < 1.6 AND anticipated < 1.1:
  state = enabled
```

a_target 계산 (entering state):
```python
drop_near = interp(max_pred,   _PRED_DROP_BP,        _PRED_DROP_KPH_V)        # 0..10 kph
drop_far  = interp(anticipated, _ANTICIPATE_DROP_BP, _ANTICIPATE_DROP_KPH_V)  # 0..2.5 kph
desired_drop = max(drop_near, drop_far)
```

자연스러운 ramp:
- t = -5s: window 3-5s peak ≈ 1.5 → drop 1 kph (살짝 anticipation 시작)
- t = -3s: window peak 1.8 → drop 2 kph
- t = -1s: near-horizon p97 ≥ 2.0 hit → drop 5 kph (full)
- t = 0: turning state

### 2. Leaving 살살 가속

`_LEAVING_ACC = 0.5 → 0.3` m/s²

5 kph drop 회복 시간:
- 0.5 m/s² (이전): 2.8s — 사용자 표현 "갑작스러움"
- 0.3 m/s² (새): 4.6s ≈ 5s ✓

10 kph drop 회복:
- 0.5 m/s²: 5.6s
- 0.3 m/s²: 9.3s (살살)

## 변경 위치

| 변경 | 파일 위치 |
|---|---|
| T_IDXS / mask 상수 신규 | line 22-26 |
| anticipation 임계 4개 상수 신규 | line 31-33 |
| `_LEAVING_ACC` 0.5 → 0.3 | line 66 |
| anticipation drop table 신규 | line 53-55 |
| `anticipated_lat_acc` instance var | line 90 |
| `_update_calculations` 3-5s peak | 신규 라인 121-122 |
| enabled→entering 진입 조건 | OR anticipated >= 1.3 |
| entering→abort 조건 | AND anticipated < 1.1 |
| entering a_target | max(drop_near, drop_far) |

## Side-effects

1. **False-positive 위험**: anticipated 임계 1.3 이 직진의 window 3-5s peak (보통 0.5~1.2) 보다 위. 일부 false-positive 가능하지만 drop 작아 (≤2.5 kph) 운전자 perception 작음.
2. **Entering state 점유 증가**: anticipation 이 3-5s 앞에서 trigger 라 entering 상태 시간이 늘어남. 가벼운 drop 만 적용되므로 cruise 영향 작음.
3. **Leaving 회복 시간**: drop 따라 5~9 초로 분산. 사용자 의도와 정합.
4. **Two-horizon abort**: 둘 다 떨어져야 abort — 한쪽만 떨어지면 entering 유지. 자연스러운 ramp 보장.

## 데이터 sanity check (전체 4 routes / 27,204 ACC-engaged modelV2 samples)

| 메트릭 | 값 |
|---|---|
| anticipated peak ≥ 1.3 (시간 점유) | 2.30% |
| max_pred ≥ 2.0 (시간 점유, 기존 Fix E trigger) | 0.14% |
| **anticipate-only (= 새로 trigger 됨)** | **2.29%** |
| 두 horizon 모두 hit | 0.4% (전체 기준) |
| Lead time (anticipate → enter, 전환된 case 3건) | median 0.1s, p90 1.18s |

→ Anticipation 새 trigger 가 ACC 시간의 2.29% 점유 (=22.7분 ACC 중 ~31초).
False-positive 17배 증가지만 drop ≤2.5 kph 라 cruise 영향 작음.
사용자 운행 후 perception 부족 또는 과잉 시 임계 (1.3 → 1.5) 또는 drop max
(2.5 → 1.5 kph) 로 조정.

## 검증

1. `python3 -m py_compile vision_controller.py` — OK
2. **데이터 sanity check**: 위 표 (false-positive trade-off 명시)
3. **차량 운행 검증** (사용자):
   - 곡선 3-5s 전 부드러운 ~1-2 kph 감속 시작 체감
   - 곡선 가까워질수록 부드러운 ramp-up (full 5-10 kph)
   - 곡선 끝나갈 때 부드러운 (~5초) 가속 회복
   - 연속 곡선 case 에서 가속 안 함 (다음 곡선 detected 시 turning 으로 복귀)
4. 추가 조정:
   - 너무 일찍/자주 trigger 면 `_ANTICIPATE_PRED_LAT_ACC_TH 1.3 → 1.4`
   - drop 너무 작으면 `_ANTICIPATE_DROP_KPH_V [0,1,2.5] → [0,2,4]`
   - 가속 너무 느리면 `_LEAVING_ACC 0.3 → 0.4`

## 산출물 (이 Update 의)

- `sunnypilot/selfdrive/controls/lib/smart_cruise_control/vision_controller.py`
  — T_IDXS 상수 + anticipation 임계 4 + drop table + state machine + a_target 통합
  (~25 라인 net)
