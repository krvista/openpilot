# Post-PR#11 Audit — Drives 14, 15, 16

**Build**: commit `e62a5257` (PR #11 merged), branch `i6n`, dirty=False — all 3 drives.

## (1) Drivelog 업로드 확인

| Drive | Route ID | rlog | qlog | seg range | 상태 |
|---|---|---|---|---|---|
| 14 | `25a48a05fa` | 37 | 37 | 0-36 | ✅ 완전 |
| 15 | `c27907e549` | 40 | 40 | 0-39 | ✅ 완전 |
| 16 | `9903825867` | 36 | 37 | 0-35 (rlog), 0-36 (qlog) | ⚠️ seg 36 rlog 누락 (드라이브 종료 직후, 일반적) |

추가로: drive 14 seg 36 rlog와 drive 15 seg 39 rlog는 압축 truncated 상태로 마지막 frame 일부 손실 (각 드라이브 마지막 seg, 비치명적).

## (3) LFA_ICON 정상화 — 데이터로 검증

CCNC_0x161.LFA_ICON (cluster green icon driver) 분포 — `mads_on` 동안 어떤 값이 떴는지 직접 측정:

| Drive | mads_on→HIDDEN(0) | mads_on→GRAY(1) | mads_on→**GREEN(2)** | mads_on→WHITE(3) |
|---|---|---|---|---|
| 14 | 1.0% | 52.7% | **32.4%** | 1.5% |
| 15 | 1.0% | 33.4% | **62.9%** | 1.4% |
| 16 | 1.1% | 35.1% | **42.6%** | 1.6% |

비교: drivelog 00000013 (PR #9 build) 에서 `mads_on|GREEN`은 **0 frames** 였음. PR #11 이후 모든 드라이브에서 GREEN이 31-63% 차지 — **자연스럽게 정상화됨**.

추정 원인: PR #10의 PR #9-mechanism revert가 squash-merge collision으로 no-op이 되어 `lfa_sync_pulse` 코드가 i6n에 그대로 남아있는데, 이게 실제로 어떤 방식이든 cluster gateway에 영향을 주거나, op의 LKAS_ALT TX 다른 필드 (LKAS_ANGLE_ACTIVE, LKA_ASSIST 등) 가 cluster를 정상 구동 중. 정확한 메커니즘은 미상이지만 **현재 동작 = 의도된 정상**.

→ **사용자 결정대로 별도 패치 불필요**.

## (4) 전체 로그 오류 audit

### Panda safety (HW 레벨)
| Drive | busOff Δ | sendErr Δ | fwdErr Δ | faults Δ |
|---|---|---|---|---|
| 14 | 0 | 0 | 0 | 0 |
| 15 | 0 | 0 | 0 | 0 |
| 16 | 0 | 0 | 0 | 0 |

→ **위험도 NONE**. 모든 panda 카운터 clean.

### carControl 무결성
- NaN steering 명령: 0 (전체 3 드라이브 660,000+ 프레임)
- 270° 이상 oversized 명령: 7-11k 프레임/드라이브 = **정상** (실제 큰 회전 시 wheel과 op 명령이 같이 큼; CC oversize ≈ CS oversize)

→ **위험도 NONE**.

### onroadEvents (수집 이벤트)

| 이벤트 | 14 | 15 | 16 | 위험도 | 분석 |
|---|---|---|---|---|---|
| `gasPressedOverride` | 992 | 1014 | 1097 | NONE | ACC 가속 페달 누름 — 정상 |
| `steerOverride` | 616 | 520 | 725 | NONE | 운전자 조향 개입 — 정상 |
| `preEnableStandstill` | 99 | 0 | 92 | NONE | 정차 진입 — 정상 |
| `laneChange` | 94 | 76 | 79 | NONE | 차선 변경 — 정상 |
| `curveSpeedAdvisory` | 40 | 78 | 53 | NONE | 곡선 속도 안내 (8차 패치) — 정상 |
| `preLaneChangeLeft/Right` | 58 | 41 | 9 | NONE | 차선 변경 준비 — 정상 |
| `commIssue` + `commIssueAvgFreq` | 14 | 14 | 19 | LOW | 부팅 직후 transient — 정상 |
| `locationdTemporaryError` | 11 | 11 | 17 | LOW | GNSS 초기화 지연 — 정상 |
| `posenetInvalid` | 9 | 9 | 13 | LOW | 카메라 초기 정렬 미완 — 정상 |
| `paramsdTemporaryError` | 9 | 9 | 14 | LOW | params daemon 부팅 race — 정상 |
| `selfdriveInitializing` | 7 | 7 | 7 | LOW | 매 부팅 1-2초 — 정상 |
| `promptDriverDistracted` + `driverDistracted` | 0 | **28** | 0 | LOW | drive 15에서 운전자 14회 시선 이탈 — DMS 정상 동작 |
| `preDriverDistracted` | 0 | 7 | 0 | LOW | 시선 이탈 직전 경고 — 정상 |
| `laneChangeBlocked` | 1 | 3 | 0 | LOW | 차선변경 시도 차단 (BSM 또는 차선 미인식) — 정상 |
| `wrongGear` | 0 | 0 | 1 | LOW | 시동 직후 1회 — 정상 |

→ **위험도 LOW**: 모두 transient 또는 정상 driver-driven 이벤트. 패치 불필요.

### MADS 상태 전이 — 이상 없음
드라이브별 24-38 회 정상적인 `None→(1,1)` (boot→engaged), `(1,1)→(0,0)` (manual disable), `(0,0)→(1,1)` (re-engage). 비정상 oscillation 없음.

### cloudlog 가시성 — **MEDIUM 위험도 (개선 권장)**

**중요 발견**: PR #10/#11에서 추가한 `cloudlog.warning("LFA_ICON transition: ...")` 등 진단 로그가 **rlog의 androidLog 채널에 없음**. `cloudlog` (swaglog daemon) 출력은 별도 경로로 가서 오프라인 rlog 분석으로 볼 수 없음.

- 영향: PR #10의 LFA_ICON diag, PR #11의 expanded state log, FAULT_LFA 로그, VM_LIMIT_TRIP 로그 등 모든 진단 cloudlog가 사후 rlog 분석으로 invisible
- 위험도: **MEDIUM** — 실제 문제 미발생 시는 무해하지만, 향후 진단 시 데이터 부재
- 개선안: 진단 정보를 cereal 메시지 (e.g. `carControlSP.lateralAlerts` 확장) 또는 dedicated `customReservedRawData0` 등에 기록하면 rlog 캡처됨. PR로 처리.

---

## (2) 60°+ 회전 + 핸들 복원 시 op 개입 — **HIGH 위험도**

### 사용자 보고 시나리오
> "60도 이상의 좌회전 또는 우회전에 운전자가 직접 핸들을 돌려 들어갔다면 핸들을 운전자에게 자연스럽게 넘겨야 함. 회전이 끝난 후 다시 핸들이 센터로 복원하는 과정에서 운전자가 손힘을 뺄 때 op가 개입하여 차량 거동이 불안해지지 않도록"

### 데이터로 확인

3 드라이브 합계 60°+ 회전: **182건**. 이 중 운전자가 손힘 뺀 후 op이 핸들 복원에 개입한 사례 분석:

| Drive | 60°+ turns | analyzed | **concerning** | worst op-wheel deviation |
|---|---|---|---|---|
| 14 | 53 | 18 | **4** | 38.4° (peak -120.6° @19.2kph blinker) |
| 15 | 69 | 19 | **3** | **122.9°** (peak -223.8° @18.4kph blinker LR) |
| 16 | 60 | 33 | **1** | 18.6° (peak -399.1° @8.8kph blinker) |

총 **8건의 우려스러운 이벤트**. 정의 = op_active + wheel step >1.5°/frame + |op_angle - wheel| max >5° 동시 만족.

### Worst case 분석 (drive 15, peak -223.8°)

5초 윈도우 timeline (50ms 해상도):

```
 t (ms)  wheel°    torque    op_angle    op-wheel   v_kmh  pressed lat_active mads blinker
   ...
  2000  -223.8°   -503 Nm    -223.8°     +0.0°     18.4    0      0           1   LR  ◀ PEAK (snap matched)
   ...
  3400  -112.8°    -26 Nm    -205.5°    -92.7°     24.0    0      1           1   LR  ◀ RELEASE
  3500  ...
  4550    -4.8°   +455 Nm     -79.1°    -74.3°     29.5    0      1           1   LR
  5000    -0.7°   +172 Nm      -8.0°     -7.3°     30.3    0      1           1   LR
  5800    +7.7°   +197 Nm      -7.2°    -14.9°     28.6    0      1           1   -
  6750    -1.1°    -66 Nm      -0.2°     +0.9°     25.3    0      1           1   -
```

진단 시퀀스:
1. **t=2000 PEAK**: 운전자가 좌회전 진입. wheel=-223.8°, torque=-503 Nm (override factor=1.0). 좌우 blinker 둘 다 켜짐 (비상등 or 차로 변경 + 비상등). `lat_active=False` (op 비활성). `apply_angle_last`은 lateral.py 강제 snap으로 wheel과 일치.
2. **t=2000-3400**: 운전자가 핸들 풀면서 caster로 wheel 자연 복원 (-223° → -112°). 도중에 `lat_active` 다시 True로 전환. 이때 `apply_angle_last`은 snap exit 시점 값(-205°쯤)에서 출발.
3. **t=3400 RELEASE 직후**: wheel은 -112°이지만 op_angle=-205.5°. **op-wheel 차이 -92.7°**. op는 모델 예측 (~0° 직선 도로)으로 수렴하려 하지만 vtau LPF + VM rate limiter가 ~0.3°/frame로 제한 → 1초 넘게 lag.
4. **t=4550까지**: wheel은 거의 0°까지 회복했지만 op_angle은 아직 -79° → 운전자가 다시 +455 Nm 토크 가해 보정.
5. **t=5000+**: 결국 op_angle이 거의 0°로 안정. wheel은 살짝 +방향 오버슈팅 (+7°)을 운전자가 +172 Nm로 카운터스티어.

→ 운전자가 회복 단계 도중 두 번 추가 개입해야 함 (4550 +455 Nm, 5800 +197 Nm). "차량 거동 불안정"의 정량적 증거.

### 원인 가설 (코드 검토 기반)

`opendbc_repo/opendbc/car/hyundai/carcontroller.py:650-690` 의 `override_snapped` 메커니즘:
- Entry: `override_factor ≥ OVERRIDE_SNAP_ENTER_FACTOR` AND (`not blinker_on` OR `|torque|>HEAVY_SNAP_OVERRIDE_TQ=200 Nm`) — 3 frames 연속.
- Exit: `override_factor ≤ EXIT_FACTOR` — N frames 연속.
- 효과: snap 중 `apply_angle_last`이 매 frame `steer_angle_safe` (실제 wheel) 로 강제 동기화.

**문제**: snap exit 직후, `apply_angle_last`은 exit 순간의 wheel 값으로 끝남. 이후 VM rate limiter (`apply_steer_angle_limits_vm`)가 통제 — 최대 jerk 한계 (`MAX_LATERAL_JERK/v_ego²`) 로 step. 9 m/s 에서 ~2°/s = 0.02°/frame. wheel은 caster로 5-10°/frame 회복.

→ **rate limiter convergence (~0.02°/frame) ≪ wheel caster velocity (~5°/frame)** → op_angle이 wheel을 못 따라감.

### 개선안 (3 후보)

#### P1 — Recovery hold (권장, 표적 정확)
snap exit 이후 |wheel|이 충분히 작아질 때까지 `apply_angle_last`을 wheel로 계속 강제 동기화:

```python
# carcontroller.py snap state machine 확장
# 신규 state: post_override_recovery
# Entry: override_snapped → False 전환 시점에 |wheel|>=30°
# Exit: |wheel|<10° (back near center) OR N=200 frames (2s timeout)
# Effect: snap-style apply_angle_last forcing 유지
```
- 변경 라인: ~10 lines
- 위험도: LOW (snap_to_wheel 기존 로직 재활용)
- 회귀 추적성: 매우 좋음 (별도 state flag로 추적 가능)

#### P2 — Adaptive rate limit during catch-up (옵션)
`|apply_angle_last - wheel| > threshold` 일 때 rate limit ceiling을 일시 상승:

```python
catch_up = abs(apply_angle_last - wheel) > 15 and abs(wheel) > 10
if catch_up:
    max_angle_delta *= 5  # 또는 다른 적절한 배수
```
- 위험도: HIGH (rate limiter는 안전 critical — 모든 시나리오 회귀 테스트 필요)
- 비권장.

#### P3 — Extend snap exit hysteresis (대안)
`OVERRIDE_SNAP_EXIT_FRAMES` 를 `|wheel|`과 동적으로 연동:
```python
exit_frames_required = base_exit + max(0, abs(wheel) - 30) * 2  # 더 큰 각도에서는 exit 지연
```
- 위험도: LOW
- 단점: snap 종료를 무한정 지연시킬 수 있어 staleness 위험. P1보다 덜 명확.

### 권장: **P1**

병행 검증 (sim 가능): drive 15 worst event에 P1 메커니즘을 numpy로 simulate, `apply_angle - wheel` 평균 절댓값이 50ms 윈도우 내 <5° 떨어지는지 확인.

---

## 종합 위험도 요약 + 권장 액션

| 항목 | 위험도 | 권장 액션 |
|---|---|---|
| Drivelog 업로드 무결성 | NONE | 변경 없음 |
| LFA_ICON 정상화 | NONE | 사용자 결정대로 패치 없음 |
| Panda safety / CC NaN | NONE | 변경 없음 |
| onroadEvents (transient) | LOW | 변경 없음 |
| **60°+ 회전 후 복원 시 op 개입** | **HIGH** | **Patch #12: P1 (recovery hold)** |
| cloudlog 가시성 (offline 미보임) | MEDIUM | 차후 별도 PR (cereal 메시지 확장) |

다음 단계: Patch #12 plan 작성 → sim 검증 → 코드 변경 → PR.
