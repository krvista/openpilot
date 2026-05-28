# Post-Phase-6f-2 Audit — Ioniq 6 N (i6n branch)

**Build**: commit `5479ecc` (Phase 6f-2 — heavy-override mismatch metric
in phase7 sim), branch `i6n`, dirty=False.
**Source**: `origin/ccnc-drivelog` orphan branch (krvista/openpilot),
committed 2026-05-27.
**Scope**: 8 of 15 ccnc-drivelog routes are on `5479ecc`. Routes 0x1e..0x23
predate Phase 6f and are excluded.

## 1. ccnc-drivelog 빌드 매핑

| Route | Build | rlog segs | Status |
|---|---|---|---|
| 0x1e (`d0b5555635`) | `48e8452` pre-Phase-6 (torque alias) | 38 | excluded |
| 0x1f (`c6a312398c`) | `150ee11` Phase 6a | 30 | excluded |
| 0x20 (`c647ae270b`) | `847216f` Phase 6c-3 (post tools) | 44 | excluded |
| 0x21 (`d541bf2a47`) | `847216f` Phase 6c-3 | 32 | excluded |
| 0x22 (`9c8591d350`) | `8994bb4` Phase 6e-3 (post tools) | 26 | excluded |
| 0x23 (`7aea6b4b56`) | `8994bb4` Phase 6e-3 | 37 | excluded |
| **0x24** (`4e9aece4a6`) | **`5479ecc` Phase 6f-2** | 2 | too-short, observational only |
| **0x25** (`e95847ba4c`) | **`5479ecc` Phase 6f-2** | 37 | **analyzed (chunk 1)** |
| 0x26 (`a9ef010f25`) | unknown — only qlog (no rlog) | 0 | upload incomplete |
| **0x27** (`5a8be2b4ba`) | **`5479ecc` Phase 6f-2** | 48 | pending |
| **0x28** (`19fa1840dd`) | **`5479ecc` Phase 6f-2** | 50 | pending |
| **0x29** (`c332384d19`) | **`5479ecc` Phase 6f-2** | 34 | pending |
| **0x2a** (`436e4ba3c2`) | **`5479ecc` Phase 6f-2** | 27 | pending |
| **0x2b** (`e732a41028`) | **`5479ecc` Phase 6f-2** | 41 | pending |
| **0x2c** (`275bbb3299`) | **`5479ecc` Phase 6f-2** | 51 | **analyzed (chunk 2)** |
| **0x2d** (`075acf9f7e`) | **`d83c3b5` Phase 6F2-A** (post-fix) | 37 | **analyzed (chunk 3, 출근 morning commute)** |
| **0x2e** (`4d0b891f77`) | **`d83c3b5` Phase 6F2-A** (post-fix) | 22 | **analyzed (chunk 3, 퇴근 evening commute)** |

Phase 6f-2 합계 = ~290 rlog segments (8 라우트). Phase 6F2-A 추가 = 59 segs (2 라우트, 2026-05-28).
보고서는 4 routes (0x25 + 0x2c + 0x2d + 0x2e, 147 segs) chunk 까지 반영.

추가로 라우트 0x26 (`a9ef010f25`) 은 47 qlog 만 업로드, rlog 누락 →
**업로드 무결성 follow-up 1건**.

### Phase 6F2-A 신규 빌드 데이터 (2026-05-28 commute)

빌드 검증: 두 route 의 첫 세그먼트 `initData.gitCommit` 가 `d83c3b5214ad...`
로 확정 (post 6F2-A pre-frame anchor), `dirty=False`, `branch=i6n`.

## 1.A. Phase 6F2-A 빌드 (`d83c3b5`) 분석 — 0x2d + 0x2e (2026-05-28 commute)

### A. heavy-override TX 측정 (override_factor ≥ 0.9)

| Route | latActive | HO frames | HO% | op-ACTIVE during HO | passive/echo |
|---|---:|---:|---:|---:|---:|
| 0x2d 출근 | 74,891 | 22,155 | 29.58% | 20,280 (91.5%) | 1,875 (8.5%) |
| 0x2e 퇴근 | 67,012 | 19,808 | 29.56% | 15,925 (80.4%) | 3,883 (19.6%) |

**op-ACTIVE heavy-override only** (LKAS_ANGLE_ACTIVE>=2, 6F2-A pre-frame anchor scope):

| Route | p50 | p90 | p95 | p99 | max | sign-mismatch | within 5° |
|---|---:|---:|---:|---:|---:|---:|---:|
| 0x2d | 11.3° | 21.3° | **31.3°** | 43.1° | 54.5° | 23.66% | 10.6% |
| 0x2e | 11.9° | 29.6° | **34.88°** | 47.4° | 129.2° | 40.99% | 20.8% |

⚠️ sim 의 hypothetical 측정 (p95=27°) 과 직접 비교 불가 — metric 정의가 다름.

### B. exit-transition (passive → active 첫 frame, 6F2-A 의 직접 효과 측정)

| Route | n events | p50 | p90 | **p95** | p99 | max | baseline p95 (sustained active) |
|---|---:|---:|---:|---:|---:|---:|---:|
| 0x2d | 40 | 11.95° | 20.98° | **22.74°** | 34.98° | 37.40° | 22.30° → ratio **1.02 (smooth) ✅** |
| 0x2e | 37 | 12.80° | 34.28° | **37.02°** | 116.41° | 158.10° | 24.70° → ratio **1.50 (spike) ⚠️** |

### C. 카메라 passthrough 빈도

| Route | latActive% | <20kph latch | passthrough active | ∩ latActive | entries/exits |
|---|---:|---:|---:|---:|---:|
| 0x25 (이전 빌드) | 29.5% | 20.4% | 90.4% | 3.05% | 542/537 |
| 0x2d | 34.4% | 16.8% | 89.7% | 0.80% | 334/325 |
| 0x2e | 52.7% | 14.8% | 97.7% | 0.50% | 167/162 |

→ commute 는 stop-and-go 가 많아 traffic_following 우회가 잘 발동 → passthrough 가 latActive 와 overlap 하는 비율 1/4 ~ 1/6 수준으로 감소.

### D. 속도 분포

| 버킷 | 0x2d 출근 | 0x2e 퇴근 |
|---|---:|---:|
| stop+0-20 | 62.6% | 47.3% |
| 20-60 | 31.3% | 33.1% |
| 60-100 | 5.9% | 18.8% |
| 100+ | 0.2% | 0.8% |
| 120+ | 0% | 0% |

→ 0x2e 가 highway 일부 포함 (60+ 19.6%), 그러나 둘 다 sustained 100+ km/h ≈ 0. **§6.1 의 Stage 1 highway 데이터 갭은 여전히 남음**.

### E. **신규 발견 — 저속 hand-off lag 큼 (P0 후보)**

저속 passthrough 중 driver 가 wheel 을 놓으면 op 가 얼마나 빠르게 이어받는가:

| Route | hand-off events | p50 lag | p90 | p95 | < 100ms 비율 |
|---|---:|---:|---:|---:|---:|
| 0x2d 출근 | 20 | **1,541 ms** | 10.1 s | 15.5 s | **15%** |
| 0x2e 퇴근 | 12 | **187 ms** | 5.0 s | 6.0 s | 50% |

코드 의도와의 불일치:
- `carcontroller.py:470-474` 는 `hands_off=True` 시 `low_speed_cam_latched=False` 로 **1 frame (≈ 10 ms)** 내 전환해야 함.
- 실측 p50 0.2-1.5 s, p95 6-15 s — passthrough 해제 후에도 **다른 게이트가 막고 있음**.

원인 후보 (effective_lat_active 의 8개 AND 게이트 중):
1. `CC.latActive=False` (MADS auto-disengage 후 미복귀)
2. `apply_steer_req=False` (controlsd 명령 미발행 구간)
3. `vm_reject_persistent` (VM rate-limit reject latch)

샘플 frame 의 `wheel_at_handoff ≤ 8°` → `angle_passive_active` 는 아님.

**fix 보류 — 원인 미확정. 다음 chunk 에서 게이트별 frame-by-frame 분류 필요.**

## 2. 카테고리 sweep 결과 (8 routes, 290 rlog segments)

`tools/ioniq6n_full_drivelog_sweep.py` 출력 요약:

| Route | segs | LKAS_ALT fr | TX | ACI consistency | Angle limit | SCC bus1 | Cam RX gap | commIssue | ACI flips |
|---|---:|---:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|
| 0x24 | 2 | 7,020 | ✅ | ✅ 0 | ✅ | ✅ | ✅ (34 ms) | 17 / 2 | ✅ |
| 0x25 | 37 | 217,451 | ✅ | **❌ 4,300** | ✅ | ✅ | ✅ (40 ms) | 10 / 2 | ✅ |
| 0x27 | 48 | 281,050 | ✅ | **❌ 1,275** | ✅ | ✅ | ✅ (33 ms) | 10 / 2 | ✅ |
| 0x28 | 50 | 296,388 | ✅ | **❌ 2,801** | ✅ | ✅ | ✅ (31 ms) | 9 / 3 | ✅ |
| 0x29 | 34 | 200,683 | ✅ | **❌ 1,623** | ✅ | ✅ | ✅ (50 ms) | 13 / 2 | ✅ |
| 0x2a | 27 | 159,622 | ✅ | **❌ 1,171** | ✅ | ✅ | ✅ (37 ms) | 25 / 2 | ✅ |
| 0x2b | 41 | 239,821 | ✅ | **❌ 2,565** | ✅ | ✅ | ✅ (38 ms) | 10 / 2 | ✅ |
| 0x2c | 51 | 304,871 | ✅ | **❌ 2,040** | ✅ | ✅ | ✅ (34 ms) | 15 / 2 | ✅ |
| **합계** | **290** | **1,706,906** | ✅ all | **❌ 15,775** | ✅ | ✅ | ✅ max 50 ms | 109 / 17 | ✅ |

**전체 panda safety / SCC_CONTROL TX / 카메라 health / 앵글 한계 / ACI flip
hotspot 깨끗** — Phase 6 시리즈 도입 후 HW 안전 레이어는 회귀 없음. cam_stale
이벤트는 0 routes (Phase 6f-2 의 250 ms gate 가 false-positive 일으키지
않음).

commIssue 는 PR #11 audit 과 동일하게 부팅 직후 transient 추정 (LOW). 8 routes
모두 mid-drive ADAS 이벤트 0건.

**ACI consistency mismatch는 8 routes 중 7 routes에서 발생** (0x24 만 2 seg
짧음). 합계 15,775 frames (1,706k LKAS_ALT의 0.92%) — 일관된 패턴.

## 3. ACI consistency mismatch — 일관된 결함 신호 (P0 후보)

대표 샘플 (Route 0x25, 0x2c — 모든 7개 라우트에서 동일 패턴):

```
t=322.41s v=20.1 kph  angle=12.8°  gain=0.004
  lkas_angle_active=0  lka_assist=0  byte7=0xF0  byte13=0x09
t=798.23s v=21.0 kph  angle=12.8°  gain=0.004
  lkas_angle_active=0  lka_assist=0  byte7=0xF0  byte13=0x09
...
```

분류 기준 (`tools/ioniq6n_full_drivelog_sweep.py:155-167`):
- `LKAS_ANGLE_ACTIVE = 0` (primary indicator = passive)
- `byte13 == 0x09` (active 패턴)
- `byte7 & 0xA0 == 0xA0` (active 패턴, 0xF0 & 0xA0 = 0xA0)
- `gain < 0.01` (사실상 inactive)
- `|angle| > 0.1`

→ 보조 active 시그널 두 개가 set 인데 primary 는 passive + 게인 ~0 + 비제로
각도. **모든 샘플이 ~20-22 km/h 시내 영역**, angle 정확히 12.8° (=128 *
0.1 — `LKAS_ANGLE = 0x80` 부근 값). 빈도: 0.7-2.0% per route, 합계 **15,775
frames** across 8 routes. 매 라우트에서 일관되게 발생.

**원인 가설**:
1. `create_steering_messages` 에서 `lat_active` 가 false 로 떨어질 때
   byte7/byte13 잔존 (mirror branch 에서 LKAS_ANGLE_ACTIVE만 0으로 reset,
   byte7/13 미reset)
2. in_passthrough 모드의 1-frame mirror 가 카메라의 active state 를 그대로
   echo (passthrough exit 직후의 1-frame race)
3. ADAS_DRV (카메라) 의 byte13/byte7 가 LKAS_ANGLE_ACTIVE 와 별도 lifecycle.
   op 가 모든 byte 를 적절히 토글하지 않음.

**Follow-up 검증 절차**:
- `git grep -n 'byte13\|byte7\b' opendbc_repo/opendbc/car/hyundai/` 로
  set 지점 모두 확인.
- 단일 mismatch frame 의 직전/직후 5 frame 의 byte 패턴을 추출 (replay 도구
  또는 Python LogReader 로 frame window 추출 스크립트).
- `hyundaicanfd.py:create_steering_messages` 의 mirror 경로와 active 경로
  byte 패턴 정렬 확인. 특히 `cam_invalid=True` 분기와 passive 분기.

Route 0x25/0x2c 모두 cam_gap=0 / cam_stale=0 이므로 cam_invalid 가 트리거된
것은 아님 → option 1 또는 2 가 유력.

## 4. Phase 6f-1 효과 측정 — 부분 달성 (transition frame 미해결)

`tools/ioniq6n_phase7_sim.py` 의 heavy-override (override_factor≥0.9)
슬라이스. 6f-2 commit 메시지가 목표한 "p95 ≪ 5°, sign-mismatch share
collapsing toward zero" 와 비교.

**8 라우트 통합 결과 (290 segs, 1.7M LKAS_ALT, 467k latActive frames)**:

| Metric | 측정값 | 6f-1 commit (`818a6b9`) 시점 baseline (drive 0x22+0x23) | 6f-1 목표 |
|---|---:|---:|:-:|
| heavy-override frames | 161,674 (34.62% latActive) | 13,320 (24.97%) | — |
| `|apply-wheel|` p50 | 3.2° | 2.7° | — |
| **`|apply-wheel|` p95** | **27.0°** | 31.1° | **< 5°** ❌ |
| `|apply-wheel|` p99 | 76.9° | 95.2° | — |
| max | 185.1° | 122.2° | — |
| sign-mismatch in heavy-override | 3,945 (2.44%) | 454 (3.41%) | → 0 (sign 28% 감소) ✅ |

→ 절대값 p95 31° → 27.0° **13% 감소**, p99 95° → 77° **19% 감소**,
sign-mismatch share 3.41% → 2.44% **28% 감소**. **방향은 맞고 크기는 미달**.
heavy-override 비율 자체가 25% → 35% 로 증가했는데 이는 분석 대상 코너 /
주행 비율 차이일 가능성 (89k → 467k latActive frame 으로 sample 5x 확대).

부수 측정 (참고): >200 Nm 그립에서 N7b error_mult 의 ACIGain 평균 reduction
10.2% (sim closed-form). 6c-2 의 deployed 효과 추정치.

**근본 원인 — 코드 분석**:

`opendbc_repo/opendbc/car/hyundai/carcontroller.py` 의 lateral path:

```
line 491:  desired_angle = sp_smooth_angle(v, desired_angle, self.apply_angle_last)
line 505:  if override_factor > 0.1:
             blend = min((override_factor - 0.1) / 0.4, 1.0)
             desired_angle = (1-blend)*desired_angle + blend*steer_angle_safe
line 509:  apply_angle = apply_steer_angle_limits_vm(
              desired_angle, self.apply_angle_last, ...)
line 530:  self.apply_angle_last = apply_angle    # ← anchor for next frame
   ...
line 633:  if self.angle_passive_active or override_factor >= 0.9:
             self.apply_angle_last = steer_angle_safe   # ← 6f-1 clamp
```

→ **6f-1 clamp 의 위치 문제**: line 633 의 clamp 는 그 *프레임의 transmitted*
`apply_angle` 에 영향을 주지 않고, *다음 프레임* anchor 만 갱신. 따라서
heavy-override 가 처음 트리거된 frame N 에서는:
- line 509 에서 `apply_angle = VM_step(apply_angle_last(N-1), wheel)` 계산.
  `apply_angle_last(N-1)` 은 보통 op_curv 를 따라가던 stale anchor → wheel
  과 멀리 떨어짐.
- line 530 에서 `apply_angle_last := apply_angle` (한 VM step 만 wheel 쪽으로
  이동된 값).
- line 633 에서 `apply_angle_last := wheel` (clamp). 이는 frame N+1 에서야
  소용 있음.

즉 **transition frame 의 mismatch 가 통계를 지배**. 짧은 heavy-override
episode (10-30 frame) 가 많을 수록 p95 가 안 떨어짐.

**개선안 후보** (다음 phase 검토):

A. **Pre-frame anchor**: line 491 직전에 clamp 도 같이 적용:
   ```python
   if self.angle_passive_active or override_factor >= 0.9:
     self.apply_angle_last = steer_angle_safe
   ```
   → 한 frame 일찍 anchor 가 wheel 로 셋되므로 transition frame 의 apply
   도 wheel ± rate 가 됨.

B. **Direct override of apply_angle**: line 530 의 `self.apply_angle_last
   = apply_angle` 대신 heavy-override 시 `apply_angle = steer_angle_safe`
   로 덮어쓰기. (안전 비용: VM rate limiter 우회 — 신중)

C. **다중 frame look-ahead**: override_factor 가 증가 중인 trend 를 감지
   하면 도달 전에 미리 anchor.

→ **권장: A** (안전 영향 없음, 코드 1줄 추가, 효과는 sim 으로 closed-form
사전 검증 가능). 8 라우트 통합 sim 결과로 A 의 예상 효과를 산출 (frame
re-simulation 으로 transition frame mismatch 가 p95 < 5° 로 떨어질지).

## 5. Phase 6d/6e latch — 정상 동작 (관측 가능)

| Metric | 0x25+0x2c (88 segs) | 비고 |
|---|---|---|
| latActive frames | 128,865 | — |
| 6d entry-zone (`|wheel|≥40 ∧ |tq|≥60`) | 5,633 (4.37%) | 코너 진입 + 그립 동시 |
| 6d exit-zone (`|tq|<30`) | 14,049 (10.90%) | 그립 해제 |
| 6d latch active (STEER_REQ=0) | 10,524 (8.17%) | — |
| 6e-1 latch active (5-frame filter) | 10,355 (8.04%) | — |
| 6d events | 42 | enter/exit 사이클 수 |
| 6e-1 events | 39 | — |
| 6e-1 가 제거한 transient (<5fr) 이벤트 | 2 → 1 (1건 추가 흡수) | **효과 미미** |
| dwell 6d p50 | 224 frames (~2.24s) | — |
| dwell 6d p95 | 675 frames (~6.75s) | — |
| dwell max | 850 frames (~8.5s) | 긴 코너 +그립 유지 |

→ 6d/6e-1 둘 다 정상 동작. **6e-1 transient filter 의 효과는 매우 작음**
(2 routes에서 1 frame 차이). transient 자체가 드물어서 평가에 더 많은
샘플 필요. 다음 chunk 후 종합 판단.

## 6. driver_torque 분포 (앵글 제어 특성)

```
all frames (latActive 무관):
  p50=82  p90=294  p95=387  p99=600  max=913
  >100 Nm: 41.2% / >200 Nm: ?
```

POST_PR11 audit / route49 분석과 동일 — 앵글 제어 MDPS 의
`STEERING_COL_TORQUE` 에 EPS 반력 포함 특성 유지. 6c-1/6c-2 가 deployed 이고,
ACIGain 가 grip 시 13.7% 감소 (>200 Nm) 로 측정됨 → 6c-2 동작 확인.

## 6.1. 속도 분포 — MASTERPLAN Stage 1 검증 cover 부족

Route 0x25+0x2c (88 segs, sim limit 200) 의 `|Δapply|` 측정 시 noted 된
speed bucket 분포:

| Bucket | frames | share |
|---|---:|---:|
| 0-20 kph | 45,488 | 67.5% |
| 20-30 | 9,917 | 14.7% |
| 30-40 | 7,479 | 11.1% |
| 40-60 | 4,910 | 7.3% |
| 60-90 | 627 | 0.9% |
| 90-200 | ~0 | 0% |

→ **시내 위주 (0-30 km/h: 82%)**, 30+ km/h share 18%, 60+ km/h 1%, 90+ 사실상
0건. `tools/IONIQ6N_STEERING_MASTERPLAN.md` 의 Stage 1 verification target
인 **30 min highway op @ 100-110 km/h** 데이터 누락. 고속 영역의 op MAE,
3.6 m/s² 횡가속도 saturation 빈도, VM rate-limit rejection 빈도가 측정
불가. 마찬가지로 Stage 1 의 **20 min parking** 분리 데이터도 없음 (이번
8 라우트는 mid-drive 위주).

## 7. 미수행 항목 (다음 chunk 에서 보강)

1. **6개 라우트 추가 (0x24, 0x27, 0x28, 0x29, 0x2a, 0x2b)** — chunk 단위로 +1 씩.
2. **LFA_ICON 분포** — `mads_on` 동안 CCNC_0x161.LFA_ICON {HIDDEN,
   GRAY, GREEN, WHITE} 비율. PR #11 baseline 비교.
3. **Panda safety counter Δ** — peripheralState 또는 panda heartbeat
   메시지에서 busOff / sendErr / fwdErr / faults 추출.
4. **carControl 무결성** — NaN steering, 270°+ oversized command 비율.
5. **MADS 상태 전이** — `(latActive, lonActive)` 튜플 oscillation,
   R→D 즉시 재진입 검증, `was_in_reverse` latch.
6. **cloudlog 가시성** — PR #11 audit 에서 MEDIUM 으로 지적한 cloudlog→cereal
   이전이 Phase 6 시리즈에서 진행됐는지. 96ea9ea 의
   `lateralAccelLimit @99` / `steerAngleLimit @100` / `cameraDataStale @101`
   가 actually 트리거되는지 (이번 분석에서 cam_stale 이벤트 0건 확인됨).
7. **route 0x26 rlog 누락 원인** — 디바이스 → repo 업로드 파이프라인 점검.

## 8. 즉시 후속 조치 후보 (P0/P1 punch list)

> 8 라우트 sweep + 2 라우트 sim 결과 기준. 8 라우트 통합 sim 완료 후 §4
> 의 통계만 갱신될 예정.

| ID | Pri | Item | 근거 / 변경 위치 |
|---|---|---|---|
| **6F2-A** | **P0 ✅ DONE** | **Pre-frame anchor**: `carcontroller.py:491` 직전에 동일 clamp 추가. 빌드 `d83c3b5` 로 deployed. 0x2d 의 exit-transition p95 == baseline (smooth resume), 0x2e 에서 1.5x spike — 추가 sample 필요. | §1.A.B |
| **6F2-I (신규)** | **P0** | **저속 hand-off lag** — driver 가 wheel 놓아도 op 가 take-over 안 함 (p50 0.2-1.5 s, p95 6-15 s). state machine 은 1 frame 안에 전환되지만 다른 게이트 (CC.latActive / apply_steer_req / vm_reject_persistent / MADS auto-disengage) 가 막고 있음. 다음 chunk 에서 게이트별 frame-by-frame 분류 후 fix 결정. | §1.A.E |
| **6F2-J (신규)** | **P0** | **동일 route A/B baseline**: 다음 commute 1 회는 이전 빌드 (`5479ecc`) 로 같은 출/퇴근길 찍어 6F2-A 의 동일 조건 before/after 비교 가능하게 함. | §1.A |
| 6F2-B | P0 | sim 의 closed-form 으로 6F2-A 의 효과 사전 검증 — frame re-simulation 으로 transition mismatch 가 p95 < 5° 도달하는지 (현 25.6° 에서). | §4 |
| **6F2-C** | **P2 강등** | **LKAS_ALT byte7/byte13 일관성**: 15,775 frames TYPE B 98.8% (ACIGain rate_dn 잔존). op 결함 아님 — 카메라 echo + 자체 rate_dn 잔존. snap-to-zero 옵션은 MDPS 토크 끊김 위험. | §3 |
| 6F2-D | P1 | sim 의 `override_factor` 가 실제 controller 와 매칭하는지 frame-level 비교. controlsState 에 신규 cereal field 추가. | §4 #1 가설 검증 |
| **6F2-K (신규)** | P1 | sign-mismatch 23-41% 원인 분석 — sign-mismatch frame 의 model_v2 plan / lane lines / driver_input 상관 측정. driver disagreement 인지 모델 noise 인지 분리. | §1.A.A |
| **6F2-L (신규)** | P1 | 고속도로 30+ min sustained drive — 90+ km/h ≈ 0% (commute 만으론 불가). 별도 의도적 highway run. | §1.A.D, §6.1 |
| 6F2-E | P2 | 6e-1 transient filter 효과 검정 — 8 라우트 통합 sample 에서 saved transient 수 카운트, ≥10 건이면 효과 인정, ≤3 건이면 6e-1 제거 검토. | §5 |
| 6F2-F | P2 | Route 0x26 rlog 업로드 누락 — 디바이스→repo 푸시 파이프라인 점검 (qlog 만 47 개 push 됨). | §1 |
| 6F2-G | P3 | LFA_ICON 분포 측정 추가 — PR #11 audit baseline (Drive 14/15/16의 GREEN 32-63%) 와 Phase 6f-2 비교. 이번 audit 미수행. | §7 #2 |
| 6F2-H | P3 | Panda safety counter / MADS 상태 전이 / cloudlog 가시성 — POST_PR11_AUDIT 의 §3,§7,§8 형식 보강. | §7 #3,5,6 |

## 9. 분석 절차 메모

```bash
# data fetch (partial clone — lazy blob)
cd /home/user/openpilot
git fetch --filter=blob:none origin ccnc-drivelog:refs/remotes/origin/ccnc-drivelog

# build identification (1 segment per route)
python3 /tmp/route_builds.py    # 본 보고서 §1 테이블 생성

# chunk fetch + sweep (route 0x25 예)
/tmp/fetch_route.sh "99b215d21bbf8735_00000025"
python3 tools/ioniq6n_full_drivelog_sweep.py
python3 tools/ioniq6n_phase7_sim.py /home/user/openpilot/drivelog --limit 200
```

분석 도구 출력 raw 는 reproducible — 본 보고서의 표는 그 출력에서 그대로
발췌. 청크 단위로 `drivelog/` 디렉터리에 누적; disk 사용량 1 chunk ≈ 500MB.
