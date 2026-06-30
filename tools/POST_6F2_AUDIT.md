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
| **0x40** (`eb2be2a919`) | **`abd7ca6` Phase 6f-5** | 44 | **analyzed (chunk 4 §1.D, 출근 morning commute, 2026-06-08)** |
| **0x41** (`3e9e6dbdb8`) | **`abd7ca6` Phase 6f-5** | 37 | **analyzed (chunk 4 §1.D, 퇴근 evening commute, 2026-06-08)** |

Phase 6f-2 합계 = ~290 rlog segments (8 라우트). Phase 6F2-A 추가 = 59 segs (2 라우트, 2026-05-28).
보고서는 4 routes (0x25 + 0x2c + 0x2d + 0x2e, 147 segs) chunk 까지 반영.

추가로 라우트 0x26 (`a9ef010f25`) 은 47 qlog 만 업로드, rlog 누락 →
**업로드 무결성 follow-up 1건**.

### Phase 6F2-A 신규 빌드 데이터 (2026-05-28 commute)

빌드 검증: 두 route 의 첫 세그먼트 `initData.gitCommit` 가 `d83c3b5214ad...`
로 확정 (post 6F2-A pre-frame anchor), `dirty=False`, `branch=i6n`.

## 1.A. Phase 6F2-A 빌드 (`d83c3b5`) vs 6f-2 (`5479ecc`) — A/B baseline 비교 (2026-05-28)

### ⚠️ 사전 정정: 이전 audit §4 의 sim p95=27° 는 **decode 버그**

`tools/ioniq6n_phase7_sim.py` + `tools/ioniq6n_full_drivelog_sweep.py` 의
LKAS_ALT angle decode 는 `int.from_bytes(dat[4:6], 'little')` 였으나, DBC
정의 `ADAS_StrAnglReqVal : 82|14@1-` 는 **byte 10-11** 에서 디코딩해야 함
(start bit 82, length 14, little-endian, signed). byte 4-5 는 사용되지 않는
영역으로 항상 0x80 → 12.8° 반환. 이전 audit §4 의 *모든* heavy-override
mismatch 통계 (p95=27°, sign-mismatch %, transition-frame 추론) 는 이 decode
오류 결과. **재측정 (byte 10-11) 결과 실제 op-active heavy-override TX 가
wheel 을 0.1° 이내로 추적 중** — 6F2-A 가 fix 할 진짜 문제 자체가 없었음.

### A/B 매칭 (GPS 으로 검증)
- **0x2b** (05-27 morning, 41 segs, `5479ecc`) ↔ **0x2d** (05-28 morning, 37 segs, `d83c3b5`)
- **0x2c** (05-27 evening, 51 segs, `5479ecc`) ↔ **0x2e** (05-28 evening, 22 segs, `d83c3b5`)

같은 home/work 경로 + 같은 시간대, 다른 빌드.

### A. heavy-override TX (op-ACTIVE only) — 모두 0.1° 이내 추적

| Route | latActive | HO% | op-active% | p50 | **p95** | p99 | max | sign-mis | within 1° |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0x2b 5479ecc | 82,146 | 27.0% | 80.8% | 0.00° | **0.10°** | 0.50° | 5.00° | 0 | 99.6% |
| 0x2c 5479ecc | 64,841 | 32.2% | 85.9% | 0.00° | **0.10°** | 0.50° | 4.40° | 0 | 99.7% |
| 0x2d d83c3b5 | 74,891 | 29.6% | 91.5% | 0.00° | **0.10°** | 0.40° | 11.40° | 2 | 99.7% |
| 0x2e d83c3b5 | 67,012 | 29.6% | 80.4% | 0.00° | **0.10°** | 0.60° | 6.80° | 1 | 99.5% |

→ 양 빌드 모두 op-active heavy-override 중 TX 가 wheel 을 **0.1° p95 (측정 한계 = CAN 양자화 0.1° 와 같음)** 로 추적. 6F2-A 효과는 floor 이하라 변별 불가. **p99 도 0.4-0.6° 로 매우 우수**.

### B. exit-transition (passive → active 첫 frame) — 모두 ≤ 2.6° p95

| Route | n events | transition p95 | baseline p95 |
|---|---:|---:|---:|
| 0x2b 5479ecc | 47 | 2.27° | 2.80° |
| 0x2c 5479ecc | 45 | 2.38° | 2.20° |
| 0x2d d83c3b5 | 40 | **1.62°** (best) | 2.40° |
| 0x2e d83c3b5 | 37 | 2.64° | 1.90° |

→ 모든 routes ≤ 3° transition spike. 6F2-A 와 baseline 사이 유의차 없음.

### C. **저속 hand-off lag — A/B 양쪽 모두 동일 문제 (6F2-A 영향 아님)**

passthrough 중 driver wheel-release → LKAS_ANGLE_ACTIVE=2 까지 latency:

| Route | events | p50 lag | p90 | p95 | < 100ms | first_TX-wheel p95 |
|---|---:|---:|---:|---:|---:|---:|
| 0x2b 5479ecc morning | 23 | 533 ms | 12.0 s | 16.9 s | 17% | 0.49° |
| 0x2c 5479ecc evening | 27 | 339 ms | 5.5 s | 11.2 s | 30% | 1.32° |
| 0x2d d83c3b5 morning | 20 | **1541 ms** | 10.1 s | 15.5 s | 15% | 0.30° |
| 0x2e d83c3b5 evening | 12 | **187 ms** | 5.0 s | 6.0 s | 50% | 4.22° |

**중요**:
- 코드 (carcontroller.py:470-474) 는 1 frame (≈ 10 ms) 내 전환 의도.
- 실측 p50 0.19-1.5 s, p95 6-17 s — **양 빌드 모두 동일** → 6F2-A 와 무관.
- 다행히 **op 가 마침내 take-over 할 때는 매끄러움** (first_TX vs wheel p95 ≤ 4.22°).

원인 후보 (effective_lat_active 의 다른 게이트):
1. `apply_steer_req=False` (controlsd 명령 발행 중단)
2. `vm_reject_persistent=True` (Phase 5e VM-reject latch 잔존)
3. `angle_passive_active` 가 이전 코너에서 latched 상태 (|tq|<30 미달로 미해제)

### D. 카메라 passthrough 빈도 — A/B 차이 작음

| Route | latActive% | <20kph latch | passthrough active | ∩ latActive | entries/exits |
|---|---:|---:|---:|---:|---:|
| 0x2b 5479ecc | 34.2% | 21.8% | 77.2% | 0.81% | 575/575 |
| 0x2c 5479ecc | 21.3% | 14.9% | 69.2% | 1.15% | 434/434 |
| 0x2d d83c3b5 | 34.4% | 16.8% | 89.7% | 0.80% | 334/325 |
| 0x2e d83c3b5 | 52.7% | 14.8% | 97.7% | 0.50% | 167/162 |

→ 의도 설계대로 동작. A/B 큰 차이 없음.

### E. 속도 분포 — highway 데이터 갭 여전

| 버킷 | 0x2b 5479ecc | 0x2d d83c3b5 |
|---|---:|---:|
| stop+0-20 | 64.5% | 62.6% |
| 20-60 | 33.1% | 31.3% |
| 60-80 | 2.4% | 4.1% |
| 80+ | 0% | 2.0% |
| 100+ | 0% | 0.2% |

→ 둘 다 sustained 100+ km/h 없음. **고속도로 데이터 별도 수집 필요**.

## 1.C. Phase HOD-clean (`c9a1ed6`) chunk — 6 신규 routes (2026-05-31 push)

빌드 검증: 4개 메인 routes (0x2f, 0x30, 0x31, 0x35) 모두 `initData.gitCommit = c9a1ed6a2ecf` (HOD bypass 제거 + carstate TODO 정리 직후), `dirty=False`, `branch=i6n`. 0x32 / 0x34 = 너무 짧음 (2 / 7 segs), 0x33 = 빠짐.

### A. 카테고리 sweep — **모두 clean** (158 segs, 933,998 LKAS_ALT)

| Route | rlog | LKAS_ALT | TX audit | Angle | SCC bus1 | Cam max gap | ACI flip | carState |
|---|---:|---:|:-:|:-:|:-:|:-:|:-:|:-:|
| 0x2f | 32 | 187,998 | ✅ | ✅ | ✅ 0 | 35 ms | ✅ | ✅ |
| 0x30 | 32 | 188,561 | ✅ | ✅ | ✅ 0 | 44 ms | ✅ | ✅ |
| 0x31 | 46 | 273,067 | ✅ | ✅ | ✅ 0 | 34 ms | ✅ | ✅ |
| 0x35 | 48 | 284,372 | ✅ | ✅ | ✅ 0 | 38 ms | ✅ | ✅ |

→ **HOD scaffold 제거 (`c9a1ed6`) 후 안전 회귀 없음**. panda safety counter, SCC bus1 collision, 카메라 health, ACI flip hotspot 모두 깨끗.

### B. Heavy-override TX (op-active, 6F2-A scope) — floor 유지

| Route | HO active | p95 `\|TX-wheel\|` | within 1° | sign-mismatch |
|---|---:|---:|---:|---:|
| 0x2f | 20,026 | **0.10°** | 99.5% | 0.06% |
| 0x30 | 22,219 | **0.10°** | 99.6% | 0.01% |
| 0x31 | 17,294 | **0.10°** | 99.8% | 0.00% |
| 0x35 | 19,425 | **0.10°** | 99.8% | 0.01% |

→ Phase 6F2-A floor (`p95 = CAN 양자화 0.1°`) 유지. HOD cleanup 이후 회귀 없음.

### C. 속도 분포 — **첫 highway 데이터** 확보 (0x30)

| 버킷 | 0x2f | 0x30 | 0x31 | 0x35 |
|---|---:|---:|---:|---:|
| stop+0-20 | 60.1% | 65.6% | n/a | n/a |
| 80-100 | 2.1% | 1.9% | n/a | n/a |
| **100-120** | 0.7% | **2.3%** | n/a | n/a |
| **120+** | 0.6% | **2.0%** | n/a | n/a |

0x30 = mixed urban + highway (100+ kph 4.3% ≈ 6 분). 0x31/0x35 = sustained latActive 31분/34분 (long drive). **§6.1 Stage 1 highway 데이터 gap 부분 해소**.

### D. ACI consistency mismatch (TYPE B 잔존)

| Route | mismatch | % of LKAS_ALT |
|---|---:|---:|
| 0x2f | 1,602 | 0.85% |
| 0x30 | 2,648 | 1.40% |
| 0x31 | **491** | **0.18%** (highway 가장 깨끗) |
| 0x35 | 1,233 | 0.43% |

8-route baseline (0.92%) 와 같은 패턴 — §3 분석 그대로 (TYPE B = ACIGain rate_dn 잔존, op 결함 아님, P2 유지).

### E. **🔴 신규 발견 — hand-off lag P0 가설이 detection artifact 였음**

이전 audit §1.A.C 에서 hand-off lag (p50 0.2-1.5 s, p95 6-17 s) 를 P0 후보로 documented. 4 routes 에 게이트별 분류 + driver torque 분포 측정 결과:

**Gate 분류 (54 hand-off events 합계)**:

| Dominant gate | n | 비율 | median lag |
|---|---:|---:|---:|
| `CC.latActive=False` | 29 | 54% | 600-900 ms |
| `angle_passive_active` | 9 | 17% | 5-10 초 |
| `in_passthrough_relapse` | 7 | 13% | 2-3 초 |
| unknown (apply_steer_req / vm_reject / cam_stale) | 7 | 13% | 2 ms (즉시) |
| `was_in_reverse` | 1 | 2% | 251 ms |

**Driver torque 측정 (31 events with lag > 500ms)**:

| Metric | 값 |
|---|---:|
| Per-event torque median (lag window 전체 평균) | **140 Nm** |
| Per-event peak torque median | 359 Nm |
| Max event peak | 1033 Nm |

→ **lag 동안 driver 가 계속 wheel 잡고 있음** (median 140 Nm, peak 359 Nm). 진짜 hand-off 가 아니라 driver continuous override 중. MADS / angle_passive / passthrough 가 yield 하는 게 **정상 동작**.

진짜 hand-off (lag < 100ms, ~13/54 events) 는 instant resolution. **즉 op 의 hand-off 메커니즘 자체에 문제 없음**.

**Phase 6d EXIT threshold sensitivity** (현재 30 Nm 에서 변경 시 해소되는 events):

| 새 threshold | 31 events 중 해소 |
|---|---:|
| 30 (현재) | 3 (10%) |
| 50 | 7 (23%) |
| 100 | 14 (45%) |
| 150 | 16 (52%) |

→ 30→50 효과 4 event 만. 100+ 필요한데 그러면 normal driving 에서 latch flap 위험. **driver 가 진짜 손 놓고 있는 게 아니라 변경 효과 한계적**.

### F. 결론 — 코드 수정 없음 (현 `c9a1ed6` 유지)

| 이전 P0 후보 | 실측 후 판정 |
|---|---|
| hand-off lag p50 0.2-1.5 s | **artifact** (driver 계속 grip, system 정상 yield) |
| Phase 6d EXIT 30→50 | 효과 < 13% (driver torque median 140 Nm) |
| MADS auto-disengage 완화 | driver 진짜 override 중일 때 정상 동작, 완화 시 의도 무시 위험 |
| passthrough hysteresis 확장 | in_passthrough_relapse 7/54, stop-and-go 의도 동작 방해 가능 |
| Sweep + ACI safety regression | **clean 확인** ✅ (이 단계가 의미 있는 P0 검증이었음) |

**HOD cleanup (`c9a1ed6`) 후 회귀 없음 + 6F2-A floor 유지 + highway 데이터 일부 확보**. 추가 코드 변경 권장 없음.

## 1.D. Phase 6f-5 빌드 (`abd7ca6`) chunk — 출퇴근 왕복 2 routes (2026-06-08 push)

빌드 검증: 두 route 첫 세그먼트 `initData.gitCommit = abd7ca682dcb...` (Phase 6f-5),
`dirty=False`, `branch=i6n`, `version 2026.001.000`, `carFingerprint=HYUNDAI_IONIQ_6_N`.
0x40 (eb2be2a919) = 출근 44 seg / 43.4 min / 13.5 km, 0x41 (3e9e6dbdb8) = 퇴근 37 seg / 36.5 min / 11.6 km.
**사용자 보고 증상 2건 검증 목적** (저속 떨림 / 차선 침범).

### A. 속도 분포 + latActive 활성 곡선 — **op 측방은 ~20 km/h 부터**

| 버킷 | 0x40 출근 | 0x41 퇴근 |
|---|---:|---:|
| stop / 1-20 / 20-50 / 50-80 / 80+ km/h | 34% / 26% / 32% / 7% / 0.5% | 38% / 30% / 20% / 9% / 4% |

latActive % by speed bin (양 route 동일 패턴):
`0-5/5-10/10-15/15-20 = 0%` → `20-25 = 74-93%` → `25 km/h↑ = ~100%`.
→ **20 km/h 미만에서 op 측방제어 0% (카메라/MDPS 패스스루).**

### B. 저속 떨림 — 6f-4 는 성공, 사용자 체감분은 op 제어 영역 밖

핸즈오프·완만곡선 band-limited deg-RMS (휠):

| 구간 | 0.5-1.5 Hz | 1.5-3 Hz | **3-5 Hz** | 5-8 Hz | 우세주파수 |
|---|---:|---:|---:|---:|---:|
| 20-35 km/h | 0.59° | 0.15° | **0.05°** | 0.02° | **0.7 Hz** |
| 35-60 km/h | 0.42° | 0.12° | **0.06°** | 0.02° | 0.6 Hz |

→ 6f-4 가 노린 4-5 Hz 시내 떨림 **제거 확인** (`desiredCurvature` 3-7 Hz RMS ~2e-5, LP 동작).
사용자 체감 "저속 떨림" 은 <20 km/h (op 비활성) → **현 파이프라인 밖 = 구조적**. 6f-3/6f-4/6f-5 무관.

### C. **🔴 신규 발견 — 코너 진입 미추종 → 차선 침범 (운전자 개입)**

op-active + 블링커 off near lane-crossing 스캔 (clearance = |laneLine.y[0]|, narrow ~2.8 m 차선 보정 감안):

| route·seg | KST/시점 | 최저 clearance | 속도 | 비고 |
|---|---|---:|---:|---|
| 0x40 **seg5** | **07:27:43-49** (GPS 확정) | **0.77 m** (우측선) | 40-47 km/h | **운전자 +40°·~1500 Nm 개입 복구** |
| 0x40 seg9 | ~10 min, 차선 끊김→재연결 | 0.53 m (좌측선) | 28 km/h | op 활성 직후 저신뢰 |
| 0x41 seg22 | 22 min | 0.33 m (좌측선) | 112 km/h | 고속 |
| 0x40 seg38 | ~38 min | 0.04 m | 44 km/h | 모델 차선 재라벨 동반(주의) |

**seg5 단별 정합** (KST 07:27:43-49): 전방 차선중심@50m −4.8 m (좌굽), **op 경로@50m −1.6 m**
(요구의 ~1/3), `desiredCurvature` ~−0.002 vs 차선 요구 ~0.0038 1/m, 휠 +0.7~+3° (거의 직진).
우측선 clearance 1.4→**0.77 m** (타이어 선 위) → 07:27:49.0 운전자 `steeringPressed=1`·토크 +700~+1550 으로
+40° 잡아챔. **op 자가복구 아님.** `laneDeparture` 이벤트는 전 구간 0건 (LDW 미발화).

근본 원인: 모델 곡률→적용 각도 사이 **보상 없는 평활 단 직렬 누적** —
신뢰도 댐핑(`controlsd.py:281-290`) + 6f-4 LP + `sp_smooth_angle` EMA(`carcontroller.py:156`,
40 km/h α0.16 / 30 km/h α0.05). 코너 대역(curv ~0.0038, ~40 km/h)이 6f-5 boost_s 타깃인데도 발생
→ lookahead 확대로는 미해결. **병목은 명령 곡률 크기/추종.**

### D. 계측 불일치 (문서 vs 구현)

- `lateralAccelLimit/steerAngleLimit/cameraDataStale` 는 plan §1.5 가 controlsState Float32 @99/100/101
  이라 적었으나 **실제 스키마는 OnroadEvent EventName 플래그**. 횡가속 3.6 m/s² 초과 프레임 출근 147 개
  (0.056%) 있었는데 `lateralAccelLimit` 이벤트 **0건** → 미배선(dead) 의심.
- `cameraDataStale` 0건 = **정상** (cam_stale 게이트 false-positive 0, plan P0 #1 충족).
- onroadEvents 양호: controlsMismatch / steerTempUnavailable / laneDeparture **0건**.

### E. heavy-override TX 추적 — floor 유지 (회귀 없음)

|drvTq|≥90 프레임 |op-wheel| : p50 0.08-0.09° (CAN 양자화 바닥, 종전과 동등).
p95~1.1°·tail 은 운전자 실제 발산(override)이지 제어 결함 아님. highway >90 km/h op-active
추적오차 p95 0.8-1.0°, max 1.5-2.2° (135 km/h 도달 = 6f-5 140 km/h base_s 노드 영역, 안정).

### F. 조치 — **Phase 6g-1 (코너 미추종 fix)** 적용

§C 가 plan §5 의 "concrete on-vehicle symptom" 조건 충족 (운전자 개입 실측) → 코드 수정 진행:
- **6g-1a** `sp_smooth_angle` slew/maneuver-aware (gap≥HI 면 α→1, jitter 는 저-α 유지).
- **6g-1b** 신뢰도 댐핑 floor (`LAT_CONF_FLOOR=0.5`).
- 안전 envelope(clip_curvature)/panda/cereal 불변, kill switch 有, **온차량 실증 TODO**. (§8 punch list 참조)

## 1.E. Phase 6g-1 빌드 (`627a715`) chunk — 출퇴근 왕복 2 routes (2026-06-09 push)

빌드 검증: 42(`8c1f634610`,출근 34seg)·43(`2cedbfe801`,퇴근 27seg) 첫 세그 `gitCommit=627a715`
(**Phase 6g-1**), branch i6n, dirty=False. **6g-1 deploy 후 첫 실측 A/B (vs 6f-5 = 0x40/0x41).**

### A. 6g-1 효과 = 코너 미추종 해소 (✓), GPS 동일지점 정합
- 어제 seg5 이벤트 좌표(37.5568,126.96898) = 오늘 **route42 seg4, KST 07:25:14–26** (GPS 1.5 m 이내).
- 그 코너: op 경로@50m −6.1 vs 차선@50m −6.0 (어제 1/3 → 오늘 **~1:1**). 집계: corner under(<0.6)
  16%→10%(출근), near-line frame 303→70(출근). **"오늘 괜찮았는데" 일치 = 미추종 fix 작동.**

### B. 🔴 신규 = 역곡선 과조향 (운전자 개입)
- 같은 구간 직후 **S자 역곡선**(route42 seg4 KST 07:25:22–25, ~38 km/h): op `desiredCurvature`
  **+0.0156**, ao **−35°**, 좌측선 clearance **0.96 m**, 운전자 −1166 Nm 개입(pressed=1). op가
  먼저(−21°@핸즈오프) 휙 친 뒤 개입.
- **원인 (lookahead/fallback 재계산으로 정정, §E 참조)**: 곡률은 폴리핏 스파이크도 conf_floor 탓도
  아님. **모델 자신(`action.desiredCurvature`)이 +0.0133까지 직접 계획**, `_lookahead_curvature`는
  그 위 **1.2–1.5×**만 얹음(finalDC ≈ lookahead). 그 구간 **lane_min 0.3–0.8(고신뢰)** → conf 댐핑
  사실상 미작동. 즉 **6g-1 의 차이는 conf_floor 가 아니라 sp_smooth release(α→1)가 모델이 계획한
  큰 곡률을 휠에 빠르게 전달**한 것 → "휙". (이전 본 절의 "conf_floor 가 스파이크 통과" 귀속은 정정됨.)

### C. 25 km/h 떨림 = 6g-1 무관 (여전, 악화 아님)
20–30 km/h op-active hands-off 2–8 Hz 휠 RMS: 6f-5(40/41) 0.13–0.15° vs 6g-1(42/43) 0.12–0.19°,
휠 반전 ~2.7–3.0 Hz **동일**. op 명령 dither(~0.25°,5–7 Hz=저속 모델 jitter)가 EMA 통과해 남음.

### D. 조치 — **Phase 6g-2** (6g-1 과조향 보정 + 25 km/h 떨림)
- **6g-2a** `conf_floor` 를 진짜 dropout(`lane_min<0.20`)에서 0 으로 taper → apex 스파이크 freeze 복원,
  재연결대(0.20–0.30)는 floor 유지(미추종 fix 보존). `controlsd.py` `CONF_FLOOR_LANE_LO/HI`.
- **6g-2b** sp_smooth release 상한 `SMOOTHING_ANGLE_RELEASE_MAX=0.7` (catch-up 에 ~30% 댐핑 잔존 → "휙" 완화).
- **6g-2c** 저속 미세 데드밴드 `SMOOTHING_ANGLE_DEADBAND_DEG=0.4`(< `DEADBAND_MAX_VEGO=11 m/s`) → 25 km/h dither 제거.
- 단위검증: 데드밴드 0.3°→hold/2°→pass, release cap 코너 95%@frame15 유지, conf taper lane_min0.07→conf0.08(freeze).
- 안전 envelope/panda/cereal 불변, kill switch 有, **온차량 실증 TODO**.
- **레버 정정(§E)**: 이 역곡선 과조향의 핵심 레버는 **6g-2b(release 상한)** — 모델이 계획한 곡률의
  휠 전달 속도를 감쇠. 6g-2a(conf taper)는 이 이벤트엔 거의 무관(고신뢰)하나 seg9류 저신뢰
  재연결엔 유효 → harmless 유지. 6g-2c 는 25 km/h dither 전용(별개).

### E. geometry clamp(6g-2d) 타당성 측정 — **기각**

과조향을 "도로 기하/모델 곡률에 묶어" 막는 두 형태를 캐시 로그로 정량:

1. **차선기하 비율 clamp** (`k_lane` = 차선중심 2nd-diff @0/25/50 m): 고신뢰 코너에서도 op/k_lane
   p95 2.4–2.9·p99 3.2–3.8 로 과조향(~2.6×)과 **겹침**. `k_lane` 노이즈 std 0.00125 ≈ 약-코너 크기.
   세기별로 보면 중코너(0.004–0.008)는 p99 1.9 로 양호하나 약코너는 꼬리가 큼(분모 노이즈). 결정적으로
   **과조향 순간 lane_min=0.07 → clamp 는 꺼져 있어야 하는 영역** → 못 잡음. F≤1.5 면 정상 코너 25–45% 깎임.
2. **lookahead/fallback 밴드 clamp**: lookahead/fallback p50 1.0·p90 1.6–1.8·p99 6–7×(직선부에서 폭발).
   **과조향 순간 비는 1.2–1.5×**(곡률이 fallback 자체) → HI=1.6× 밴드는 못 자르고 정상 코너 12% 깎음.

→ **두 형태 모두 이 과조향을 못 잡고 정상 주행만 손상** → **6g-2d 도입 안 함**. 실집행 레버(6g-2b release
감쇠)가 정답. 부족 시 후속: (a) RELEASE_MAX 0.7→0.5, (b) clip_curvature 곡률-증가방향 jerk 비대칭 강화.
도구: `tools/ioniq6n_lane_tracking_audit.py`(+ lookahead/fallback 재계산은 `action.desiredCurvature` 추출 필요).

## 1.F. 6g-2 빌드 (`441e665`) 온차량 실측 — W0 게이트 (2026-06-10 출퇴근, 0x44/0x48)

빌드 검증 (라우트별 initData 확인): **0x44**(`2ebc2f3a4e`, 37seg, 07:44–08:15 KST 출근)·
**0x48**(`3c01135925`, 30seg, 21:11–21:36 KST 퇴근) 모두 `gitCommit=441e665` = **6g-2**,
branch i6n, dirty=False. 0x45(2seg)=주차장, 제외. 디바이스는 6h(`85f3f80`) 푸시 전 상태였음.

### A. 🔴 W0 핵심: 6g-2c 데드밴드 staircase **실차 확인** (사용자 "35kph 떨림 악화" 보고와 일치)
33–38 km/h op-active hands-off: apply-angle **hold(Δ=0) 46–58%**, 움직일 때 스텝 p90 0.42–0.68°,
휠 2–8 Hz RMS 0.094–0.158°. 연속 미세떨림이 **stick-slip 계단 틱**으로 바뀜 → 손끝 체감 악화.
핸드오프 §5의 "staircase 미확인" 리스크가 이 데이터로 **확인**됨. **6h-1(deadband 0.4→0.1 +
상류 τ(v) 연속 평활)이 정조준 — 6h 플랜 유효성 실증 1.**

### B. 코너 부정확은 "진입 lag"에 집중 (6h-1 lead 정조준 — 실증 2)
고신뢰 코너 op/k_lane: **진입(0–1 s) p50 0.83–0.93, under<0.6 = 35–41%** vs **지속 p50 1.21–1.59,
under 8–29%**. 직선 센터링은 이미 우수(차선중심 오프셋 p50 0.07–0.09 m, p90 0.19–0.23 m)
→ 오프셋 피드백 불필요, 병목은 진입 타이밍. 지속·고속 초과(48 sustained 1.59)는 6h-2 대상.

### C. 고속 near-line 다수는 차선 재라벨 아티팩트 (정직성 기록)
near-line(clr<0.8 m) 44=19건/48=27건 중 최악 사례 직독 결과: 0x48 seg15 t+29 s (R 0.26 m @84)는
yR +2.46→+0.52 **순간 점프**(합류부 재라벨), 0x44 seg17 t+27 s 도 저신뢰(prob 0.07) 구간 —
실제 드리프트 아님. 단 0x48 seg15 t+47 s (R 0.70 m @72, 지속 접근)는 진성.

### D. 6g-4 (op-active LDW) 설계 확정 — desirePrediction 만으론 불가
- `ldw.py`의 `ldw_allowed = ... and not CC.latActive` → **op 조향 중 LDW 구조적 OFF** (MADS 에선
  사실상 상시 OFF). 부작용: controlsd 의 lane-departure 억제 블록도 같은 플래그 소비 →
  **op-active 중 죽은 코드**였음 (`ldw` 이벤트 전 라우트 0건 확인).
- 최소수정(`not latActive` 제거)은 **무효**: 두 near-line 사건에서 `desirePrediction` 내내 0.000
  (op 가 차선유지 의도인 한 모델은 변경을 예측 안 함).
- **검증된 설계**: prob>0.5 지속 K=6프레임 + clr<0.7 m + 단조 접근(Δclr<0.02/frame, 누적 −0.10 m)
  + 재라벨 점프 배제(|Δclr|<0.3) + 블링커/laneChange off + v>30 → 0x44/0x48 재생 시
  **오발화 0, 진성 1건(0x48 seg15 t+47 s)만 포착**. 이 트리거를 ldw.py op-active 분기로 추가
  (선검증 완료 → **구현됨**: 6라우트 ~3.2 h 재생 오발화 0/진성 2, §8 6g-4 행 참조. 경보 위주라
  6h W1/W2 조향 측정을 오염시키지 않아 같은 빌드에 탑재).

### E. 판정
6g-2c(deadband 0.4)는 **순비용 회귀로 확정** → 6h-1 이 제거(이미 i6n `85f3f80` 푸시).
6h 플랜 전 항목이 이번 실측과 정합 — **W1(6h-1)·W2(6h-2) 게이트를 0x44/0x48 을 baseline 으로 측정**.

## 1.G. 6h+R1-4+6g4 빌드 (`8559f81`) 첫 실측 — W게이트 + 저속 떨림 근본원인 (2026-06-11, 0x49/0x4a)

빌드 검증: **0x49**(`962e7a6b9b`, 출근 07:37 KST, 31 min)·**0x4a**(`9e426514bb`, 퇴근 18:42 KST, 32 min)
모두 `gitCommit=8559f81`. baseline = 6g-2(0x44/0x48).

### A. W게이트 결과
| 게이트 | 0x49/0x4a | 판정 |
|---|---|---|
| W1 35 kph 떨림 | 휠 2–8 Hz 0.08–0.16°, held 48–55% | ❌ 미개선 (노치만 제거, 진폭 유지 → §B) |
| W2 코너 진입 | under<0.6 32–34%, 센터링 0.08–0.10 m | △ 미미 — 모델 plan 한계로 수용 |
| heavy-override | p95 0.49–0.59° | ✅ 무회귀 |
| R4 pressed flips | 31–32/min (구 38–56) | ✅ −30% |
| 6g-4 LDW 검출 | 0x4a seg31 2건(우 0.60–0.62 m @71–87), 오발화 0 | ✅ 검출 정상 / ⚠ `ldw` 이벤트 미표출 → §C |

### B. 🔴 저속 떨림 근본원인 = roll 보상 노이즈 (전 빌드 공통) → **Phase 6h-6a**
2–8 Hz 분해(20–40 kph hands-off): desiredCurvature **0.18e-4/m(매끄러움 — 6h-1 작동)** →
달성곡률 **0.45e-4/m(2× 주입)**. 주입의 ~50% = `g·roll/u²` roll 보상(latcontrol_angle) —
raw roll 2–8 Hz 0.16 mrad 가 저속 1/u² 증폭으로 **0.138e-4/m**. roll 은 준정적(뱅킹)이므로
2–8 Hz 성분은 전부 추정 노이즈. **fix: roll LP τ0.6 s** — 오프라인 재생 **82% 제거**
(0.138→0.024), 뱅킹 정상상태 손실 ~0(3 s 램프 잔차 0.15 mrad), 4 Hz 93% 감쇠 단위검증.
6f-4/6h-1 이 못 잡은 이유 = roll 가산이 곡률 평활의 하류. kill switch `ROLL_LP_TAU=0`.
잔여 ~50%는 MDPS/EPS+VM 변환(구조적, 6h-4 full-step 이 부차 레버 — W4 별도).

### C. LDW 이벤트 미표출 → **Phase 6h-6b**
검출기 발화(0x4a seg31)에도 `onroadEvents`에 `ldw` 없음 — selfdrived 의 `IsLdwEnabled`
수동주행용 토글 게이트가 op-active 경보까지 침묵시킴. fix: latActive 중에는 토글과 무관히
경보 표출(수동주행 desirePrediction 경로는 토글 유지). 6라우트 0오발화 검증 위에서만 작동.

## 1.H. Phase 7 풀스택 (`d1ade48`) 첫 실차 검증 — W-P7 게이트 (2026-06-12, 0x4b/0x4c)

빌드 검증: **0x4b**(`c19d4405f7`, 출근 07:39 KST, 31 min)·**0x4c**(`e5a9f98546`, 퇴근 18:58 KST, 32 min)
모두 `gitCommit=d1ade48` = 6h+R1-4+6g4+6h-6(roll-LP)+7a/7b/7c+mici. baseline = pre-7(0x49/0x4a, `8559f81`).
사용자 주관 평가 "아주 만족". 정량 확인:

| 게이트 | pre-7 (0x49/0x4a) | Phase 7 (0x4b/0x4c) | 판정 |
|---|---|---|---|
| **저속 떨림** 20-30 kph 휠 2-8 Hz | 0.142–0.144° | **0.111–0.120°** | ✅ −17~22% (roll-LP 예측 −25% 부합) |
| op 명령 2-8 Hz (동구간) | 0.326–0.333° | 0.240–0.426° | ↘ (4b 0.24 뚜렷, 4c 노이즈) |
| **near-line wide-run** frame | 217 / 178 | **121 / 84** | ✅ **−44~53%** (최대 user-facing 개선) |
| 코너 차선중심 오프셋 p90 | 0.85 / 0.68 m | 0.64 / 0.68 m | ✅ 타이트화(4b 0.85→0.64) |
| 코너 지속 over>1.5 | 23 / **47%** | 24 / **19%** | ✅ 과조향 감소(4a→4c) |
| heavy-override |op-wheel| p95 | floor | 0.69 / 0.90° (p50 0.10°) | ✅ 무회귀 |
| **LDW 이벤트** | 0 | 0 (검출기 재생 0발화) | ✅ 정상 — wide-run 급감으로 진성 이탈 자체가 없음 |
| 직선 센터링 | 0.07–0.09 m | 0.05–0.09 m | ✅ 유지/개선 |

### 정직한 한계 — 7a 적분기 cap 포화 (다음 사이클 레버)
곡률 추종 deficit(`|desired−achieved|` 1s-LP, 코너) p50 ~12e-4/m 는 거의 불변(achieved/desired ~0.75→0.76).
원인: MDPS 과소전달(achieved≈0.75×desired)을 메우려면 중강 코너에서 trim>cap(4e-4)이 필요해 **적분기가 포화**.
그럼에도 lane 결과(near-line −45%·센터링)는 개선 — 이는 7a(추종)보다 **7c(명령 가산)**가 코너에서 차를 더 안쪽으로
겨눠 wide-run을 줄이기 때문. 향후 레버: `LAT_FB_CAP` 상향(과조향 위험 측정 후) 또는 비례항 소량 추가.
(주의: 0x4c 저녁은 latActive 66–77%로 표본 작음 — 정체로 op 해제 많았음, 회귀 아님.)

### 판정
6h-6a(roll-LP)·6g-4(LDW)·7a/7b/7c 모두 **회귀 없이 의도대로**: 떨림 −20%, wide-run −45%, 과조향 −60%,
센터링 유지. "아주 만족" 주관과 데이터 일치. 다음 사이클은 7a cap 미세상향 후보만 남음.

## 1.I. 저속 떨림 잔차 분해 (`36eedf2`) — 명령측 근본원인 + Phase 8 (2026-06-15, 0x4e/0x4f)

빌드 검증: **0x4e**(`1a6d2d26cf`, 출근)·**0x4f**(`059e8a2966`, 퇴근) 모두 `gitCommit=36eedf2`
(= Phase 7a-2, cap 10e-4), branch i6n, dirty=False. 목적: roll-LP(6h-6) 이후 남은 저속 떨림 잔차를
**op-controllable vs plant**로 분해. 이를 위해 소스 변경 없이 rlog만으로 신규 3채널 추출
(`extract3.py`): EPS 토크(`steeringTorqueEps`), ACIGain(LKAS_ALT byte12 ×0.004), 실제 TX 조향각
(`ADAS_StrAnglReqVal` bit82, 14b signed, ×0.1).

### A. 🔴 떨림은 명령발이다 (기존 "구조적 플랜트 바닥" 결론 뒤집음)
20–40 kph 핸즈오프 2–8 Hz 분해:

| 단 | 2–8 Hz RMS | 비고 |
|---|---|---|
| carcontroller 입력 (`carControl.actuators` = latcontrol 출력) | **0.176°** | controlsState.angleState와 동일(확인) |
| carcontroller 출력 (`carOutput` = 실제 CAN TX, ACIGain·각 디코드와 일치) | **0.345°** | **입력의 ~2×** |
| 휠(달성, `steeringAngleDeg`) | ~0.11° | EPS 플랜트가 ÷3 저역통과 |

- **휠–명령 상관 R² = 0.56–0.99** (여러 구간·빌드) → 휠 떨림은 노면/EPS가 만든 게 아니라 **명령에 이미 있고
  EPS가 깎아서** 남는 것. 즉 플랜트는 떨림을 **줄여준다**.
- EMA·rate-limiter는 정현파 RMS를 키울 수 없는데 출력>입력 → **자기생성 리미트사이클**(고정 ~2.4 Hz,
  속도·빌드 무관, FB on/off 무관). 모델기여(geo, desiredCurvature→각) 0.05–0.17°, 나머지 ~0.16–0.23°는
  carcontroller 하류 주입.

### B. 🔴 근본원인 = 컬럼토크 DC 오프셋 → grip-blend 오발동 → 휠노이즈 되먹임 (오프라인 재생 확정)
crcmod 없이 VehicleModel+lateral 단독 재생으로 carcontroller 각도 체인을 충실 복제:
- `sp_smooth` EMA + 0.1° 데드밴드 + 이중 rate-limit만으로는 doubling **재현 안 됨**(출력≈입력 0.174).
- **grip-blend(Phase 5a heavy-grip yield)를 넣으니 재현**: 입력 0.176 → 0.341 (실측 0.345와 일치).
- 원인: CCNC-ALT 컬럼토크(`STEERING_COL_TORQUE`)가 **직진 핸즈오프에서도 +90~+180 Nm 양의 정적 바이어스**
  (부호평균 seg10 +92 / seg12 +176, |wheel|<3°에서도 동일 → 코너 반력 아닌 센서 오프셋). 이게 100 Nm 데드존을
  **핸즈오프 프레임의 59–75%** 초과 → 블렌드가 **노이즈 있는 측정 휠각을 명령에 섞음** → 휠 2–8 Hz 노이즈가
  TX돼 EPS가 일부 재추종(부분 양의 되먹임). `apply_angle_last:=wheel` 리셋(override≥0.9)도 부수 주입.

roll-LP(6h-6)와 합쳐 저속 떨림의 두 명령측 주입원(roll 보상 노이즈 / grip-blend 오프셋)이 모두 규명됨.

### C. 조치 — **Phase 8** (오버라이드 토크 오프셋 보정, `5b512c3`)
`carcontroller.py`: not-pressed AND `|tq|<OFFSET_MAX(250)`일 때 느린 EMA(τ45 s)로 정적 바이어스 추적 후
override 데드존 **전에** 차감. `|tq|` 게이트는 0에서 수렴 가능(바이어스가 데드존보다 큼) + 큰 동적 yank 배제.
동적(진짜) 운전자 토크는 그대로 → **오버라이드 yield 권한 불변**. 상수 `values.py`,
kill switch `DRIVER_TORQUE_OFFSET_TAU=0`(bit-identical). 안전중립: override_factor는 op측 comfort/yield 전용,
panda는 독립적으로 자체 한계 강제.

| 검증(오프라인 A/B, 0x4e seg10) | OFF | ON |
|---|---|---|
| override>0.1 프레임 | 59% | **31%** |
| TX 2–8 Hz | 0.341° | **0.295°** (−14%) |
| **안전**: +150 오프셋 위 +320 Nm 동적 그립 | — | override_factor **1.0**(완전 yield 유지) |

### 정직한 한계
- 효과 **보수적**(seg10 −14%): 정적 오프셋만 제거, 동적 yield 보존 때문. 휠 비례 하락 + roll-LP와 합산 기대.
- seg12류 **고곡률 = 모델 지터 지배** 구간(입력 0.485)은 grip 기여가 작아 평탄 → 보조 레버 P1
  (controlsd τ(v) 저속단 ≤8 m/s τ 0.20→0.30–0.40) 후보로 보류.

### 판정 / 차기 로그
온차량 검증 항목(0x50~): ① 핸즈오프 20–40 kph 휠 2–8 Hz 하락, ② 오버라이드 이벤트 yield 무회귀,
③ 오프셋 수렴값(예측 +80~+150 Nm). 약하면 P1 추가.

## 1.J. Phase 8 온차량 반증 + LDW 토글 회귀 (`43312af`) — 실제 CAN TX 검증 (2026-06-16, 0x50/0x51)

빌드 검증: **0x50**(`0258ce1cd4`)·**0x51**(`6f31941a00`) 둘 다 `gitCommit=43312af`(=Phase 8 `5b512c3` 포함),
branch i6n, dirty=False. 이번엔 **실제 송신 조향각**(원시 CAN `ADAS_StrAnglReqVal`, addr 272 bit82 14b signed
×0.1)을 디코드해 §1.I의 replay 기반 주장을 ground-truth로 검증.

### A. 🔴 Phase 8 무효 — §1.I "doubling/grip-blend" 가설 반증
핸즈오프 20–40 kph, per-run 2–8 Hz RMS(노면 교란 줄이려 input 대비 gain으로 정규화):

| 증거 | 값 | 함의 |
|---|---|---|
| replay TX vs **실제 CAN** TX | replay 0.54 / **실제 0.15** (≈3.6× 과대) | §1.I "0.176→0.345 doubling", A/B "0.341→0.295" = **replay/windowing 아티팩트** |
| carcontroller 2–8 Hz gain (TX/input) | pre-P8 **1.32×** / P8 **1.44×** | Phase 8이 gain을 **못 낮춤**(오히려↑, 노이즈 범위) |
| 🔴 gain @ grip-blend 거의 미발화 | 0x4e seg10: **1.37× at \|tq\|>100 단 23%** | 증폭원 = **rate-limiter/sp_smooth/이중 VM-limit 체인**, grip-blend 아님 |

→ Phase 8은 **엉뚱한 메커니즘**(grip-blend)을 건드림. 증폭은 grip 발화와 무관하게 각도 리미팅/평활 체인에서
발생. **조치: kill switch TAU=0으로 비활성화**(Phase 7 baseline과 bit-identical, 코드는 기록용 보존).
교훈: 절대 2–8 Hz를 replay로 추정하지 말 것 — 반드시 실제 CAN TX로 검증. (R²·DC추종으로 디코드 타당성 확인:
clean run corr 0.9–0.98.)

### B. 떨림 현 위치 — carcontroller floor, 입력측 잔존
- 실제 CAN: TX ≈ input × ~1.3, 휠 ≈ TX(EPS가 추가 저역통과). 즉 **2–8 Hz는 입력(controlsd 저속 모델-곡률
  지터)이 지배**하고 carcontroller는 ~1.3× 통과. clean 저-input 구간 휠 2–8 Hz ~0.02–0.14°(Phase 7 만족
  수준 유지). rough-input 구간은 입력 지터로 상승.
- **판정**: carcontroller 측은 하한선 근접(grip-blend 추가 손질 무의미). 추가 여지는 **입력측**뿐 →
  controlsd τ(v) 저속단(≤8 m/s) 0.20→0.30–0.40 (P1, §8). 단 절대값이 작고 Phase 7 "아주 만족"이라
  **수익체감** — 측정 가능한 회귀 없으면 보류 권장.

### C. 🔴 LDW 토글 회귀 수정 (Phase 6h-6 부작용)
사용자 보고: IsLdwEnabled 토글 OFF인데 comma4 화면에 LDW 경보 표출. 원인 = `selfdrived.py` Phase 6h-6의
`ldw_alert_allowed = is_ldw_enabled or latActive` — op 조향 중이면 토글 무시. 6h-6 당시 "0x4a에서 플래그는
떴는데 이벤트 미출현"은 **그때 토글이 OFF였던 것**을 오진한 것(표시 경로는 `EventName.ldw` 단일 — mici UI에
별도 lane-depart 렌더 없음). **수정: 토글 존중으로 복원**(`if self.is_ldw_enabled and ...`). controlsd의
카메라-ECU lane-departure 조향억제는 별도 상시 안전동작이라 무영향.

## 1.K. 2-8Hz 증폭원 정밀 국소화 + Phase 8b (`d81cf85`, 0x52/0x53)

빌드 `d81cf85`(LDW 수정 + Phase 8 off) 첫 로그. 실제 CAN TX(`ADAS_StrAnglReqVal`)로 input→TX 체인을
요소별 replay로 분해, 2-8Hz 증폭원(input→TX gain ~2×, coherence ~0.1)을 단 하나로 확정.

### A. 국소화 — grip-blend가 유일한 증폭원 (데드밴드 가설 기각)
EPS는 충실 통과(TX→wheel gain 1.0, coh 0.79–0.95). 증폭은 input→TX. replay 요소 토글(<11m/s 핸즈오프):

| 체인 | 2-8Hz | |
|---|---|---|
| real input / real TX | 0.106 / **0.190** | 증폭 대상 |
| EMA+데드밴드+양자화 | 0.094 | ❌ 증폭 안 함(데드밴드는 오히려 ↓) |
| +VM rate-limiter | 0.094 | ❌ 안전요소 무죄 |
| **+grip-blend** | **0.333** | 🔴 유일 증폭원 |

근본: 컬럼토크 오프셋 → `override_factor>0.1`이 **핸즈오프 78%** 발화 → 블렌드가 노이즈 있는 측정 휠각을
명령에 주입. 즉 §1.I의 첫 진단(grip-blend)이 옳았고, §1.J에서 의심한 sp_smooth 데드밴드/EMA·VM은 모두 무죄.
Phase 8(오프셋 추정)이 겨눈 원인은 맞았으나 방식이 실CAN에서 무효였음.

### B. 조치 — Phase 8b: 블렌드 참조 휠각 저역통과
오프셋을 추정하는 대신(Phase 8 실패) 블렌드가 섞는 **휠각만 LP**(τ=0.15s): yield 위치추종(<1Hz)은 보존,
2-8Hz 노이즈만 차단. 안전중립(VM-limit 前 참조만 평활, 권한·한계 불변). replay(절대 과대평가, 상대로 읽음):
2-8Hz −22%, yield 0.2-1Hz −2~3%. 킬스위치 `DRIVER_GRIP_BLEND_WHEEL_LP_TAU=0`. 차기 로그에서 실CAN
input→TX gain 하락으로 확정 예정(replay 불신, 실측 검증).

### C. LDW (0x52/0x53)
driverAssistance 이탈 플래그 0 / ldw 이벤트 0 → 오발화 없음 확인. 단 실이탈 부재로 토글 ON/OFF 동작 자체는 미검증.

## 1.L. Phase 9 — yield-by-authority 아키텍처 (설계+구현, A/B 대기)

Phase 8b 실CAN 평가(0x54/0x55, §위)에서 코너 parasitic 초과분 −22%지만 input→TX gain 불변 → 점수정
한계 확인. §1.K가 2-8Hz 유일 증폭원을 **명령측 grip-blend**(노이즈 휠을 명령에 섞음)로 확정했으므로,
yield를 명령축에서 **ACIGain 권한축**으로 옮기는 구조 변경.

### 핵심 통찰
권한-기반 yield는 이미 존재(`compute_torque_reduction_gain`: 토크↑ → ACIGain↓ → MDPS가 op를 덜 추종).
재설계 = 새 메커니즘이 아니라 **명령측 블렌드 제거 + 권한축으로 yield 흡수**. 권한 감소는 **오프셋 강건**
(노이즈 주입 0, 그냥 덜 밀 뿐) — Phase 8/8b가 못 푼 오프셋 문제를 구조적으로 우회.

### 구현 (`YIELD_BY_AUTHORITY` 마스터 스위치, A/B 토글)
1. **명령측 블렌드 OFF** — op는 자기 깨끗한 궤적만 명령(2-8Hz 휠노이즈 주입 차단).
2. **권한 yield, pressed 게이트** — ACIGain 감소를 **디바운스 steeringPressed**에 게이트(오프셋 토크가
   아님). 핸즈오프 = **레거시와 bit-identical**(권한·드리프트복구 보존). 그립 시 [100,260]Nm에서 ceiling→
   0.10으로 공격 강하(레거시 0.41@250 → 0.14). 블렌드 yield를 권한이 대체.
3. **error-boost 억제** — 블렌드 없으면 grip 중 steering_error가 운전자 발산을 반영 → boost가 MDPS를
   op각도로 밀어 싸움. pressed 시 boost OFF(핸즈오프 드리프트복구는 유지).
4. resume 앵커(≥0.9)·angle_passive 래치 등 풀릴리스 경로는 그대로.

오프라인 확인: 핸즈오프(오프셋 90-180Nm 포함) ACIGain 레거시와 동일, 그립 시만 변화. 안전중립(panda
envelope·VM angle-limit 불변). 킬스위치 `YIELD_BY_AUTHORITY=False` → Phase 8b와 bit-identical.

### A/B 검증 (대기) — 통제 토글
같은 노선으로 `True`(신)/`False`(구) 빌드 비교. 관전 포인트: ① 코너 input→TX 2-8Hz gain이 실제로
하락(구조적 제거 확인), ② **override 용이성**(그립 시 op가 덜 싸우나 — 핵심 리스크), ③ 핸즈오프 추종
무회귀(설계상 bit-identical). 1차 튜너블: `ACIGAIN_GRIP_FULL_NM=260`, `ACIGAIN_GRIP_FLOOR=0.10`.

### 코드 재검토 (2026-06-19) — 앵커 일관성 수정
전체 재검토에서 유일한 결함: 저속 핸즈오프에서 오프셋이 `override_factor>=0.9`(저속 full_override 180 Nm)를
트립해 **heavy-override 앵커**(`apply_angle_last := wheel`)가 발화 → 블렌드를 없앤 신 모드에서도 raw 휠
2-8Hz가 거기서 재주입(저속 핸즈오프 ~2.7% 프레임). Phase 9 철학(그립 신호는 디바운스 pressed로 게이트)과
불일치. **수정**: 앵커를 `heavy_grip_anchor`(신 모드 = override>=0.9 AND pressed)로 게이트 — 실제 heavy
그립은 그대로 앵커(resume 보존), 오프셋-only 핸즈오프는 앵커 안 함(주입 제거). 레거시는 순수 override 게이트
유지(bit-identical). 이로써 코드측은 완결 — 남은 건 실주행 검증뿐.

## 1.M. Phase 7a-3 — 코너 곡률추종 cap 상향 (저속진동 소진 후 전환)

저속 1-2Hz 진동은 데이터로 **소진**(원인=모델곡률 지터→MDPS 플랜트 외란, roll 아님; op 권한 밖, 능동댐핑도
플랜트 식별 불가 coh 0.19/위상 std 105°로 불가). 전체 구조 survey에서 **유일하게 실효·고가치 미해결 = 코너
곡률추종 deficit**로 전환.

### 데이터 (풀링 0x50-0x5b, 빌드 무관)
핸즈오프 >35km/h 코너(|κ|>0.003) n=3705: **achieved/desired 비율 p50 0.82, p25 0.65** → op가 코너에서
~18% 바깥. 분포 거의 다 under(over-correction 드묾=헤드룸). §1.H의 0.75-0.82·cap 28-31% 포화와 일치.

### 조치
`LAT_FB_CAP 10e-4 → 14e-4`(~deficit 중앙값). 안전: 트림가속 v²·cap=0.39<0.5 m/s²@60km/h(accel cap이
80km/h↑서 바인딩), ERR_MAX(15e-4)>cap 유지(게이트 역전 없음), worst-case release ~1.4° wheel. 킬스위치
LAT_FB_KI=0. **한계**: deficit 일부는 모델 plan(7a 불가시) — cap으로 전부는 못 잡음. 순간비율 0.82엔 진입
lag도 혼재.

### A/B 검증 (대기)
코너 많은 노선 주행 → achieved/desired 비율 상승(0.82→?) + over-correction(비율>1.1, inside-cut) 감시.
올라가되 over-correction 안 늘면 성공. 늘면 cap back-off 또는 ERR_MAX 동반 검토.

## 1.N. Phase 7a-4 (ERR_MAX 안전버전) + lookahead 진입 lag 분석

### 데이터 — deficit 대역 (1s-LP 정상상태, lag 분리)
풀링 0x50-0x5b 코너 n=4782: 순간 비율 0.82 → **정상상태 0.91**(차이 = 진입 lag = "밀림"의 절반).
대역: <10e-4 **68%**(구 cap 처리) / 10-15e-4 **9%**(신 cap=14 타깃) / **>15e-4 23%**(ERR_MAX가 막음).
즉 cap 상향(7a-3)은 9%만 직접 개선, 진짜 미해결은 **23%의 sharp 코너(ERR_MAX 잠금)** + 진입 lag.

### 조치 — ERR_MAX 안전버전 (7a-4)
>15e-4의 23%는 1s-LP 정상값이라 **지속 sharp 코너**(스파이크 아님). 단 ERR_MAX 게이트는 **순간** fb_err에
걸려 올리면 windup 위험. 안전버전: 게이트를 **0.3s LP 오차**에 걸어 스파이크는 거르고 지속 deficit만 통과,
임계 15e-4→**22e-4**(지속), 별도 **순간 hard guard 30e-4** 유지, steeringPressed 게이트 불변. 트림은
여전히 cap(14e-4)+accel cap이 묶으므로 **권한·안전 envelope 불변**(어느 코너가 cap에 도달하나만 바뀜).
킬스위치 `LAT_FB_ERR_LP_TAU=0`+`ERR_MAX=15e-4`.

### lookahead 진입 lag — 분석만 (구현 보류)
desired→achieved 곡률 교차상관 lag(post-lookahead, n=9 windows): **net lag p50 ~170ms**(35-55kph 155ms).
즉 lookahead가 플랜트 지연을 **~170ms 덜 보상** → lead 더 줄 여지 있음. **단 ① 표본 9개로 얇음 ②
lookahead는 과조향(627a715) 민감 경로**(현재 conf-floor taper+7a 피드백으로 가드되나). → **이번엔 구현
안 함**(미검증 변경 3중첩 방지). 코너 많은 출퇴근 로그로 lag 재확인 후, 확정되면 **t_ahead 캡(0.25→0.30s)**
별도 검증 스텝. (정정: lead 레버는 boost_s가 아님 — 코너에서 base+boost=0.30이 이미 0.25 캡에 포화하므로
boost_s는 무효. 캡이 진짜 레버이고, lead는 6h-2의 J=0.7 jerk-budget으로 bounded.)

### A/B (대기): cap(7a-3)+ERR_MAX(7a-4) 합산 = "7a가 코너 deficit을 더 메움". 비율↑·over-correction 무증
확인. 진입 lag는 별도(lookahead).

## 1.O. Phase 7a-3/7a-4 온차량 검증 — 🔴 over-correction 회귀 → REVERT (2026-06-23, 0x5d-0x61)

빌드 `63bac87`(7a-3 cap14 + 7a-4 ERR_MAX sustained + Phase 9) 첫 실측. 사용자 정정: 출퇴근은 **이전과
동일코스**, 블루핸즈 왕복(점심)만 새 sharp 코스 → 동일코스 A/B 가능.

### 결과 (코너, hands-off >35kph)
| | OLD(cap10/ERR15) | 0x61(동일코스) | 0x5d(sharp) |
|---|---|---|---|
| \|κ\|p90 | 0.0098 | 0.0108(≈OLD) | 0.0254 |
| ratio p50 | 0.90 | **0.95** | 0.70 |
| **over>1.1(inside-cut)** | 11% | **23%** | 22% |

**동일코스(0x61)에서도** over-correction이 11%→23%로 2배. 중앙값은 0.90→0.95로 *개선*(7a가 deficit은
메움)이나 **꼬리에서 inside-cut**. 사용자 주관도 "안쪽 컷/과조향" 확인. 매칭 bin에서 sharp 코너 over-correction
0%→23-26%(7a-4가 푼 영역) — cap·ERR_MAX 둘 다 과함.

### 판정 — REVERT (Phase 7a-5)
deficit-닫힘과 over-correction이 **결합**(트림 세게 밀면 평균은 좁히되 꼬리서 inside-cut). baseline 0.90
ratio가 **실질 한계** — 그 이상은 model-plan deficit+진입 lag라 desired-vs-achieved 트림으로는 over-cut
없이 못 메움. `LAT_FB_CAP 14→10`, `LAT_FB_ERR_LP_TAU 0.3→0`, `ERR_MAX 22→15`(7a-2와 bit-identical).
killswitch 코드는 보존. lookahead lead 증가(진입 lag)는 **과조향 방향이라 보류**(over-correction 상황서
aggression 추가는 부적절).

### 🔴 안전사건 — 서울역 S자 차선침범 (0x5d seg2, GPS 0m 일치) = 이 over-correction의 실차 발현
사용자가 "늘 테스트하는 S자에서 다른 차선으로 넘어감" 보고. 좌향 bend apex 정량: 명령각이 desK 요구각보다
**+3.3° 초과**(7a cap ~3.6°에 일치), achieved/desired 곡률비가 **1.0→1.55로 overshoot**(inside-cut) →
좌선 접근(L -1.19) → 좌선 검출 점프(-1.17→-2.74)+prob붕괴(0.23, 차선 재검출=이탈) → 운전자 1733Nm 개입,
op 해제. **옛 baseline에선 이 S자 deficit(~20e-4)이 ERR_MAX 15e-4 게이트에 막혀 7a 미개입**(그래서 이전엔
정상)이었는데 7a-4가 게이트를 풀어 +3.3° 과조향을 주입한 것. **7a-5 revert가 게이트를 복원해 직접 제거.**
다음 주행 그 S자에서 재확인 필요.

### 무회귀 확인
- **Phase 9 yield: 사용자 "양보 자연스러움"** → ✅ 정상, 유지.
- LDW: 이탈 0건(토글 여전 미검증).

### 교훈
코너 곡률추종은 baseline(7a-2, cap10)이 sweet spot. 향후 더 짜려면 7a 트림(over-cut)이 아니라 **model-plan/
진입 lag** 쪽을 별도로 봐야 함(단 lookahead는 과조향 민감, t_ahead 캡이 레버).

## 1.P. Phase 7c-2 — lookahead lead 소폭 상향 (진입 lag용, 적분 대신 feed-forward)

7a-5 revert로 코너 트림은 baseline 복귀. 남은 under-steer는 **대부분 진입 lag**(0.82 순간 vs 0.90 정상,
~170ms 잔여)이고, **lag는 적분(7a)으론 와인드업→over-cut 없이 못 메움**(서울역 S자 사건이 그 증거).
정도(正道) = **feed-forward lead**(반응 아닌 예측 → 와인드업 없음). lookahead `t_ahead` 캡을 **0.25→0.27s
(+20ms)** 보수적 소폭 상향. additive lead는 여전히 J=0.7 jerk-budget(dk_max)으로 bounded → 불확실
reverse 커브서도 envelope 초과 불가. 7a와 직교(메커니즘 다름)라 동일 빌드서도 분석 분리 가능
(over-correction=7a / 진입lag=lookahead). 킬스위치 `LOOKAHEAD_T_AHEAD_CAP=0.25`.

### A/B (대기) — S자 우선 검증
① **서울역 S자에서 inside-cut 사라졌나**(revert 확인) + **lookahead가 과조향 안 만드나**(과조향 최악
케이스) ② 일반 코너 진입 lag(achieved/desired 순간비율) 개선 ③ over-correction 무증. S자 안전하면
다음 스텝(0.27→0.30) 검토, 과조향이면 즉시 0.25 복귀.

## 1.Q. 센터링 쏠림 근본원인 추적 — 아키텍처 갭(횡 미보상) (2026-06-25, 0x66/0x67)

**증상**: 직선 센터링이 한쪽으로 상수 쏠림 +0.14–0.25 m(두 로그 같은 방향). 이탈/inside-cut 안전
이벤트는 0건.

**정체 = yaw 오차 아닌 고정 횡 오프셋.** lateral profile: idx0(0 m)=+0.25 m, ~12 m=+0.22 m(거의 일정),
먼 거리서 약간 감소. yaw 오정렬이면 0 m에서 0이고 멀수록 커져야 하는데 **0 m에서 최대 → 회전이 아니라
프레임 전체의 상수 횡 평행이동 바이어스.**

**파이프라인 추적 — 보상 단 부재(아키텍처 갭):**
- 캘리브(`rpyCalib`) = roll/pitch/yaw 회전 + height(수직)만. `get_view_frame_from_road_frame` translation
  `[[0],[height],[0]]` — 수직 성분만, **횡(x) 항 = 0**.
- 모델 워프(`get_warp_matrix`) = `intrinsics @ view_frame_from_device @ device_from_calib`, **회전 전용**.
- `device_from_calib = rot_from_euler` 순수 회전 — 카메라 횡 장착 오프셋 보정 항 없음.
- `CameraOffset` Param 존재하나 `model_renderer.py` **UI 표시 전용**(params_keys.h:229), e2e 제어
  경로(controlsd → `model_v2.action.desiredCurvature`)에 미배선.
- → **횡 장착/모델 횡 바이어스를 보정하는 레버가 e2e 제어 경로 전체에 없음.** 빌드 무관하게 쏠림이
  남는 이유.

**두 성분의 합으로 좁힘:**
- (a) 카메라 횡 장착 오프셋 — 고정분(작음). 0.25 m는 윈드실드 mount 치곤 과대.
- (b) 모델 횡 바이어스 + 노면 의존 변동 — **주성분(가변).** 0.14→0.25 m 날짜별 변동은 고정 mount로
  설명 불가 → 모델이 차로 중심을 한쪽으로 약간 치우치게 추정 → desiredCurvature 정렬 → op 충실 추종.

**측정 한계**: 이 metric만으론 "차가 실제 한쪽 치우침" vs "차는 중앙·카메라가 옆으로 봄"을 분리 불가
(둘 다 거리무관 상수 오프셋). 분리엔 외부 ground truth 필요(없음).

**판정 = 수용(A).** 근본원인이 op 제어단 밖(모델 횡 바이어스 + 미보상 mount). PD 위치-피드백(B)은
모델 노이즈까지 되먹여 7a류 over-correction/inside-cut 위험 + 가변분이라 게인 고정 곤란. 차로폭(~3.5 m)
안 상수 쏠림이라 이탈 위험 없음(0x66/0x67 이탈 0건 확인). 근본 해결은 업스트림 모델/캘리브 개선 대기.

## 1.R. §1.Q 정정 — 우측 정상 오프셋의 roll-comp 기여 (제로베이스 재검토, faae679e, 0x63/66/67)

§1.Q는 lean을 "인지/평행이동 측 + 모델 바이어스, op-side 레버 없음 → 수용"으로 닫았다.
별도 세션의 제로베이스 재검토가 **부호/일관성**(§1.Q가 안 본 축)을 측정해 그 결론을 정교화:

**측정**: signed center-offset이 전 세그먼트·전 속도(44–114 km/h) **동부호 우측**,
직선 mean **+0.118 m**(n=15,886), 코너 **+0.127 m**(n=1,108). 세그먼트별 +0.05~+0.30 m 모두 양(우).

**메커니즘(코드 확인됨)**: localizer roll(`liveParameters.roll`)이 준-DC **+0.036~0.049 rad
(2.1–2.8°)**, 세그 내 std 0.006(사실상 상수), offset과 corr +0.05(무상관). 이 DC roll이
- `paramsd.py:17` ROLL_MIN/MAX ±10° 안이라 clip 안 되고 KF로 통과,
- `latcontrol_angle.py:142` ≥15 m/s(54 km/h) full roll gain(1.0),
- `vehicle_model.py:121` `roll_compensation=g·roll/(1/sf−u²)`를 `get_steer_from_curvature`가 매 프레임 차감
→ 존재하지 않는 뱅킹을 상시 메우는 방향으로 명령각 bias(50 km/h 등가곡률 ~0.002 1/m)
→ 차로유지 루프와 평형 → 상수 우측 오프셋. order-of-magnitude로 0.12 m와 정합.

**판별(준-DC bias vs 실뱅킹)**: 세그 내 roll std 0.006으로 상수인데 offset std 0.24로 변동
→ 실뱅킹이면 offset이 roll을 따라야 하나 안 따름. 한국 시내 크라운은 방향이 자주 바뀌므로
진짜 roll이면 분산이 커야 함 → **마운팅/캘리 roll bias가 road-roll로 귀속**이 유력.

**§1.Q "no clean op-side lever" 정정**: 틀렸음. roll-comp 경로를 (저속 댐핑 때문에) 닫아버린 게
누락 원인. 실제로는 두 레버 존재 — (1) **재캘리브레이션(코드 0)**: roll DC가 캘리 artifact면 해결,
(2) **roll DC 차감**(`_filtered_roll`에 τ~120s HPF, 6h-6 LP와 직교): 캘리로 안 빠지는 잔여 DC만 제거.

**caveat**: 인과 미확정(angleOffset −0.44°/카메라 yaw/실평균 크라운 공동 기여 가능). 세그 간
roll DC 차이(0.036 vs 0.049)는 부분 실뱅킹 혼재 시사 → **roll DC 차감을 무리하게 걸면 상시
편경사 도로(고가도로)에서 보상 깎는 부작용** → 코드 수정 전 진단 필수.

## 1.S. ZB-1 Step A 진단 완료 — roll bias = 차량고정(노면뱅킹 아님) 확정 (2026-06-29, 0x66/0x67 qlog 직독)

ccnc-drivelog 브랜치에서 0x66(39seg)·0x67(30seg) qlog 다운로드 → pycapnp 직독으로
liveParameters.roll + gpsLocationExternal(lat/lon/bearing) + carState 추출, GPS-시간정렬.
(빌드 faa679e. qlog엔 modelV2 없어 offset 직접 재측정은 rlog 필요 — 본 진단은 roll+GPS로 충분.)

**roll DC (lpvalid):** 0x66 mean **+0.0397 rad(+2.28°)** med +0.043 std 0.015 / 0x67 mean
**+0.0294 rad(+1.69°)** med +0.031 std 0.015. **두 주행 같은 방향(+).** angleOffsetAvg −0.06°
(무시 가능 → 원인 후보 제외).

**🔴 결정적 판별 — 같은 도로 반대 방향(출근/퇴근):** 실뱅킹(크라운)이면 진행방향 반대 시 roll
부호가 뒤집혀야 함. 차량고정 bias면 부호 유지.
- **8개 heading 섹터 전부 roll 양수** (+0.017~+0.043 rad). 4개 반대-섹터 쌍(0/180, 45/225,
  90/270, 135/315) **전부 same sign**.
- **같은 GPS 셀·반대 heading(>120°)**: 111m 격자 n=3 → roll **sign-agree 100%**, 둘 다 양수
  (0x66 +0.040 / 0x67 +0.036); 222m 격자 동일. **roll 부호 안 뒤집힘 = 노면 뱅킹 결정적 배제.**
→ **roll = 차량고정 DC bias(마운팅/캘리) 확정.** 평균 ~+0.033 rad(+1.9°).

**메커니즘 정량(VM, 타이어강성 근사)**: phantom roll +0.033이 full roll-comp(≥54 km/h) 통과 →
컨트롤러가 **상시 ~+0.5° 조향휠 bias** 유지(속도 무관) → 차로유지 루프와 평형 → 상수 횡 오프셋
(§1.R 실측 +0.12 m와 정합). g·roll 등가 상시 횡가속 ~0.32 m/s²(고속 점근).

**캘리 vs 마운트 성분**: 두 주행 roll DC 차이(+0.040 vs +0.032, ~0.008 rad)는 캘리가 주행마다
약간 다르게 수렴함을 시사 → 캘리 성분 존재. 그러나 큰 동부호 평균이 유지됨 → 캘리가 못 잡는
물리 마운트 tilt도 혼재 가능 → **재캘리 단독으론 완전 제거 안 될 수 있음.**

**판정/권고 (§1.Q "수용"·§1.R "진단 먼저" 갱신):** roll bias가 노면 아닌 차량고정으로 **확정**되어
op-side 수정이 정당화됨. 2-track:
1. **재캘리브레이션(코드 0)** — 먼저. 캘리 성분(~0.008 rad day-to-day) 제거. 차량에서 사용자 실행.
2. **roll DC 차감**(`latcontrol_angle._filtered_roll`에 τ~120s HPF, kill `ROLL_DC_TAU=0`) — 재캘리로
   안 빠지는 잔여 마운트 DC 제거. 6h-6 LP(고주파)와 직교. **caveat**: 분 단위 상시 편경사 도로
   (장대 고가 곡선)에선 뱅킹 보상도 서서히 깎임 — 단 본 데이터가 크라운 빈번 전환(AC 성분 존재)을
   입증하므로 τ120s는 진짜 DC만 포착. 게이트: 직선 signed offset +0.12→±0.03 m, 진동/추종 무회귀,
   상시편경사 구간 offset 감시.
- 다음 스텝: 사용자 재캘리 → 다음 주행 로그로 roll DC·offset 재측정 → 잔여 DC 있으면 roll DC 차감 구현.

## 1.T. 업스트림 sync 검토 — sunnypilot hkg-angle-steering-2025 (2026-06-30)

i6n(현 HEAD **faa679e**)과 sunnypilot 업스트림 비교. 결론: **차용할 것 없음 — angle-steering
영역에서 i6n이 기능·실차검증으로 앞서 있고, 업스트림 고유 변경은 i6n에 부적합.**

**ref 좌표 (재조사 불필요하도록 고정):**
| 항목 | commit | 날짜 | opendbc pin |
|---|---|---|---|
| **fork 지점** | `fffb5ab` "sunnypilot v2026.03.13-4326" | 2026-03-13 | `966c60b8d593` (업스트림 force-push로 GC됨, raw 404) |
| Kay Oh 첫 커밋 | `c91727f` "Update values.py" | 2026-03-31 | — |
| 사용자가 링크한 `…-2025-prebuilt` | `705bfb5` (github-actions bot 빌드) | **2026-04-10 (STALE)** | — |
| **활성 소스** `hkg-angle-steering-2025` | `282f517` | **2026-06-09** | `0b5dacee34fa` |
| i6n HEAD | `faa679e` | (현재) | opendbc_repo **vendored**(서브모듈 아님) |

opendbc 서브모듈 repo = **github.com/sunnypilot/opendbc**. 업스트림이 두 브랜치를 force-push/
rebuild → fork base와 **공통 조상 끊김**(`No common ancestor`) → git-compare 불가, content/
commit-history 분석으로 우회. (proxy: krvista/openpilot만 git smart-HTTP 허용, 그 외는 raw/API만.
cross-fork compare selector 차단.)

**fork 이후 car/hyundai 변경 = 64 커밋(3개월).** 분류:
- **ACIGain/토크감쇠 대수술**: `TorqueReductionGainController` 클래스 제거(`00d5c0de`)→함수형 전환,
  speed/error 보정 추가 후 다시 단순화(`b6e30040`/`5278c0fb`/`b3913542`). → **업스트림도 독립적으로
  클래스/shelf 정리 방향에 수렴**(i6n Phase 10 + dead-code prune과 동일 결론).
- **lookahead/안전위반 예측 실험 전부 REVERT**(`cf3223a5`/`4cce6392`/`c941e665` 등) = net no-op.
  → i6n 7c-2 feed-forward lead가 더 앞섬.
- **신규 차종/핑거프린트**: Ioniq 6(non-HDA-II) `3e4da61b`, Ioniq 9 `1954b1cb`, Ioniq 5 PE,
  Kona EV, Niro, Santa Fe PHEV, **Sorento HDA2 angle** `115b68b5` — i6n 거동 무관(참고용만).
- **comma/opendbc 인프라 sync**: DBC→generator `f1ec12a4`, CAN FD DBC 갱신, 범용 BSM, CarState.brake
  deprecate — opendbc 전체 resync 시에만 의미.

**상세 비교 2건:**
1. **EPS-whine smoothing `88bb52cf`** (2026-04-03, +34/-5 carcontroller): `sp_smooth_angle` 속도-EMA
   재구현 + 0.1° deadzone. 저속 α 0.05~0.10(매우 무겁게)→80km/h α 1.0. **i6n과 동일 함수(공통 조상)**.
   i6n은 Phase 6c-3에서 같은 저속 무거운 α를 했다가 **6g-1 코너 wide-running으로 기각 → 6h-1에서
   floor 0.30 + jitter 흡수를 controlsd 곡률-LP(matched lead)로 이전.** 업스트림 α0.05(τ~0.2s lag)는
   i6n이 의도적으로 피한 코너-진입 lag. **포팅 시 6g-1 회귀.** 저속진동(§1.J plant-지배, <20km/h
   op-inactive)엔 각도-EMA 무효.
2. **ACIGain 최종 `compute_torque_reduction_gain` @0b5dace**: 업스트림 = 4-bp **mid-torque SHELF**
   곡선(ceiling→shelf→shelf→floor), 토크 breakpoint **속도-스케일**(저속 bp1=75Nm→고속 125Nm),
   m/s 단위, rate −0.014/+0.004 고정, **steering_error/blinker/grip 없음**. i6n = 2-bp(shelf 없음),
   error_mult ceiling boost + blinker yield + Phase 9 grip-band + error rate_up. 업스트림 고유 2개 모두
   부적합: **(a) SHELF = i6n Phase 10에서 전라우트 A/B로 net-negative 기각필. (b) 속도-스케일 bp =
   저속 bp1=75Nm가 i6n 컬럼-토크 오프셋(+90~180Nm hands-off)에 걸려 저속 hands-off authority 헛감소
   → i6n Phase 9 grip-state 게이팅이 센서오프셋에 안 걸리는 올바른 축.** 기능상 i6n이 superset.

**권고**: 업스트림 sync 불필요. 단 차종 포팅(신규 핑거프린트)·comma 인프라 변경이 필요해지면 그때
opendbc resync 별도 검토. 업스트림=단순화 방향 / i6n=차-특화 정교화 — i6n 센서오프셋·실차검증 감안 시
i6n 방향 유지가 맞음.

## 1.B. A/B 비교 결론

| 항목 | 5479ecc baseline | d83c3b5 6F2-A | 판정 |
|---|---|---|---|
| HO op-active TX 추적 | p95 0.10° | p95 0.10° | **둘 다 floor (동등)** |
| exit-transition spike | p95 2.27-2.38° | p95 1.62-2.64° | **유의차 없음** |
| hand-off lag p50 | 339-533 ms | 187-1541 ms | **빌드 무관 — 다른 게이트** |
| sign-mismatch | 0-0.00% | 0.01% | **둘 다 사실상 0** |
| 안전 회귀 (TX/cam/panda) | clean | clean | **무회귀** |

**핵심 발견:**
1. **이전 audit §4 "p95=27° heavy-override 문제" 는 decode 버그**. 실제 deployed code 는 양 빌드 모두 wheel 을 0.1° 이내로 추적 중이었음. **6F2-A 가 fix 할 진짜 문제 없었음** (harmless code change — 회귀도 없음).
2. **신규 P0 = hand-off lag (p50 0.2-1.5 s)** 양 빌드 동일 → 다른 게이트 (apply_steer_req / vm_reject_persistent / angle_passive_active latch) 의 frame-by-frame 분류 필요.

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

> **⚠️ 신뢰 불가 (2026-06-10, Phase 6h COMMIT 0):** 본 절의 "15,775 frames TYPE B"
> 통계는 `ioniq6n_full_drivelog_sweep.py`의 **구 디코드(byte4-6 / gain÷255)** 산출물.
> §1.A 에서 sim 도구의 디코드만 고치고 sweep 도구는 누락됐었음 — COMMIT 0 에서
> byte10-11 / gain×0.004 로 수정 완료. **재측정 전까지 본 절 수치 인용 금지.**

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
| **6g-1a** | **P0 ✅ DONE** | **코너 미추종 fix — `sp_smooth_angle` slew/maneuver-aware**: gap `|desired-apply_last|` ≥ `SMOOTHING_ANGLE_RELEASE_HI_DEG(4°)` 면 α→1, ≤`LO(1°)` jitter 는 저-α 유지. 저속 떨림 흡수 보존 + 코너 응답 복원(단위 데모: 15° 램프 95% 도달 24→15 frame). kill switch=HI 거대화. | `carcontroller.py:156`, `values.py:43` / §1.D.C |
| **6g-1b** | **P0 ✅ DONE** | **신뢰도 댐핑 floor** — `confidence=max(min(conf_y,conf_l), LAT_CONF_FLOOR=0.5)`. 저신뢰 코너진입(교차로 직후 차선 재연결)에서 명령을 직진으로 얼리지 않게. kill switch=`LAT_CONF_FLOOR=0.0`. | `controlsd.py:286` / §1.D.C |
| **6g-2a** | **P0 ✅ DONE** | **과조향 보정 — conf_floor taper**: `confidence=max(min(conf_y,conf_l), LAT_CONF_FLOOR*clip((lane_min-LO)/(HI-LO)))`, `LO=0.20/HI=0.30`. apex 붕괴(lane_min<0.20)에서 floor→0 freeze(스파이크 차단), 재연결대 유지. kill=`LAT_CONF_FLOOR=0`. | `controlsd.py` / §1.E.B,D |
| **6g-2b** | **P0 ✅ DONE** | **release 상한** `SMOOTHING_ANGLE_RELEASE_MAX=0.7` — 코너 catch-up 에 ~30% 댐핑 잔존("휙" 완화). kill=1.0. | `carcontroller.py:156`, `values.py` / §1.E.D |
| **6g-2c** | **P0 ✅ DONE** | **저속 미세 데드밴드** `SMOOTHING_ANGLE_DEADBAND_DEG=0.4`(<`DEADBAND_MAX_VEGO=11m/s`) — 25 km/h 5-7 Hz dither 제거. kill=0. | `carcontroller.py:156`, `values.py` / §1.E.C |
| **6g-2v** | **P0 ✅ DONE(W0)** | **6g-2 온차량 실증** — §1.F: deadband staircase 실차 확인(hold 46-58%, 사용자 체감 악화 보고), 진입 lag 분해(under 35-41% 진입 집중), 센터링 우수(0.07-0.09 m). 6h-1 정당화 완료. | §1.F |
| **6g-4** | **P1 ✅ DONE** | **op-active LDW 구현** — `ldw.py`에 연속-접근 검출기 분기(prob>0.5×7f, clr<0.7 m, 단조접근 ≥0.10 m, 재라벨 점프 배제, laneChangeState=off, v>50 km/h). **6라우트(0x40–44/48, ~3.2 h) 재생: 오발화 0 / 진성 2건만 포착**(0x41 seg22 113 km/h 좌측 0.65 m, 0x48 seg15 72 km/h 우측 0.70 m — 둘 다 기지 near-line 사건). 부수효과: controlsd 차선이탈 억제 블록이 op-active 중 처음으로 유효해짐(차선 쪽 추가 조향만 동결, 발화율 ~0.6회/h 전수 검사). kill switch `OP_LDW_CLR_M=0`. 수동주행 경로 불변. | §1.F.D |
| **U-3296** | **검토 후 비반영** | upstream opendbc#3296 (CANFD BCW 1→2bit) — i6n 은 CCNC `_ALT` 비트(138/141)를 읽으며, 4라우트 원시 0x1BA 스캔: 같은쪽 블링커+감지(=점멸조건) 731프레임 전부 인디케이터 비트 1 유지, 2-bit state-2 패턴 출현 0회(우리 `LEFT_MB`/`MORE_LEFT_PROB` 30/32비트도 동일). **버그 시그니처가 이 플랫폼에 부재** — 수정 불요. | raw CAN 검증 |
| **U-3305** | **검토 후 비반영** | upstream opendbc#3305 (SCC cancel 버튼 100 ms 지연) — PR 자체가 `CANFD_ALT_BUTTONS` 제외 + i6n CCNC-ALT 경로는 cancel 시 **아무것도 TX 안 함**(`if not ccnc_lka_alt:` 가드, 팩토리 SCC 자체관리). 이중 비해당. | carcontroller.py:870 |
| **6g-2d** | **❌ 기각(측정)** | **geometry clamp** — 차선기하 비율/lookahead-fallback 밴드 둘 다 과조향(곡률이 모델 fallback 자체·저신뢰 구간)을 못 잡고 정상 코너만 손상. 실집행 레버(6g-2b)가 정답. 후속: RELEASE_MAX 0.7→0.5, clip_curvature jerk 비대칭. | §1.E.E |
| **6g-3 (신규)** | P1 | **계측 배선** — `lateralAccelLimit/steerAngleLimit` EventName 이 실제 publish 되는지 확인·수정 (147 saturate frame 에 0 event). 또는 plan §1.5 문서 정정. | §1.D.D |
| **6g-4 (신규)** | P1 | **LDW 미발화** — seg5/seg9/seg22 차선 침범에 `laneDeparture` 0건. driverAssistance/LDW 게이트 점검. | §1.D.C |
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
