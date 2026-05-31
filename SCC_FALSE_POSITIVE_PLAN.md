# SCC-V 직진 false-positive 감속 — 분석 + 수정 (구현 완료)

차량: **2020 Jeep Grand Cherokee**, 동글 `0fb02cc3a5abcc2f`. 브랜치: **sunny-tizi-v6**.
증상: SCC-V + SCC-M 둘 다 ON 으로 직진 도로에서 부당하게 속도가 줄어, 운전자가 액셀 재가속 또는
ACC 해제.

## ★ 빌드 검증 (중요 — 어떤 로그가 v6 인가)
rlog `initData.gitCommit` 으로 각 route 가 어느 빌드에서 기록됐는지 확인 (`tools/wk2_build_check.py`):

| route | 빌드 | gitCommit | branch | SCC-M param |
|---|---|---|---|---|
| 0,1,5,6 | 2026.001.**004** | e1fe9abd | sunny-**tizi** | **0 (off)** |
| **10,11,12** | **2026.001.006** | **644e7f06** | **sunny-tizi-v6** | **1 (on)** |

→ **route 10/11/12 만 v6(644e7f06) = 현재 수정 대상 빌드.** route 0~6 은 구버전(v004, Fix E/F/G
없음, SCC-M off) → v6 검증에 부적합 → 분석/검증에서 제외. 아래 모든 수치는 v6 routes 만 사용.

## 데이터 (실제 rlog, 검증됨)
`wk2-drivelog` 브랜치 `drivelog/` 의 v6 routes (10/11/12) 를 cereal 직접 디코딩으로 풀레이트 분석.
도구는 `tools/wk2_*.py` (LogReader 가 하드웨어 스택을 끌어와 막혀, pycapnp 로 `cereal/log.capnp`
직접 로드해 `.zst` 디코딩). 직진 = 실측 횡가속 `v_ego²·|curvature| < 0.7 m/s²`.

## 근본 원인 (v6, 프레임 단위 확정)
- **SCC-V 직진 오진입** (route11 `..1b2deee273`): 직진에서 `entering` 8회(69~83km/h), 2회 ACC
  해제. 8건 모두 `maxPredictedLateralAccel` 1.30~1.47 = near-horizon 임계 2.0 미만 →
  **anticipation 경로(`_ANTICIPATE_PRED_LAT_ACC_TH=1.3`)로 진입**.
- **분포** (`tools/wk2_dist.py`, route11 직진 18,424프레임): anticipation 윈도우 값이
  **p99≈1.4, 단발 max≈2.4** → 임계 1.3 이 직진 노이즈 안에 있음.
- **SCC-M 직진 큰 감속** (route12 `..e36e15a16f`): 완전 직진에서 맵이 **30km/h 타깃 =
  24~28km/h 한방 드롭** (turning 76프레임), 그 동안 카메라 예측 횡가속 max 0.09 (=직진). →
  운전자 ACC 해제. **한국 OSM/Google 맵 데이터가 신뢰 불가**.

## 구현된 수정 (sunny-tizi-v6)

### SCC-V — `vision_controller.py` (코드 수정)
1. `_ANTICIPATE_PRED_LAT_ACC_TH` **1.3 → 1.8** (직진 p99 1.4 위로 분리), abort `1.1 → 1.4`.
2. `_ENTERING_DEBOUNCE_FRAMES = 3` + enabled→entering 연속프레임 디바운스(>3 = 4프레임 ≈0.2s).
   직진 단발 spike(최대 2.4)가 1프레임 임계를 넘겨도 진입 안 함.
3. near-horizon 임계(2.0) · drop 커브(`_PRED_DROP*`, `_ANTICIPATE_DROP*`) 는 **그대로** →
   실제 곡선의 near-horizon 처리/감속감 유지.

### SCC-M — 코드 변경 없음 (사용자가 UI 에서 OFF)
한국에서 OSM/Google 맵 데이터가 정확하지 않아 false-positive 위험이 큼. 사용자 결정:
**코드 변경 대신 UI 토글로 SmartCruiseControlMap OFF** (설정 → Cruise). 맵 데이터 품질이
회복/한국 지원 개선 시 UI 에서 다시 켤 수 있음.

route12 의 SCC-M turning 76프레임은 코드 수정 근거가 아니라 **SCC-M 을 끄도록 권장하는 근거**.

## 검증 (리플레이, v6 routes only — `tools/wk2_validate.py`)
OLD(=v6 기존 SCC-V 로직) vs NEW(=구현 config) 를 v6 rlog 에 리플레이. 직진 entering = 직진에서의
enabled→entering 진입 수. (절대값은 long_enabled 게이트 미반영으로 과대계상되나 OLD/NEW
**상대 비교**가 유효.)

| route(v6) | 직진 entering OLD→NEW | 곡선 active 프레임 OLD→NEW (보존율) |
|---|---|---|
| route11 (SCC-V FP) | **24 → 2** (-92%) | 1300 → 590 (45%) |
| route10 | 16 → 2 | 1402 → 612 (44%) |
| route12 | 11 → 2 | 612 → 251 (41%) |

SCC-M: 사용자가 UI 에서 OFF → route12 의 24-28km/h 직진 한방 감속이 원천 차단.

## 트레이드오프 / 한계 (정직 기록 — 실주행 확인 필요)
- **SCC-V 곡선 active 가 ~41~45% 로 감소.** near-horizon(2.0) 경로는 유지되므로 실제 곡선 진입
  자체는 작동하나, "곡선 3-5초 전 미리 살짝 줄이는" 조기 anticipation 이 약해짐. 횡제어/메인 long
  planner 가 곡선을 처리하므로 안전 영향은 적으나 **체감/거동은 실차 확인 필요**.
- **SCC-M 은 UI 토글로 OFF 가 운영 정책.** 한국 도로/맵 데이터 환경 때문이며 코드 변경 아님.
- **실주행 검증 불가**(샌드박스). 검증 = py_compile + 기록 rlog 리플레이까지. 실차 거동 미확인.
- 임계는 이 차량/v6 3개 route 기준. 다른 차량/도로 일반화엔 추가 데이터 필요.

## 재현/분석 도구 (`tools/`, 커밋됨)
- `wk2_scc_scan.py`  : 직진 SCC engagement + 운전자 gas/cancel 스캐너(직접 capnp).
- `wk2_build_check.py`: rlog initData 의 빌드(gitCommit/branch/param) 추출 — **로그-빌드 매칭 필수**.
- `wk2_dist.py`      : 직진 vs 곡선 예측횡가속 분포(임계 근거).
- `wk2_episodes.py`  : entering/turning 에피소드 길이(디바운스 근거).
- `wk2_validate.py`  : OLD vs 후보 config 리플레이 비교.
- `wk2_map_check.py` : 맵 turning 시 vision corroboration 측정(SCC-M 끄기 권장 근거).
- 사용: `PYTHONPATH=$PWD python3 tools/wk2_xxx.py <out.json> '<rlog glob>'`.
  드라이브로그: `git checkout origin/wk2-drivelog -- drivelog/<route>--<seg>--rlog.zst`.
  (cereal 스키마 로드에 `opendbc_repo/opendbc/car/car.capnp` 필요; submodule 미체크아웃 시 다른
  브랜치 blob 으로 임시 생성. drivelog/*.zst 와 wk2_*.json 은 .gitignore.)
