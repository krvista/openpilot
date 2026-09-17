# WK2 NNLC 자가 학습 — 준비 체크리스트

기준 빌드: `krvista/openpilot` 브랜치 `wk2-fixes-release-mici` (tip **d67c5fe**: v2026.002.002 + 크래시픽스 + 토크시드 + 54km/h 홀드)

## A. 장치 (주행 전 1회)
1. 빌드 업데이트: `cd /data/openpilot && git fetch origin wk2-fixes-release-mici && git reset --hard origin/wk2-fixes-release-mici && sudo reboot` → Settings › Software 에서 Commit **d67c5fe** 확인
2. **컨트롤러 설정 (순서 중요)**: Settings › sunnypilot › **Steering** 에서
   ① "Neural Network Lateral Control" **OFF** → ② "Enforce Torque Lateral Control" **ON**
   WK2는 opendbc 기본 횡제어가 **PID**라서, NNLC와 Enforce Torque가 둘 다 꺼지면 토크 컨트롤러가 아닌 PID로 조향합니다
   (`sunnypilot/selfdrive/car/interfaces.py`: `if nnlc_enabled or enforce_torque: configure_torque_tune`). PID 로그(`pidState`)는 NNLC 학습에 쓸 수 없습니다.
   두 토글이 동시에 ON이면 UI가 둘 다 자동으로 꺼버리므로 반드시 NNLC를 먼저 끄고 Enforce Torque를 켭니다.
   확인: 첫 주행 세그먼트 하나로 `nnlc-extract <seg_dir> -o /tmp/d.csv --limit 1` → `lateral_control_type counts: {'torqueState': …}` 이어야 함.
3. 학습 파라미터 살리기: `/data/params` 절대 초기화하지 말 것. 2–3회 주행 후 `collect_check.py`로 liveTorque `useParams=True`, liveDelay `estimated` 확인 후 본수집 시작
4. 저장공간: 카메라 포함 시간당 ~2–3GB. 장치가 꽉 차면 오래된 라우트를 자동 삭제하므로 **주행 후 그날 안에 PC로 동기화**
5. 기계 상태 고정: 얼라인먼트 완료 상태 유지, 타이어/공기압 변경 없이 수집 기간 통일

## B. 주행 프로토콜
- 목표 **5–10시간 / 20–30라우트**, 라우트당 2분 이상 활성 구간
- 63km/h 이상에서 인게이지(이후 54까지 유지됨), **손은 얹되 힘 빼기** — 목표: 활성 중 |steeringTorque| p90 < 40 (현재 176)
- 좌/우 코너 균형, 마른 노면, 경사·롤 다양하게, 정체/공사/주차장 회피
- 개입은 최소 (라우트 점수: 개입 >10% 시 감점, 포화 >5%, 비활성 >20%)
- 커버리지 플롯의 빨간 빈(<50샘플)이 **도달 가능한 영역(≥54km/h)** 에 남아 있으면 그 조건을 겨냥해 추가 주행 (예: 70–90km/h 램프·와인딩 = 고횡가속 빈)

## C. PC (WSL) 준비
1. `git clone https://github.com/amzoo/openpilot-nnlc-tools.git && cd openpilot-nnlc-tools`
2. 패치 적용: `git apply /path/to/nnlc-tools.patch` (sunnypilot 스키마 · MADS active 판정 · 세그먼트 자연정렬 · 라우트경계 temporal 보정)
3. `uv venv && uv pip install -e .` (또는 `bash scripts/setup.sh`)
4. Julia: `curl -fsSL https://install.julialang.org | sh` (juliaup) → `julia training/install_packages.jl` · NVIDIA면 CUDA 자동, 없으면 `--cpu`
5. 환경변수: `export NNLC_CEREAL_DIR=/path/to/sunnypilot/cereal` (sunnypilot 리포의 cereal 디렉토리 — 로그를 만든 빌드와 같은 브랜치)

## D. 수집 후 파이프라인 (라우트 단위로 반복)
```bash
# 1) 장치에서 rlog 동기화 (per-segment 디렉토리 레이아웃으로 저장됨)
uv run nnlc-sync -d <장치IP> -o ./data
#    (이미 flat 파일로 받아둔 경우) python3 layout_rlogs.py <flat_dir> ./data --dongle 99b215d21bbf8735
# 2) 추출 (temporal 필수)
NNLC_CEREAL_DIR=... uv run nnlc-extract ./data -o ./output/JEEP_GRAND_CHEROKEE_2019.csv --temporal
# 3) 점수·프루닝·커버리지
uv run nnlc-score  ./output/JEEP_GRAND_CHEROKEE_2019.csv
uv run nnlc-prune-routes ./output/JEEP_GRAND_CHEROKEE_2019.csv --min-score 60 -o ./output/routes_pruned.csv
uv run nnlc-visualize ./output/routes_pruned.csv -o ./output/coverage.png
uv run nnlc-interventions ./output/routes_pruned.csv --prune both --prune-output ./output/pruned.csv
# 4) WK2 전용: 서브임계 그립(40~120) 프레임 제거
python3 prune_grip.py ./output/pruned.csv -o ./train/JEEP_GRAND_CHEROKEE_2019.csv --max-torque 40
```
- CSV 파일명 = 핑거프린트명(`JEEP_GRAND_CHEROKEE_2019.csv`) — 학습 스크립트가 파일명에서 차종/출력명을 정함
- 2)~4)는 라우트 업로드만 해주면 서버(Claude) 쪽에서 대신 실행 가능

## E. 학습 → 배포 → 검증
```bash
bash training/run.sh ./train/            # GPU 없으면 뒤에 --cpu ; 결과: ./train/training_results/<n>_JEEP_GRAND_CHEROKEE_2019/JEEP_GRAND_CHEROKEE_2019.json
scp .../JEEP_GRAND_CHEROKEE_2019.json comma@<장치IP>:/data/openpilot/sunnypilot/neural_network_data/neural_network_lateral_control/
```
- 기존 JSON은 백업(`.orig`) 후 교체, 장치 재부팅, **NNLC 토글 ON**
- 이 폴더는 서브모듈이라 브랜치 업데이트/reset 시 원복됨 → JSON 사본 보관
- 첫 주행은 손 얹고 관찰 → 로그 업로드 → `lat_quality` 비교(교체 전/후 추종오차·떨림)

## F. 수집 시작 전 최종 점검 (`collect_check.py`)
```bash
python3 collect_check.py <route>--0--qlog.zst <route>--1*--qlog.zst   # 세그먼트 0 포함 필수
```
기대 출력: build `d67c5fe` · 컨트롤러 `torqueState`(pidState면 Enforce Torque 미설정) · feedforward `LINEAR (NNLC OFF)` · liveTorque `useParams=True` · liveDelay `estimated` · active below minSteerSpeed > 0% · grip p90 < 40

## H. 대용량 rlog를 GitHub에 올리지 않는 운용 (권장) — PC에서 추출, CSV만 공유
장치에 쌓인 rlog가 GB 단위라면 G 대신 이 흐름을 씁니다. 9.6GB를 git에 올리면 쿼터를 넘고, 브랜치를 지워도 히스토리에 남아 용량이 안 줄어듭니다.
1. 최초 1회 (WSL):
   ```bash
   git clone https://github.com/amzoo/openpilot-nnlc-tools.git && cd openpilot-nnlc-tools
   git apply /path/to/tools/wk2_nnlc/nnlc-tools.patch && uv venv && uv pip install -e . && source .venv/bin/activate
   git clone --depth 1 --filter=blob:none --sparse -b wk2-fixes-release-mici https://github.com/krvista/openpilot.git ~/sp-cereal \
     && git -C ~/sp-cereal sparse-checkout set cereal opendbc_repo/opendbc/car   # log.capnp -> car.capnp 심링크가 opendbc를 가리킴
   export NNLC_CEREAL_DIR=~/sp-cereal/cereal      # 로그를 만든 빌드와 같은 브랜치의 cereal
   ```
2. 라우트 수집 후 (장치와 같은 LAN):
   ```bash
   bash tools/wk2_nnlc/pc_pipeline.sh -d comma@192.168.1.155 -o ~/wk2-nnlc --expect-commit d67c5fe
   #   -r "0000002b--ec8d875f19 0000002a--9e72ae868f"  로 라우트 제한 가능,  --delete-rlogs 로 추출 후 즉시 삭제
   ```
   결과 `~/wk2-nnlc/train/`: `JEEP_GRAND_CHEROKEE_2019.csv`(학습용) + `.csv.gz`(공유용) + `coverage.png` + `score.txt` + `audit.txt`(라우트별 빌드/NNLC on·off/그립 판정) + `keep_routes.txt`
3. `audit.txt`에서 **NNLC on 또는 빌드 불일치 라우트는 자동 제외**됩니다 — 표를 보고 의도와 다르면 알려주세요.
4. 학습: `bash training/run.sh ~/wk2-nnlc/train/` (CSV가 이미 로컬에 있으므로 브랜치를 거칠 필요 없음)
5. 공유(선택, 수십 MB): `train/` 안의 gz·png·txt만 `wk2-nnlc-train` 브랜치에 푸시 → 커버리지·점수 리뷰를 서버에서 진행
6. rlog는 모델이 검증될 때까지 `~/wk2-nnlc/rlogs`에 두었다가(재추출 대비) 삭제

## 결정 기록 (record-the-why)
- **2026-09-17 자가 NNLC 학습 중단, 기존 모델 사용** — 이유: 학습에 필요한 데이터(토크 컨트롤러 + NNLC OFF + 가벼운 그립, 5–10시간)를 새로 모아야 하는데, 이 차의 개입 가능 영역(≥54km/h, 횡가속 ≤1.3)이 좁아 기존 모델 대비 기대 개선폭이 작고, 지난 한 달치 로그는 PID 컨트롤러 로그라 사용 불가로 판명됨. 대안으로 남긴 것: 라이브 파라미터 수렴·보존, 그립 개선, 커뮤니티 모델 갱신에 로그 기여.
- **2026-09 PID 회귀 사고** — NNLC를 끄자 토크 컨트롤러가 아니라 opendbc 기본 PID로 떨어짐(`sunnypilot/selfdrive/car/interfaces.py`: `if nnlc_enabled or enforce_torque`). 원인은 안내 누락("Enforce Torque Lateral Control"도 켜야 함). 결과: 21개 라우트(~15h)가 `pidState`로 기록돼 학습 불가, 토크 경로 학습도 한 달간 정지. 교훈: 설정 변경 후 첫 세그먼트에서 `lateral_control_type`을 확인하는 단계를 A-2에 추가.
- **토크 시드(1.95/0.175)는 잠정값** — torqued 학습이 서브임계 그립(활성·미개입 프레임의 49%가 40–120 토크)에 오염됐을 수 있어 latAccelFactor가 낮게 편향됐을 가능성. 가벼운 그립으로 2–3회 주행 후 수렴값과 대조해 확정.
- **SCC-V 리튠 폐기** — 사용자가 SCC-V를 쓰지 않아 브랜치·PR에서 제거(2026-08).
- **홀드 하한 54km/h(15.0m/s)** — EPS가 13m/s까지 받는다는 upstream 주석은 이 차 실측이 아니며, LKAS 비트 하한(14.5)보다 위여야 램프다운이 유효하므로 보수적으로 시작. 실주행 로그로 63 미만 조향이 확인되면 52까지 확장.
