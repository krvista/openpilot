# WK2 NNLC 자가 학습 — 준비 체크리스트

기준 빌드: `krvista/openpilot` 브랜치 `wk2-fixes-release-mici` (tip **d67c5fe**: v2026.002.002 + 크래시픽스 + 토크시드 + 54km/h 홀드)

## A. 장치 (주행 전 1회)
1. 빌드 업데이트: `cd /data/openpilot && git fetch origin wk2-fixes-release-mici && git reset --hard origin/wk2-fixes-release-mici && sudo reboot` → Settings › Software 에서 Commit **d67c5fe** 확인
2. **NNLC 끄기**: Settings › sunnypilot › **Steering** › "Neural Network Lateral Control" **OFF** (조향 자체는 그대로 동작, 피드포워드만 선형식으로 바뀜)
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
기대 출력: build `d67c5fe` · feedforward `LINEAR (NNLC OFF)` · liveTorque `useParams=True` · liveDelay `estimated` · active below minSteerSpeed > 0% · grip p90 < 40
