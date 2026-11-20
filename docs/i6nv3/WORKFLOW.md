# i6nv3 작업 규칙

## 변경 기록
- 동작이 바뀌는 코드(조향·제어·부팅·UI 등)를 바꿀 때마다 `CHANGELOG.md` 를 **같은 커밋에서** 갱신한다.
  - 위치: 맨 위 `i6nv3 ...` 블록의 Phase 목록, 최신 항목이 위.
  - 형식: `* Phase N (MM-DD, 근거 라우트): 무엇을 왜 바꿨는지 — 근거 수치(전/후) (Kill: 되돌리는 스위치)`.
  - 분석만 하고 코드가 바뀌지 않은 작업은 CHANGELOG 대신 `tools/i6nv3_bench/ROUTE8_REPORT.md` 에 새 절(§)로 남긴다.
- 분석·검증 결과는 `ROUTE8_REPORT.md` 에, 쓴 프로브는 `tools/i6nv3_bench/probes/qa/` 에 커밋한다.

## 코드 변경
- 조향 동작을 바꾸는 Phase 는 kill 스위치(값 하나로 이전 동작)를 두고 `phase_tests/test_phaseN.py` 를 추가한다.
- `phase_tests` 전체가 통과해야 커밋한다.

## 커밋
- 작성자 krvista <krvista@gmail.com>. 커밋 메시지는 변경 내용과 근거만 적고 다른 서명 줄은 넣지 않는다.

## 푸시 후 보고
- i6nv3 에 푸시한 뒤에는 푸시된 커밋의 짧은 해시(빌드 번호, 예: `9aae6071`)를 결과 보고에 항상 함께 적는다 — 다음 주행 로그가 어느 빌드인지 대조하는 기준.

## 순정 LFA 수집 주행 (Phase 70 / 70b)
- 목적: 순정 LFA 가 이 차 MDPS 를 직접 조향할 때의 요청·권한 값·응답을 기록(평소 로그에는 대기 프레임만 있다, 보고서 §91.5).
- 켜기(지금): 시동을 건 뒤 **정차 상태·미인게이지**에서 설정 → 개발자 화면 끝의 `stock LFA capture`(quickboot mode 다음). openpilot 이 한 번
  재시작하고(수 초) 화면에 "Dashcam Mode" 가 뜨면 수집 중이다. 그 주행 동안 sunnypilot 은 조향·가감속을 하지 않고 기록만 한다 — 순정 LFA/HDA2 를
  평소처럼 켜서 운전. (b62f36a0 에서는 켤 때마다 계기판 ADAS 경고가 떴다 — 릴레이가 가로챈 채 무송신, 보고서 §94.1. 70b 에서 수정.)
- 켜기(예비 — 지금 전환에서 계기판 ADAS 경고가 뜰 때): `stock LFA capture` 를 다시 끄면 경고는 1–2 s 안에 풀린다. `stock LFA next start` 를
  켠 다음(주행 중 아무 때나 — 그 주행은 달라지지 않는다) 시동을 껐다 켠다. 그 시동은 **처음부터** 수집 주행이고 재시작도 릴레이 전환도 없다. 예약은 1 회용.
- 끄기: 시동을 끄면 자동 해제(다음 시동은 원래의 sunnypilot). 주행 중 되돌리려면 정차 후 `stock LFA capture` 를 끈다.
- 권장 주행: 10–15 분, 고속도로 직선·완만한 커브와 40–70 km/h 커브 진입 몇 번을 순정 LFA 로.
- 분석: 구간별 `probes/qa/e121_angle_frames_extract.py <rlog> <npz>` → `probes/qa/e135_stock_lfa_analysis.py <npz ...>`.
  수집이 제대로 됐는지는 `probes/qa/e139_capture_session_scan.py <rlog ...>` — 안전 모드가 elm327 로 유지되고 "reaches the car from the camera"
  가 0 이 아니며 camera faults 가 없어야 한다. 안전 모드 전환 순간은 `e138_handover_gap.py`, 통과 모드의 버스 상태는 `e137_can_state_by_mode.py`.
- 업로드: 이 차 로그는 `bash drivelog.sh upload --profile ccnc`(`ccnc-drivelog`). `wk2` 프로파일은 다른 차다.
