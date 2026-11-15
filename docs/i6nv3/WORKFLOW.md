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

## 순정 LFA 수집 주행 (Phase 70)
- 목적: 순정 LFA 가 이 차 MDPS 를 직접 조향할 때의 요청·권한 값·응답을 기록(평소 로그에는 대기 프레임만 있다, 보고서 §91.5).
- 켜기: 시동을 건 뒤 **정차 상태**에서 설정 → 개발자 화면 맨 끝 `stock LFA capture`(quickboot mode 다음). openpilot 이 한 번 재시작하고
  화면에 "Dashcam Mode" 가 뜨면 수집 중이다. 그 주행 동안 sunnypilot 은 조향·가감속을 하지 않고 기록만 한다 — 순정 LFA/HDA2 를 평소처럼 켜서 운전.
- 끄기: 시동을 끄면 자동 해제(다음 시동은 원래의 sunnypilot). 주행 중 되돌리려면 정차 후 같은 토글을 끈다.
- 권장 주행: 10–15 분, 고속도로 직선·완만한 커브와 40–70 km/h 커브 진입 몇 번을 순정 LFA 로.
- 분석: 구간별 `probes/qa/e121_angle_frames_extract.py <rlog> <npz>` → `probes/qa/e135_stock_lfa_analysis.py <npz ...>`.
