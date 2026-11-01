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
