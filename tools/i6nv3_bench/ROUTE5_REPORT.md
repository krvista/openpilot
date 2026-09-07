# 라우트 00000005 (2026-09-07 아침 출근, 빌드 f852d9c6 = 17af98b5 제어코드) 오류 전수 분석

38 세그먼트, 38 분. 스캔 항목: selfdriveState 알림, onroadEvents, 거부 에코(0x110 src 192),
controlsState.laneDropout, 조향 폴트, pandaStates, managerState, 로그 메시지(ERROR).

## 발견된 오류 3종과 원인

| # | 증상 | 시각 | 원인 (로그 근거) |
|---|---|---|---|
| 1 | comma4 "TAKE CONTROL IMMEDIATELY" 빨간 경고 + 경고음, 소프트 해제 | seg 19 t=1159.7 s, 14 km/h | hardwared 의 2 Hz deviceState 가 7.2 s 끊김(1155.2→1162.3). 제어 경로(carState/modelV2/controlsState/can)는 한 프레임도 안 빠짐. selfdrived 가 deviceState 를 alive 필수로 봐서 commIssue → commIssueAvgFreq(critical). managerState 1.0 s, procLog 2.0 s 도 같은 시각에 한 틱씩 늦음 = 시스템 전체 순간 정지, hardwared 만 오래 멈춤. 근본 원인은 로그로 특정 불가(CPU 온도 68 °C 정상, mem 62 %, 스토리지 84 %). |
| 2 | 계기판 ADAS 경고 짧게 | seg 18 t=1082.6 s, 91 km/h | ALC 진행 중 운전자가 193 Nm 로 핸들을 35 °/s 로 되돌림. 판다는 ACTIVE 프레임의 각도를 마지막 수락값 대비 프레임당 ~0.2 ° 만 허용하고 위반 시 기준을 핸들각으로 리셋하는데, 우리 핸들각 복사본은 1–2 프레임 늦어 두 프레임에 한 번씩 거부(6 프레임, src 192). 거버너/에코 재정렬로는 못 막는 구조. |
| 3 | 차선 드롭아웃 래치(Phase 39) 남발: "Lane Lines Lost" 74 회 / 175 s | 전 구간 | 39 회는 25 km/h 미만(교차로 회전·무표시 차로: 차선이 카메라를 벗어날 뿐 계획은 옳음), 33 회는 방향지시등·ALC 중(선을 넘으면 확률이 떨어지는 게 정상), 41 회는 3 s 상한 후 6 s 마다 재래치(무표시 도로), 실제 커브에서는 직진으로 감쇠해 op 가 커브를 놓침(seg 8: 명령 −9.1e-3 → −1.2e-3, 도로 −14e-3, 운전자 664 Nm). |
| 부 | 부팅 직후 거부 에코 11 프레임 | seg 0 t=11.65 s | 판다 안전모드 전환 직후 0.1 s 동안 측정각 샘플 창이 비어 passive 프레임도 거부. 무해(주행 전). |
| 부 | 판다 faultTemp(interruptRateCan2), harness flipped | 부팅부터 | 부팅 시 FW 조회로 버스 2 인터럽트율 초과 플래그가 남음. 라우트 4 에는 없음. 동작 영향 없음. |

steerFault 0, 프로세스 크래시 0, managerState 비정상 0.

## 해결책 (이 커밋)

1. **Phase 39-2 래치 재설계 (controlsd)** — 직진 감쇠 대신 드롭아웃 직전 명령을 **홀드**(0.3 s 히스토리),
   진입은 차선확률 0.15 미만이 0.3 s 연속 + 40 km/h 이상일 때만, 방향지시등·차선변경 중과 그 후 1 s 는 금지
   (진행 중 래치도 즉시 해제), 해제 후에는 차선이 0.30 이상으로 1 s 돌아와야 재무장(무표시 도로 = 한 번만).
2. **Phase 38-3 핸들 추월 시 passive 프레임 (carstate/carcontroller/hyundaicanfd/values)** — MDPS 핸들각의
   CAN 샘플 단위 스텝(프레임 병합에 영향 없음)이 판다 허용치+0.2 ° 를 2 프레임 연속 넘으면 passive 프레임
   (비활성 비트, 게인 0, 각도 = 핸들각) 송신, 5 프레임 조용하거나 1 s 가 지나면 핸들각에서 active 재개.
   아이콘·경고·assist 필드는 그대로 active 상태를 유지(계기판 깜빡임·카메라 경고 역류 없음). passive 프레임은
   판다의 6-샘플 핸들각 창으로 판정돼 항상 수락됨. 이때 ACIGain 은 이미 양보 상태.
3. **selfdrived** — deviceState 를 alive/평균주파수 검사에서 제외하되, 15 s 이상 침묵하면(진짜 멈춘 hardwared)
   commIssue 로 승격. 프로세스 사망은 기존 managerState 경로가 잡음. hardwared 가 1 s 이상 멈추면 원인 절반
   (sm.update 대 본체·params 쓰기)을 로그에 남김(다음 발생 시 원인 특정용).

## 오프라인 검증 (수정 후)

| 검증 | 결과 |
|---|---|
| 라우트 5 전 38 세그먼트 폐루프 프리플라이트 | 223,786 프레임 중 거부 0 (실차 로그: 17); 라우트 3/4 도 3/1 |
| seg 18 실제 card 리플레이 (오버라이드 구간 1082.4–1083.0 s) | passive 11 프레임 / active 49 — 거부 대신 passive 로 통과 |
| seg 19 실제 selfdrived 리플레이 | commIssue 없음, 인수 경고 없음 |
| 라우트 5 최악 5 세그먼트(8·13·18·34·35) 실제 controlsd 리플레이 | 래치 0 프레임 (실차 로그 6,956 프레임) |
| 라우트 4 seg 8 (원래 사건) | 524.0 s 에 1 회 3.0 s 홀드 (커브 명령 유지), 차선변경 구간은 억제 |
| phase_tests 250, 안전 스위트, 정적 점검, 수용 관문 7 다리 | 녹색 |
| 독립 검증 에이전트 리뷰 | 1차 FIX FIRST(차단 2·개선 2·메모 3) → 전부 반영 후 2차 SHIP |

## 실차에서 볼 것
- 계기판 ADAS 경고가 강한 오버라이드 중에도 안 뜨는지(거부 에코 0 목표).
- "Lane Lines Lost" 가 고속 직선/완만 커브의 진짜 드롭아웃에서만 드물게 뜨는지.
- hardwared 스톨 로그(`hardwared loop stall`)가 다시 찍히면 그 절반 정보로 원인 추적.
