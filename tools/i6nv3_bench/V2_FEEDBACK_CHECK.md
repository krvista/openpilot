# v2 피드백·요청 사항의 v3(i6nv3) 구현 확인표

자동 생성(`python3 tools/i6nv3_bench/gen_v2_checklist.py`): 각 항목의 식별자가 트리에 존재하는지와 이를 검증하는 테스트가 있는지를 grep -F로 확인. 컨트롤러 출력 패리티(같은 npz 로그를 v2·v3 트리의 CarController에 넣어 송신각·gain을 프레임별 비교)는 tools/ccnc_analysis/parity_dump.py로 별도 확인한다.

| 항목 | 식별자 | 코드 | 테스트 |
|---|---|---|---|
| 31 유지토크 모델(hold comp: 속도 기준선+lat_acc 포화) | ACIGAIN_HOLD_BASE_SPEEDS_MS, ACIGAIN_HOLD_LAGAIN_V, ACIGAIN_HOLD_LA_BP | 있음 | test_driver_domain 있음 |
| 34a 저속 명령 LP τ 0.30 | LAT_CMD_SMOOTH_TAU_V  = [0.30, 0.12, 0.08] | 있음 | test_noncontrol_controlsd 있음 |
| 34b 정지-직진 크립 게이트 (파지 인식 원시 300 Nm) | ACIGAIN_CRAWL_STILL_COMP_NM, CRAWL_STILL_RATE_EXIT | 있음 | TestPhase34CrawlGate 있음 |
| 34c LDW 깜빡이 쿨다운 5 s | last_blinker_frame) * DT_MDL < 5.0 | 있음 | TestBlinkerCooldownRate 있음 |
| 35a 고속 파지 즉각·깊은 양보 (용산터널 그립 기준) | ACIGAIN_GRIP_FLOOR35_V, ACIGAIN_GRIP_FULL35_V, ACIGAIN_GRIP_RATE_DN_GATE_NM | 있음 | TestPhase35GripAtSpeed 있음 |
| 35b/35c 한남대교 합류 해제 후 회복 가속 (arm 게이트) | ANCHORED_RECOVERY_FRAMES, ANCHORED_RECOVERY_RATE_UP, REANCHOR_ARM_FRAMES | 있음 | TestPhase35AnchoredRecovery 있음 |
| 36 저속 커브 조건부 천장 (연속 램프, 문턱 없음, 비대칭 EMA) | ACIGAIN_CURVE_CEILING_V, ACIGAIN_CURVE_RAMP_DEG, ACIGAIN_CURVE_MEAS_TAU_RISE_S, ACIGAIN_CURVE_MEAS_MAX_RISE_DPS | 있음 | TestPhase36CurveCeiling 있음 |
| 37a 고속 회복 완만화 (상승률 캡·저크 캡) + 와이퍼 강우 모드 | ACIGAIN_RATE_UP_CAP_V, RECOVERY_JERK_CAP_V, RAIN_WIPER_ON_FRAMES, CCNC_WIPER | 있음 | test_phase37a 있음 |
| 37b 사각지대 3계층 (ALC 중단 1.5 s·지시등 양보 차단·BSM 차선 가드 2 s 타임아웃) | LANE_CHANGE_BSM_ABORT_MAX_S = 1.5, BSM_BLINKER_NO_CONCESSION, BSM_LANE_GUARD_MAX_S, lateralLaneChangeActive | 있음 | test_phase37b 있음 |
| 37c 양보 시작점 속도 테이블 (60 km/h+ 50 Nm, 강우 30) | ACIGAIN_GRIP_START_V, ACIGAIN_GRIP_START_RAIN_NM     = 30.0 | 있음 | test_phase37c 있음 |
| 38 송신각 거버너 + 비활성 gain 0 + 실측각(MDPS_2) 비활성 프레임 | TX_GOVERNOR, meas_angle, mdps_angle_2 | 있음 | test_phase38 있음 |
| 38-2 거부 에코 기반 기준 재정렬 | tx_rejected, Bus.loopback | 있음 | TestPhase38EchoResync 있음 |
| 39-2 차선 드롭아웃 래치 (직전 명령 홀드, 0.15/0.3 s 진입, 40 km/h+, 지시등·차선변경 금지, 재무장) | LANE_DROPOUT_LATCH, LANE_DROPOUT_ENTRY_LM, LANE_DROPOUT_REARM_S | 있음 | TestLaneDropoutLatch 있음 |
| 38-3 핸들 추월 시 passive 프레임 (샘플 단위 스텝, 2프레임 진입, 1 s 상한) | WHEEL_OUTRUN_PASSIVE, mdps_angle_2_step_can, wire_active | 있음 | TestPhase38_3WheelOutrunPassive 있음 |
| i6nv3 안전 계층: CCNC 2048, 모델 11, 안전 스위트 1907+159 | HYUNDAI_ANGLE_MODEL_HYUNDAI_IONIQ_6_N, HYUNDAI_PARAM_CCNC | 있음 | test_hyundai_canfd_i6n 있음 |

결과: 모든 항목 존재

실차 대비 관문(acceptance.sh): phase_tests, 안전 스위트(상류+i6n), 정적 점검, 생성 DBC 중복, 프로세스 리플레이(selfdrived·controlsd), 사전 점검 리플레이(preflight_replay). 로컬 드라이브로그가 있으면 뒤의 두 관문이 실제 로그로 돈다.
