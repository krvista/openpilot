# i6nv3 벤치 검증 가이드 (0단계 산출물)

i6nv3 = i6nv2(34페이즈, 도로 검증 완료)를 sunnypilot `hkg-angle-steering-2025`
(cfb38312, master 2026-08-27 동기) 위로 이식한 리베이스 트랙. 이식 정합성은
교차검증 3라운드(SHIP)로 마감. **실차 사용 전 이 문서의 1a → 1b → 2 → 3 관문을
순서대로 통과해야 한다.**

## 0. 수용 기준 3각 (개발기, 매 변경마다)
```
bash tools/i6nv3_bench/acceptance.sh
```
| 다리 | 대상 | 커버 |
|---|---|---|
| phase_tests 172 | CarController/controlsd 파이썬 거동 | 페이즈 상태기계 |
| 안전 스위트 1907 + i6n 파일 159 | opendbc C 안전층 | 한계/whitelist/rx, **CCNC 강제 각도집행, 모델 id 선택기(EV9 반증)** |
| test_dbc_frames + 중복 SG_ 스캔 | DBC 패킹 | 명령 프레임 바이트(i6nv2 동일 증명 시점 골든) |

세 결함 클래스가 각각 "당시 녹색이던 스위트에 안 보이던" 경험에서 온 구조 —
하나라도 빨간 상태로 장치에 올리지 않는다.

## 0-1. 정적 점검 결론 (장치 첫 부팅 플로우)
- **빌드**: i6nv3에는 `prebuilt` 마커가 없다 → 첫 부팅에서 `system/manager/build.py`가
  scons 전체 빌드(panda 펌웨어 + **모델 tinygrad 컴파일 포함**). 예상 20~40분,
  화면에 빌드 진행 표시. 실패 시 화면에 오류 고정 + `/tmp/launch_log`.
- **서브모듈**: opendbc_repo/panda는 인트리 벤더링(빌드 대상 아님). 나머지
  (msgq/tinygrad/rednose/teleoprtc/neural_network_data)는 업데이터가
  `submodule update --init --recursive`로 초기화하며, SSH 전환 시엔
  `switch_branch.sh`가 동일하게 처리. `launch_chffrplus.sh`의 심링크
  (`opendbc -> opendbc_repo/opendbc` 등)는 벤더링 구조와 정합.
- **panda 펌웨어**: `panda/SConscript`가 `opendbc.INCLUDE_PATH`(벤더링된 opendbc의
  safety)를 포함해 `panda/board/obj/panda_h7.bin.signed`를 생성 → pandad가 내장 panda
  서명과 비교해 다르면 플래시. **차 연결 없이 전원만으로 진행됨.**
- **롤백 메커니즘**: i6nv2는 `prebuilt` 마커(무빌드) + 커밋된 d60eecd 펌웨어
  바이너리(19개 obj 파일 추적). i6nv3의 obj/는 gitignore 대상이라 i6nv2 체크아웃이
  추적 파일로 덮어씀 → pandad가 서명 불일치로 **자동 재플래시**. 스크립트가 obj/를
  선제 정리해 잔재 혼입을 막는다.
- **안전 파라미터 기대치** (fingerprint 후 interface.py 산출):
  `EV_GAS(1) | CANFD_LKA_STEER_MSG(16) | CANFD_ALT_BUTTONS(32) | CANFD_LKA_STEER_MSG_ALT(128) | CCNC(2048)` = 2225,
  SP 파라미터 = 모델 id 11 << 4 = 176. C 헤더와 파이썬 enum 비트 일치는 검증 완료.
- **모델 id 경로**: 소프트웨어로 반증 완료(`test_hyundai_canfd_i6n.py`):
  같은 CCNC 파라미터에서 모델 id를 EV9로 바꾸면 집행 포락선이 EV9 물리로 바뀜 →
  encode→decode→테이블 선택기가 살아있음. 따라서 차량에서 임시 구별값 실험은
  **선택 사항**(장치 배선까지 포함해 보고 싶을 때만).

## 1a. 책상 벤치 (차 없음 — 위험 0)
전제: C4에 하네스 커넥터로 12V 공급(comma 파워 어댑터), WiFi.
1. SSH: `bash tools/i6nv3_bench/switch_branch.sh i6nv3` → `sudo reboot`
2. 빌드 완주 대기(화면). 실패 시: `/tmp/launch_log` 저장 → 롤백(아래) → 로그 분석.
3. `python3 tools/i6nv3_bench/panda_check.py` → 합격:
   - `PASS firmware signature matches` (pandad가 새 빌드를 플래시함)
   - `PASS no panda fault`, harness 상태 정상, heartbeat_lost False
   - safety_mode는 이 단계에서 noOutput/elm327 — 정상(차 미연결)
4. **롤백 리허설(필수)**: `switch_branch.sh i6nv2` → reboot → `panda_check.py`가
   i6nv2 서명 일치를 보고하는지. 왕복이 되면 1b로.
5. 다시 `switch_branch.sh i6nv3` → reboot → 1b 준비 완료 상태로 둔다.

## 1b. 차량 벤치 (정지 상태 — 여기부터 차 필요)
전제: 주차, Ready, 손은 핸들에, 주변 안전. **주행 금지.**
1. 시동 → onroad 전환 → 핑거프린트 확인(설정 화면 차종 = Ioniq 6 N).
2. `python3 tools/i6nv3_bench/panda_check.py --onroad 30` → 합격:
   - `PASS CarParams safetyParam has CCNC(2048)` + expected set
   - `PASS CarParamsSP angle model id = 11`
   - `PASS safety model is hyundaiCanfd`, `PASS panda safetyParam == CarParams`
   - `PASS rx checks valid` — CCNC 완화 rx 스펙이 실제 게이트웨이 트래픽에서 성립
   - `PASS no new rx-invalid`
3. MADS(LFA 버튼) 활성 → `controlsAllowedLateral=True` 확인, txBlocked 증가 없음.
4. 핸들을 가볍게 돌려 EPS 파지 → op 양보(ACIGain 하강) 확인, txBlocked 폭증 없음.
5. 30초 이상 활성 유지 후 이상 없으면 2단계로. **어느 항목이든 FAIL → 즉시 롤백.**

중단 조건: panda fault, safetyRxChecksInvalid True 지속, txBlocked 지속 증가,
경고음/에러 알림, 예상 밖 조향 토크.

## 2. 모델 고정 + 주차장 스모크
- 설정 → Model Selector에서 **i6nv2와 같은 계열의 모델 고정**(베이스 이식과 모델
  변경을 분리). 같은 계열이 없으면 기록하고 32/34a 재측정 계획을 앞당긴다.
- 주차장 저속(선행차 없으면 Phase 18로 passive가 정상 — 정체 추종 시 활성) →
  faults/알림 없음, 로그 생성 확인.

## 3. 첫 도로 주행 → 데이터 게이트
낮·한산한 직선로·짧게·손 가볍게. 로그 업로드 후 상시 검출기 + i6nv2 기준선
(셰이크 0.277, 커브 w/p 0.56, 릴리즈 p50 30°/s, pressed 8.7/min, faults 0)
대비 동등성 확인 → 통과 후에만 통상 주행 편입. 모델 업그레이드는 그 뒤 별도 단계.

## 롤백 (언제든)
```
bash tools/i6nv3_bench/switch_branch.sh i6nv2 && sudo reboot
```
부팅 후 `panda_check.py`로 i6nv2 서명 일치 확인. (1a-4에서 리허설된 경로)

## 대기 항목
- i6n 알림 스키마 추가(벤더링으로 가능; 알림 카운터→드라이버 연결)
- 모델 11 실 CarSpecs(wb 2.965/sr 14.96) 전환 — 벤치 통과 후 의식적 결정
- i6nv2에도 존재하는 LDW 쿨다운 25초 결함(보수 방향) 반영 여부
