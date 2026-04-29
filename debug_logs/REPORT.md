# sunny-release-tizi drivelog 분석 결과

총 라우트 5개, 사건 431건


## 라우트별 요약

| route | 세그 | 시간(분) | fingerprint | 사건 종류 | 심각도 |
|---|---|---|---|---|---|
| `0fb02cc3a5abcc2f_00000006` | 34 | 561.6 | JEEP_GRAND_CHEROKEE_2019 | error_log=55, process_crash=1, disengage=11 | error=50, info=2, warning=14, critical=1 |
| `0fb02cc3a5abcc2f_00000007` | 37 | 703.3 | JEEP_GRAND_CHEROKEE_2019 | error_log=64, process_crash=2, disengage=58 | error=60, info=2, warning=61, critical=1 |
| `0fb02cc3a5abcc2f_00000008` | 19 | 171.3 | JEEP_GRAND_CHEROKEE_2019 | error_log=57, process_crash=1, disengage=11 | error=54, info=2, warning=12, critical=1 |
| `0fb02cc3a5abcc2f_0000000a` | 49 | 1176.8 | JEEP_GRAND_CHEROKEE_2019 | error_log=56, process_crash=1, disengage=17 | error=52, info=2, warning=19, critical=1 |
| `0fb02cc3a5abcc2f_0000000b` | 39 | 780.9 | JEEP_GRAND_CHEROKEE_2019 | error_log=65, process_crash=1, disengage=31 | error=47, info=2, warning=47, critical=1 |

## 전체 패턴 (5개 라우트 합산)

### 사건 종류별

- **error_log**: 297건
- **disengage**: 128건
- **process_crash**: 6건

### 데몬별 (Top 15)

- `controlsd`: 113건
- `qcomgpsd`: 60건
- `micd`: 57건
- `soundd`: 50건
- `card`: 50건
- `selfdrive.modeld.modeld_tinygrad`: 28건
- `selfdrived`: 27건
- `locationd_llk`: 24건
- `athenad`: 8건
- `sunnylinkd`: 6건
- `backup_manager`: 5건

## Critical 사건 (5건)


### 0fb02cc3a5abcc2f_00000006

- seg 0 (~0.1 min) **disengage** `controlsd` — controlsMismatch (immediateDisable)

### 0fb02cc3a5abcc2f_00000007

- seg 0 (~0.4 min) **process_crash** `micd` — exitCode=1 shouldBeRunning=True

### 0fb02cc3a5abcc2f_00000008

- seg 0 (~0.1 min) **disengage** `controlsd` — controlsMismatch (immediateDisable)

### 0fb02cc3a5abcc2f_0000000a

- seg 0 (~0.1 min) **disengage** `controlsd` — controlsMismatch (immediateDisable)

### 0fb02cc3a5abcc2f_0000000b

- seg 0 (~0.1 min) **disengage** `controlsd` — controlsMismatch (immediateDisable)

## 반복되는 에러 메시지 Top 30

| 횟수 | daemon | message |
|---|---|---|
| 52 | `micd` | get_stream failed, trying again |
| 50 | `soundd` | get_stream failed, trying again |
| 50 | `qcomgpsd` | inject_assistance failed, trying again |
| 20 | `locationd_llk` | Locationd vs ubloxLocation position difference too large, kalman reset |
| 10 | `card` | got vin with request=b'\t\x02' |
| 10 | `card` | iso-tp query response pending: (1860, None) |
| 10 | `card` | iso-tp query bad response: (2016, None) - 0x7f2212 |
| 10 | `card` | iso-tp query bad response: (2017, None) - 0x7f2231 |
| 10 | `selfdrive.modeld.modeld_tinygrad` | skipping model eval. Dropped 3 frames |
| 10 | `qcomgpsd` | inject_assistance failed after retry |
| 8 | `athenad` | athenad.main.exception |
| 8 | `selfdrive.modeld.modeld_tinygrad` | skipping model eval. Dropped 1 frames |
| 6 | `sunnylinkd` | sunnylinkd.main.OSError.EPERM (1) |
| 5 | `card` | {"event": "fingerprinted", "car_fingerprint": "JEEP_GRAND_CHEROKEE_2019", "source": 1, "fuzzy": false, "cached": false,  |
| 5 | `card` | {'event': 'fingerprinted', 'car_fingerprint': 'JEEP_GRAND_CHEROKEE_2019', 'source': 1, 'fuzzy': False, 'cached': False,  |
| 4 | `locationd_llk` | Locationd vs ubloxLocation orientation difference too large, kalman reset |
| 3 | `` | athenad.ws_recv.exception |
| 2 | `selfdrive.modeld.modeld_tinygrad` | skipping model eval. Dropped 95 frames |
| 2 | `selfdrive.modeld.modeld_tinygrad` | skipping model eval. Dropped 89 frames |
| 2 | `micd` | crash |
| 2 | `micd` | logged crash to ['/data/community/crashes/2025-07-02--14-05-19.log', '/data/community/crashes/error.log'] |
| 2 | `selfdrive.modeld.modeld_tinygrad` | skipping model eval. Dropped 83 frames |
| 2 | `selfdrive.modeld.modeld_tinygrad` | skipping model eval. Dropped 96 frames |
| 2 | `selfdrive.modeld.modeld_tinygrad` | skipping model eval. Dropped 117 frames |
| 1 | `selfdrived` | {"event": "selfdrived.initialized", "dt": 4.21, "timeout": false, "canValid": true, "invalid": ["alertDebug", "modelData |
| 1 | `selfdrived` | {'event': 'selfdrived.initialized', 'dt': 4.21, 'timeout': False, 'canValid': True, 'invalid': ['alertDebug', 'modelData |
| 1 | `selfdrived` | {"event": "selfdrived.initialized", "dt": 4.43, "timeout": false, "canValid": true, "invalid": ["alertDebug", "modelData |
| 1 | `selfdrived` | {'event': 'selfdrived.initialized', 'dt': 4.43, 'timeout': False, 'canValid': True, 'invalid': ['alertDebug', 'modelData |
| 1 | `selfdrived` | {"event": "process_not_running", "not_running": "{'micd'}", "error": true} |
| 1 | `selfdrived` | {'event': 'process_not_running', 'not_running': "{'micd'}", 'error': True} |

## Disengage 이벤트 분포

- `preEnableStandstill`: 54건
- `processNotRunning`: 37건
- `wrongGear`: 5건
- `reverseGear`: 5건
- `controlsMismatch`: 4건
- `seatbeltNotLatched`: 4건
- `doorOpen`: 4건

## 프로세스 종료/크래시 (6건)

- `backup_manager`: 5건  (exitCode 분포: [(0, 5)])
- `micd`: 1건  (exitCode 분포: [(1, 1)])
