# i6n 업스트림 SYNC → 수정사항 재적용 플레이북

목적: 모델/OS 업그레이드 테스트를 위해 결국 sunnypilot 업스트림과 sync해야 할 때,
i6n의 모든 수정사항을 **유실 없이 + 안전하게 + 점진 검증으로** 다시 얹기 위한 절차.
(작성 2026-06-30, i6n HEAD `faa679e` 기준. 업스트림 좌표는 POST_6F2_AUDIT §1.T 참조.)

---

## 0. 하드 제약 — 먼저 읽을 것

**모델 bump = 튜닝 검증 전면 무효화.** 업스트림 sync는 driving model(modeld/tinygrad)과
agnos OS를 통째로 올린다. i6n의 모든 on-road 튜닝(Phase 6~10, ZB-1)은 **특정 모델 출력의
거동에 맞춰 데이터-검증**된 것이라, 모델이 바뀌면 W-게이트를 **전부 재검증**해야 한다.
→ sync는 "튜닝 브랜치에 섞는 작업"이 아니라 **별도 작업 + 전면 재검증 사이클**로 다룬다.

**sync는 가볍게 하지 않는다.** §1.T 결론: fork 이후 업스트림 본체 변경의 ~90%는
comma/openpilot 인프라(modeld 재작성/tinygrad/agnos/빌드)이고 차용할 핵심은 없다.
sync의 유일한 정당화 = **새 모델/OS로 업그레이드 테스트를 하고 싶을 때.**

---

## 1. 분기 구조의 함정 (왜 `git rebase`가 안 되나)

- 업스트림이 `hkg-angle-steering-2025`(-prebuilt 포함)를 **force-push/rebuild** → fork base
  `fffb5ab`와 **공통 조상 끊김**(`No common ancestor`). fork 시점 opendbc 커밋(`966c60b`)은
  업스트림에서 **GC됨**(raw 404). → 깨끗한 `git rebase --onto upstream` 불가.
- fork 시점 `opendbc_repo`는 **서브모듈**이었으나 i6n에서 **vendored tree**로 전환됨. 새 업스트림은
  다시 서브모듈일 수 있다 → opendbc 재적용은 서브모듈/vendor 구조부터 결정해야 함.
- i6n 거동-소스 커밋 = **217개**(반복·revert 포함). 217개를 그대로 cherry-pick하면
  **죽은 시도(예: 7a-3→7a-5 revert, lateral_mismatch→revert)까지 재생**된다. → cherry-pick 금지.

---

## 2. 권장 메커니즘 — "최종상태 파일 포팅 + kill-switch OFF 베이스라인 + 점진 활성화"

1. **새 브랜치를 새 업스트림 HEAD에서** 생성(`hkg-angle-steering-2025` 최신, §1.T 좌표 갱신).
2. **§3 manifest의 각 소유 파일을 "최종상태"로 3-way 포팅**(217 커밋 재생 아님). 각 파일의 현재
   i6n 버전을 새 업스트림 같은 파일 위에 머지 — 업스트림이 그 파일을 옮겼으면 수동 정합.
3. **모든 kill-switch를 OFF로 먼저 착지**(§4 표). 이 상태면 거동이 새 업스트림과 ~bit-identical
   → **깨끗한 베이스라인**에서 모델/OS만 바뀐 효과를 먼저 확인.
4. **kill-switch를 Phase 단위로 하나씩 ON** → 각 단계마다 §5 게이트 재측정 → 회귀 없으면 다음.
   (i6n이 원래 거쳐온 순서를 역추적: roll-LP → 곡률 LP/lookahead → ACIGain/Phase9 → FB trim.)
5. 각 단계는 POST_6F2_AUDIT의 해당 §가 "왜/검증수치"를 이미 보유 → 재검증 비교 기준으로 사용.

---

## 3. PORT MANIFEST — 재적용 대상 (유실 금지 체크리스트)

### 🔴 안전-critical (panda 수용테스트 필수 + 업스트림 충돌 점검)
| 파일 | 내용 | 주의 |
|---|---|---|
| `opendbc_repo/opendbc/safety/modes/hyundai_canfd.h` | **CCNC angle-steering 안전경로 전체** — `hyundai_ccnc` 플래그, `HYUNDAI_PARAM_CCNC=1024`, `HYUNDAI_CANFD_ANGLE_STEERING_LIMITS`, gain_violation 체크, CCNC_0x161/0x162 TX, 2-layer angle validation, check_relay/counter fix | 업스트림이 자체 angle-steering 안전(Sorento HDA2 등)을 추가했을 수 있어 **충돌 가능** → blind 재적용 금지, 3-way 머지 + `safety_replay`/panda 테스트 통과 확인 |

### 차 제어 (opendbc/car/hyundai)
| 파일 | 내용 |
|---|---|
| `carcontroller.py` | `compute_torque_reduction_gain`(ACIGain), `sp_smooth_angle`, angle 경로, Phase 9 yield-by-authority, override/anchor. **업스트림과 같은 함수의 진화형** → 업스트림 최종본(shelf/속도-bp)과 충돌, i6n 버전 유지(§1.T) |
| `values.py` | `CarControllerParams` — 모든 named 상수/kill-switch(§4) |
| `hyundaicanfd.py` | ADAS 각도 요청 TX (272/bit82 등) |
| `interface.py`, `carstate.py` | i6n 핑거프린트/플래그/CCNC carstate |

### openpilot 제어
| 파일 | 내용 |
|---|---|
| `selfdrive/controls/controlsd.py` | lookahead curvature lead(7c-2), 곡률 LP(`LAT_CMD_SMOOTH_TAU`), conf floor taper(6g-2a) |
| `selfdrive/controls/lib/latcontrol_angle.py` | roll LP(6h-6), roll gain(6h-5), FB 곡률 trim(7a), entry boost(7b). **ZB-1 roll DC 차감 미구현 — sync 후 여기에 추가 예정** |

### 기타 거동
| 파일 | 내용 | 주의 |
|---|---|---|
| `selfdrive/selfdrived/selfdrived.py` | op-active LDW surfacing(`IsLdwEnabled` 게이트, 6g-4) | |
| `sunnypilot/mads/mads.py` | always-active steering + boot race fix(allow_always CANFD Hyundai). **lateral mismatch guard는 추가했다 부팅크래시로 revert** | **업스트림 #1801**(MADS safety: heartbeat+lateral mismatch, panda+cereal로 구현)이 네가 시도하다 만 그 기능 → sync 후 #1801로 대체 검토(§6) |
| `selfdrive/ui/{mici,sunnypilot}/layouts/settings/developer.py` | Stock LFA Passthrough 토글 | **add+revert로 net-zero** → 재적용 불필요(잔여 없는지만 확인) |

### 비-거동 (그대로 이관, 검증 불필요)
- `tools/POST_6F2_AUDIT.md`(§1.A~§1.T 기관기억), `tools/IONIQ6N_STEERING_MASTERPLAN.md`,
  `docs/i6n_corner_entry_plan.md`, `tools/ioniq6n_*.py`(분석 스크립트). **이 플레이북도 이관.**

### ❌ 재적용 금지 (재생성/데이터)
- `panda/board/obj/*` (재빌드 펌웨어 바이너리 — 소스 아님, 새 빌드가 생성)
- `drivelog/*.zst`, 루트 `*--*.zst` (커밋된 주행로그 — 코드 아님)

---

## 4. KILL-SWITCH 레지스트리 (OFF = 업스트림-동등; 착지 후 점진 ON)

| 스위치 | 위치 | OFF값(업스트림 동등) | 현재 |
|---|---|---|---|
| `ROLL_LP_TAU` | latcontrol_angle.py | `0.0` (raw roll) | 0.6 |
| `LAT_FB_KI` | latcontrol_angle.py | `0.0` (FB trim 없음, bit-identical) | 0.8 |
| `LAT_FB_ENTRY_BOOST` | latcontrol_angle.py | `1.0` | 2.5 |
| `LOOKAHEAD_T_AHEAD_CAP` | controlsd.py | `0.25` (pre-7c-2) | 0.27 |
| `LOOKAHEAD_JERK_BUDGET` | controlsd.py | (lookahead lead 자체 비활성은 6h-2 경로) | 0.7 |
| `LAT_CONF_FLOOR` | controlsd.py | `0.0` (floor taper off) | 0.5 |
| `LAT_CMD_SMOOTH_TAU_V` | controlsd.py | `[0,0,0]` (곡률 LP off) | [0.20,0.12,0.08] |
| `SMOOTHING_ANGLE_RELEASE_HI_DEG` | values.py | `1e6` (이미 release off, 6h-1) | 1e6 |
| `SMOOTHING_ANGLE_DEADBAND_DEG` | values.py | `0.0` | 0.1 |
| ACIGAIN grip-band (Phase 9) | carcontroller 호출부 | legacy band(350/0.19) 전달 | 260/0.10 (grip 시) |
| `ROLL_DC_TAU` (ZB-1, 미구현) | latcontrol_angle.py | `0.0` (현 동작) | — sync 후 추가 |

> 원리: 모든 거동 변경은 named kill-switch + 소스 주석 + POST_6F2_AUDIT의 "Kill switch:" 라인을
> 가진다. 새 업스트림 위에 **전부 OFF로 착지 → bit-identical 베이스라인 확인 → Phase별 ON+게이트**.

---

## 5. 재검증 게이트 (모델 bump 시 전부 재실행)

POST_6F2_AUDIT W-게이트 + ZB-1. 모델이 바뀌면 **이전 수치는 무효**, 새 모델 베이스라인으로 재측정:
1. **W1** 35 km/h 휠 2-8 Hz 떨림 (Phase 9 yield 효과 무회귀)
2. **W2** 코너 진입 under-response / over-correction (achieved/desired, inside-cut 0)
3. **센터링** signed offset (ZB-1: roll DC bias / 우측 +0.12 m 추적)
4. **grip yield** heavy-override floor, pressed-flip rate
5. **S자(서울역)** inside-cut 무발생 (7a-5 revert 보존 + lookahead 과조향 0)
6. **LDW** `IsLdwEnabled` 발화 정상
7. **panda safety** Ioniq6N angle 수용(가장 먼저, 코드 착지 직후)

---

## 6. 알려진 충돌/교차 리스크

- **carcontroller ACIGain**: 업스트림이 `TorqueReductionGainController` 제거 후 함수형으로 재작성 +
  mid-torque shelf/속도-스케일 bp 유지. i6n은 shelf를 A/B로 기각(Phase 10)·error/blinker/Phase9
  superset. → **i6n 최종본 유지**, 업스트림 버전으로 덮어쓰지 말 것(§1.T).
- **panda hyundai_canfd.h**: 업스트림 자체 angle-steering 안전(Sorento HDA2 등) 추가 가능 → CCNC
  경로 충돌 점검 필수.
- **sp_smooth_angle**: 업스트림 EPS-whine 튜닝(저속 α0.05)은 i6n이 6g-1에서 기각한 방향 → i6n
  최종본(floor 0.30 + 곡률-LP 이전) 유지(§1.T).
- **MADS #1801**(heartbeat + lateral controls mismatch, panda+cereal): i6n이 시도하다 부팅크래시로
  revert한 lateral_mismatch_counter의 **제대로 된 업스트림 구현**. sync 시 자연 포함되면 i6n의
  always-active/boot-race fix와 상호작용 재확인 후 채택.

---

## 7. 한 줄 요약
sync는 모델/OS 업그레이드용이며 **튜닝 전면 재검증을 동반**한다. 217 커밋 재생 대신 **§3 manifest의
최종상태 파일을 새 업스트림에 3-way 포팅 → §4 kill-switch 전부 OFF로 bit-identical 베이스라인 →
Phase별 ON + §5 게이트**. 안전경로(hyundai_canfd.h)와 ACIGain/sp_smooth는 업스트림 자체 변경과
충돌하므로 i6n 최종본 우선.
