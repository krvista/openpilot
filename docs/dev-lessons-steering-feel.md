# Steering Feel Development Lessons

openpilot / sunnypilot Hyundai CCNC (Ioniq 6 N) angle-control steering feel
개선 작업 중 발생한 오류와 방지 노하우 기록.

Branch: `ccnc-port-prebuilt`, `claude/steering-feel-masterplan-BIIQD`
Date: 2026-04-21


## 1. capnp _DynamicListReader slicing 금지

### 증상
- controlsd 크래시 (processNotRunning)
- ACC 눌러서 주행 시작하는 순간 발생
- LFA만 사용 시에는 미발생 (조기 fallback으로 문제 라인에 도달하지 않았기 때문)

### 근본 원인
```python
# WRONG - capnp list는 Python slice를 지원하지 않음
pos_x = model_v2.position.x   # capnp _DynamicListReader
x = np.array(pos_x[:n], ...)  # TypeError: an integer is required
```
capnp `_DynamicListReader.__getitem__`은 정수만 허용. `slice` 객체를 넣으면
`TypeError`가 발생한다.

### 수정
```python
# OK - 정수 인덱싱으로 element-wise 접근
x = np.fromiter((pos_x[i] for i in range(n)), dtype=np.float64, count=n)
```
대안:
```python
x = np.array(list(pos_x)[:n], dtype=np.float64)   # list() 변환 후 슬라이스
x = np.array([pos_x[i] for i in range(n)], ...)    # list comprehension
```

### 방지 규칙
- **cereal/capnp 메시지의 List 필드에는 절대 `[:]`, `[:n]`, `[a:b]` 슬라이스를 쓰지 않는다.**
- capnp list를 numpy로 변환할 때는 `np.fromiter` + generator 또는 `list()` 변환 후 사용.
- **Mock 테스트만으로 capnp 코드를 검증하지 않는다.** 반드시 실제 rlog의 capnp 메시지로 재생 테스트.


## 2. cereal custom.capnp 스키마 확장 시 sync 필요 파일

### 배경
`CarStateSP`에 MDPS 진단 필드 4개 추가 시 아래 3개 파일을 동시에 수정해야 함.

### 필수 수정 파일
| 파일 | 역할 | 빠뜨리면 |
|------|------|----------|
| `cereal/custom.capnp` | capnp 스키마 정의 | 직렬화/역직렬화 실패 |
| `opendbc_repo/opendbc/car/structs.py` | Python dataclass (런타임 구조체) | AttributeError |
| 해당 차종 `carstate_ext.py` | CAN 값 실제 populate | 필드가 항상 0 |

### 방지 규칙
- 세 파일을 **하나의 커밋**에 넣는다. 분리하면 중간 상태에서 다른 프로세스가 깨질 수 있음.
- 스키마 추가 후 반드시 round-trip 테스트:
  ```python
  from cereal import log
  ev = log.Event.new_message()
  cs = ev.init('carStateSP')
  cs.newField = value
  data = ev.to_bytes()
  with log.Event.from_bytes(data) as ev2:
      assert ev2.carStateSP.newField == value
  ```
- capnp 필드 추가는 하위 호환성 보장 (이전 rlog 재생 시 기본값 0/false). 필드 삭제/타입 변경은 금물.


## 3. controlsd 크래시의 파급 효과

### 증상 체인
```
controlsd 크래시
  -> processNotRunning 이벤트 발생 (selfdrived 감지)
  -> carControl, controlsState 메시지 중단
  -> commIssue 이벤트 (carOutput, longitudinalPlan 등 invalid)
  -> ADAS 기능 전체 비활성화
  -> 매니저가 controlsd 재시작 시도하지만 동일 조건에서 반복 크래시
  -> 해당 드라이브 동안 ADAS 영구 사용 불가
```

### 방지 규칙
- controlsd `state_control()` 내 신규 코드는 **예외가 절대 전파되면 안 된다.**
- 외부 데이터(model_v2, liveParameters 등)를 가공하는 함수에는 최소한 fallback 값이 보장되어야 함.
- 새 기능 추가 시 `CC.latActive=True + 주행 중` 조합으로 반드시 테스트. 정차 상태만 테스트하면 특정 코드 경로를 놓침.


## 4. Unit test에서 capnp 객체를 정확히 모사하기

### 문제
Phase 7 look-ahead 로직을 Mock Python list로 테스트 -> 통과.
실제 capnp `_DynamicListReader`에서는 slicing 불가 -> 프로덕션 크래시.

### 방지 규칙
capnp 메시지를 다루는 코드를 테스트할 때:
```python
# WRONG - Python list는 슬라이싱을 지원하므로 버그를 놓침
model_v2 = MockModel(x=[0.0, 0.5, ...], y=[...])

# RIGHT - 실제 rlog에서 capnp 메시지를 읽어서 테스트
for msg in LogReader("segment.rlog.zst"):
    if msg.which() == 'modelV2':
        result = my_function(msg.modelV2, v_ego=15.0)
```
또는 최소한 capnp 메시지를 직접 생성:
```python
from cereal import log
ev = log.Event.new_message()
mv = ev.init('modelV2')
pos = mv.init('position')
xs = pos.init('x', 33)
for i in range(33): xs[i] = float(i)
# 이렇게 만든 mv.position.x는 실제 _DynamicListReader
```


## 5. git 브랜치 cherry-pick / 체크아웃 주의사항

### 문제
- feature 브랜치와 ccnc 브랜치가 분기된 상태에서 cherry-pick
- 대용량 drivelog 바이너리가 있는 상태에서 체크아웃 시 `.git/index.lock` 잔류

### 방지 규칙
- `git checkout` 전에 `git status`로 uncommitted 변경 확인. drivelog 등 대용량 파일이 staged 상태면 `git reset HEAD drivelog/`로 unstage.
- `index.lock` 에러 발생 시 다른 git 프로세스가 background에서 돌고 있지 않은지 확인 후 삭제.
- cherry-pick 후 remote에 drivelog 커밋이 추가된 상태면 `git pull --rebase` 후 push.
- 두 브랜치의 수정 파일이 동일한지 반드시 `git diff branch1 branch2 -- <files>` 확인.


## 6. CAN 파서와 DBC 시그널 구독 방식

### 발견
opendbc의 `CANParser`는 메시지 단위로 구독한다. `cp.vl["MDPS"]["STEERING_COL_TORQUE"]`를
한 번이라도 접근하면 MDPS 메시지(0xEA)의 **모든 시그널**이 파싱됨. 따라서 새 시그널을
읽으려면 DBC에 정의만 되어 있으면 별도 구독 추가 없이 `cp.vl["MDPS"]["NEW_SIGNAL"]`로
접근 가능.

### 방지 규칙
- DBC에 시그널이 있는지 먼저 확인: `CANParser(dbc, [('MDPS', 100)], 0).vl['MDPS'].keys()`
- 새 시그널 추가 시 DBC 수정 불필요 (이미 있음). `carstate.py`나 `carstate_ext.py`에서 읽기만 하면 됨.
- 단, `COUNTER`나 `CHECKSUM` 시그널은 CANParser의 `ignore_counter` / `ignore_alive` 설정과 충돌 가능. 시그널 값을 읽는 것과 카운터 검증은 별개.


## 7. controlsd의 CC.latActive 조건 분석

### 발견
```python
CC.latActive = _lat_active
    and not CS.steerFaultTemporary
    and not CS.steerFaultPermanent
    and (not standstill or self.CP.steerAtStandstill)
```
- `standstill`: `abs(CS.vEgo) <= max(self.CP.minSteerSpeed, 0.3) or CS.standstill`
- 정차 중에는 `CC.latActive=False` -> `_lookahead_curvature` 미호출
- 주행 시작 직후 처음으로 True가 됨 -> 새 코드의 첫 실행 지점

### 방지 규칙
- `CC.latActive` 조건부 코드를 추가할 때는 **"정차 -> 출발 전환 시점"을 반드시 테스트 시나리오에 포함**.
- 이 전환 시점에서 model_v2 데이터가 불완전하거나 capnp 기본값일 수 있음 (position.x가 전부 0 등).


## 부록: 이 세션의 커밋 이력

| SHA | 내용 | 결과 |
|-----|------|------|
| `fce7575` | Phase 7 adaptive look-ahead curvature | capnp slicing 크래시 유발 |
| `c202d6d` | MDPS 진단 로깅 (CarStateSP 확장) | 정상 |
| `bd05edf` | capnp slicing 크래시 수정 (np.fromiter) | 문제 해결 |
