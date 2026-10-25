# PLAN — FMA-001 주차 제어권 반납 · FMA-003 stop_line_distance 유효기간

> 작성 2026-10-25. 출처: `PLAN-REVIEW-LOG.md` Round 1 (Codex) FMA-001, FMA-003.
> 원칙 유지: **완주 로직(pure_pursuit·mux) 수정 금지.** 고치는 파일은 parking_node 와
> longitudinal_controller 두 개 + 테스트.

## 1. 목표

### FMA-001 — 주차가 끝나거나 취소되면 `/teleop/cmd_vel` 을 놓는다
지금: `start_cb` 가 `False` 를 버린다(`parking_node.py:201`). DONE/ABORT/FINISHED 에서
`tick` 이 0 명령을 **영원히** 발행한다(`:397-400`). mux 는 신선한 teleop 을 자율보다
우선한다(`vehicle_cmd_mux_node.py:310`). → 시퀀서가 주차를 disarm 해도 차가 그 자리에
선다 = 1분 정지 탈락.

### FMA-003 — 오래된 `/stop_line_distance` 는 버린다
지금: `stop_cb` 가 시각 없이 값만 저장한다(`longitudinal_controller_node.py:180`).
정지 노드가 HOLD(0) 중에 죽으면 목표속도 0 이 **영원히** 유지되고, mux 입력 타임아웃은
`/target_speed` 가 계속 오므로 못 잡는다.

## 2. 수용 기준 (관찰 가능)

FMA-001 (`parking_node`)
- A1. `/parking/start` 가 **True→False 하강엣지**이고 주차가 제어권을 가진 상태
  (DRIVE, CUSP, EXIT, DONE, ABORT, FINISHED 중 follower 가 있는 상태)면 `RELEASING` 으로 간다.
- A2. `RELEASING` 동안 0 명령을 발행한다. `release_hold_s`(기본 1.0s) 이상 지났고
  `|linear_speed| < stopped_speed` 이면, 또는 `release_max_s`(기본 3.0s) 가 지나면
  `RELEASED` 로 가고 **그 뒤로 `/teleop/cmd_vel`·`/parking/done` 을 하나도 발행하지 않는다**
  (`/parking/state` 만 발행).
- A3. follower 를 한 번도 만들지 않은 상태(WAIT, 계획 실패 ABORT)에서 하강엣지가 오면
  0 명령을 내지 않고 바로 `RELEASED`(계획 실패) 또는 무시(WAIT) 한다 — 자율 주행을 잠깐이라도
  덮어쓰지 않는다.
- A4. **시작 실패는 전부** follower 없는 `ABORT` 로 간다 — 계획 실패(`plan is None`)뿐 아니라
  자리 pose 파일 없음·측위 미수신(`begin()` 의 앞쪽 두 return, 지금은 WAIT 에 남고 `_started`
  래치 때문에 재시도도 안 됨)도. follower 없는 ABORT 에서는 `/parking/done=False` 를 발행하고
  teleop 은 발행하지 않는다 — 지금은 아무것도 안 내서 시퀀서가 50s 타임아웃까지 다른 미션을
  못 켠다. 재시도는 기존처럼 `/parking/reset` 으로만.
- A5. **False 수준이 계속 오는 것**(시퀀서가 arm 전부터 10Hz 로 보냄)만으로는 아무 일도 안 일어난다.
  하강엣지 = 직전에 `/parking/start` True 를 받은 적이 있음.
- A6. `/goal_reached` 는 **시작 트리거로만** 쓴다(True 만 의미). 하강엣지 판단에 섞이지 않는다.
- A7. 하강엣지가 없으면(손으로 `ros2 topic pub --once /parking/start true`, 또는
  `/goal_reached` 트리거) **`/teleop/cmd_vel` 발행 순서가 기존과 완전히 같다** — 완주 후 0 을
  계속 냄(데드맨). `/parking/done` 도 같다. 단 A4 의 시작 실패(pose 파일 없음·측위 미수신)에서만
  기존(아무것도 안 냄)과 달리 done=False 를 낸다.
- A9. 종료(`main()` finally)의 0 명령은 **주차가 제어권을 가질 때만** 낸다(follower 있음 &&
  상태가 RELEASED 아님). WAIT·RELEASED·follower 없는 ABORT 에서는 내지 않는다
  — 내면 mux 가 0.5s 동안 자율을 덮어쓴다. 판단은 메서드 `_owns_control()` 로 뽑아 테스트한다.
- A10. 해제는 멱등이다 — `RELEASING`/`RELEASED` 에서 하강엣지가 다시 와도 상태·해제 시각이
  바뀌지 않는다.
- A8. `RELEASING` 중 `/parking/reset` 은 무시한다. `RELEASED` 에서 reset 은 기존처럼 WAIT 로 돌린다.
  `RELEASED` 뒤 True 가 다시 와도 `_started` 래치 때문에 재시작하지 않는다(기존 규칙).

FMA-003 (`longitudinal_controller`)
- B1. 새 파라미터 `stop_line_timeout`(기본 1.0s). `stop_cb` 가 수신 시각을 **`time.monotonic()`**
  으로 기록하고 만료 판정도 monotonic 으로 한다(벽시계가 NTP/GPS 로 뒤로 가도 만료가 늦어지지
  않게). 기존 곡률·경사 필드의 `time.time()` 은 건드리지 않는다.
- B2. `control_loop` 에서 마지막 수신 후 `stop_line_timeout` 초과 && 값이 999 가 아니면
  `stop_line_dist = 999.0` 으로 되돌리고 경고를 **한 번** 낸다(만료마다 1회).
- B3. `stop_line_timeout <= 0` 이면 기존 동작과 동일(만료 없음).
- B4. 신선한 입력에 대해서는 `decide_target()` 출력이 기존과 비트 단위로 같다
  (`decide_target` 자체는 고치지 않는다).
- B5. 정상 disarm(999 한 번 쏘고 발행 중단) 뒤 만료되어도 경고를 내지 않는다(값이 이미 999).

## 3. 접근

### parking_node.py
- `/parking/start` → `arm_cb(msg)`: 수준 `self._arm_level` 을 저장하고
  `True` 면 기존 `start_cb` 로직(`_trigger()`), `prev True → now False` 면 `release('disarm')`.
- `/goal_reached` → `goal_cb(msg)`: True 면 `_trigger()`. False 는 무시.
- `begin()` 의 자리없음·측위없음 return 직전에 `state='ABORT'`(follower None) — A4.
- `_owns_control()`: `follower is not None and state not in ('WAIT','RELEASED')`.
  `main()` finally 는 이게 True 일 때만 0 을 낸다 — A9.
- `release(reason)`: **멱등** — 상태가 `RELEASING`·`RELEASED` 면 상태와 `_release_since` 를
  바꾸지 않고 바로 return(RELEASED 뒤 True→False 가 다시 와도 0 명령 재개 없음, RELEASING 중
  반복 엣지가 3s 상한을 늘리지 못함). 그다음 follower 가 없으면 A3 처리. 있으면 `state='RELEASING'`,
  `_release_since=now`, 경고 로그.
- `tick` 맨 앞(상태 발행 다음)에 `RELEASING`/`RELEASED` 처리. 그다음 follower None 이고
  ABORT 면 done False 발행(A4). 나머지 기존 그대로.
- 파라미터 `release_hold_s`(1.0), `release_max_s`(3.0) 추가. 모듈 docstring 의 데드맨 문단에
  "시퀀서 disarm 시 놓는다" 를 덧붙인다.

### longitudinal_controller_node.py
- 파라미터 `stop_line_timeout`(1.0), 상태 `_stop_t=None`, `_stop_expired_warned`.
- `stop_cb` 가 `_stop_t=time.monotonic()` 기록. `_expire_stop_line(now)` 메서드를
  `control_loop` 의 `decide_target()` 호출 직전에 부른다. `decide_target` 은 안 건드린다.

### 왜 이 방식인가 / 트레이드오프
- **하강엣지 + 소유 확인**: 시퀀서는 arm 전부터 False 를 10Hz 로 보낸다. 수준으로 반응하면
  `/goal_reached` 트리거 경로를 시퀀서가 즉시 꺼버린다(지금 launch 는 둘을 같이 안 켜지만
  파라미터 하나로 바뀐다).
- **완료만으로는 놓지 않는다**(disarm 이 와야 놓는다): 시퀀서 없이 손 트리거로 쓸 때
  "주차 끝 → 0.5s 뒤 자율 재출발" 을 막는 기존 데드맨 설계를 유지한다. 시퀀서 경로는
  `done_cb` 가 done(True/False) 수신 즉시 disarm 하므로 완료·실패 모두 반납된다.
- **기동 중 취소 → 세운 뒤 자율 복귀**: 시퀀서 원칙("포기하고 자율 복귀, 정지는 탈락")과 같다.
  ⚠ 리스크: 주차칸 안에서 자율로 돌아가면 pure_pursuit 가 칸 밖으로 빠져나가는 궤적이
  이상할 수 있다. 정지 유지(탈락 확정)보다 낫다는 판단. 실차 확인 필요.
- **1.0s 유효기간**: 세 발행자 모두 arm 중 10Hz 로 매 tick 재발행(상태 전이 tick 에서 최대
  1 tick 공백). 1.0s = 10 메시지 누락. `ramp_arm_timeout` 과 같은 값. ⚠ 신호대기 중 신호
  노드가 1s 넘게 멎으면 빨간불에 출발할 수 있다 — 감점(신호위반) vs 탈락(정지 고착) 중 감점 쪽.

## 4. 비목표
- FMA-002(발행자 3개 중재) — 별도 작업.
- `/obstacle_distance` 의 같은 무기한 유지 문제 — 장애물 정지는 안전기능이라 만료 정책을 따로 정해야 함.
- mux·pure_pursuit·mission_sequencer·`parking_direct.py`(launch 미사용) 수정 없음.
- 시퀀서 프로세스가 죽는 경우(arm 이 끊김) — 이번엔 하강엣지만 다룬다.

## 5. 검증

이 PC(Windows)에는 rclpy 가 없다. 테스트는 rclpy/메시지 모듈이 없으면 최소 가짜 모듈을
`sys.modules` 에 넣고, **실제 노드 클래스의 메서드**(`arm_cb`, `goal_cb`, `tick`, `reset_cb`,
`stop_cb`, `control_loop`, `_expire_stop_line`)를 Stub self 로 부른다. 로봇(rclpy 있음)에서도 그대로 돈다.

```
python tools/test_parking_release.py      # 기대: 마지막 줄 "ALL PASS", exit 0
python tools/test_stop_line_lease.py      # 기대: 마지막 줄 "ALL PASS", exit 0
python -m py_compile src/mission_perception/mission_perception/parking_node.py src/velocity_controller/velocity_controller/longitudinal_controller_node.py
```

test_parking_release.py 시나리오
1. 시퀀서 실제 발행 패턴: arm 전 False×N → True×N(DRIVE) → follower DONE/FINISHED → False×N.
   기대: RELEASING 동안 0 명령, 정지+1s 후 RELEASED, 이후 teleop·done 발행 0건.
2. DRIVE 중 False(시퀀서 타임아웃): RELEASING → 속도가 안 줄어도 3s 에 RELEASED.
3. 시작 실패 ABORT 세 가지(계획 실패·pose 파일 없음·측위 미수신, 실제 `_trigger`→`begin` 경로):
   done False 발행, teleop 0건, 하강엣지 시 teleop 0건으로 RELEASED.
9. 멱등성: RELEASING 중 True→False 반복 → `_release_since` 불변, 3s 에 RELEASED.
   RELEASED 뒤 True→False → teleop 0건 유지.
8. `_owns_control()`: DRIVE/DONE/RELEASING 에서 True, WAIT/RELEASED/follower 없는 ABORT 에서 False.
4. arm 전 False 만 계속: 상태 WAIT, 발행 0건.
5. `/goal_reached` True 트리거 + `/parking/start` False 수준 계속: 해제 안 됨.
6. **기존/신규 비교**: `git show 3752505:...parking_node.py`(수정 직전 커밋 **고정** — HEAD 아님) 를 임시 모듈로 불러, 하강엣지 없는
   같은 입력 열(--once True, goal_reached)에 대해 `/teleop/cmd_vel`·`/parking/done` 발행 열이 같음.
7. reset: RELEASING 중 무시, RELEASED 에서 WAIT.
(follower 는 상태를 조종할 수 있는 가짜 follower 로 대체 — 기하는 기존 test_parking_* 이 맡는다.)

test_stop_line_lease.py 시나리오
1. 0 수신 후 1.1s 무수신(monotonic 주입) → 999 로 복귀, 경고 1회. 0.9s 는 유지.
2. 10Hz 로 0 계속 → 만료 없음, 목표속도 0 유지.
3. 999 한 번 후 무수신 → 경고 없음.
4. `stop_line_timeout=0` → 만료 없음(기존 동작).
6. 벽시계 역행: `time.time` 을 1시간 뒤로 돌려도 monotonic 기준 1.1s 뒤 만료된다.
5. **기존/신규 비교**: 수정 직전 커밋 `3752505` 고정 버전과 신규 버전 `control_loop` 를 같은 신선 입력 열(곡률·정지거리
   격자, 시간 주입)로 돌려 `/target_speed` 열이 같음.

로봇 실차/HIL 확인 (수동, 이 PC 에선 불가 — 결과 보고 시 미실행으로 표시)
- `colcon build` 후 `sequencer:=true parking:=true` 로 띄우고
  `ros2 topic pub -r 10 /parking/start std_msgs/Bool "{data: true}"` → 끊고 `"{data: false}"` 로
  바꿨을 때 `ros2 topic hz /teleop/cmd_vel` 이 ~1–3s 뒤 0 이 되는지.
- crosswalk 노드를 HOLD 중 `kill` → 1s 뒤 `/target_speed` 가 0 에서 올라가는지.

## 6. 툴체인
- Claude(빌더): Python 3.12(이 PC), git. ROS 불필요.
- Codex(리뷰·검사): 읽기 전용. 실행 없음.
