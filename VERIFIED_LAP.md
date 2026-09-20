# ★ 검증된 본선 주행 명령 — 2026-09-19 15:36 용인

> **이대로 간다.** 이 문서의 명령을 글자 하나 바꾸지 말 것.
> 근거 로그: `data/2026-09-19/mainlap_1536/console.log` (555행)
> 파라미터 덤프: `data/2026-09-19/mainlap_1536/params/` (14개)

---

## 1. ★ 내일 아침 명령 — 터미널 4개, 이 순서대로

### 터미널 1 — 정리 (제일 먼저, 한 번만)
```bash
pkill -f teleop_keyboard; cd /home/han/racing_ws && bash tools/ros_cleanup.sh
```
`✅ 없음` 이 떠야 한다.

### 터미널 1 — 라이다 드라이버 (계속 켜 둘 것)
```bash
cd /home/han/racing_ws && bash tools/lidar_up.sh 12
```
⚠ 이 스크립트의 `✅` 는 **발행자 수만 본다. 믿지 말 것**(9/19 에 세 번 오판했다).
반드시 데이터로 확인한다 — **11~12 Hz** 가 찍혀야 한다:
```bash
cd /home/han/racing_ws && source install/setup.bash && ros2 topic hz /scan_front
```
확인되면 이 `hz` 만 Ctrl+C. **드라이버 터미널은 닫지 않는다.**
안 뜨면 `lidar_up.sh` 를 다시. 여러 번 실패해도 갑자기 붙는다(간헐적이다).
끝내 안 붙으면 아래에서 `lidar:=true` → `lidar:=false` 로만 바꾼다.

### 터미널 2 — 주행

차를 **코스 출발점 (56.09, 5.39)** 에, **앞바퀴 중앙**(ADC 412 ±41)으로 놓는다.

```bash
cd /home/han/racing_ws && source install/setup.bash && ros2 launch gps_localization bringup.launch.py control:=true sequencer:=false lidar:=true rviz:=true auto_calib:=true waypoints:=/home/han/racing_ws/config/yongin_2026-09-05/wp_yongin_drive_0.5.yaml max_speed:=1.6 curvature_gain:=6.0 calib_distance:=10.0 auto_calib_speed:=0.5 calib_jump_speed:=3.0 ff_mode:=ros ff_static:=46.0 ff_gain:=42.6 ff_min_pwm:=0.0 ff_breakaway_pwm:=90.0 ff_breakaway_ms:=1200.0 grade_ff_gain:=1.0 grade_ff_max:=95.0 gov_pwm:=60.0 gov_min_grade:=0.06 gov_gain:=300.0 gov_lead_s:=0.3 gov_deadband:=0.20 2>&1 | tee /tmp/run.log
```

**손 안 댄다.** 5초 카운트다운 → 스스로 10m 직진 → 캘리브 → AUTO.

### 터미널 3 — 기록 (선택, 권장)
```bash
cd /home/han/racing_ws && bash tools/record_run.sh mainlap_full
```

### 터미널 4 — teleop (비상정지) — **`모드: AUTO` 가 뜬 뒤에만**
```bash
cd /home/han/racing_ws && source install/setup.bash && ros2 run velocity_controller teleop_keyboard
```
`E` = 정지 토글. 한 번 더 누르면 그 자리에서 이어서 간다.
⚠ **끌 때는 `E` 로 해제부터.** Ctrl+C 로 끄면 `/e_stop=False` 를 쏴서 차가 출발한다.
⚠ **AUTO 전에 띄우면 차가 안 나간다**(§1-B).

---

## 1-A. 출발 전 4가지 (9/19 에 이걸로 네 번 막혔다)

```
① 앞바퀴 중앙        꺾여 있으면 heading_init 이 출발을 거부한다
② teleop 꺼져 있나    떠 있으면 /teleop/cmd_vel 에 0 을 덮어쓴다
③ 차가 출발점에      (56.09, 5.39)
④ 라이다 11~12 Hz    발행자 수 말고 hz 로 확인
```

## 1-B. ★ 순서 — 이걸 어기면 차가 안 나간다 (2026-09-19 16:16 에 당했다)

```
① bash tools/ros_cleanup.sh   +   pkill -f teleop_keyboard
② (라이다 쓸 때만) 터미널1: bash tools/lidar_up.sh 12   ← 계속 켜 둔다
③ 터미널2: 위 §1 런치.  차는 출발점에, 앞바퀴는 중앙(ADC 412±41)
④ **`모드: AUTO` 가 뜬 뒤에** 터미널3: ros2 run velocity_controller teleop_keyboard
```

⚠ **teleop 을 캘리브 전에 띄우면 차가 못 나간다.**
  `heading_init` 의 자가 직진은 **`/teleop/cmd_vel` 에 0.5 를 발행**한다.
  teleop 은 W/S 를 안 눌러도 **데드맨으로 0 을 20Hz 로 발행**한다.
  같은 토픽이라 0 이 덮어쓴다 — 실측 분포 `{0.0: 60건, 0.5: 34건}`.
  증상은 조용하다: 로그가 `직진 기준 헤딩 고정` 에서 멈추고 차는 안 움직인다
  (`직진 중... 1.0/10m` 은 1m 를 가야 찍히므로 아예 안 나온다).

⚠ **teleop 은 한 개만.** 두 개면 E-stop 상태가 서로 엇갈린다(16:12 에 2개 떠 있었다).

⚠ **teleop 종료는 반드시 `E` 키로 해제 후.** `Ctrl+C` 로 끄면 종료 시
  `/e_stop=False` 를 발행해 **차가 출발한다**(teleop_keyboard_node.py:143).

⚠ **`lidar:=true` 는 드라이버를 안 띄운다.** 클러스터링만 띄우고 `/scan_front` 는
  외부(`lidar_up.sh`)에서 오길 기대한다. 그 드라이버를 죽이면 런치가 통째로 내려간다.

## 2. 이 줄들이 나오면 정상 (15:36 실측)

```
RTK 수렴 (수평 σ=1.5cm (상한 5cm), status=1) — 헤딩 캘리브를 시작한다.
직진 중... 0.0/10m  →  9.0/10m
✅ 헤딩 초기화 완료: 10.3m 직진 (최대 편차 1°).
   GPS course=159.4°, IMU yaw=120.0°, → yaw_offset=39.5°
헤딩 캘리브 완료 (39.5°) — 자율 허용
모드: AUTO
```

캘리브 주행은 **14.0초에 10.3m = 평균 0.74 m/s** 였다.

## 3. 그날 실제로 일어난 것

```
출발 (56.2, 3.8)  →  마지막 확인 (0.75, −44.26)
AUTO 주행 3초 샘플 32건, 직선거리 합 약 186m
거버너 개입 10회
VMIN 4106 mV (리셋선 3483 — 뚫지 않았다)
```

## 4. ⚠ 알고 쓰는 것 — 고치지 말고 알고만 있을 것

**① 개루프가 명령보다 훨씬 빠르다.**
```
명령 1.50 → 측정 2.37 m/s   (거버너가 제동 −60 으로 잡았다)
구간 최고 측정 3.09 m/s      (max_speed 는 1.6)
```
`ff_static:=46.0` 은 **정지마찰을 넘기려고** 올린 값이라 고속에서 과하다.
내리면 저속에서 차가 안 나간다(`ff-static-cut-breaks-stiction`). 그래서 그대로 둔다.
**거버너가 이 과속을 잡아 주고 있다** — `gov_pwm:=60` 을 끄면 안 된다.

⚠ 이 코스 최소R 2.71m(급커브 19개)에서 3.09 m/s 는 이탈 여유가 없다.
   속도를 낮추려면 `ff_static` 이 아니라 **`max_speed` 를 내릴 것.**

**② 거버너 자세 게이트가 평지에서도 열린다.**
`gov_min_grade:=0.06` 인데 평지 주행 중 10회 걸렸다. 가속 오염 보정
(`48bd0f5`)을 넣었어도 남는다. 지금은 **과속을 잡아 주는 쪽으로 작동**하므로
그대로 둔다. 문턱을 올리면 이 과속을 못 잡는다.

**③ 캘리브 전에는 거버너가 캘리브를 깎는다.**
```
거버너 — 측정 0.60+0.24(선행) > 명령 0.50 (+0.34) → PWM 69 → 28
```
그래도 캘리브는 **편차 1°** 로 통과했다. 건드리지 말 것.

**④ 전압.** 무부하 4092~4137 mV (preflight 문턱 4300). 주행 중 VMIN 4106.
과거 리셋선 3483 을 이번엔 안 뚫었지만 여유가 없다. **주행 전 충전할 것.**

**⑤ 라이다 꺼져 있다.** 본선의 S자 장애물·돌발정지를 하려면 `lidar:=true`.
붙는지 먼저: `bash tools/lidar_up.sh 12`

**⑥ 활성 미션 0개 · 정지점 0개.** 자율 완주만 한다(미션은 전부 감점).
시퀀서를 켜려면 `config/mission_plan.yaml` 에서 s 를 재고 `enabled: true`.

## 5. 실패하면

| 증상 | 원인 | 조치 |
|---|---|---|
| heading_init 이 **로그를 한 줄도 안 낸다** | 재시도를 다 써서 `drive_aborted` — 이 상태에선 영원히 조용하다 | 런치를 죽이고 다시 띄운다 |
| `❌ 헤딩 캘리브 무효 ... 속도상한` | 개루프 과속 | 이미 고쳤다(`283d654`). `ff_mode:=ros` 면 `auto_speed_cap` 이 2.0 으로 자동 전달된다 — `ros2 param get /gps_heading_init auto_speed_cap` 으로 확인 |
| `↩ 차를 시작점으로 되돌리세요` | 실패 후 10m 앞에 서 있다 | 출발점으로 되돌리면 자동 재출발 |
| 스택이 두 벌 | 정리 없이 겹쳐 띄웠다. `/global_path` 에 경로 2종이 번갈아 올라온다 | 반드시 `ros_cleanup.sh` 먼저 |
| 차가 안 나가고 로그가 `직진 기준 헤딩 고정` 에서 멈춤 | **teleop 이 떠 있다** — `/teleop/cmd_vel` 에 0 을 덮어쓴다 | `pkill -f teleop_keyboard`. 런치는 그대로 둬도 바로 출발한다 |
| `❌ 헤딩 캘리브 무효: 주행 중 측위 점프 ... = 2.0m/s (상한 2.0m/s)` | 개루프 캘리브가 **가속한다**(실측 1.01→1.69 m/s). 진짜 점프가 아니다 | `calib_jump_speed:=3.0`. 2026-08-24 사고는 19m/s 라 3.0 으로도 잡힌다 |
| `앞바퀴가 우로 N° 꺾여 있다` + `STALL: steer=1` | 정지 상태 아스팔트 조향이 못 이긴다 | 손으로 앞바퀴를 중앙에. 차를 몇 cm 굴리면 스스로 찾는다 |
| `FollowGap: mode=BLOCKED, front=0.3~0.9m` | 라이다가 지면/차체를 때린다 | `python3 tools/lidar_mount_check.py` 로 마운트 확인 |

---

# ★ 2차 주행 — 돌발정지 포함 (2026-09-20)

1차 완주(a6d737f) 확보 후 미션에 도전하는 설정이다.

```bash
cd /home/han/racing_ws && source install/setup.bash && ros2 launch gps_localization bringup.launch.py control:=true sequencer:=true sudden_stop:=true sudden_stop_dwell:=5.0 obstacle_stop_dist:=1.0 lidar:=true rviz:=true auto_calib:=true waypoints:=/home/han/racing_ws/config/yongin_2026-09-05/wp_yongin_drive_0.5.yaml max_speed:=1.6 curvature_gain:=6.0 calib_distance:=10.0 auto_calib_speed:=0.5 calib_jump_speed:=3.0 ff_mode:=ros ff_static:=46.0 ff_gain:=42.6 ff_min_pwm:=0.0 ff_breakaway_pwm:=90.0 ff_breakaway_ms:=1200.0 grade_ff_gain:=1.0 grade_ff_max:=95.0 gov_pwm:=60.0 gov_min_grade:=0.06 gov_gain:=300.0 gov_lead_s:=0.3 gov_deadband:=0.20 2>&1 | tee /tmp/run2.log
```

## 1차 대비 바뀐 것 4개뿐

| 인자 | 1차 | 2차 | 왜 |
|---|---|---|---|
| `sequencer` | false | **true** | 미션 arm/disarm |
| `sudden_stop` | (없음) | **true** | 돌발정지 노드 |
| `sudden_stop_dwell` | — | **5.0** | 규정 최소 3초 + 여유 2초 |
| `obstacle_stop_dist` | 0.8 | **1.0** | 관성 여유 (아래 계산) |

## 구간 배치 (config/mission_plan.yaml)

```
sudden_stop   s=470~515   운전자가 지도에서 짚은 483~499 + 앞뒤 여유
lidar_avoid   s= 12~648   allow_full_course:true
S자 구간      s=166~250   ← 돌발 구간과 안 겹친다. 회피 정상
```

| 구간 | 회피 조향 | 감속·정지 | 3초 홀드 |
|---|---|---|---|
| s=0~12 (캘리브) | — | ✅ | ❌ |
| s=12~470 | ✅ | ✅ | ❌ |
| **s=470~515 (돌발)** | ❌ 규정상 차단 | ✅ | ✅ |
| s=515~648 | ✅ | ✅ | ❌ |

## ⚠ obstacle_stop_dist 는 '멈출 거리' 가 아니다

런치 주석: **"동력을 끊을 거리지 멈출 거리가 아니다"**.
`ff_brake_pwm:=0` (능동제동 꺼짐)이라 끊은 뒤 관성으로 더 간다.
모델 `a = −(0.861·v + 0.37)` 로 계산한 관성 거리:

```
차단 시 0.5 m/s → 0.20 m      1.5 m/s → 0.99 m
        1.0 m/s → 0.56 m      2.0 m/s → 1.46 m
```

4m 부터 감속 램프가 걸리므로 실제로는 훨씬 느리게 도달하지만,
개루프라 명령보다 빠르다. 그래서 0.8 → **1.0** 으로 0.2m 벌었다.

⚠ **1.2 이상으로 올리지 말 것.** sudden_stop_node 의 `trigger_dist`
가 1.2 이고 "obstacle_stop_dist(0.8)보다 조금 크게 잡아 하드정지를
인지한다" 는 전제다. 뒤집히면 dwell 타이머가 안 돌아 미션이 조용히 실패한다.

⚠ **능동제동(`ff_brake_pwm`)은 켜지 않았다.** 급제동이 전원을 끌어당기는데
오늘 VMIN 4230mV 다. 그 자리에서 멈추면 1분 넘어 탈락이다.

## 검증한 것

```
✅ 런치 배선   require_arm=True · dwell=5.0 · require_arm_for_steer=True
✅ 코스 대조   계획 648.3m · 실제 648.27m
❌ 겹침 경고   sudden_stop↔lidar_avoid — **의도한 것**
```

겹침 ❌ 는 규정(항목 7 "회피기동은 미션 성공으로 인정하지 않음")이 요구하는
동작이다. 검사기는 과거 사고(돌발이 회피를 꺼 의자를 못 피함) 때문에 경고한다.
→ **s=470~515 안에 더미가 아닌 장애물이 있으면 못 피한다. 다만 정지는 한다.**

## 더미 처리

```
완전정지 → 5초 유지 → 더미를 3m(clear_dist) 밖으로 치우면 재출발
10초(clear_timeout) 안 치우면 스스로 우회(PASSING) — 규정상 허용
```

⚠ `sequencer:=true` 는 오늘 실차로 한 번도 안 돌려봤다. 1차 완주는 false 였다.
