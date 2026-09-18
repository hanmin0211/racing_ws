# 오늘·내일 할 일 (2026-09-18)

```
9/18 (오늘)  마지막 준비일 — 용인 없이 답 낼 수 있는 것 전부
9/19 (내일)  용인 현장 — 유일한 현장 날. 순서가 생명이다
9/20         본경기
```

> **오늘의 원칙**: 내일은 시간이 없다. **현장 없이 알 수 있는 것은 오늘 전부 알아낸다.**
> IMU 피치 부호는 주차 상태 30분이면 끝나는데, 내일로 미루면 경사로를 아예 못 켠다.

| 실패 비용 | |
|---|---|
| 도로/연석 이탈 · 1분 이상 정지 | **탈락** |
| 미션 실패 · 경사로 부진 | 감점 (회복 가능) |
| 8분 초과 | 감점 1분당 5점 |

---

# 오늘 (9/18)

## 0. 전원·연결 — 순서를 지킬 것 (5분)

반대로 하면 엔코더가 0을 센다.

```
1) 차량 배터리 ON
2) Arduino USB
3) GPS(u-blox) USB
4) IMU USB
```

```bash
lsusb | grep -iE "u-blox|2341|10c4"
```
```bash
ls -l /dev/imu /dev/arduino
```

- `/dev/imu` 없으면 → 라이다와 같은 칩(10c4:ea60)이라 **시리얼 `0001`** 로 구분한다
  (`/etc/udev/rules.d/99-imu.rules`). 다른 CP2102 가 먼저 잡혔는지 확인.
- **USB 허브를 쓰지 말 것** — 허브 깊이가 시리얼을 끊는다(dmesg `-32` = EPIPE). 직결.
- **배터리 완충.** ③의 판정이 배터리 상태에 좌우된다. 과거에 방전을 하드웨어
  고장으로 오진한 적이 있다.

---

## ① IMU 피치 부호 + 실제 경사도 — 30분 · 주차 상태 🔴 최우선

**차가 움직이지 않는다.** 본 주행과 같은 노드 설정으로 영점을 잡아야 하므로
평소 런치를 그대로 쓴다 — `control` 기본값이 false 라 모터 체인이 안 켜진다.

```bash
cd /home/han/racing_ws && source install/setup.bash && ros2 launch gps_localization bringup.launch.py rviz:=false
```
```bash
cd /home/han/racing_ws && source install/setup.bash && ros2 topic hz handsfree/imu
```

`직진하면 헤딩 초기화...` 메시지가 보여도 **무시한다.** `auto_calib` 기본 false 라
차는 안 움직이고, 캘리브는 yaw 만 건드려 피치와 무관하다.

**평지에 세우고 영점** (손 떼고 대기):
```bash
cd /home/han/racing_ws && source install/setup.bash && python3 tools/imu_grade.py --level
```
- 기울기 **5° 넘으면** 마운트가 삐뚤거나 바닥이 안 평평하다
- 표준편차 **1° 넘으면** 차가 흔들린다 — 다시

**경사 중턱, 코를 위로 두고 측정**:
```bash
cd /home/han/racing_ws && source install/setup.bash && python3 tools/imu_grade.py --measure --expect 12
```

`--expect 12` 는 **부호만 보려고** 넣는 값이다(판정이 `grade * expect < 0`).
실제 경사가 10이든 15든 상관없다.

| 출력 | 조치 |
|---|---|
| `▶ 보정 피치 +7.1° → 경사 +12.5%` | ✅ 통과. **그 12.5% 가 실제 경사도다 — 적어 둘 것** |
| `❌ 부호가 반대다` | `python3 tools/imu_grade.py --level --sign -1` 후 ② 재측정 |
| 값이 0 근처 | 차가 경사에 제대로 안 올라가 있다 |

> **경사로가 없으면 앞바퀴를 괴면 된다.** 축거 0.785 m 라 `sinθ = h/0.785`:
> 10 cm → 12.9%(법정과 같음) · 15 cm → 19.5%. 양쪽 앞바퀴를 같은 높이로.
> 서스펜션이 눌려 크기는 조금 작게 나올 수 있으나 **부호 판정에는 영향 없다.**

**★ 잰 경사도로 파라미터를 정한다**

| 실측 경사 | `grade_ff_max` |
|---|---|
| ~14 % | 70 (기본) |
| **15 % 이상** | **95** ← 기본 70 은 14.4% 까지만 완전보상 |

---

## ② 조향 전원 — 15분 · 바퀴 들고 🔴 가장 큰 미지수

```bash
cd /home/han/racing_ws && source install/setup.bash && python3 tools/drive_diag.py --yes
```

**측정된 것**: 조향 ±18° 에서 레일이 4198 → **2038 mV** 로 무너진다(무부하).
**미확인**: 그게 실제로 리셋·링크 끊김을 일으키는가. 오히려 **기록된 주행은
VMIN 2282~2528 로도 완주**했다(과거 리셋선 3483 아래인데도).

**볼 것**: VCC 가 떨어질 때 **재연결·리셋 로그가 찍히는가.**

| 결과 | 판단 |
|---|---|
| 재연결 로그 없음 | 이 항목 내려놓고 경사로에 집중 |
| 재연결 로그 있음 | **내일 최우선.** S자·굴절에서 조향 공백 → 이탈 위험 |

⚠ "조향 = 탈락" 은 아직 **가설**이다. 사실처럼 말하지 말 것.

---

## ③ 오르막 실주행 — 30분 (①이 통과한 뒤에만)

경사로 **3~5 m 앞**에 세운다. 터미널 2개.

```bash
cd /home/han/racing_ws && source install/setup.bash && ros2 launch tools/teleop_drive.launch.py ff_mode:=ros max_speed:=2.0 ff_min_pwm:=0.0 ff_breakaway_pwm:=60.0 grade_ff_gain:=0.5
```
```bash
cd /home/han/racing_ws && source install/setup.bash && python3 tools/ramp_test.py --label 오르막반 --drive 12 --speed 2.0 --abort-speed 2.8 --stuck-after 5 --seconds 40
```

**키보드 안 쓴다.** `ramp_test --drive` 가 `/teleop/cmd_vel` 로 자율 주행한다 —
엔터 치면 차가 12 m 가고 선다. E-stop 만 쥐고 있으면 된다.

- `--drive` = 조주 4 m + 경사 8 m. **경사 진입 후 8 m 면 정상상태에 닿는다**
- 경사로에서 서면 **뒤로 밀린다**(홀드 없음). `PWM:` 명령은 펌웨어 PID 를
  우회하므로 모터 부하는 0 이다. 뒤를 비워 둘 것 — 밀림 가속도 약 0.86 m/s²

**볼 숫자** (12.5% 기준 예측):

| | 예상 |
|---|---|
| 진입 직후 최저 | 1.5 m/s 로 한 번 떨어짐 (정상) |
| **정상상태 속도** | **1.8 m/s** |
| **정상상태 PWM** | **156** (포화 255) |

PWM 이 255 에 붙으면 경사가 예상보다 급하거나 배터리가 빠진 것이다.
잘 오르면 `grade_ff_gain:=1.0` 으로 한 번 더. **CSV 경로를 남길 것.**

**★ 이 주행이 부호를 다시 증명한다.** 정상상태는 가속도가 0이므로 PWM·속도만으로
경사를 역산할 수 있다 — IMU 를 안 쓰고:

```
sinθ = (0.0202·PWM − 0.861·v − 0.37) / 9.81
```

PWM 156, v 1.80 → 12.6%. ①의 IMU 측정과 **같은 부호·같은 크기면 확정**이다.

---

## ④ 커밋 · 배터리 충전 — 20분

대회 이틀 전에 미커밋 상태로 두지 말 것. 배터리는 전부 충전.

---

# 내일 (9/19) 용인 — 순서를 바꾸면 하루가 날아간다

```
1. bash tools/use_site.sh yongin      ← 원점이 76.8km 어긋나 있다. 이거 먼저
2. python3 tools/preflight.py         ← 원점·경로·정지점 일괄 검사
3. 헤딩 캘리브 + 시운전 1랩            ← 미션 전부 off, 완주만 확인
4. python3 tools/mission_s.py --live  ← 경사로 시작/정상부/끝 포함, 전 미션
5. mission_plan.yaml 채우고 mission_plan_check.py 통과 → enabled:true
6. 경사로 구간 켜고 1랩
7. 정지점 재기록 (현재 149.7km 어긋남)
```

⚠ **용인 전체 코스이므로 `config/mission_plan.yaml` 을 쓴다.**
`config/ramp_course/mission_plan.ramp.yaml` 은 **단독 경사로 코스용**이다.
섞으면 `check_course` 가 길이 불일치를 잡아 **미션을 전부 끈다**(조용히).

## 첫 랩 진행 흐름

```
① RTK Fixed 대기
② 5초 카운트다운
③ 차가 스스로 10m 직진          ← ⚠ 앞을 비우고 E-STOP 쥘 것
④ 정지 → 직진성 검증
⑤ 통과 → yaw_offset 발행
⑥ 먹스: "헤딩 캘리브 완료 — 자율 허용"
⑦ 모드: AUTO → 경로 추종 시작
```

**⚠ ③은 헤딩을 모르는 개루프 직진이다.** 차가 지금 향한 쪽으로 그냥 간다.
경로 시작 방향으로 대충 맞춰 놓고 시작할 것. 직선 12 m 필요.

## 경사로를 본 주행에 얹는 명령

```bash
cd /home/han/racing_ws && source install/setup.bash && ros2 launch gps_localization bringup.launch.py control:=true lidar:=true sequencer:=true auto_calib:=true rviz:=false max_speed:=2.0 ff_mode:=ros ff_min_pwm:=0.0 ramp_up_arm_topic:=/ramp/up_arm v_ramp_up:=2.0 ramp_down_arm_topic:=/ramp/down_arm v_ramp_down:=1.11 grade_ff_gain:=1.0 gov_pwm:=50.0
```

⚠ **소수점을 꼭 찍을 것.** `v_ramp_up:=2` 는 INTEGER 로 들어가 노드가 즉사한다.

시작 로그에서 **네 줄**을 확인한다. 없으면 그 기능은 안 켜진 것이다:

| 로그 | 없으면 |
|---|---|
| `★ 오르막 구간 속도 켜짐` | `ramp_up_arm_topic`/`v_ramp_up` 미적용 |
| `★ 내리막 구간 속도 켜짐` | `ramp_down_arm_topic`/`v_ramp_down` 미적용 |
| `★ 경사 보상 켜짐` + 영점·부호 | ①을 안 했거나 `grade_ff_gain` 이 0 |
| `헤딩 초기화(1회성) 시작: 10m` | `auto_calib` 꺼짐 |

주행 중 arm 이 실제로 오는지:
```bash
ros2 topic echo /ramp/up_arm
```

## 차가 안 갈 때 — 로그로 원인 판별

| 로그 | 원인 | 조치 |
|---|---|---|
| `모드: STOP(헤딩 캘리브 전)` | 캘리브 미완료 | ③~⑤ 다시. 거부됐으면 3초 뒤 자동 재시도 |
| `모드: STOP(입력끊김)` | `/target_speed` 또는 `/steering_cmd` 없음 | 제어 노드가 떴는지 확인 |
| `⚠ 경로 ...s 끊김 — 완주 신호 없음` | GPS/IMU 끊김 | `gps_check` 로 RTK, IMU 재연결 |
| `정지: lookahead가 차량 뒤` | 헤딩 뒤집힘 의심 | `invert_imu_yaw:=true` 로 재시작 |
| `직진성 검증 실패 (편차 ...°)` | 캘리브 중 휘었다 | 자동 재시도. 계속 실패하면 더 평탄한 곳 |
| 내리막에서 **오히려 빨라진다** | **피치 부호 반대** | **즉시 `grade_ff_gain:=0.0`** → ① 다시 |

## 완주 판정

```
[local_sliding_window_node]  ★ 경로 완주 → 정지 신호 발행
[longitudinal_controller]    🏁 완주 신호 수신 — 감속 정지합니다 (고장 아님)
```

**`🏁` 가 뜨면 완주다. `⚠` 는 고장이다.** 이 구분을 위해 넣은 것이니 헷갈리지 말 것.

---

# 판단 근거 — 왜 이 순서인가

| 항목 | 실패하면 | 오늘 가능? | 상태 |
|---|---|---|---|
| 조향 전원 | **탈락 가능** | ✅ ② | ❓ 미확인 |
| 경로 추종 | 탈락 | ✅ 완료 | ✅ 여유 충분 |
| 경사로 | 감점만 | ✅ ①③ | 예측 완료 |
| 미션 8개 | 감점만 | ❌ | 내일 s 실측 |
| 8분 초과 | 감점 5점/분 | — | 무리할 이유 없음 |

## 경로 추종은 폐루프 시뮬로 확인했다 (`tools/tracking_sim.py`)

용인 648 m · `wp_yongin_drive_0.5.yaml`:

| 조건 | 완주 | 최대 이탈 | 조향 포화 |
|---|---|---|---|
| 오차 없음 v=2.0 | ✅ | 0.15 m | 0 % (12.9°/18°) |
| v=2.8 (bringup 기본) | ✅ | 0.19 m | 0 % |
| 헤딩 편향 5° | ✅ | 0.32 m | 0 % |
| 헤딩 편향 10° | ✅ | 0.50 m | 0 % |
| 조향 중립 3° 어긋남 | ✅ | 0.25 m | 0 % |
| v=2.8 + 헤딩5° + 중립2° | ✅ | 0.27 m | 0 % |

이탈 경고선 0.72 m · 복귀 불가선 0.90 m. **코스 기하는 완주의 병목이 아니다.**
시뮬이 **안 보는 것**: 조향 전원 강하 · GPS 끊김 · 라이다 회피 override.

## 경사로는 grade_ff 없이는 안 된다 (`tools/ramp_profile.py`)

| 경사 | grade_ff 켬 | 끔 | 끔 + 구동 15% 약화 |
|---|---|---|---|
| 10 % | 1.61 m/s | 0.66 | 0.33 |
| **12.5 %** | **1.56** | **0.37** | **0.04 = 못 넘음** |
| 15 % | 1.51 | **못 올라감** | 못 올라감 |

어제 로그가 같은 말을 한다(`data/2026-09-17-ramp/ramp_PWM160_2302.csv`):
**PWM 105 에서 0.37 → 0.23 으로 꺼져 갔고, 160 으로 올리니 올라갔다.**
자율 FF 가 2.0 m/s 에 내는 PWM 은 **94.8** — 이미 실패한 105 보다 낮다.
grade_ff 가 +60 을 더해 155 를 만든다 = 어제 손으로 올린 160 과 같은 자리.

---

# 상시 참고

### 뭔가 이상하면 제일 먼저
```bash
bash /home/han/racing_ws/tools/ros_cleanup.sh
```
`pkill -f ros2` 는 런처만 죽인다. 고아 노드가 쌓이면 NTRIP 401 · USB 충돌 ·
yaw_offset 덮어쓰기가 난다.

### E-stop
```bash
ros2 topic pub --once /e_stop std_msgs/msg/Bool "{data: true}"
```

### 수동 조작 (별도 터미널)
```bash
cd /home/han/racing_ws && source install/setup.bash && ros2 run velocity_controller teleop_keyboard
```
※ `teleop:=true` 런치 인자는 쓰지 말 것 — xterm 이 없어 키 입력을 못 받는다.
※ **한글 입력 상태면 W 가 ㅈ 으로 들어가 차가 안 움직인다.** 영문 전환 확인.

### 이어서 볼 문서

| 문서 | 내용 |
|---|---|
| `RAMP_RUNBOOK.md` | 경사로 현장 절차 ①~⑦ |
| `HANDOFF_2026-09-18.md` | 저장소 작업 원칙 · 물리 상수 · 실행 위생 |
| `config/mission_plan.yaml` | 용인 미션 계획 (주석에 규정 조항) |
