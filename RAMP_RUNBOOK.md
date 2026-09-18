# 경사로 주행 — 현장 절차 (2026-09-18 작성)

오르막을 올라가고 내리막을 **버티면서** 내려온다. 웨이포인트(GPS+IMU)로
구간을 알고, IMU 피치로 중력을 상쇄하고, 거버너로 과속을 막는다.

> ⚠ **이 문서의 절차를 건너뛰지 말 것.** 경사 보상은 부호가 반대면
> 내리막에서 **가속한다**. 그래서 기본이 꺼짐이고, ①②를 통과해야 켠다.

---

## 세 가지가 각각 하는 일 — 겹치지 않는다

| | 무엇을 정하나 | 근거 | 웨이포인트 필요? |
|---|---|---|---|
| **grade_ff** | 그 속도를 내려면 **PWM 이 얼마여야 하는지** | IMU 피치 | ❌ 없어도 된다 |
| **구간 속도** | **어디서 얼마의 속도**를 낼지 | course_s | ✅ 필요 |
| **거버너** | 그래도 빨라지면 **깎는다** | 엔코더 위치차분 | ❌ |

grade_ff 는 지도에 없는 경사도 잡는다. 구간 속도는 지도가 있어야 한다.
거버너는 둘 다 실패했을 때의 마지막 방어선이다.

### 경사가 먹는 PWM (k = 0.0202 m/s²/PWM, 실측)

| 경사 | g·sinθ/k | 4km/h 유지 PWM |
|---|---|---|
| 법정 오르막 10% | +48 | +109 |
| 법정 오르막 12.5% | **+60** | +120 |
| 법정 내리막 6.5% | −32 | +29 |
| 법정 내리막 9% | **−44** | +17 |
| 시험 경사 19.3% | ±92 | −33 (제동) |

**법정 내리막은 제동이 아니라 스로틀만 줄이면 된다.** 플러깅 전류가 안 흐른다.

---

## ① 평지에서 영점 (5분)

차를 **평평한 곳**에 세우고, IMU 가 발행 중인 상태에서:

```bash
cd /home/han/racing_ws && source install/setup.bash && python3 tools/imu_grade.py --level
```

`config/imu_pitch_offset.yaml` 에 마운트 기울기가 저장된다.

- 기울기가 **5° 넘으면** IMU 가 삐뚤게 붙었거나 바닥이 안 평평하다. 확인할 것.
- 표준편차가 **1° 넘으면** 차가 흔들리고 있다. 손 떼고 다시.

## ② 경사로에서 부호 확인 (5분) — 여기가 제일 중요하다

차를 경사 **한가운데**, 코를 **위로** 두고 세운다.
(시작·끝에 걸치면 차체가 경사를 다 안 탄다)

```bash
cd /home/han/racing_ws && source install/setup.bash && python3 tools/imu_grade.py --measure --expect 12.5
```

`--expect` 는 그 경사로의 실제 값을 넣는다. 법정 오르막이면 10~12.5.

- **✅ 통과** → ③으로.
- **❌ 부호가 반대다** 가 뜨면, 시키는 대로 평지에서 다시:
  ```bash
  cd /home/han/racing_ws && source install/setup.bash && python3 tools/imu_grade.py --level --sign -1
  ```
  그리고 ②를 다시 한다.
- 값이 20% 넘게 어긋나면 영점을 다시 잡거나 차 위치를 확인한다.

> 어제 내리막 런에서 **운동방정식으로 역산한 19.3%** 가 독립적인 검증값이다
> (`data/2026-09-17-ramp/ramp_4키로_2349.csv`). 같은 경사에서 재면 맞아야 한다.

## ③ 경사 보상을 절반만 켜고 오르막 (10분)

부호가 맞아도 처음부터 완전 보상은 쓰지 않는다. **절반부터.**

```bash
pgrep -f 'install/[a-z_]+/lib/' | xargs -r kill -9
```
```bash
cd /home/han/racing_ws && source install/setup.bash && ros2 launch tools/teleop_drive.launch.py ff_mode:=ros max_speed:=2.0 ff_min_pwm:=0.0 ff_breakaway_pwm:=60.0 gov_pwm:=50.0 grade_ff_gain:=0.5
```

시작 로그에 **`★ 경사 보상 켜짐`** 과 영점·부호가 찍히는지 확인할 것.
`⚠ IMU 영점 파일이 없다` 가 뜨면 ①을 안 한 것이다.

```bash
cd /home/han/racing_ws && source install/setup.bash && python3 tools/ramp_test.py --label 오르막반 --drive 8 --speed 1.11 --abort-speed 2.5 --stuck-after 10 --seconds 30
```

**보는 것:** `실제` 열이 1.0 근처를 유지하며 8m 를 완주하는가.
잘 오르면 `grade_ff_gain:=1.0` 으로 올려 한 번 더.

## ④ 내리막 (10분)

같은 런치에 `gov_pwm` 을 경사에 맞게:

| 경사 | gov_pwm |
|---|---|
| 법정 6.5~9% | **40** |
| 12.5% | 50 |
| 시험 19.3% | 60 |

```bash
cd /home/han/racing_ws && source install/setup.bash && python3 tools/ramp_test.py --label 내리막 --drive 8 --speed 1.11 --abort-speed 2.2 --stuck-after 10 --seconds 30
```

**성공 기준** (속도값이 아니다):
- PWM 이 매끄럽게 움직인다 (71→23→59 같은 왕복 없음)
- 중단 없이 완주
- 속도가 한 값에 물린다 (계속 붙지 않는다)

경사 보상이 켜져 있으면 어제보다 **목표에 가까워야 한다.** 어제는
거버너 단독으로 5.7km/h 에 물렸다(목표 4.0). 경사 보상이 FF 단계에서
미리 빼주므로 거버너가 덜 일한다.

끝나면 CSV 경로를 남길 것 — `tools/replay_governor.py` 로 따질 수 있다.

## ⑤ 웨이포인트와 계획 파일 (15분) — 여기서 가장 조용한 고장이 난다

어제까지는 teleop 직진이라 웨이포인트가 없었다. 오늘은 GPS+IMU 헤딩으로
자율 주행하므로 **경로와 계획이 서로 맞아야** 구간 속도가 켜진다.

### 5-1. 웨이포인트를 기록한다

캘리브 10m 구간이 **직선**이어야 한다. `heading_init_node` 는 웨이포인트를
안 보고 '차가 향한 방향' 으로 직진하기 때문이다(그 방향이 곧 헤딩 기준이 된다).

기록한 뒤 길이를 읽는다:

```bash
cd /home/han/racing_ws && source install/setup.bash && python3 tools/list_waypoints.py
```

### 5-2. 세 지점의 s 를 잰다

차를 그 자리에 세우고 각각 실행한다:

```bash
cd /home/han/racing_ws && source install/setup.bash && python3 tools/mission_s.py --waypoints <위 파일> --live --name ramp_up
```

- 오르막 시작 (경사가 실제로 붙는 지점)
- 정상부 시작
- 내리막 끝 (평지 복귀)

### 5-3. 계획 파일 — ⚠ 용인 계획을 그대로 쓰면 안 된다

`config/mission_plan.yaml` 은 **용인 648m 코스**의 것이다. 경사로만 찍은 짧은
경로로 달리면서 그걸 쓰면 `mission_sequencer.check_course()` 가 길이 불일치를
잡아 **모든 미션을 건너뛴다.** 설계된 동작이다(엉뚱한 데서 켜면 탈락).

증상이 고약하다 — 에러도 없고 노드도 정상인데 `/ramp/up_arm` 이 한 번도 안
나가서 구간 속도가 통째로 안 걸린다. "기능이 고장났나" 로 반나절이 간다.

그래서 경사로 단독 코스는 전용 계획을 쓴다:

```
config/ramp_course/mission_plan.ramp.yaml
```

파일 안의 ①~⑥ 순서대로 `course.waypoints` · `path_length_m` · 세 s 값을
채운다. **용인 전체 코스를 도는 날에는 반대로** `config/mission_plan.yaml` 의
`ramp_up`/`ramp_down` 을 채운다.

### 5-4. 검사한다 — 통과 전에는 주행하지 않는다

```bash
cd /home/han/racing_ws && source install/setup.bash && python3 tools/mission_plan_check.py config/ramp_course/mission_plan.ramp.yaml --include-disabled
```

`path_length_m` 이 실제와 10% 넘게 다르면 여기서 잡힌다. 통과하면
`enabled: true` 로 바꾼다. **순서를 바꾸지 말 것.**

---

## ⑥ 거리 예산 — 21m 를 어떻게 나누는가

```
s=0 ──── 헤딩 캘리브 10m ────► s=10 (차가 선다) ──── 가속 11m ────► s=21 경사로
```

★ **캘리브가 끝나면 차는 정지한다.** `heading_init_node` 가 10m 를 채운 뒤
`_stop_driving('캘리브 완료')` 로 1초간 0 을 쏘고 토픽을 놓는다. 먹스의
`teleop_timeout`(0.5s)이 지나면 자율로 인계된다. 즉 **자율 주행은 s≈10m 에서
속도 0 으로 시작하고, 경사로까지 남은 11m 가 가속에 쓸 수 있는 전부다.**

그래서 계획 파일의 `ramp_up.s_enter` 를 오르막 입구가 아니라 **10.0** 에 둔다.
입구에서야 속도를 올리면 슬루레이트(`max_accel` 1.0 m/s²)가 경사로 **위에서**
가속을 시작한다 — 중력을 지고 가속하는 제일 불리한 자리다.

⚠ 10.0 미만으로 내리지 말 것. 그 구간은 먹스에서 teleop 이 자율보다
우선이라 arm 해도 안 먹고, `mission_plan_check.py` 가 실패로 잡는다.

### 11m 로 충분한가 — 실측과 예측

`data/2026-09-17-ramp/ramp_평지기준_2259.csv` (평지, PWM 105 고정):

| 도달 속도 | 걸린 거리 | 걸린 시간 |
|---|---|---|
| 1.11 m/s | 0.7 m | 0.9 s |
| 1.60 m/s | 2.6 m | 2.2 s |
| 2.00 m/s | 6.8 m | 4.4 s |

11m 면 닿는다. 다만 개루프 FF 는 정상편차가 있어 **2.0 을 시켜도 약 1.8 에서
물린다** (FF 식 `38.8·v+17.2` 이 실측보다 기울기가 얕다). 고장이 아니다.

### 예측을 돌린다 — 경사도는 ② 에서 잰 값을 넣는다

```bash
cd /home/han/racing_ws && source install/setup.bash && python3 tools/ramp_profile.py --grade 12.5 --approach 11 --ramp-len 8 --down-len 8 --v-up 2.0 --v-down 1.11 --grade-ff-gain 1.0 --gov-pwm 50
```

실제 제어 코드(`decide_target`·`_grade_pwm`·`_governor_pwm`·`_ff_output`)를
그대로 불러 돌린다. 차량 모델은 위 로그에서 뽑아 **다른 두 로그로
교차검증**했다(평지 +0.8%, 등반 로그 +19.3% — §3-1 의 독립 역산값과 일치).

**`❌ 못 올라감` 이나 `❌ 여유없음` 이 뜨면 파라미터를 고치고 다시 볼 것.**

경사도를 아직 모르면 훑어서 본다:

```bash
cd /home/han/racing_ws && source install/setup.bash && python3 tools/ramp_profile.py --sweep 8,10,12.5,15,19.3 --v-up 2.0 --grade-ff-gain 1.0 --gov-pwm 50
```

### 왜 grade_ff 가 선택이 아닌가 (예측 결과)

| 경사 | grade_ff 켬 등반최저 | 끔 등반최저 | 끔 + 구동 15% 약화 |
|---|---|---|---|
| 10 % | 1.61 m/s | 0.66 m/s | 0.33 m/s |
| 12.5 % | 1.56 m/s | **0.37 m/s** | **0.04 = 못 넘음** |
| 15 % | 1.51 m/s | **못 올라감** | 못 올라감 |
| 19.3 % | 1.24 m/s | **못 올라감** | 못 올라감 |

'구동 15% 약화' 를 같이 보는 이유: 9/17 로그는 같은 PWM 에서 최고속도가
1.4~2.3 m/s 로 흩어졌다(배터리·노면). 한 점이 통과했다고 '된다' 고 쓰면
현장에서 그 흩어짐의 나쁜 쪽에 걸린다.

---

## ⑦ 본 주행에 얹기

⚠ **2026-09-18 이전에는 이게 불가능했다.** `gov_pwm`·`grade_ff_gain` 은
`tools/teleop_drive.launch.py` 에만 있었고, `ramp_up_arm_topic` 은
`control.launch.py` 에 선언만 돼 있고 `bringup` 이 안 넘겼다. 그래서 자율
주행으로는 경사로 기능을 하나도 켤 수 없었다. 지금은 셋 다 이어져 있고,
`tools/test_ramp_launch_wiring.py` 가 그 배선을 지킨다.

```bash
pgrep -f 'install/[a-z_]+/lib/' | xargs -r kill -9
```
```bash
cd /home/han/racing_ws && source install/setup.bash && ros2 launch gps_localization bringup.launch.py control:=true lidar:=true sequencer:=true waypoints:=<경사로 wp> mission_plan:=/home/han/racing_ws/config/ramp_course/mission_plan.ramp.yaml max_speed:=2.0 auto_calib:=true calib_distance:=10.0 ff_mode:=ros ff_min_pwm:=0.0 ramp_up_arm_topic:=/ramp/up_arm v_ramp_up:=2.0 ramp_down_arm_topic:=/ramp/down_arm v_ramp_down:=1.11 grade_ff_gain:=1.0 gov_pwm:=50.0
```

⚠ **소수점을 꼭 찍을 것.** `v_ramp_up:=2` 는 INTEGER 로 들어가 노드가 즉사한다.

시작 로그에서 **네 줄**을 확인한다. 하나라도 없으면 그 기능은 안 켜진 것이다:

| 로그 | 안 나오면 |
|---|---|
| `★ 오르막 구간 속도 켜짐` | `ramp_up_arm_topic`/`v_ramp_up` 이 안 들어갔다 |
| `★ 내리막 구간 속도 켜짐` | `ramp_down_arm_topic`/`v_ramp_down` 이 안 들어갔다 |
| `★ 경사 보상 켜짐` + 영점·부호 | ① 을 안 했거나 `grade_ff_gain` 이 0 이다 |
| `헤딩 초기화(1회성) 시작: 10m` | `auto_calib` 이 꺼져 있다 (사람이 몰아야 한다) |

그리고 주행 중 `/ramp/up_arm` 이 실제로 오는지 본다 — 안 오면 계획의 s 나
`check_course` 다:

```bash
ros2 topic echo /ramp/up_arm
```

---

## 안 되면 — 되돌리는 법

전부 **기본이 꺼짐**이라 인자만 빼면 원래대로 돌아온다.

| 증상 | 조치 |
|---|---|
| 내리막에서 오히려 빨라진다 | **즉시 `grade_ff_gain:=0.0`**. 부호가 반대다 → ② 다시 |
| 오르막에서 못 올라간다 | `grade_ff_gain:=1.0` 으로 올린다. 그래도 안 되면 `max_speed` 를 올려 FF 여유를 준다 |
| 엉뚱한 데서 속도가 바뀐다 | 계획의 s 가 틀렸다 → `enabled:false` 로 내리고 ⑤ 다시 |
| 시퀀서가 죽었다 | 1초 뒤 자동으로 v_max 로 복귀한다 (설계됨) |
| IMU 가 끊긴다 | 0.5초 뒤 보상이 0 이 된다 (설계됨). `tools/imu_link_check.py` 로 케이블 확인 |

---

## 아직 검증 안 된 것 (정직하게)

- **grade_ff 는 실차에서 한 번도 안 돌았다.** 시험 29건(`test_grade_ff`)은
  전부 합성 입력이고, `tools/ramp_profile.py` 의 예측도 **모델**이다.
  물리식·부호·안전장치는 못 박았지만, **IMU 실제 부호는 ②에서 처음 확인된다.**
  예측이 말해 주는 건 "부호만 맞으면 넘는다" 까지다.
- **구간 속도도 실차 미검증.** 시험 21건(`test_ramp_section`)은
  `decide_target()` 을 직접 부른 것이다. 시퀀서 연동(arm 토픽이 실제로
  오는지)은 현장이 처음이다 — ⑦ 의 `ros2 topic echo /ramp/up_arm` 으로 본다.
- **자율(웨이포인트) 경사로 주행 자체가 처음이다.** 어제까지는 teleop
  직진이었다. 캘리브 → 정지 → AUTO 인계 → 가속 → 경사로 순서는 코드상
  맞지만(⑥), 실제로 이어서 돌아간 적은 없다.
- 차량 모델(`ramp_profile.py`)이 안 보는 것: **바퀴 슬립 · 배터리 전압 강하 ·
  조향 부하.** 그래서 '구동 15% 약화' 칸을 같이 본다. 그 칸이 빠듯하면
  예측이 맞아도 현장에서 갈린다.
- 경사로 **정지·출발**(규정 항목, `ramp` 미션)은 이번 작업 범위 밖이다.
  3초 정지 후 재출발, 후방 50cm 이내 — 별도로 검증해야 한다.
- 어제 내리막에서 **정지 후 뒤로 밀렸다**(엔코더 홀드 없음). 경사로 정지·출발
  미션을 켜기 전에 반드시 해결할 것. 규정은 50cm 이상 밀리면 5점이다.

## 우선순위 (9/19)

1. **조향 전원 강하** — VMIN 2038mV, 구동이 아니라 조향이다. S자·굴절에서
   링크가 끊기면 차가 선다. 1분 정지 = **탈락**. 이게 1순위다.
2. `bash tools/use_site.sh yongin` — 원점이 76.8km 어긋나 있다.
3. 미션 s 실측 (위 ⑤ 포함).
4. 경사로 주행 (이 문서).
