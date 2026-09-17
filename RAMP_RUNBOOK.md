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

## ⑤ 구간 s 실측 (10분)

경사로 시작/정상부/끝에 차를 세우고 각각:

```bash
cd /home/han/racing_ws && source install/setup.bash && python3 tools/mission_s.py --waypoints config/yongin_2026-09-05/wp_yongin_drive_0.5.yaml --live --name ramp_up
```

`config/mission_plan.yaml` 의 `ramp_up` / `ramp_down` 에 채운다.

- `ramp_up`: 오르막 진입 **3m 앞** → 정상부
- `ramp_down`: 정상부 → 내리막 끝 **3m 뒤**

채운 뒤 반드시:

```bash
cd /home/han/racing_ws && source install/setup.bash && python3 tools/mission_plan_check.py config/mission_plan.yaml --include-disabled
```

그 다음 `enabled: true` 로 바꾼다. **순서를 바꾸지 말 것** — s 를 모르는 채로
켜는 것이 이 대회에서 가장 비싼 실수다(엉뚱한 데서 실행 = 이탈 = 탈락).

## ⑥ 본 주행에 얹기

```bash
ros2 launch pure_pursuit_pkg control.launch.py ramp_up_arm_topic:=/ramp/up_arm v_ramp_up:=1.6 ramp_down_arm_topic:=/ramp/down_arm v_ramp_down:=1.11
```

⚠ **소수점을 꼭 찍을 것.** `v_ramp_up:=2` 는 INTEGER 로 들어가 노드가 즉사한다.

시작 로그에 `★ 오르막 구간 속도 켜짐` / `★ 내리막 구간 속도 켜짐` 이
찍히는지 확인한다. 안 찍히면 파라미터가 안 들어간 것이다.

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

- **grade_ff 는 실차에서 한 번도 안 돌았다.** 시험 28건은 전부 합성 입력이다.
  물리식·부호·안전장치는 못 박았지만, IMU 실제 부호는 ②에서 처음 확인된다.
- **구간 속도도 실차 미검증.** 시험 23건은 `decide_target()` 을 직접 부른 것이다.
  시퀀서와의 연동(arm 토픽이 실제로 오는지)은 현장이 처음이다.
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
