# 미션 구현 계획 — 후진주차 + 횡단보도 정지
### (2026-08-23 작성 / 2026-08-24 갱신, 대회 D-1)

> **작성 배경**: 후진주차를 못하면 완주로 인정 안 됨 → 후진주차가 최우선.
> 그 다음 완주 안정화. 현재 맵은 대구권(대회장). 아래 순서를 그대로 따를 것.
> 원칙: **예측 말고 측정. 검증된 완주 코드(pure_pursuit)는 건드리지 않는다.**
>
> **08-24 갱신 요약**: 두 미션의 노드를 **구현·폐루프 검증 완료**했다(아래 8절).
> 남은 것은 전부 **현장 작업**이다 — 대회장에서 좌표를 찍고 실차로 확인하는 것.
> 코드는 더 쓸 게 없다.

## 미션 2 — 횡단보도 정지 (2026-08-24 추가)

- 정지선 **64cm 이내**에 정지 → **3초** 대기 → 다시 직진.
- 정지선 위치도 **대회장에서 미리 웨이포인트로 찍는다**(후진주차와 같은 방식).
- 제어는 이미 있던 것을 쓴다: `longitudinal_controller` 가 `/stop_line_distance`
  하나만 보고 √(2ad) 로 감속하고 0.3m 이하에서 선다. **없던 건 "정지 후 3초 뒤
  재출발" 하나뿐**이라 그 상태기계만 새 노드로 얹었다.
- ⚠ `mission:=true`(신호등)와 **동시에 켜지 말 것** — 둘 다 `/stop_line_distance`
  를 발행해 서로 덮어쓴다. 노드가 기동 시 발행자 수를 세어 경고한다.

---

## 0. 미션 정의 (확정)

- 주차공간 3곳. 대회 때 **1곳을 지정** → 그 자리로 **후진 진입**.
- 자리 위치는 **대회장에서 미리 웨이포인트로 찍을 수 있다**.
- ⇒ 라이다·카메라 불필요. 자리가 고정 좌표이므로 GPS(RTK cm급)만으로 가능.

## 1. 확인된 사실 — 후진 인프라는 이미 존재한다 (코드 실측)

| 계층 | 후진 지원 | 근거 |
|---|---|---|
| 펌웨어 | ✅ | `target_velocity = constrain(v, -3.0, 3.0)`, 모터 pwm<0 방향반전 |
| serial_bridge | ✅ | `/cmd_vel` linear.x 부호 그대로 `VEL:` 로 전달 |
| **vehicle_cmd_mux** | ✅ | `min_speed = -0.6` (후진 한계). teleop 이 자율을 덮어씀 |
| pure_pursuit | ❌ 전진 전용 | reverse 로직 없음 → **건드리지 않고 별도 노드로 우회** |

**설계 핵심**: 주차 노드는 `/teleop/cmd_vel`(Twist)로 명령을 낸다.
- mux 우선순위(E-stop > teleop > 자율) 덕에 자율(pure_pursuit) 명령을 자동으로 덮어쓴다.
- `teleop_timeout = 0.5s` 데드맨 → 주차 노드는 **10~20Hz 로 계속 발행**해야 하며, 노드가
  죽으면 0.5초 안에 자동 정지(안전).
- E-stop 은 여전히 최상위 → 비상정지 그대로 유효.
- heading 캘리브 여부와 무관하게 teleop 은 통과(완주 뒤라 어차피 캘리브 완료 상태).

---

## PHASE 0 — 후진 최소 검증 (벤치, 바퀴 공중) ★가장 먼저

목적: "후진이 물리적으로 되는가"를 측정으로 확정. 이게 안 되면 아래 전부 무의미.
**차량 연결 + 바퀴 공중 필수.**

- **0-1 후진 구동 부호**: 개루프 `PWM:-60` 인가.
  - 바퀴가 **후진 방향**으로 도는가(눈으로).
  - 엔코더 ENC1 이 **증가**하는가(전진이 감소였으므로 후진은 증가여야 함).
  - 계산 속도가 **음수**로 나오는가.
  - → 셋 다 맞으면 후진 부호 정상. 하나라도 어긋나면 기록 후 중단하고 원인부터.
- **0-2 후진 중 조향**: `PWM:-60` + `STEER:+10` / `STEER:-10` 를 번갈아.
  - 조향 ADC 가 목표로 가는가(조향은 구동과 독립이라 될 것이나 확인).
- **0-3 mux 후진 통과**: 노드 띄운 상태에서
  `ros2 topic pub /teleop/cmd_vel geometry_msgs/Twist "{linear:{x:-0.3}}"` →
  `ros2 topic echo /cmd_vel` 에 **linear.x=-0.3** 이 나오는가(min_speed -0.6 이라 통과 기대).

**게이트**: 0-1/0-2/0-3 전부 통과해야 PHASE 1 로. 부분 실패 시 그 지점부터 디버그.

---

## PHASE 1 — 노드 구현 ✅ 완료 (2026-08-24)

### 1-A 주차 궤적 기록 `parking_recorder` ✅
- teleop 으로 접근→후진→정차 를 몰면서 (x, y, yaw, speed, **gear**) 를 기록.
- gear 는 최근 5샘플 평균 속도 부호로 자동 판정(0.1m/s 데드밴드), 실패 시 이동방향 폴백.
- **원점(site_origin)을 파일에 같이 저장**하도록 고쳤다. 기록에 원점이 없으면
  장소가 바뀌었을 때 검증할 방법이 없다(2026-08-17 에 150km 어긋난 전례).
  `parking_node` 가 적재할 때 현재 원점과 대조해 1m 이상 다르면 **거부**한다.

### 1-B 주차 실행 노드 `parking_node` ✅
`src/mission_perception/mission_perception/parking_node.py`
- `/teleop/cmd_vel` 로 20Hz 발행 → mux 우선순위로 자율을 덮어쓴다. **완주 코드 무수정.**
- 궤적을 기어별 구간으로 쪼개고, 구간 끝에서 **완전정지(|v|<0.05) + 1초 대기 후**
  기어를 바꾼다(cusp). 서지 않고 기어를 바꾸면 구동계가 상한다.
- 구간 끝을 향해 √(2ad) 로 감속하고, 가감속·조향 슬루레이트를 건다.
- 안전 중단(ABORT): 궤적에서 1m 이탈 / 전진구간인데 목표점이 뒤 / 그 반대(헤딩 이상).
- 완료·중단 후에도 **0 명령을 계속 발행**한다. 멈추면 0.5초 뒤 mux 데드맨이 풀려
  자율로 되돌아가 차가 다시 출발한다.
- **시작 전에는 아무것도 발행하지 않는다** — `/parking/start` (Bool) 를 받아야 한다.
  자리 번호는 `slot` 파라미터 또는 `/parking/select` (Int32).

#### ★ 후진 조향 부호 — 전진과 **같다** (유도 + 시뮬로 확인)
자전거모델 rear-axle 에서 원점에 접하고 목표점 P(x_ld,y_ld) 를 지나는 원호의
곡률 κ=2·y_ld/ld² 은 P 가 앞이든 뒤든 성립한다. θ̇=v·tanδ/L 에 v<0 을 넣고 적분하면
δ>0(좌타)일 때 차가 **후방-좌측**으로 들어간다. 즉 δ=atan(L·κ) 를 그대로 쓴다.
- 시뮬 검증: 부호 +1 → 오차 14cm 수렴 / 부호 -1 → 1m 이탈(제대로 실패).
- ⚠ 그래도 `steer_sign_reverse` 파라미터로 뒤집을 수 있게 뒀다.
  **PHASE 0-2 벤치 측정에서 반대로 나오면 `-1.0` 을 주면 된다**(예측 금지 원칙).

### 1-C 폐루프 사전검증 ✅ (하드웨어 없이)
```bash
python3 tools/test_parking_geometry.py   # 후진 기하·조향부호 (함수 단위)
python3 tools/test_parking_node.py       # 실제 노드 + 가짜 차량 (상태기계까지)
python3 tools/test_crosswalk_stop.py     # 횡단보도 정지 3초 재출발
```
결과(2026-08-24): 주차 오차 **14cm**, cusp 정지 정상, 완료 후 정지 유지 정상.
횡단보도는 정지선 **33cm 앞** 정지(규정 64cm 이내) + 대기 **3.10초** + 재출발 정상.
실제로 찍은 궤적으로도 돌려볼 수 있다: `test_parking_node.py --file ~/parking_1.yaml`

**게이트**: ✅ 통과. 남은 것은 실차 검증(PHASE 0)과 현장 기록(PHASE 2).

---

## PHASE 2 — 대회장 주차자리 기록 (현장, 대회 당일 아침)

목적: 실제 3개 자리의 진입 궤적을 확보. **현장에서만 가능.**

- 절차(자리마다 반복):
  1. GPS+NTRIP 띄우고 RTK **Fixed** 확인(h_acc < 0.05m).
  2. `parking_recorder` 실행.
  3. 차량을 **접근점 → 후진 진입 → 정차** 로 수동(teleop) 주행하며 기록.
  4. `parking_1/2/3.yaml` 저장.
- ⚠ **원점(site_origin)이 대회장 값인지 먼저 확인**. 원점 틀리면 전부 어긋난다.
- 기록 직후 `tools/list_waypoints.py` 로 위치가 대회장인지, 길이/형태가 맞는지 확인.

**게이트**: 3개 파일이 올바른 위치·형태로 저장됐고, 각각 HIL 로 한 번씩 재현 확인.

### 2-B 횡단보도 정지선 기록 (같은 날 아침, 같은 방식)

⚠ **현재 `~/stop_points.yaml` 은 충주 시절 파일이다**(2026-08-16, 옛 원점 399848/4092209).
대구 경로에서 **149.6km 떨어져 있어** bringup 이 자동으로 비활성화한다. 반드시 새로 찍을 것.

```bash
ros2 run mission_perception stop_point_recorder     # 정지선에 세우고 엔터 → Ctrl-C
```
- **차를 세우고 싶은 자세 그대로 세운 뒤** 찍는다(안테나 위치가 기록된다).
- ⚠ `longitudinal_controller` 는 0.3m 이하에서 하드정지하므로, 정지선 위에 찍으면
  차는 그보다 **약 33cm 앞에서** 선다. 규정 64cm 안이라 그대로 두면 되고,
  더 붙이고 싶으면 `stop_bias` 로 보정한다(양수 = 더 가까이).
- 기록 후 bringup 로그에 `정지지점 N개 로드` 가 뜨는지 확인. `경로에서 멀다` 가
  뜨면 원점이 틀렸거나 다른 장소 파일이다.

---

## PHASE 3 — 완주 ↔ 주차 통합

목적: "완주 후 지정 자리로 후진주차" 를 하나의 흐름으로.

- 완주(pure_pursuit)로 트랙을 돈다 → **주차 접근점 도달** 감지.
- `parking_node` 활성화(주차자리 번호 지정) → teleop 채널로 후진주차 실행.
- 완주 로직은 그대로 두고, **접근점 이후 제어권만 주차 노드로 넘긴다**(teleop 우선순위 활용).
- 실차 저속으로 전체 시나리오 1회 리허설. E-stop 손에 쥐고.

**게이트**: 완주→주차 전 과정이 실차에서 1회 성공.

---

## PHASE 4 — 완주 안정화 (주차 검증 후)

주차가 확보된 뒤에 손대는 잔여 튜닝. 완주 자체를 흔들지 않는 선에서.

- **lookahead 재튜닝**: 2.3m(급커브 이탈) / 1.6m(직진 악화) 사이 → **1.9m** 시험.
  대구 경로 최소R 2.68m 기준으로 로그 보고 판단.
- **직진 차선밟기**: `drive_log.py` 로 RTK Float 비율·헤딩오차 재확인.
  (지난 로그 분석: 헤딩·중앙은 정상, Float 구간에서 오차 급증. Fixed 유지가 관건)
- 속도 상향 필요 시 `MAX_DRIVE_PWM` 을 VMIN 보며 단계적으로.

---

## 절대 규칙 (실수 방지)

1. **pure_pursuit / 완주 로직 수정 금지.** 주차는 teleop 채널로만 얹는다.
2. **후진 부호·조향 방향은 반드시 PHASE 0 측정으로 확정.** 코드 주석·기억으로 넘겨짚지 말 것.
3. **주차 노드는 계속 발행(≥2Hz).** 멈추면 mux 데드맨(0.5s)이 정지시킨다(안전장치이자 함정).
4. **속도는 mux min_speed(-0.6) 안에서** 보수적으로(-0.3 권장).
5. **원점(site_origin) 확인 먼저.** 장소 바뀌면 이것부터. 틀리면 150km 어긋난다.
6. **매 실차 단계 E-stop 손에.** 무부하(공중) 테스트로 로직 먼저 잡고 실차.
7. 보드 교체 시 [[board-variants-racing-ws]] 절차대로 재측정(조향핀·엔코더·극성).

---

## 8. 대회 당일 실행 순서 (그대로 따라할 것)

### ① 아침 — 좌표 찍기 (현장에서만 가능, 제일 먼저)
```bash
bash tools/ros_cleanup.sh
ros2 launch gps_localization bringup.launch.py control:=true rviz:=true
```
RTK **Fixed** 확인(h_acc < 5cm) 후, teleop 으로 몰면서:
```bash
# 주차 3자리 — 자리마다 접근→후진→정차 를 수동 주행
ros2 run mission_perception parking_recorder --ros-args -p slot:=1
ros2 run mission_perception parking_recorder --ros-args -p slot:=2
ros2 run mission_perception parking_recorder --ros-args -p slot:=3
# 횡단보도 정지선 — 세우고 엔터
ros2 run mission_perception stop_point_recorder
```
찍은 즉시 시뮬로 재현되는지 확인(현장에서 30초면 된다):
```bash
python3 tools/test_parking_node.py --file ~/parking_1.yaml
```

### ② 주행 — 지정 자리를 알고 나서
`parking_slot` 에 **지정받은 번호**를 넣는다. 이것만 바꾸면 된다.
```bash
bash tools/ros_cleanup.sh
# 터미널1
ros2 launch gps_localization bringup.launch.py \
    control:=true auto_calib:=true rviz:=false max_speed:=1.0 \
    crosswalk:=true parking:=true parking_slot:=1
# 터미널2 — 로그 (★ 반드시 같이 켤 것)
python3 tools/drive_log.py
```
- 횡단보도는 **자동**이다(정지선 앞 정지 → 3초 → 재출발).
- 주차는 완주 후 **사람이 트리거**한다. 접근점 근처에서:
```bash
ros2 topic pub --once /parking/start std_msgs/Bool "{data: true}"
```
  자리를 바꿔야 하면 시작 전에:
```bash
ros2 topic pub --once /parking/select std_msgs/Int32 "{data: 2}"
```
- 상태 확인: `ros2 topic echo /parking/state` / `/crosswalk/state`
- ⚠ `mission:=true`(신호등)는 `crosswalk:=true` 와 **같이 켜지 말 것**.

---

## 현재 상태 스냅샷 (이어받는 세션용)

- 맵: 대구권(원점 477800.0/3964400.0), 경로 378점/188m/최소R 2.68m, 못도는구간 0.
- 펌웨어: A15 + SPI엔코더(CS22/23) + NO_ENCODER 0 + 좌26.2/우20.6 + center412.
  `MAX_DRIVE_PWM` 은 최근 150 으로 설정(전압여유). 실차 속도는 max_speed 로 제어.
- lookahead: 현재 **1.6m**(충주에서 직진 악화 관측 → 대구선 1.9m 재시험 권장).
- 완주: HIL 2회 성공, 실차 부분주행 성공(엔코더·헤딩·조향 정상, RTK Fixed 94.8%).
- **후진주차·횡단보도: 노드 구현 + 폐루프 검증 완료(08-24). 실차 미검증.**
  - 코드는 다 됐다. 남은 건 **현장 좌표 기록**과 **실차 확인**뿐이다.
  - `~/parking_1/2/3.yaml` **없음** — 대회장에서 찍어야 함.
  - `~/stop_points.yaml` 은 **충주 시절 파일** — 대구에서 다시 찍어야 함.
  - PHASE 0(후진 벤치 검증) 미실시 → 실차 붙이면 제일 먼저 할 것.
- 미해결: 완주 lookahead 튜닝, 직진 Float 구간 오차.
