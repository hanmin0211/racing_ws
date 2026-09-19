# 무조건 출발 카드 — 용인 (2026-09-19/20)

> 급할 때 이것만. 근거는 `HANDOFF_2026-09-19-night.md`.
> **이 문서는 용인 기준이다.** 학교로 돌아가면 §마지막 참고.

---

## 0. ★ 원점이 용인인지 먼저 확인 — 이게 틀리면 차가 절대 안 움직인다

```bash
cd /home/han/racing_ws && bash tools/use_site.sh
```

`site: "용인 (2026-09-05)"` · `origin_x: 332200.0` 이어야 한다. 아니면:

```bash
cd /home/han/racing_ws && bash tools/use_site.sh yongin
```

**왜**: `global_path_publisher` 가 웨이포인트의 원점 스탬프를 `config/site_origin.yaml`
과 대조해 **1km 이상 차이 나면 전역 경로를 발행하지 않는다**(`origin_check` 기본 strict).
충주 ↔ 용인은 **76.8km** 다. 원점이 충주로 남아 있으면 용인 경로가 통째로 거부되고
**차는 캘리브까지 하고도 한 발짝도 안 간다.**

## 1. 사전점검 (차 연결 후)

```bash
cd /home/han/racing_ws && source install/setup.bash && python3 tools/preflight.py --waypoints config/yongin_2026-09-05/wp_yongin_drive_0.5.yaml --calib-distance 10.0 --skip-parking
```

**`치명적 문제 없음`** 이어야 한다. 2026-09-19 오전 기준 상태:

```
✅ 경로: 원점 일치 (용인)          1298점 · 648.3m · 최소R 2.71m (타각 16.2°/18°)
✅ run-up 직선 — 여기서 캘리브하면 안전   (편차 3.2°, 이탈 0.08m)
⚠  빠듯한 커브 19개 (R<3.5m)      → max_speed 1.6 이면 횡가속 0.60 m/s² 로 안전
⚠  정지점 0개                      → 의도된 것. 용인 정지선은 아직 미측정
⚠  활성 미션 0개                   → 트랙 주행 테스트는 이게 맞다
```

## 2. 장치 확인 (30초)

```bash
ls -l /dev/imu /dev/arduino /dev/ldlidar_front && lsusb | grep -icE "10c4:ea60" && stty -F /dev/imu 921600 raw -echo && timeout 2 head -c 64 /dev/imu | wc -c
```

**심링크 3개 · CP210x 2 · 마지막 64**

## 3. 정리

```bash
cd /home/han/racing_ws && bash tools/ros_cleanup.sh
```

`✅ 없음` 확인.

## 4. ⚠ 라이다 마운트 (주행 전 30초, 건너뛰지 마라)

```bash
cd /home/han/racing_ws && source install/setup.bash && ros2 launch lidar_clustering lidar_dual.launch.py rear:=false
```
별 터미널:
```bash
cd /home/han/racing_ws && source install/setup.bash && python3 tools/lidar_mount_check.py
```

지면을 때리면 전방이 통째로 장애물이 되어 **차가 영원히 선다**(2회 재발).

## 5. ★ 출발 — 용인 트랙 주행

```bash
cd /home/han/racing_ws && bash tools/drive_school.sh --speed 1.6 --min-pwm 0 --waypoints /home/han/racing_ws/config/yongin_2026-09-05/wp_yongin_drive_0.5.yaml --extra ff_mode:=ros --extra ff_static:=46.0 --extra ff_gain:=42.6 --extra ff_breakaway_pwm:=90.0 --extra ff_breakaway_ms:=1200.0 --extra auto_calib_speed:=0.5 --extra calib_distance:=10.0 --extra curvature_gain:=6.0 --extra avoid_steer_rate_deg:=90.0 --extra grade_ff_gain:=1.0 --extra grade_ff_max:=95.0 --extra gov_pwm:=60.0 --extra gov_min_grade:=0.06 --extra gov_gain:=300.0 --extra gov_lead_s:=0.3 --extra gov_deadband:=0.20
```

기록 (별 터미널):
```bash
cd /home/han/racing_ws && source install/setup.bash && python3 tools/drive_record.py --yaw-offset 180 --out /tmp/yongin_$(date +%H%M).csv
```

**`exec` 직전 출력에서 눈으로 확인:**

```
waypoints:=...yongin_2026-09-05/wp_yongin_drive_0.5.yaml
ff_static:=46.0        ← 17.2 면 다른 차다
grade_ff_gain:=1.0     ← 없으면 경사로에서 죽는다
max_speed:=1.6
✅ /scan_front 발행 시작
```

런치 로그에서:
```
전역 경로 로드 완료: ... (총 1298개 점, 648.3m)   ← 안 뜨면 원점 문제(§0)
```

## 6. 예상 — 이 숫자에서 벗어나면 멈춰라

```
완주 시간    약 5.1분 (648.3m, 평균 2.13 m/s)  · 8분 예산에 2.9분 여유
평지 속도    약 2.4 m/s   (명령 1.6)
빠듯한 커브  약 1.42 m/s  (명령 0.57, 바닥에 걸린다) · 횡가속 0.60 m/s²
경사로       약 1.8 m/s   (용인 12.5% 기준)
종점         1~2m 지나서 정지 (역토크 없음. 정상이다)
```

---

# 안 될 때 — 증상별

| 증상 | 원인 | 처방 |
|---|---|---|
| **캘리브은 되는데 차가 안 감** | **원점이 충주** | `bash tools/use_site.sh yongin` (§0) |
| `전역 경로 로드 완료` 가 안 뜸 | 같음 | 같음 |
| `/dev/imu` 없음 | 안 꽂힘 | 꽂아라. `ttyUSB` 번호는 무시(udev 가 시리얼로 잡는다) |
| IMU 바이트 0 | 보드레이트 or 죽은 포트 | **먼저** `stty -F /dev/imu 921600 raw -echo`. 그래도 0 이면 뽑고 **10초 뒤** 재삽입 |
| `RTK 수렴 대기 중` | RTK 미수렴 | 하늘 트인 곳. ublox 로그 `degraded mode` 확인. **무시 금지 — 8/24 충돌 원인** |
| `❌ 캘리브 무효`(직진성) | 휘었다 | 앞바퀴 중앙(`teleop_keyboard`→`SPACE`), 차 **정중앙 뒤**에서 밀기 |
| `측위 점프 감지` | 너무 빨리 밀었다 | 2 m/s 이하 |
| 캘리브 시작조차 안 함 | 앞바퀴가 꺾여 있다 | `teleop_keyboard` → `SPACE` |
| 차가 아예 안 움직임 | `ff_static` 이 낮다 | `ff_static:=46.0` 확인. 17.2 면 틀렸다 |
| **경사로에서 서서히 죽음** | **`grade_ff_gain` 누락** | `--extra grade_ff_gain:=1.0 --extra grade_ff_max:=95.0` |
| 라이다 있는데 안 피함 | 드라이버 or scan_topic | `ros2 topic hz /scan_front` |
| 출발하자마자 영원히 정지 | 라이다가 지면을 때림 | `python3 tools/lidar_mount_check.py` |
| 스택이 시작 거부 | 노드가 이미 돈다 | `bash tools/ros_cleanup.sh` |
| 캘리브가 기어감 | 거버너 헛개입 | `gov_deadband:=0.20` 확인 |
| **`❌ 캘리브 무효: 10m 를 N초 만에`** | **`auto_calib_speed` 가 전압 때문에 올라갔다** | `--extra auto_calib_speed:=0.5` 가 있는지 확인. 없으면 스크립트가 전압 4700mV 밑에서 최대 1.2 까지 올린다 → 게이트 여유 +49%→−3% |
| 경사로에서 1.4 m/s 밑 | 속도가 낮다 | `--speed 1.8` |
| 커브에서 크게 벌어짐 | 속도가 높다 | `--speed 1.4`. 8분 예산에 여유가 있다(1.2 여도 6.0분) |

---

# 절대 하지 말 것

- **런치를 Ctrl+C** — 캘리브(`yaw_offset`)가 날아간다. 예선 대기 중에 끄지 마라
- **`gov_min_grade` 올리기** — 게이트가 안 열리면 내리막에서 못 잡아 **탈락**
- **`obstacle_stop_dist` 올리기** — 장애물마다 멀리서 서고 8초 뒤 회피를 포기한다
- **`ff_static`/`ff_gain` 바꾸기** — 양방향 완주한 값이다
- **USB 허브 깊게 쓰기** — 시리얼이 끊긴다
- **`wasd_teleop`** — 먹스를 우회해 `/cmd_vel` 을 두고 싸운다. `teleop_keyboard` 를 써라
- **`--extra waypoints:=`** — 인자가 중복된다. **`--waypoints` 를 써라**(2026-09-19 추가)
- **`mission:=true` 와 `crosswalk:=true` 동시에** — 둘 다 `/stop_line_distance` 를 덮어쓴다

---

# 캘리브만 따로 (컨트롤러 없이, 밀어서)

```bash
cd /home/han/racing_ws && source install/setup.bash && ros2 launch gps_localization bringup.launch.py control:=true sequencer:=false lidar:=false rviz:=false auto_calib:=false calib_distance:=8.0 waypoints:=/home/han/racing_ws/config/yongin_2026-09-05/wp_yongin_drive_0.5.yaml max_speed:=1.6 curvature_gain:=6.0 ff_mode:=ros ff_static:=46.0 ff_gain:=42.6 ff_min_pwm:=0.0 ff_breakaway_pwm:=90.0 ff_breakaway_ms:=1200.0 grade_ff_gain:=1.0 grade_ff_max:=95.0 gov_pwm:=60.0 gov_min_grade:=0.06 gov_gain:=300.0 gov_lead_s:=0.3 gov_deadband:=0.20
```

별 터미널:
```bash
cd /home/han/racing_ws && source install/setup.bash && ros2 run velocity_controller teleop_keyboard
```
`SPACE` = 조향 중앙. 그다음 **손으로 8m 곧게** 민다.

**10m 가 안 나오면** `calib_distance:=5.0` (헤딩 오차 0.23°). 3m 밑은 금지. **5m 는 미검증.**

**캘리브는 코스 밖에서 해도 된다.** `yaw_offset` 은 상수 회전이고 래치로 발행된다.
런치만 안 끄면 차를 옮겨도 산다. 단 **회전시켜 옮기는 건 미검증** —
`python3 tools/heading_check.py` 로 곡선 포함 20~30m 확인(평균 오차 < 20°).

---

# 학교(충주)로 돌아갈 때

```bash
cd /home/han/racing_ws && bash tools/use_site.sh chungju
```
경로도 `config/chungju_school/...` 로 바꿀 것. **원점만 바꾸고 경로를 안 바꾸면
(또는 그 반대면) 발행이 거부된다** — 그게 안전장치다.

---

# 구간 감속 (굴절코스·S자를 5 km/h 로 고정)

**필요할 때만.** 트랙 주행이 먼저 되고 나서 얹어라.

급커브 자체는 곡률 제한기가 이미 5.0 km/h 로 누른다. 문제는 **커브 사이 직선에서
8.9 km/h 로 튀는 것**이다. 구간 전체를 고정하려면 구간속도를 쓴다.

```
                 지금              v_ramp 0.55
굴절코스     5.0~8.9 km/h  →   4.8~5.0 km/h
S자+장애물   5.2~8.9 km/h  →   4.8~5.0 km/h
나머지 구간  5.1~8.9 km/h  →   그대로 (안 건드린다)
완주         5.1분         →   5.6분  (+33초, 8분에 2.4분 여유)
```

⚠ **이 차의 최저 속도는 4.8 km/h 다.** 4 km/h 는 PWM 58 이라 정지마찰을 못 넘어
차가 선다 — 불가능하다.

**순서**

1. s 를 실측한다 (추정값으로 켜지 말 것)
```bash
cd /home/han/racing_ws && source install/setup.bash && python3 tools/mission_s.py --waypoints config/yongin_2026-09-05/wp_yongin_drive_0.5.yaml --live
```
2. `config/mission_plan.yaml` 의 `ramp_up`(굴절) · `ramp_down`(S자) 에 s 를 넣고
   `enabled: true` 로 바꾼다
3. 주행 명령에 인자 5개를 더한다

```
--extra sequencer:=true --extra ramp_up_arm_topic:=/ramp/up_arm --extra ramp_down_arm_topic:=/ramp/down_arm --extra v_ramp_up:=0.55 --extra v_ramp_down:=0.55
```

⚠ **`0.55` 는 소수점을 꼭 찍어라.** `v_ramp_up:=1` 처럼 정수면 노드가 즉사한다.

**안전장치 (실제 노드 코드로 확인함)**
- 기준속도만 바꾼다 — 곡률·정지선·장애물 감속은 그대로 걸린다
- 시퀀서가 죽으면 1초 뒤 `v_max` 로 복귀한다
- s 가 틀려도 엉뚱한 데서 느려질 뿐이다(안전 문제 아님)
