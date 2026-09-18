# 무조건 출발 카드 — 2026-09-19

> 급할 때 이것만. 이유·근거는 `HANDOFF_2026-09-19-night.md`.

## 1. 꽂고 확인 (30초)

```bash
ls -l /dev/imu /dev/arduino /dev/ldlidar_front && lsusb | grep -icE "10c4:ea60" && stty -F /dev/imu 921600 raw -echo && timeout 2 head -c 64 /dev/imu | wc -c
```

**심링크 3개 · CP210x 2 · 마지막 64** 이어야 한다.

## 2. 정리

```bash
cd /home/han/racing_ws && bash tools/ros_cleanup.sh
```

`✅ 없음` 확인.

## 3. 출발

```bash
cd /home/han/racing_ws && bash tools/drive_school.sh --speed 1.6 --min-pwm 0 --extra waypoints:=<경로>.yaml --extra ff_mode:=ros --extra ff_static:=46.0 --extra ff_gain:=42.6 --extra ff_breakaway_pwm:=90.0 --extra ff_breakaway_ms:=1200.0 --extra auto_calib_speed:=0.5 --extra calib_distance:=10.0 --extra curvature_gain:=6.0 --extra avoid_steer_rate_deg:=90.0 --extra grade_ff_gain:=1.0 --extra grade_ff_max:=95.0 --extra gov_pwm:=60.0 --extra gov_min_grade:=0.06 --extra gov_gain:=300.0 --extra gov_lead_s:=0.3 --extra gov_deadband:=0.20
```

**`exec` 직전 출력에서 눈으로 확인:**

```
ff_static:=46.0      ← 17.2 면 다른 차다
grade_ff_gain:=1.0   ← 없으면 경사로에서 죽는다
max_speed:=1.6
✅ /scan_front 발행 시작
```

---

# 안 될 때 — 증상별

| 증상 | 원인 | 처방 |
|---|---|---|
| `/dev/imu` 없음 | 안 꽂힘 | 꽂아라. `ttyUSB` 번호는 신경 쓰지 마라(udev 가 시리얼로 잡는다) |
| IMU 바이트 0 | 보드레이트 or 죽은 포트 | **먼저** `stty -F /dev/imu 921600 raw -echo`. 그래도 0 이면 뽑고 **10초 뒤** 재삽입 |
| `RTK 수렴 대기 중` | RTK 미수렴 | 하늘 트인 곳으로. ublox 로그에 `degraded mode` 확인. **무시하지 마라 — 2026-08-24 충돌 원인** |
| `❌ 캘리브 무효` (직진성) | 밀 때 휘었다 | 앞바퀴 중앙(`teleop_keyboard` → `SPACE`), 차 **정중앙 뒤**에서 밀기 |
| `측위 점프 감지` | 너무 빨리 밀었다 | 2 m/s 이하(걷는 속도) |
| 캘리브 시작조차 안 함 | 앞바퀴가 꺾여 있다 | `teleop_keyboard` → `SPACE` |
| 차가 아예 안 움직임 | `ff_static` 이 낮다 | `ff_static:=46.0` 확인. 17.2 면 틀렸다 |
| **경사로에서 서서히 죽음** | **`grade_ff_gain` 누락** | `--extra grade_ff_gain:=1.0 --extra grade_ff_max:=95.0` |
| 라이다 있는데 안 피함 | 드라이버 or scan_topic | `ros2 topic hz /scan_front`. `drive_school.sh` 는 자기가 띄운다 |
| 출발하자마자 영원히 정지 | 라이다가 지면을 때림 | `python3 tools/lidar_mount_check.py` |
| 스택이 시작 거부 | 노드가 이미 돈다 | `bash tools/ros_cleanup.sh` |
| 캘리브가 기어감 | 거버너 헛개입 | `gov_deadband:=0.20` 확인 |
| 경사로에서 1.4 m/s 밑 | 속도가 낮다 | `--speed 1.8` 로 올려라 |

---

# 절대 하지 말 것

- **런치를 Ctrl+C** — 캘리브(`yaw_offset`)가 날아간다. 예선 대기 중에 끄지 마라
- **`gov_min_grade` 올리기** — 게이트가 안 열리면 내리막에서 못 잡아 **탈락**
- **`obstacle_stop_dist` 올리기** — 장애물마다 멀리서 서고 8초 뒤 회피를 포기한다
- **`ff_static`/`ff_gain` 바꾸기** — 양방향 완주한 값이다
- **USB 허브 깊게 쓰기** — 시리얼이 끊긴다
- **`wasd_teleop`** — 먹스를 우회해 `/cmd_vel` 을 두고 싸운다. `teleop_keyboard` 를 써라

---

# 캘리브만 따로 (컨트롤러 없이, 밀어서)

```bash
cd /home/han/racing_ws && source install/setup.bash && ros2 launch gps_localization bringup.launch.py control:=true sequencer:=false lidar:=false rviz:=false auto_calib:=false calib_distance:=8.0 waypoints:=<경로>.yaml max_speed:=1.6 curvature_gain:=6.0 ff_mode:=ros ff_static:=46.0 ff_gain:=42.6 ff_min_pwm:=0.0 ff_breakaway_pwm:=90.0 ff_breakaway_ms:=1200.0 grade_ff_gain:=1.0 grade_ff_max:=95.0 gov_pwm:=60.0 gov_min_grade:=0.06 gov_gain:=300.0 gov_lead_s:=0.3 gov_deadband:=0.20
```

별 터미널:
```bash
cd /home/han/racing_ws && source install/setup.bash && ros2 run velocity_controller teleop_keyboard
```
`SPACE` = 조향 중앙. 그다음 **손으로 8m 곧게** 민다.

**10m 가 안 나오면** `calib_distance:=5.0` (헤딩 오차 0.23° — 충분하다). 3m 밑은 금지.

**캘리브는 코스 밖에서 해도 된다.** `yaw_offset` 은 상수 회전이고 래치로 발행된다.
런치만 안 끄면 차를 옮겨도 산다. 단 **회전시켜 옮기는 건 미검증** — 옮겼으면
`python3 tools/heading_check.py` 로 곡선 포함 20~30m 확인(평균 오차 < 20°).
