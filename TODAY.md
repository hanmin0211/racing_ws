# 오늘 현장 순서 (2026-08-17) — 목표: **맵 완주**

> 원칙: **완주를 먼저 확보하고, 그다음 속도를 올린다.**
> 지금 설정으로도 완주는 된다(0.33 m/s, 약 8.6분). FF 작업은 속도를 위한 것이지
> 완주를 위한 게 아니다. 필수 목표를 미검증 변경에 걸지 않는다.

---

## 0. 도착 직후 — 전원과 연결 (5분)

**순서를 지킬 것.** 반대로 하면 엔코더가 0을 센다.

```
1) 차량 배터리 ON
2) 그다음 Arduino USB 연결
3) GPS(u-blox) USB 연결
4) IMU USB 연결
```

확인:
```bash
lsusb | grep -iE "u-blox|2341"     # GPS + Arduino 둘 다 보여야 함
ls -l /dev/imu                      # 심볼릭 링크 존재
```

- `/dev/imu` 가 없으면 → udev 규칙 확인 (HANDOFF 2장)
- Arduino 안 보이면 → 케이블/허브 교체

**배터리 완충 확인.** 오늘은 랩(8.6분) + 스윕 + 두 번째 랩이라 부하가 크다.
전에 방전 때문에 조향 breakaway 실패를 하드웨어 고장으로 오진한 적 있다.

---

## 1. RTK 확인 (5분)

### 터미널 A — RTK
```bash
cd ~/racing_ws && source install/setup.bash && bash tools/ros_cleanup.sh && ros2 launch ngii_ntrip ngii_rtk.launch.py
```
`[NTRIP 연결됨] RTCM ...프레임 발행` 이 반복되면 정상.

### 터미널 B — 품질 점검
```bash
cd ~/racing_ws && source install/setup.bash && python3 tools/gps_check.py
```

```
  FIXED ✅   σ=  0.6cm | 10.0Hz | RTCM  4.0Hz | x=  -18.19 y=  118.26 | ...
```

| 상태 | 조치 |
|---|---|
| `FIXED ✅` σ ≤ 5cm | 진행 |
| `FLOAT ⚠` | 1~2분 더 대기. 하늘 트인 곳으로 이동 |
| `단독측위 ❌` / RTCM 0Hz | NTRIP 문제 → `Ctrl-C` 후 `bash tools/ros_cleanup.sh` 하고 재시작 (중복 클라이언트면 401) |

**여기서 `Ctrl-C` 로 터미널 A 를 끈다.** 다음 단계의 bringup 이 RTK 를 포함하고 있어서,
켜둔 채로 bringup 을 띄우면 NTRIP 이 2개가 되어 401 이 난다.

---

## 2. ★ 첫 랩 — 최우선 (15분)

### 터미널 A
```bash
cd ~/racing_ws && source install/setup.bash && bash tools/ros_cleanup.sh && ros2 launch gps_localization bringup.launch.py control:=true auto_calib:=true rviz:=false
```

`rviz:=false` 인 이유: rviz2 가 CPU 112% 를 먹어 토픽 조회가 간헐 실패한 적 있다.
경로를 보고 싶으면 랩이 끝난 뒤에 켤 것.

### 진행 흐름

```
① RTK Fixed 대기
② 5초 카운트다운
③ 차가 스스로 0.3 m/s 로 10m 직진      ← ⚠ 앞을 비우고 E-STOP 쥘 것
④ 정지 → 직진성 검증
⑤ 통과 → yaw_offset 발행
⑥ 먹스: "헤딩 캘리브 완료 — 자율 허용"
⑦ 모드: AUTO → 경로 추종 시작
```

**⚠ ③은 헤딩을 모르는 개루프 직진이다.** 차가 지금 향한 쪽으로 그냥 간다.
차를 경로 시작 방향으로 대충 맞춰 놓고 시작할 것. 직선 12m 필요.

### 확인할 로그

```
[gps_heading_init]  헤딩 오프셋 ...°  ← 캘리브 성공
[vehicle_cmd_mux]   헤딩 캘리브 완료 (...°) — 자율 허용
[vehicle_cmd_mux]   모드: AUTO         ← 이게 떠야 달린다
```

### 차가 안 갈 때 — 로그로 원인 판별

| 로그 | 원인 | 조치 |
|---|---|---|
| `모드: STOP(헤딩 캘리브 전)` | 캘리브 미완료 | ③~⑤ 다시. 거부됐으면 3초 뒤 자동 재시도 |
| `모드: STOP(입력끊김)` | `/target_speed` 또는 `/steering_cmd` 없음 | 제어 노드가 떴는지 확인 |
| `⚠ 경로 ...s 끊김 — 완주 신호 없음` | GPS/IMU 끊김 | `gps_check` 로 RTK 확인, IMU 재연결 |
| `정지: lookahead가 차량 뒤` | 헤딩이 뒤집혔을 가능성 | `invert_imu_yaw:=true` 로 재시작 |
| `직진성 검증 실패 (편차 ...°)` | 캘리브 중 휘었다 | 자동 재시도됨. 계속 실패하면 더 평탄한 곳으로 |

### 완주 판정

```
[local_sliding_window_node]  ★ 경로 완주 (남은 거리 ...m) → 정지 신호 발행
[longitudinal_controller]    🏁 완주 신호 수신 — 감속 정지합니다 (고장 아님)
[local_pure_pursuit]         정지: 🏁 완주 (정상 종료)
```

**`🏁` 가 뜨면 완주다.** `⚠` 는 고장이다. 이 구분을 위해 넣은 것이니 헷갈리지 말 것.

### 캘리브 보너스 — 놓치지 말 것

자동 직진은 조향 0°를 명령하므로, 그래도 호를 그렸다면 **기계의 계통 오차**다.
노드가 활꼴 높이로 `STEER_CENTER` 오차를 역산해 권장값을 찍는다:

```
[gps_heading_init] 직진 편차 좌 0.24m → 조향 바이어스 +1.8°, 권장 STEER_CENTER 386
```

이 값이 나오면 **기록해 둘 것.** 펌웨어에 반영하면 랩 품질이 올라간다.

---

## 3. FF 스윕 — 완주 확보 후에만 (15분)

### ⚠ 반드시 2단계 스택을 완전히 종료할 것

제어 체인이 `VEL:` 을 보내면 개루프가 **즉시 해제된다.** 동시 실행하면 스윕이 실패한다.

```bash
bash ~/racing_ws/tools/ros_cleanup.sh
```

### 터미널 A — RTK 만
```bash
cd ~/racing_ws && source install/setup.bash && ros2 launch ngii_ntrip ngii_rtk.launch.py
```

### 터미널 B — serial_bridge
```bash
cd ~/racing_ws && source install/setup.bash && ros2 run velocity_controller serial_bridge
```

### 터미널 C — 스윕
```bash
cd ~/racing_ws && source install/setup.bash && ros2 run velocity_controller ff_sweep --ros-args -p max_speed:=1.0 -p ramp_rate:=4.0
```

| 조건 | |
|---|---|
| 장소 | 코스 바닥면 직선 (x −37 ~ +8, 약 45m) |
| 필요 공간 | **직선 40m** (주행 23m + 감속 여유) |
| 소요 | 55초 |
| **최고 속도** | **0.91 m/s — 지금의 약 3배** |

⚠ **바퀴 네 개 접지 필수.** 뜬 상태면 1m 지점에서 자동 중단된다.
⚠ 개루프라 PID 보정이 없다. **E-stop 쥐고, 앞 비우고.**

### 결과 판정

```
  이동거리 대조 : 엔코더 23.1m / RTK 22.8m       ← 비슷해야 정상
  ★ STATIC_FF        = xx.x
  ★ VELOCITY_FF_GAIN = xxx.x                     ← 150~300 이면 정상
  ★ MAX_DRIVE_PWM 권장 = xxx
  ★ PID 권장 : velocity_kp = xx, velocity_ki = xx
```

- **기울기가 60 미만이면 경고가 뜬다 → 그 값은 절대 펌웨어에 넣지 말 것** (바퀴가 떴다는 뜻)
- **중단 메시지가 뜨면** 결과가 폐기된다. 접지 확인 후 재실행

**결과 전체를 AI에게 붙여넣을 것.** 펌웨어 값을 확정해서 플래시한다.

---

## 4. 플래시 + 두 번째 랩 (20분)

FF 반영 후 2장을 반복. 목표 **4분대**.

```bash
cd ~/racing_ws && source install/setup.bash && bash tools/ros_cleanup.sh && ros2 launch gps_localization bringup.launch.py control:=true auto_calib:=true rviz:=false
```

---

## 5. 정지지점 기록 — 시간 남으면 (10분)

**현장에서만 가능하다.** 기존 `~/stop_points.yaml` 은 충주 좌표라 자동 비활성화돼 있다.

```bash
cd ~/racing_ws && source install/setup.bash && ros2 run mission_perception stop_point_recorder
```
차를 정지선에 세우고 **엔터**. 여러 개면 반복. 끝나면 `Ctrl-C`.

---

## 상시 참고

### 뭔가 이상하면 제일 먼저
```bash
bash ~/racing_ws/tools/ros_cleanup.sh
```
`pkill -f ros2` 는 런처만 죽인다. 고아 노드가 쌓이면 NTRIP 401 · USB 충돌 ·
yaw_offset 덮어쓰기가 난다.

### 수동 조작이 필요하면 (별도 터미널)
```bash
cd ~/racing_ws && source install/setup.bash && ros2 run velocity_controller teleop_keyboard
```
※ `teleop:=true` 런치 인자는 쓰지 말 것 — xterm 이 없어 키 입력을 못 받는다.
※ **한글 입력 상태면 W 가 ㅈ 으로 들어가 차가 안 움직인다.** 영문 전환 확인.

### E-stop
```bash
ros2 topic pub --once /e_stop std_msgs/msg/Bool "{data: true}"
```

### 시간 배분 (총 약 70분)
```
0. 연결        5분
1. RTK 확인    5분
2. 첫 랩      15분   ← 필수
3. FF 스윕    15분
4. 두 번째 랩 20분
5. 정지지점   10분
```

**2번까지 끝나면 오늘 목표는 달성이다.** 3~5는 보너스.
