# 구동 캘리브레이션 절차 (야외 1회 세션)

> 목적: 엔코더 스케일 확정 → FF 재식별 → MAX_DRIVE_PWM 상향 → PID 재튜닝
> 소요: 약 1~1.5시간. **이 순서를 지켜야 한다** (앞 단계가 틀리면 뒤가 전부 헛수고)

---

## 0. 준비

**장소**
- 개활지(RTK Fixed 필요), **직선 30m 이상** 확보
- 바퀴 접지, 진행 방향 전방이 트여 있을 것


> ## ⚠️ 전원 순서 (반드시 지킬 것)
> **① 차량 배터리 ON  →  ② 아두이노 USB 연결 / serial_bridge 실행**
>
> 엔코더는 SPI 카운터 칩이고, 설정은 아두이노 `setup()` 의 `initEncoders()` 에서
> **부팅 때 한 번만** 들어간다. 배터리가 꺼진 상태로 아두이노가 부팅되면 칩에
> 전원이 없어 설정이 안 들어가고, **나중에 배터리를 켜도 카운트하지 않는다.**
> (2026-08-15 현장: 83.6m 주행에 엔코더 2카운트 → 캘리브 결과가 통째로 무의미)
>
> 순서가 꼬였다면 **배터리를 켠 뒤 serial_bridge 를 재실행**하면 된다
> (포트를 열 때 아두이노가 자동 리셋되어 initEncoders 가 다시 실행됨).
> 확인: `ros2 topic echo /encoder_count` 하고 1m 밀었을 때 수백 단위로 변해야 정상.

**차량**
- 배터리 **완충** (방전되면 토크 부족 + 전압 붕괴로 결과가 왜곡된다)
- 아두이노 USB는 허브 말고 **PC 직결** 권장

**빌드**
```bash
cd /home/han/racing_ws && colcon build && source install/setup.bash
```

**RTK 확인** (터미널 1)
```bash
NGII_PW=ngii ros2 launch ngii_ntrip ngii_rtk.launch.py
```
```bash
ros2 topic echo /ubx_nav_status --field carr_soln.status   # 2 = Fixed
```
> ⚠️ Fixed(2) 아니면 진행하지 말 것. 엔코더 스케일 기준이 무너진다.

**로컬라이제이션** (터미널 2)
```bash
ros2 launch gps_localization bringup.launch.py rviz:=false
```
→ 10m 직진해서 헤딩 초기화 완료 확인 (직진성 검증이 들어가 있어 휘면 거부됨)

---

## 1단계: 엔코더 스케일 검증 ★ 가장 중요

**터미널 3**
```bash
ros2 run velocity_controller serial_bridge
```

**터미널 4**
```bash
ros2 run velocity_controller encoder_calib
```

**주행**: 차량을 **직진**으로 15~20m. teleop으로 몰든 손으로 밀든 무관
(밀어도 엔코더와 RTK 둘 다 움직이므로 측정된다. 오히려 더 안전하다.)

```bash
# teleop으로 몰 경우 (터미널 5)
ros2 run velocity_controller teleop_keyboard
```

**결과**: `Ctrl-C` 하면 보정계수와 적용할 값이 출력된다.

```
★ 보정계수 = 1.0xxx
   wheel_radius : 0.13 → 0.1xxx
```

**적용**: `arduino/henes_firmware/henes_firmware.ino` 에서 `wheel_radius` 수정 후 플래시
```bash
arduino --upload --board arduino:avr:mega:cpu=atmega2560 --port /dev/ttyACM0 \
  arduino/henes_firmware/henes_firmware.ino
```

> 검증: 다시 encoder_calib 돌려서 보정계수가 **1.00 ± 0.02** 나오면 통과

---

## 2단계: FF 재식별

**터미널 3** (serial_bridge 계속 실행)

**터미널 4**
```bash
ros2 run velocity_controller ff_sweep --ros-args -p max_speed:=1.2 -p ramp_rate:=4.0
```

PWM이 초당 4씩 천천히 올라가며 차량이 서서히 가속한다.
목표 속도(1.2 m/s)에 닿으면 **자동 정지**한다.

- 직선 30m를 다 쓰면 `Ctrl-C` → 그때까지 데이터로 결과 산출
- 여러 번 나눠 해도 되지만, 한 번에 끊김 없이 받는 게 정확하다

**결과**:
```
★ STATIC_FF        = xx.x   (현재 35.0)
★ VELOCITY_FF_GAIN = xx.x   (현재 60.0)
★ MAX_DRIVE_PWM 권장 = xxx  (현재 80)
```

**적용**: 펌웨어의 `STATIC_FF`, `VELOCITY_FF_GAIN`, `MAX_DRIVE_PWM` 수정 후 플래시

---

## 3단계: PID 재튜닝

FF가 정확해지면 PID는 작은 오차만 보정하면 되므로 **게인을 낮춰야 한다.**
현재 `kp=30, ki=24` 는 틀린 FF를 억지로 메우던 값이라 그대로 두면 진동한다.

**시작값 권장**: `kp=8, ki=4, kd=0.2`

**검증 주행**
```bash
ros2 run velocity_controller teleop_keyboard   # 속도 0.5 → 1.0 단계별
```
```bash
ros2 topic echo /current_speed      # 명령 대비 추종 확인
```

판정 기준:
- 오버슈트 < 15%
- 정상상태 오차 < 0.05 m/s
- 진동(헌팅) 없을 것

진동하면 `kp` 내리고, 느리면 `kp` 올린다. 정상상태 오차가 남으면 `ki` 올린다.

---

## 4단계: 통합 확인

```bash
ros2 launch gps_localization bringup.launch.py control:=true max_speed:=0.6
```
- 먼저 **낮은 속도(0.6)** 로 경로 추종 확인
- `/steering_error` 로 조향 추종 감시
```bash
ros2 topic echo /steering_error
```
- 문제없으면 max_speed 단계적 상향

---

## 안전 수칙

- **항상 E-stop 준비**: teleop 창에서 `E` 키 (또는 `SPACE`)
- 사람이 차량 전방에 서지 말 것
- 이상하면 즉시 `Ctrl-C` — 모든 노드가 워치독으로 0.5초 내 정지한다
- 배터리 전압이 떨어지면 결과가 왜곡되니, 중간에 이상하면 배터리부터 확인

## 문제 해결

| 증상 | 확인 |
|---|---|
| encoder_calib 데이터 없음 | `/odometry/filtered`, `/encoder_count` 나오는지 |
| 보정계수가 1에서 크게 벗어남 | 직진 주행이었는지(궤적 길이 vs 직선거리 경고 확인) |
| ff_sweep 샘플 부족 | 실제로 움직였는지, ramp_rate 낮췄는지 |
| 차가 안 움직임 | 배터리, MAX_OPENLOOP_PWM(140) 상한, 스톨 로그 |
