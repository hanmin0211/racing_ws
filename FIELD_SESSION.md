# 야외 세션 종합 체크리스트

> 2026-08-15 기준. 실내에서 할 수 있는 건 전부 끝났고, **남은 건 모두 야외 작업**이다.
> 순서에 의존관계가 있으니 **위에서부터** 진행할 것. 앞 단계가 틀리면 뒤가 전부 헛수고다.
>
> 세션 A(기본검증+구동) 약 1.5시간 → 세션 B(경로+통합) 약 2시간

---

## 챙길 것

- [ ] **차량 배터리 완충** ← 방전되면 조향 breakaway 실패 + 전압 붕괴로 보드 리셋
- [ ] 노트북 (충전), 아두이노 USB 케이블 (허브 말고 **PC 직결**)
- [ ] 인터넷 (NTRIP용 — 폰 핫스팟)
- [ ] 줄자, 각도기 (조향 반대쪽 타각 실측용)
- [ ] 차량 받침대 (앞축 들어올릴 것 — 조향 확인용)

**장소 조건**: 하늘 트인 개활지(RTK 필수), **직선 30m 이상**, 코스 주행 공간

---

# 세션 A — 기본 검증 + 구동 캘리브

## A-0. 시동 (15분)

```bash
cd /home/han/racing_ws && colcon build && source install/setup.bash
```

**★ 재실행 전에는 반드시 완전 정리** (`pkill -f ros2` 로는 부족하다)
```bash
bash tools/ros_cleanup.sh
```
> 노드 실행파일 경로가 `/opt/ros/humble/lib/...` 라 `ros2` 패턴에 안 걸린다.
> Ctrl-C 해도 **런처만 죽고 자식 노드가 고아로 남아** 쌓이고, NTRIP 클라이언트가
> 여러 개가 되면 NGII 1접속 제한에 걸려 **401 Unauthorized / RTCM 0프레임**이 된다.
> (2026-08-15 현장에서 NTRIP 3개·global_path_publisher 4개가 쌓여 RTK가 계속 401)

**★ bringup.launch.py 가 RTK(ngii_rtk)를 이미 포함한다.**
따로 `ngii_rtk.launch.py` 를 켜면 **NTRIP 클라이언트가 2개**가 되어
NGII 계정당 1접속 제한에 걸려 `401 Unauthorized` 가 나고 RTCM이 0프레임이 된다.
(ublox_dgnss 도 중복 실행되어 USB 충돌) → **bringup 하나만 켤 것.**

**udev 규칙 확인** (없으면 ublox 가 LIBUSB_ERROR_ACCESS 로 죽는다)
```bash
ls /etc/udev/rules.d/99-ublox-dgnss.rules
```
없으면:
```bash
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="1546", ATTRS{idProduct}=="01a9", MODE="0666", GROUP="plugdev"' | sudo tee /etc/udev/rules.d/99-ublox-dgnss.rules && sudo udevadm control --reload-rules && sudo udevadm trigger
```
→ 넣은 뒤 **F9P USB 재연결**

**터미널 1 — 전체 스택 (RTK 포함)**
```bash
NGII_PW=ngii ros2 launch gps_localization bringup.launch.py
```
**확인 (터미널 2)**
```bash
ros2 topic echo /ubx_nav_status --field carr_soln.status    # 2 = Fixed
```
> ⚠️ **Fixed(2) 아니면 여기서 멈춰라.** 1~3분 대기. 이게 안 되면 이후 전부 무의미하다.

> RTK만 따로 시험하고 싶을 때만 `ros2 launch ngii_ntrip ngii_rtk.launch.py` 를 쓰고,
> 그때는 bringup 을 동시에 켜지 말 것.

---

## A-1. 헤딩 초기화 + ★반전 확정 (15분)  — 미해결 항목

차량을 **곧게 10m 직진**시킨다.
- 직진성 검증이 들어가 있어 휘면 `❌ 직진이 아닙니다` 뜨고 재시도를 요구한다
- 완료 시 `✅ 헤딩 초기화 완료 ... yaw_offset=xx°`

**★ RViz에서 반드시 확인** (이게 아직 확정 안 된 항목):
| 확인 | 정상 |
|---|---|
| 전진할 때 차량 화살표 | 진행 방향을 향함 |
| **좌회전할 때 화살표** | **같이 좌로 회전** |

화살표가 **반대로 돌면** 헤딩 부호가 뒤집힌 것:
```bash
# 터미널 2 재실행
ros2 launch gps_localization bringup.launch.py invert_imu_yaw:=true
```
→ 다시 10m 직진해서 재확인

**통과 기준**: 화살표 방향·회전이 실제와 일치. 초록 `/global_path`, 빨강 `/local_path`가 차량 앞으로 뻗음

---

## A-2. 지상 조향 확인 (15분)  — 공중과 다름

공중에선 완벽했지만 **땅에서는 조향 부하가 몇 배**다(dry steering).

**터미널 3**
```bash
ros2 run velocity_controller serial_bridge
```
**터미널 4 — 추종 오차 감시**
```bash
ros2 topic echo /steering_error
```
**터미널 5 — 조향 명령**
```bash
ros2 run velocity_controller steering_demo --ros-args -p amplitude:=15.0
```

**통과 기준**
- `/steering_error` 절댓값 **2° 이내** 유지
- 스톨 로그(`STALL: steer=1`) 없음
- 정지 상태에서도 조향이 뜯김

**실패 시**: `MAX_STEER_PWM` 130 → 160~180 상향 후 재플래시
```bash
arduino --upload --board arduino:avr:mega:cpu=atmega2560 --port /dev/ttyACM0 \
  arduino/henes_firmware/henes_firmware.ino
```

> 덤: 시간 되면 **반대쪽 최대 타각을 각도기로 실측**. `STEER_COUNTS_PER_DEG 21.2`를
> 정확히 맞출 수 있고, 지금의 보수적 언더스티어 여유를 되찾는다.

---

## A-3 ~ A-5. 구동 캘리브 (1시간)

**→ [DRIVE_CALIB.md](DRIVE_CALIB.md) 절차를 그대로 따를 것**

요약:
| 단계 | 명령 | 얻는 것 |
|---|---|---|
| **엔코더 스케일** | `ros2 run velocity_controller encoder_calib` + 직진 15~20m | `wheel_radius` 보정값 |
| **FF 식별** | `ros2 run velocity_controller ff_sweep --ros-args -p max_speed:=1.2` | `STATIC_FF`, `GAIN`, `MAX_DRIVE_PWM` |
| **PID 재튜닝** | teleop으로 속도 단계 확인 | `kp≈8, ki≈4` 부근 |

각 도구가 **펌웨어에 넣을 숫자를 그대로 출력**한다. 반영 후 재플래시.

**통과 기준**
- 재검증 시 보정계수 **1.00 ± 0.02**
- 속도 오버슈트 < 15%, 정상상태 오차 < 0.05 m/s, 헌팅 없음

---

# 세션 B — 경로 + 통합 주행

## B-1. 본 경로 기록 (30분)

```bash
ros2 run waypoint_follower waypoint_recorder
```
- 대회 코스를 **실제로 주행**하며 기록 (1m마다 자동, 0.5m 리샘플 저장)
- ⚠️ `require_rtk` 옵션 **쓰지 말 것** (`/fix.status`가 Fixed여도 1로 나옴)

**★ 코너 반경 제약**
```
18° 사용 시 최소 회전반경 = 2.42m  (물리적 한계선)
→ 여유 포함 최소 코너 반경 2.8m 이상으로 돌 것
```

**기록 후 즉시 오프라인 검사** (야외에서 바로 가능):
```bash
python3 tools/local_path_harness.py src/pure_pursuit_pkg/config/waypoints_recorded_resampled_0.5.yaml
```
- `윈도우 이음매 점프`, `경로 생성 실패`가 **없음**이어야 통과
- 문제 있으면 그 자리에서 다시 기록 (돌아와서 발견하면 늦다)

---

## B-2. 통합 자율주행 (1시간+)  — 처음 하는 것

```bash
ros2 launch gps_localization bringup.launch.py control:=true max_speed:=0.6
```

**감시할 토픽** (별도 터미널)
```bash
ros2 topic echo /steering_error      # 조향 추종
ros2 topic echo /goal_reached        # 완주 판정
ros2 topic echo /current_speed       # 속도 추종
```

**진행 방식**
1. `max_speed:=0.6` 저속으로 **한 바퀴**
2. 문제없으면 0.8 → 1.0 단계적 상향
3. 이탈·진동 나면 즉시 E-stop, 원인 확인 후 재시도

**튜닝 포인트** (`pure_pursuit_params.yaml`)
- 코너에서 안쪽으로 파고들면 → `k_ld`, `min_lookahead` 올림
- 직선에서 좌우로 흔들리면 → `min_lookahead` 올림
- 코너에서 밖으로 밀리면 → `k_ld` 내림 / `curvature_speed_gain` 올림(더 감속)

---

# 안전 수칙 (전 세션 공통)

- **E-stop 항상 준비**: teleop 창에서 `E` 키. 먹스가 모든 명령을 즉시 차단한다
- 차량 **전방에 사람 금지**
- 이상하면 `Ctrl-C` — 모든 노드가 워치독으로 0.5초 내 정지
- 중간에 거동이 이상해지면 **배터리 전압부터 확인** (오늘 방전으로 조향 실패 사례 있음)

---

# 우선순위 (시간이 부족하면)

| 순위 | 항목 | 이유 |
|---|---|---|
| 1 | **A-1 헤딩 반전 확정** | 틀리면 경로를 반대로 따라감. 30분 |
| 2 | **A-3 엔코더 스케일** | 모든 속도 제어의 기준. 20분 |
| 3 | A-2 지상 조향 | 못 뜯기면 주행 자체가 불가 |
| 4 | A-4 FF 식별 | 목표속도 도달 가능해짐 |
| 5 | B-1 경로 기록 | 코스 확보돼야 가능 |
| 6 | B-2 통합 주행 | 위가 다 돼야 의미 있음 |

**1·2번만 해도 완주 확률이 크게 오른다.** 시간 없으면 이 둘부터.
