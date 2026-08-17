# HENES T870 자율주행 — 인수인계 (2026-08-17 기준)

> 이 문서 하나로 다음 작업자가 현재 상태를 파악하고 이어받을 수 있게 정리했다.
> **이전 버전(2026-08-13)은 값이 여러 개 틀렸다** — 최대타각 30°, 조향 부호, 캘리브 등.
> 이 문서가 최신이며, 값이 충돌하면 **이 문서와 실제 코드**를 따를 것.
>
> 워크스페이스: `/home/han/racing_ws` (ROS 2 Humble, Ubuntu 22.04)
> git: `git@github.com:hanmin0211/racing_ws.git` (**Private**), SSH 키 `~/.ssh/id_ed25519`

---

## 0. 한 줄 요약

**실전 트랙 웨이포인트를 찍었고(382점 / 190.4m), 횡방향은 폐루프 시뮬로 검증 끝났다.**
평균 횡오차 2.4cm, 조향 캘리브 ±3°·최대타각 12°까지 낮춰도 완주한다.

**남은 병목은 구동 하나다.** 0.4 명령에 0.33만 나온다.
2026-08-17 확인: 이전 `ff_sweep` 결과(STATIC_FF 40.5 / GAIN 20.7)는 **바퀴가 접지되지 않은
상태에서 측정된 값**이다. 지면 실측 2점(PWM 80→0.198, PWM 111→0.33)으로 역산한 실제
기울기는 **200 이상**이고, `MAX_DRIVE_PWM 111` 이 지금 천장으로 작동 중이다.
→ **지면에서 `ff_sweep` 재실행이 다음 작업.** (RTK 교차검증을 넣어 공중 측정은 이제 자동 중단된다)

**실전 트랙에서 통합 자율주행 랩은 아직 한 번도 안 돌렸다.** 이게 가장 큰 미검증 항목.

---

## 1. 지금 어디까지 됐나

### ✅ 완료 (실측 검증됨)

| 항목 | 상태 |
|---|---|
| RTK GPS (F9P + NGII VRS NTRIP) | Fixed, h_acc 1.4cm |
| IMU 헤딩 초기화 | 10m 직진 캘리브 + **직진성 검증**(휘면 거부) |
| 로컬라이제이션 | `/odometry/filtered` 30Hz, GPS/IMU 끊김 감지 |
| 로컬 경로 생성 | 호길이 매개변수 3차 피팅, 열린경로 지원, 전방 곡률 |
| **조향** | 캘리브·부호·정확도(≤1°)·안전장치 전부 완료 |
| 구동 기본 | 방향(전/후진), 스톨가드, 엔코더 스케일 실측 |
| 안전 | E-stop, 워치독, 엔드스톱, 스톨 컷 — 전부 실하드웨어 검증 |
| **통합 자율주행** | **1회 성공** (30.7m, 평균오차 0.32m) — 학교 임의 트랙 |
| **신호등 인식** (파이5+Hailo-8) | 실동작 — 30Hz 발행, 노트북 수신 확인 (2026-08-16) |
| **실전 트랙 경로** | 382점 / 190.4m 기록 완료 (2026-08-17, RTK σ≤5cm 필터) |
| **엔코더 스케일** | RTK 대조 오차 **0.17%** — 정확. 손대지 말 것 |
| **횡방향 추종 (시뮬)** | 폐루프 검증. 평균 2.4cm, 조향 ±3°/최대타각 12°/출발 2m 이탈에도 완주 |
| **완주·고장 구분** | `/goal_reached` 를 종·횡방향 제어가 구독 (🏁 vs ⚠) |

### ⬜ 남은 것

| 항목 | 비고 |
|---|---|
| **★ 구동 FF 재측정** | 이전 스윕이 **바퀴 뜬 상태**에서 측정됨. 실제 기울기는 20.7이 아니라 200 이상. `MAX_DRIVE_PWM 111` 이 천장으로 작동 중 → **최우선** |
| **실전 트랙 통합 랩** | **0회.** 가장 큰 미검증 항목 |
| 코너 추종 개선 | **불필요.** 시뮬상 `k_ld` 0.6→0.4 차이가 0.216→0.195m 로 미미. 이전 0.45m 오차는 실차 이슈였다 |
| **정지선·횡단보도 인지** | 모델에 없음 (신호등 2클래스뿐). 별도 학습+HEF 재컴파일 or 고전 영상처리 |
| **정지 지점 기록** | `traffic_light_bridge` 로 연동은 됐다(제어 코드 무수정). **대회장에서 좌표를 찍어야** 동작한다 — `stop_point_recorder` |

---

## 2. 하드웨어

| 부품 | 사양 | 연결 |
|---|---|---|
| GPS | u-blox **ZED-F9P** | USB (1546:01a9), libusb |
| IMU | HandsFree **A9** | `/dev/imu` (udev, 10c4:ea60), **300Hz** |
| MCU | Arduino **Mega 2560** | `/dev/ttyACM0` (2341:0042) |
| 모터 드라이버 | **aMAP 보드** | Mega에 스택 |
| 구동 | **24V 듀얼 DC, 16000rpm**, 정격 240W | MOTOR1 PWM5/ENA6/ENB7, MOTOR2 PWM2/ENA3/ENB4 |
| 조향 | DC모터 + 포텐셔미터(A15) | MOTOR3 PWM8/ENA9/ENB10 |
| 엔코더 | SPI 카운터 | CS 22, 23 |
| 소나 3개 | | 11, 12, 13 |
| 차량 | HENES T870, **축거 0.785m** | 전폭 775 / 전장 1400mm |
| **인지 보드** | Raspberry Pi 5 8GB + **AI HAT+ (Hailo-8, 26TOPS)** | 랜선 직결 **192.168.99.8** (`pi` 계정, 키 등록됨) |
| **카메라** | Logitech **C920** | **파이 USB** `/dev/video0` (노트북 아님!) |

### udev 규칙 (없으면 동작 안 함)
```
/etc/udev/rules.d/99-ublox-dgnss.rules
  SUBSYSTEM=="usb", ATTRS{idVendor}=="1546", ATTRS{idProduct}=="01a9", MODE="0666", GROUP="plugdev"
/etc/udev/rules.d/99-imu.rules
  SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="imu", MODE="0666"
```

---

## 3. ★ 캘리브레이션 값 (전부 실측, 근거 포함)

| 항목 | 값 | 근거 |
|---|---|---|
| UTM 원점 | x=399848.522, y=4092209.171 (EPSG:32652) | 웨이포인트 생성 원점 |
| 축거 | **0.785 m** | 줄자 실측 |
| 조향 중앙 | **ADC 424** | 손으로 직진 맞춘 뒤 실측 (2026-08-15). `auto_calib` 쓰면 매 세션 자동 점검됨 (ADC 1 ≈ 10m당 5cm) |
| 조향 좌끝/우끝 | ADC 0 / 949 | 실측 |
| **최대 타각** | **20°** | 실측 (30°는 틀린 가정이었음) |
| **조향 부호** | **+각도 = 좌 = ADC 증가** | 육안 확정 (2026-08-15) |
| counts/도 | **21.2** (단일 선형) | 보수적 채택 — 아래 주석 참조 |
| 실사용 타각 | **±18°** | 20°에서 pot이 ADC 0으로 포화하므로 마진 |
| **wheel_radius** | **0.1327 m** | RTK 실측 보정계수 1.0208 (2026-08-15) |
| counts_per_revolution | -290 | (wheel_radius 쪽을 보정했으므로 그대로) |
| NGII NTRIP | RTS2.ngii.go.kr:2101 / VRS-RTCM32 / <NGII_ID> / `ngii` | |

**조향 counts/도가 21.2인 이유**: 최대타각 20°를 **한쪽만 실측**했다. 포텐셔미터가
조향축 직결이면 counts/도는 전 구간 일정한 게 옳으므로 단일 스케일을 쓰되,
보수적으로 작은 값(424counts/20°)을 채택했다. 실제가 26.25면 언더스티어(안전한 방향)로만
틀린다. **반대쪽 타각을 실측하면 `STEER_COUNTS_PER_DEG` 한 줄만 고치면 된다.**

**경로 제약**: 18°의 최소 회전반경 = 2.42m → **본 경로는 최소 코너반경 2.8m 이상**으로
기록할 것. (불확실성 감안하면 3.2m 권장)

---

## 4. 아키텍처

```
[NGII NTRIP] ─RTCM→ vrs_ntrip_client ─→ ublox_dgnss ─→ /fix
handsfree/imu (A9 300Hz) ──┐
gps_heading_init (10m 직진, 1회성) ─/heading/yaw_offset(래치)─┐
                            ↓                                ↓
              direct_localization → /odometry/filtered + map→base_link TF
                            ↓
global_path_publisher → /global_path ─┐
                                      ↓
              local_sliding_window → /local_path + /curvature + /goal_reached
                            ↓
              local_pure_pursuit → /steering_cmd (조향각[도])
              longitudinal_controller → /target_speed (m/s)
                            ↓
              vehicle_cmd_mux → /cmd_vel  ★단일 출구 (E-stop > teleop > 자율)
                            ↓
              serial_bridge → "VEL:x,STEER:y" → Arduino(henes_firmware)
                            ↑ 텔레메트리
   /current_speed /steering_angle /steering_error /encoder_count /obstacle_distance /vehicle_stall

[C920] → [파이5 + Hailo-8, YOLOv8s] → /traffic_light_state ─랜선(DDS)→ traffic_light_bridge
              별도 보드 192.168.99.8         "RED"/"GREEN"/"NONE" 30Hz        + GPS 정지지점
                                                                                    ↓
                                                            /stop_line_distance → longitudinal_controller
```

### ★ 역할 분담 원칙
- **PWM은 펌웨어가 소유한다.** ROS는 목표속도(m/s)·조향각(도)까지만.
  펌웨어에 속도PID+FF+스톨가드가 있고, ROS가 raw PWM을 쏘면 **안전가드가 무력화**된다.
- **`/cmd_vel` 규약**: `linear.x` = 속도[m/s], `angular.z` = **조향각[도]** (rad/s 아님!)
- **속도는 `longitudinal_controller`가 소유.** `pure_pursuit`의 속도 파라미터는
  `standalone:=true`일 때만 쓰인다. (이걸 몰라서 엉뚱한 파일을 튜닝한 적 있음)

---

## 5. 파일 지도

| 경로 | 내용 |
|---|---|
| `arduino/henes_firmware/henes_firmware.ino` | **펌웨어** (조향/구동 PID, 안전가드, 개루프모드) |
| `arduino/steering_calib/` | 조향 ADC 읽기 전용(모터 미구동, 비상용) |
| `src/gps_localization/` | `direct_localization_node`, **마스터 런치 `bringup.launch.py`** |
| `src/gps_heading_init/` | `heading_init_node` (직진성 검증 포함) |
| `src/ngii_ntrip/` | `vrs_ntrip_client`, `ngii_rtk.launch.py` |
| `src/waypoint_follower/` | `local_path_core.py`(계산부), `local_sliding_window_node`, `global_path_publisher`, `waypoint_recorder`, **`tracking_monitor`** |
| `src/pure_pursuit_pkg/` | `local_pure_pursuit_node`, `control.launch.py`, `config/pure_pursuit_params.yaml` |
| `src/velocity_controller/` | `serial_bridge`, `vehicle_cmd_mux`, `longitudinal_controller`, `teleop_keyboard`, `wasd_teleop`, **`encoder_calib`**, **`ff_sweep`**, `steering_demo/sweep` |
| `src/mission_perception/` | `camera_node`, **`traffic_light_bridge`**(신호등→정지거리), **`stop_point_recorder`**(대회장에서 정지지점 찍기) |
| `src/mission_perception/pi_deploy/` | **파이 배포 원본** — `detector_node.py`, README. 파이 SD카드가 죽어도 여기 남는다 |
| `tools/pi_link.sh` | 파이 랜선 연결 (**매 부팅 후 1회**) + `--check` 진단 |
| `tools/local_path_harness.py` | **오프라인 경로 검증** (야외 없이 회귀 테스트) |
| `tools/heading_check.py` | 헤딩 부호 검증 (곡선 주행 필요) |
| `tools/ros_cleanup.sh` | **ROS 고아 프로세스 정리** (필수, 아래 함정 참조) |
| `FIELD_SESSION.md` | 야외 세션 종합 절차 |
| `DRIVE_CALIB.md` | 구동 캘리브 상세 절차 |

---

## 6. 실행

### 전체 자율주행
```bash
cd /home/han/racing_ws && source install/setup.bash
NGII_PW=ngii ros2 launch gps_localization bringup.launch.py control:=true max_speed:=0.4
```
- `control:=false`(기본)면 제어 없이 로컬라이제이션·경로만
- `max_speed:=0.0`이면 조향만 (구동 0) — 안전한 조향 확인용
- `invert_imu_yaw:=true`는 **지금 불필요** (부호 정상 확인됨)
- `auto_calib:=true` 면 차량이 스스로 10m 직진해 헤딩을 잡는다 (**실차 미검증**).
  헤딩을 모르는 개루프 전진이라 앞을 비우고 E-stop 준비 필수. 끝나면
  STEER_CENTER 점검값도 로그에 나온다. 안 붙이면 기존대로 사람이 밀어야 한다.

### 신호등 인식 (파이5 + Hailo-8)
```bash
bash tools/pi_link.sh              # 노트북 쪽 네트워크 — 매 부팅 후 1회
bash tools/pi_link.sh --check      # 링크·dnsmasq·포워딩·SSH 한 번에 진단

ssh pi@192.168.99.8
  source ~/ros2_humble/install/setup.bash && source ~/ros2_ws/install/setup.bash
  nohup ros2 run traffic_light_detector traffic_light_node > ~/tld.log 2>&1 < /dev/null & disown
```
노트북에서 `ros2 topic echo /traffic_light_state` 로 확인된다(별도 DDS 설정 불필요).
파이 노드 종료는 `pkill -f '[t]raffic_light_node'` — 대괄호 없으면 SSH 세션이 죽는다.

### 재실행 전 반드시
```bash
bash tools/ros_cleanup.sh
```

### 진단
```bash
ros2 run waypoint_follower tracking_monitor   # 횡방향 오차 실시간
ros2 topic echo /steering_error               # 조향 추종 오차
ros2 topic echo /ubx_nav_status --field carr_soln.status   # 2=RTK Fixed
```

### 펌웨어 플래시
```bash
# serial_bridge 먼저 종료 (포트 점유)
arduino --upload --board arduino:avr:mega:cpu=atmega2560 --port /dev/ttyACM0 \
  arduino/henes_firmware/henes_firmware.ino
```

---

## 7. ★★ 함정 모음 (전부 실제로 당한 것들)

| # | 함정 | 대응 |
|---|---|---|
| 1 | **`pkill -f ros2`는 런처만 죽인다** — 노드가 `/opt/ros/humble/lib/...`라 패턴에 안 걸림. 고아가 쌓여 NTRIP 다중접속(401)·USB 충돌·yaw_offset 덮어쓰기 발생 | **`bash tools/ros_cleanup.sh`** |
| 2 | **bringup이 ngii_rtk를 이미 포함** — 따로 켜면 NTRIP 2개 → 401 | bringup 하나만 |
| 3 | **전원 순서**: 차량 배터리 ON → 그다음 아두이노. 반대면 엔코더 SPI 칩 초기화가 안 되어 **카운트 0** | 꼬였으면 serial_bridge 재실행(아두이노 리셋) |
| 4 | **키보드 한글 입력** — teleop의 `W`가 `ㅈ`으로 들어가 차가 안 움직임. 엔코더 고장으로 오인해 크게 헤맴 | 영문 전환, 또는 `ros2 topic pub`으로 직접 명령 |
| 5 | **rviz2가 CPU 112% 점유** → `ros2 topic echo --once`가 간헐 실패해 "토픽 없음"으로 오진 | 캘리브 중엔 rviz 끄기 |
| 6 | **프로세스 개수 확인**: `pgrep -f`는 자기 명령줄 오탐, `pgrep -x`는 comm이 15자로 잘려 미탐 | `ps -eo comm` + 15자 비교 |
| 7 | **펌웨어 텔레메트리에 필드 추가하면 serial_bridge 정규식이 깨진다** (MODE= 추가 시 전 토픽 죽음) | 필드 추가 시 정규식 확인 |
| 8 | `/fix.status.status`는 RTK Fixed여도 1 | `carr_soln.status`로 판단 |
| 9 | IMU는 **재시작마다 yaw 기준 리셋** | 매 세션 10m 재캘리브 필수 |
| 10 | 배터리 방전 시 조향 breakaway 실패 + 보드 리셋 | 완충 확인 |
| 11 | **헤딩 캘리브 거부 뒤 110ms 만에 재측정 시작** — 차를 되돌리는 동작이 다음 시도 앞구간으로 기록돼 연속 실패. 2026-08-16에 43°→157°로 2연속 당함 | **고쳐짐**(`7ed98c8`): 3초 정지 확인 후 재시작 |
| 12 | **파이5 USB-C는 전원 전용** — 파이4의 USB 가젯 모드가 없어 PC에 꽂아도 네트워크가 안 된다. 전력도 부족(5V/5A 필요) | 랜선 또는 WiFi로 연결. 전원은 27W 이상 어댑터 |
| 13 | **IP 포워딩·iptables NAT는 재부팅하면 날아간다** (ufw 규칙·고정 IP는 파일로 남아 유지됨) | 매 부팅 후 `bash tools/pi_link.sh` |
| 15 | **`ff_sweep` 을 바퀴가 뜬 채로 돌리면 FF가 10배 틀리게 나온다** — 무부하 바퀴는 PWM 62에 1.15m/s가 나온다. 그 값을 펌웨어에 넣어 차가 목표속도의 60%밖에 못 냈다 | **고쳐짐**: RTK 이동거리와 엔코더를 대조해 35% 이상 어긋나면 자동 중단·결과 폐기 |
| 16 | **`require_rtk` 가 아무 점도 기록하지 않는다** — `waypoint_recorder` 가 `status.status>=2`로 판정하는데 `ublox_nav_sat_fix_hp` 는 Fixed에서도 **status=1**만 준다 (공분산 2.9mm인데 status=1) | **고쳐짐**: 공분산 기준(수평 σ ≤ 5cm)으로 판정 |
| 17 | **원점이 5개 파일에 흩어져 있었다** — 장소가 바뀌었는데 원점은 충주(149.7km 밖)로 남아 좌표가 `x=78019, y=-127771`. 제어는 차량기준이라 안 깨졌지만 **RViz로 경로를 볼 수 없었다** | **고쳐짐**: `config/site_origin.yaml` 이 유일한 정본. 장소 바뀌면 이 파일만 수정 |
| 18 | **`encoder_calib` 이 낡은 상수로 계산했다** — 툴은 `wheel_radius=0.13`, 펌웨어는 `0.1327`. "보정계수 1.019(2% 과소)"라는 가짜 결과가 나왔다. 실제로는 0.998로 이미 정확했다 | **고쳐짐**: 상수 동기화 + 3% 이내면 "수정 불필요" 판정 |
| 19 | **완주와 고장이 로그로 구분되지 않았다** — 경로 끝에서 `/local_path` 발행이 멈춰 하류가 '경로 0.5s 끊김'으로 정지. 완주인지 GPS 끊김인지 알 수 없었다 | **고쳐짐**: `/goal_reached` 를 종·횡방향 제어가 구독. 완주는 `🏁`, 고장은 `⚠` |
| 20 | **`~/stop_points.yaml` 이 다른 장소 좌표로 남아 있었다** — 원점도 옛 충주. 그대로 `mission:=true` 하면 조용히 아무 데서도 안 선다 | **고쳐짐**: bringup 이 원점 환산 + 중복 제거 + 경로에서 50m 넘으면 비활성화하고 경고 |
| 21 | **★ 아두이노가 모터 전류만 흐르면 브라운아웃 리셋** — 무부하 VMIN 4018mV, 모터 부하 시 **3483mV** (BOD 문턱 약 4.3V 미달). USB 장치번호가 2~3초마다 증가하고 serial_bridge 가 무한 재연결. **차량 배터리를 교체해도 전압 그대로**(4018→3987mV)라 배터리가 아니라 **전원 경로** 문제 | 실내 멀티미터 진단: ①5V핀 ②**VIN핀(0V면 USB 전원만 쓰는 중 — 유력)** ③배터리 단자 ④실드 5V 출력. VIN 에 7~12V 를 넣어 온보드 레귤레이터를 쓰게 하는 게 1순위 |
| 22 | **serial_bridge 를 두 개 띄우면 아두이노가 무한 리셋** — 포트 open 시 DTR 토글로 보드가 리셋되는데, 두 인스턴스가 서로의 연결을 끊고 재연결하며 루프를 만든다. 에러 메시지의 "multiple access on port" 가 그대로 맞는 말이었다 | bringup 이 이미 포함하므로 **절대 따로 실행하지 말 것**. ModemManager 도 같은 방식으로 관여 가능 → `99-arduino.rules` 로 `ID_MM_DEVICE_IGNORE=1` 권장 |
| 14 | **신호등 1프레임 오탐** — 카메라가 신호등을 안 봐도 6.6초에 4번 GREEN 이 튀었다. 제어에 물리면 한 프레임이 '출발' 명령이 된다 | **고쳐짐**(`d52374a`): 5프레임 연속 일치해야 확정. 로그의 `걸러낸튐` 이 늘면 threshold 조정 |

---

## 8. 다음 작업 (우선순위)

### 1) 구동 FF **재**측정 — 유일한 병목, 현장 필요

이전 `ff_sweep` 결과는 **바퀴가 접지되지 않은 상태에서 측정된 값**이다(2026-08-17 판명).
그 값(`STATIC_FF 40.5 / GAIN 20.7 / MAX_DRIVE_PWM 111`)이 지금 펌웨어에 들어가 있고,
`MAX_DRIVE_PWM 111` 이 실제 천장으로 작동해 차가 0.33 m/s 이상 못 낸다.

**근거 — 지면 실측 2점으로 역산한 실제 기울기:**

| MAX_DRIVE_PWM | 실측 속도 |
|---|---|
| 80 | 0.198 m/s |
| 111 | 0.33 m/s |

→ 기울기 ≈ **200~240 PWM per m/s** (스윕이 뱉은 20.7의 10배).
스윕이 PWM 62에서 1.15 m/s를 기록한 건 무부하 바퀴가 아니면 불가능한 값이다.

**엔코더는 정확하다** — RTK 30.229m vs 엔코더 30.280m, 오차 0.17%. 여기는 손대지 말 것.

```bash
ros2 run velocity_controller ff_sweep --ros-args -p max_speed:=1.0 -p ramp_rate:=3.0
```
바퀴 접지 필수. RTK 교차검증이 들어가 공중 측정이면 1m 지점에서 자동 중단된다.
결과의 기울기가 60 미만이면 경고가 뜨고, 그 값은 펌웨어에 넣으면 안 된다.

식별 후 펌웨어에 반영할 것(스윕이 권장값을 같이 출력한다):
`STATIC_FF`, `VELOCITY_FF_GAIN`, `MAX_DRIVE_PWM`, `velocity_kp/ki`, 적분 클램프.
※ `kp/ki` 는 **플랜트 기울기에 비례**해야 한다. 기울기 200에 `kp=8` 이면
0.1 m/s 오차에 0.8 PWM — P 제어가 사실상 없는 것과 같다.

### 2) 정지지점 기록 — 현장에서만 가능

```bash
ros2 run mission_perception stop_point_recorder
```
차를 정지선에 세우고 엔터. `~/stop_points.yaml` 에 저장된다.
기존 파일은 **다른 장소(충주) 좌표**라 bringup 이 자동으로 비활성화한다 — 반드시 다시 찍을 것.
FF 재측정과 같은 날 하면 이동을 아낀다.

### 3) 실전 트랙 통합 자율주행 랩 × 2~3회 ← **완주의 실질 검증**

시뮬은 횡방향 로직이 맞다는 것만 증명한다. 시뮬에 없는 것:
IMU 동결, GPS 끊김, 조향모터 스톨, 배터리 새그, NTRIP 네트워크.
**실제로 겪은 사고는 전부 이 목록에 있다.**

190.4m 는 학교 트랙(89m)의 2.1배다. 주행시간이 2배면 그 사고들의 발생 확률도 2배다.

```bash
ros2 launch gps_localization bringup.launch.py control:=true
```

### 4) 신호등 연동 검증 — 3) 성공 후

```bash
ros2 launch gps_localization bringup.launch.py control:=true mission:=true
```
`traffic_light_bridge` 가 bringup 에 통합됐다(`mission:=true`).
정지지점은 `~/stop_points.yaml` 에서 자동으로 읽고, 원점 환산·중복 제거·
경로 이탈 검사를 거친다. 파이(Hailo)가 `/traffic_light_state` 를 쏘고 있어야 한다.

### 실내에서 가능한 회귀 검증

```bash
python3 tools/tracking_sim.py --plot /tmp/t.png       # 폐루프 추종 (수치)
ros2 launch waypoint_follower sim_tracking.launch.py  # 폐루프 추종 (RViz)
```
`--steer-bias`, `--max-steer-deg`, `--init-offset` 등으로 오차 내성을 볼 수 있다.
**주의**: 기존 `sim_rviz.launch.py` 는 가상 차량을 경로 위에 강제로 올려놓아
추종 오차가 원리적으로 0이다. 검증에는 위 두 개를 쓸 것.

---

## 9. 현재 파라미터 요약

**펌웨어** (`henes_firmware.ino`)
```
STEER_CENTER 424 | STEER_MAX_ANGLE 20.0 | STEER_COUNTS_PER_DEG 21.2
MAX_STEER_PWM 130 | STEER_DEADBAND 16 | STEER_RESUME 24 | STEER_MIN_MOVE 34
STEER_BOOST_STEP 6 (정지마찰 탈출) | STEER_STALL_PWM 30 / 250ms / 쿨다운 1500ms
MAX_DRIVE_PWM 111 | STATIC_FF 40.5 | VELOCITY_FF_GAIN 20.7 | kp8/ki4/kd0.2
   ↑ ff_sweep 실측 반영 (2026-08-16, 103샘플/잔차 4.3PWM) — 구식 값 35/60/80 아님
wheel_radius 0.1327 | counts_per_revolution -290
PWM 주파수: 구동 488Hz(드라이버 한계), 조향 3.9kHz(소음저감)
개루프 모드: PWM:x 명령 (FF 식별용), MAX_OPENLOOP_PWM 140
```

**ROS**
```
local_pure_pursuit : max_steering_deg 18, k_ld 0.6, min_lookahead 1.8,
                     max_lookahead 4.0, max_steer_rate 60
longitudinal_ctrl  : v_max(launch), v_min 0.25, curvature_gain 6.0
vehicle_cmd_mux    : max_steer 18°, max_speed(launch)
local_sliding_window: n_back 5, n_forward 20, poly_order 3,
                     lookahead 10m, curvature_preview 4m
```

---

## 10. 일정

대회 **2026-08-22** (약 6일 남음).

| 항목 | 완성도 | 비고 |
|---|---|---|
| 완주(주행) | **약 82%** | FF 식별 완료(+30%). 남은 건 명령 대비 64% 잔차 — 엔코더/경사 판별 |
| 신호등 인식 | **약 90%** | 파이+Hailo 실동작. 실외 검증만 남음 |
| 정지선·횡단보도 | **0%** | 모델에 클래스 자체가 없음 |
| 인지 → 주행 연동 | **0%** | 토픽은 오는데 소비자가 없음 |

**속도 추종 잔차 규명이 최우선이다.** FF 는 잡혔지만 아직 명령의 64% 라, 2번(코너
튜닝)과 auto_calib 검증이 여기 물려 있다. 다음 수는 ff_sweep 재실행이 아니라
**주행 중 encoder_calib 동시 실행**(엔코더 과대측정인지 경사인지 판별)이다.

**미결(외부 확인 필요)**: 대회 규칙상 미션 구간을 **GPS 사전기록 경로**로
통과해도 되는가. 가능하면 정지선 인지 없이도 신호등만으로 미션이 성립하고,
불가하면 정지선 인지를 새로 만들어야 한다 — 남은 일정 배분이 여기서 갈린다.
