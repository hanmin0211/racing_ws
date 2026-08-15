# HENES T870 자율주행 — 인수인계 문서 (HANDOFF)

> 작성 2026-08-13. 이 문서 하나로 다른 작업자가 이 프로젝트를 완전히 이해하고,
> 이어받아 수정·보완·고도화할 수 있도록 매우 상세히 정리한다.
> 워크스페이스: **`/home/han/racing_ws`** (ROS 2 Humble, Ubuntu 22.04)

---

## 0. TL;DR (한 눈에)

- **차량**: HENES T870 브룬(유아용 승용완구 개조). 대회용 자율주행.
- **파이프라인 상태**: RTK GPS → 로컬라이제이션 → 전역/지역경로 → 횡방향 제어(pure pursuit) → `/cmd_vel` → **Arduino 펌웨어(하위제어)** 까지 구현. GPS/로컬라이제이션/경로는 **야외 실측 검증 완료**. 펌웨어 조향은 **벤치 검증 중**(방향·엔드스톱·스톨 통과, 저속 스무스니스 튜닝 남음). 구동모터는 미검증.
- **핵심 원칙**: 모터가 과거 2번 탔음 → **"무조건 안전"** (펌웨어 안전가드가 스톨/과부하 시 모터 컷).
- **다음 큰 작업**: ① 펌웨어 구동모터 벤치 검증 ② `/cmd_vel`↔펌웨어 통합 ③ EKF 대체 결정 정리 ④ 실차 통합 주행.

---

## 1. 하드웨어

| 부품 | 모델/사양 | 연결 | 비고 |
|---|---|---|---|
| GPS | u-blox **ZED-F9P** (RTK) | USB 허브 → `/dev/ttyACM*` (libusb, vendor 1546 product 01a9) | RTK Fixed 시 1~2cm |
| IMU | HandsFree **A9** (9축) | USB-시리얼(CP210x, vendor 10c4 product ea60) → `/dev/imu`(udev) | **재시작 시 yaw 기준 리셋됨** |
| MCU | Arduino **Mega 2560** | USB → `/dev/ttyACM0` (vendor 2341 product 0042) | 하위제어 |
| 차량 | HENES T870 브룬 | — | 전폭 775 / 전장 1400 / 전고 531 mm, **축거(wheelbase) 0.785 m**(실측), 최대타각 30°(가정) |

### 1.1 udev 규칙 (필수, 이미 설정 요망)
포트 번호가 재부팅/재연결마다 바뀌는 걸 막기 위해:
```
# /etc/udev/rules.d/99-ublox-dgnss.rules  (GPS libusb 접근권한)
SUBSYSTEM=="usb", ATTRS{idVendor}=="1546", ATTRS{idProduct}=="01a9", MODE="0666", GROUP="plugdev"
# /etc/udev/rules.d/99-imu.rules  (IMU 고정 이름 /dev/imu)
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="imu", MODE="0666"
```
적용: `sudo udevadm control --reload-rules && sudo udevadm trigger` 후 재연결.

### 1.2 Arduino 배선 (henes_firmware.ino, 검증된 배선)
- 전륜구동 MOTOR1: PWM=5, ENA=6, ENB=7
- 후륜구동 MOTOR2: PWM=2, ENA=3, ENB=4
- 조향   MOTOR3: PWM=8, ENA=9, ENB=10  (DC모터, **서보 아님**)
- 조향 각도센서(포텐셔미터): A15
- 엔코더(SPI): CS ENC1=22, ENC2=23
- 소나 3개: 11, 12, 13

---

## 2. 전체 시스템 아키텍처 (데이터 흐름)

```
[NGII VRS NTRIP] ──RTCM──▶ vrs_ntrip_client ──/ntrip_client/rtcm──▶ ublox_dgnss_node ──┐
                            (GGA 전송)                                (F9P USB 주입)     │
                                                                                        ▼
                                                          ublox_nav_sat_fix_hp ──▶ /fix (RTK 위경도)
handsfree/imu (A9 raw) ─────────────────────────────┐                                   │
                                                     │        ┌──────────────────────────┤
gps_heading_init (10m 직진 캘리브, 1회성) ──/heading/yaw_offset(래치)──┐                  │
                                                     ▼                 ▼                  ▼
                                      direct_localization_node (raw IMU + offset + /fix)
                                        → /odometry/filtered (map 프레임 x,y,yaw) + TF map→base_link
                                                     │
global_path_publisher (웨이포인트 yaml) ──/global_path──┐  │
                                                        ▼  ▼
                                      local_sliding_window_node
                                        (과거5+전방20 윈도우 → 3차곡선 피팅 → 앞10m 미래점)
                                        → /local_path (base_link 프레임) + /poly_coeffs + /curvature
                                                     │
                                      local_pure_pursuit_node (축거 0.785, lookahead, 슬루레이트)
                                        → /cmd_vel (linear.x=속도[m/s], angular.z=조향[도])
                                                     │
                                      serial_bridge_node ──"VEL:x,STEER:y"──▶ Arduino (henes_firmware)
                                                                                조향PID+구동PID+★안전가드
                                        ◀──"STATUS_10ms: ENC.. VEL.. PWM.. SONAR.."──
```

**좌표계**: 전부 map 프레임(UTM 52N − 고정원점). 원점 `x=399848.522, y=4092209.171` (= lat 36.970661, lon 127.874852). 웨이포인트·로컬라이제이션·경로가 모두 이 원점을 공유해야 어긋나지 않음.

---

## 3. 소프트웨어 패키지 (핵심만; `/home/han/racing_ws/src/`)

### 3.1 신규 작성 (이번 재구축의 핵심)
| 패키지 | 노드 | 역할 |
|---|---|---|
| **ngii_ntrip** | `vrs_ntrip_client` | NGII VRS NTRIP 클라이언트(**GGA 전송** — 기본 ntrip_client_node는 GGA 미지원이라 직접 작성). RTCM3 프레임→`rtcm_msgs/Message`→`/ntrip_client/rtcm` |
| **gps_heading_init** | `heading_init_node` | 10m 직진 GPS-course로 IMU yaw 오프셋 계산 → `/heading/yaw_offset`(래치) → **자동 종료**(1회성) |
| **gps_localization** | `direct_localization_node` | `/fix`(UTM변환) + raw IMU + yaw_offset → `/odometry/filtered`(map) + TF. **navsat/EKF 대체(RTK cm급이라 단순 직접방식 채택)** |
| waypoint_follower | `global_path_publisher` | 웨이포인트 yaml → `/global_path` |
| " | `local_sliding_window_node` | 슬라이딩윈도우+3차곡선 → `/local_path`(앞10m) + `/curvature` |
| " | `waypoint_recorder` | `/fix` 1m마다 기록 + 0.5m 리샘플 자동저장 |
| " | `resample_waypoints` | 웨이포인트 균일간격 리샘플 유틸 |
| " | `sim_odom_publisher` | 시뮬용 가짜 odom(경로 따라 이동) |
| **pure_pursuit_pkg** | `local_pure_pursuit_node` | `/local_path` 기반 pure pursuit(축거0.785)+슬루레이트 → `/cmd_vel` |

### 3.2 기존/참고 (일부 미사용)
- `pure_pursuit_pkg/pure_pursuit_node` : **구버전**(옛 `/vehicle_local_pose`+파일로딩). 미사용(→ `local_pure_pursuit_node`로 대체).
- `robot_localization_config` : EKF+navsat 설정. **현재 미사용**(direct_localization으로 대체, navsat이 map→base_link TF 없어 (0,0) 출력하는 문제). `.bak` 백업 있음. 고급화 시 재검토 대상.
- `velocity_controller/serial_bridge_node` : `/cmd_vel`(Twist)→시리얼. **펌웨어 프로토콜과 정합 확인 필요**(아래 8.2).
- `gps_local_bridge_pkg`, `vehicle_transform_pkg`, `gps_local_realtime_node` : 옛 파이프라인 잔재(3갈래 분산). 미사용.
- `ros2-ublox-zedf9p-master` : 옛 ublox 드라이버(**RTCM 주입 미지원**이라 폐기, `ublox_dgnss`로 대체). `zed_f9p.yaml`에 멀티GNSS 켜둔 흔적 있음.

### 3.3 외부 설치(apt)
`ros-humble-ublox-dgnss` (F9P를 libusb로 다뤄 USB 하나로 RTCM 주입 + NavSatFix). 함께 `ntrip-client-node`(GGA 미지원이라 미사용), `ublox-nav-sat-fix-hp-node`(사용), `rtcm-msgs`.

---

## 4. 캘리브레이션 값 (전부 실측/확정)

| 항목 | 값 | 출처 |
|---|---|---|
| UTM 원점 | x=399848.522, y=4092209.171 (lat 36.970661, lon 127.874852, EPSG:32652) | 웨이포인트 생성 원점 |
| 축거(wheelbase) | **0.785 m** (실측 78~79cm) | 줄자 |
| 최대 타각 | 30° (가정, 실측 권장) | pure_pursuit_params, 펌웨어 |
| 조향센서 ADC | **좌끝=0, 직진=459, 우끝=949** (방향: 낮=좌, 높=우) | 2026-08-13 실측(steering_calib.ino) |
| 조향 각도→ADC | `459 − 각도×(좌 15.30 / 우 16.33)` counts/도 | 위에서 유도 |
| 바퀴 반지름 | 0.13 m, counts/rev = −290 | 펌웨어(POWERPACK 계승) |
| NGII NTRIP | 캐스터 RTS2.ngii.go.kr:2101, 마운트 VRS-RTCM32, id <NGII_ID>, 비번 `ngii` | **비번은 map.ngii.go.kr VRS ID 관리에서 재설정 가능. 특수문자(@등) 쓰면 str2str 깨짐** |

---

## 5. 실행 방법

### 5.1 전체 스택 (한 방에) — 야외
```bash
cd /home/han/racing_ws && colcon build && source install/setup.bash
ros2 launch gps_localization bringup.launch.py
# 인자: rviz:=false, waypoints:=<yaml>, calib_distance:=10.0, imu_port:=/dev/imu
```
포함: RTK GPS(ngii_rtk) + IMU + heading_init + direct_localization + global_path + local_sliding_window + RViz.
**주의**: 실행 전 `pkill -f ros2` 로 중복 제거(NGII 계정당 1접속 제한).

### 5.2 현장 절차
1. IMU·GPS 연결 확인(`ls /dev/imu`, `lsusb|grep u-blox`)
2. bringup 실행 → **RTK Fixed 대기**: `ros2 topic echo /ubx_nav_status --field carr_soln.status` 가 **2**면 Fixed
3. **차량 정방향으로 10m 직진** → heading_init이 yaw_offset 계산 후 자동종료, direct_localization 로그에 "헤딩 오프셋 수신"
4. RViz: 초록 `/global_path`, 빨강 `/local_path`, 차량 화살표 확인
5. 웨이포인트 새로 찍기: `ros2 run waypoint_follower waypoint_recorder` (require_rtk 쓰지 말 것 — 8.4 참고)

### 5.3 RTK 정확도/상태 확인
```bash
ros2 topic echo /ubx_nav_status --field carr_soln.status   # 2=Fixed 1=Float 0=none
ros2 topic echo /ubx_nav_hp_pos_llh --field h_acc           # ×0.1mm (141≈1.4cm=Fixed)
```

---

## 6. 현재 상태: 검증됨 vs 미완

### ✅ 검증 완료
- RTK Fixed 야외 실측(carr_soln=2, h_acc~1.4cm, 주행 중 유지)
- NGII NTRIP RTCM 수신(GGA 포함)
- IMU 10m 헤딩 캘리브(yaw_offset 계산, /odometry/filtered yaw가 GPS course와 일치)
- 웨이포인트 기록(82점)+0.5m 리샘플(179점) 저장
- direct_localization → /odometry/filtered → local_sliding_window → /local_path 전체 체인 야외 동작
- 곡률 발행, pure pursuit 조향 로직(시뮬 검증), 슬루레이트
- **Arduino 펌웨어**: 부팅/시리얼/워치독, 조향 **방향(좌/우) 정확**, 목표도달, **엔드스톱 컷**, **조향 스톨감지 컷**(★핵심 안전 검증 통과), 정지 시 버즈 없음

### ⬜ 미완/미검증
- **펌웨어 조향 저속 스무스니스**: 저속 스윕 시 스텝감(툭툭) 있음. friction FF는 떨림/버즈 유발해 제거함. → **PWM 디더링 등 고급기법 필요**(9절).
- **펌웨어 구동모터 전체**: 방향/속도PID/구동스톨 **미검증**(바퀴 들고 벤치 필요).
- `/cmd_vel` ↔ serial_bridge ↔ 펌웨어 **통합 미검증**(프로토콜 정합 8.2).
- **헤딩 반전 이슈**: RViz에서 헤딩/미래점이 반대로 보인 적 있음(원인 미확정 — 정방향 캘리브 vs IMU 부호). 8.3 참고.
- EKF(robot_localization) 경로는 폐기 상태 — 고급화 시 재도입 검토.
- 종방향(속도) 프로파일 다듬기(현재 곡률 감속만).

---

## 7. Arduino 펌웨어 상세 (`arduino/henes_firmware/henes_firmware.ino`)

### 7.1 설계 철학
```
[구동PID / 조향PID] → PWM → [★safety_guard()] → 모터
                              ↑ 스톨/엔드스톱/워치독 → 강제 0
```
**PID가 뭘 하든 안전가드가 스톨/과부하를 잘라 모터를 못 태운다.** (모터 2번 탔던 이력)

### 7.2 안전장치 (현재값 = 벤치 초기 보수값)
| 항목 | 값 | 의미 |
|---|---|---|
| MAX_DRIVE_PWM | 80 | 구동 상한(검증 후 255까지 상향) |
| MAX_STEER_PWM | 90 | 조향 상한 |
| DRIVE_STALL_PWM/MS | 55 / 300 | 구동 PWM>55인데 속도<0.05가 300ms → 컷 |
| STEER_STALL_PWM/ERR/MS | 60 / 15 / 250 | 조향 PWM>60인데 ADC 안변하고 오차>15가 250ms → 컷 |
| SERIAL_TIMEOUT_MS | 500 | 명령 없으면 정지+조향중앙 |

### 7.3 조향 제어 (현재)
- 각도(도)→목표ADC: `steerAngleToADC()` (캘리브 459/0/949, 30°)
- 위치 PID: `steering_kp=1.0, kd=0.2`, **데드밴드 18(±1.2° → PWM 0)**, **MIN_MOVE 35**(스티션 극복)
- ⚠ **한계**: 데드밴드+MIN_MOVE 방식이라 저속에서 스텝감. friction FF(연속 이동)는 시도했으나 **목표근처 고주파 떨림/모터 버즈** 유발해 제거함. → 9절 고급화.

### 7.4 구동 제어 (POWERPACK.ino 계승, 미검증)
FeedForward(STATIC_FF=35, GAIN=60) + PID(kp30/ki24/kd0.5) + 조건부 안티와인드업 + 소프트스타트(가속0.5/제동0.8 m/s²). 엔코더 10ms 이동평균 속도.

### 7.5 시리얼 프로토콜
- 수신: `VEL:{m/s},STEER:{도}\n` (CRC 없음)
- 송신(20Hz): `STATUS_10ms: ENC1=.. VEL=.. TARGET=.. PWM=.. SLOPE=.. SONAR1/2/3=..` + `STEER: ADC=.. TGT=.. PWM=.. ANG=..`(조향 진단) + 스톨 시 `STALL: drive=.. steer=..`

### 7.6 플래시 방법 (arduino-cli 없음, IDE 1.8.19 CLI 사용)
```bash
arduino --upload --board arduino:avr:mega:cpu=atmega2560 --port /dev/ttyACM0 \
  /home/han/racing_ws/arduino/henes_firmware/henes_firmware.ino
```
센서만 읽는 **안전(모터 미구동) 스케치**: `arduino/steering_calib/steering_calib.ino` (조향 ADC 캘리브·비상정지용).
시리얼 읽기(파이썬): `serial.Serial('/dev/ttyACM0',57600)`, 포트 열면 리셋되니 2.5s 대기. **명령은 20Hz 이상 연속 전송(워치독 500ms)**.

### 7.7 벤치 테스트 절차 (필수, 바퀴 공중)
0. 바퀴 들기 → 1. 조향 방향(작은 각도) → 2. 엔드스톱(±30°) → 3. 조향 스톨(손으로 막기) → 4. 구동 방향(0.3m/s) → 5. 구동 스톨(바퀴 막기) → 6. MAX_PWM 상향+PID 튜닝 → 7. ROS 통합.
**각 단계 통과 전 다음 안 감. 스톨 컷 실제 확인 필수.**

---

## 8. 알려진 이슈/함정 (반드시 숙지)

1. **QoS**: `/fix`는 BEST_EFFORT(센서QoS)로 발행됨. 구독 노드는 `qos_profile_sensor_data`로 구독해야 받음(RELIABLE로 하면 0개 수신). heading_init/vrs_ntrip_client/waypoint_recorder/direct_localization 다 적용됨.
2. **serial_bridge ↔ 펌웨어 정합**: serial_bridge는 `STATUS_10ms:...SLOPE...`(구형)과 CRC 없는 명령을 씀. 펌웨어도 STATUS_10ms 포맷으로 맞춰뒀으나, **serial_bridge의 정확한 파싱 정규식/명령포맷을 펌웨어와 재대조** 필요(직접 통합 테스트 안 됨).
3. **헤딩 반전**: A9는 **재시작마다 yaw 기준 리셋** → 매 세션 10m 재캘리브 필수. RViz에서 헤딩/미래점 반대로 보인 적 있음 → (a)정방향 캘리브인지 (b)IMU yaw 부호반전인지 벤치/야외서 구분 필요(전진 시 화살표 방향 / 좌회전 시 화살표 회전방향으로 판별).
4. **recorder require_rtk 금지**: `/fix.status.status`가 RTK Fixed인데도 1로 나옴 → require_rtk=true면 아무것도 기록 안 함. RTK는 `carr_soln.status`로 판단. (개선: recorder가 carr_soln 직접 보게 수정 가능)
5. **NGII 단일접속**: 계정당 1개. 중복 vrs_ntrip_client 뜨면 401. 실행 전 pkill.
6. **NGII 비번 특수문자**: str2str는 `@` 등 URL특수문자 파싱 실패. 비번은 영숫자로.
7. **ublox_dgnss "Missing response CFG_*"**: degraded mode 경고 떠도 RTK 정상 동작 → 무시 가능(또는 CFG 파라미터 최소화).
8. **시리얼 포트 점유**: 파이썬 스크립트가 안 죽고 포트 잡으면 다음 접속 블록. `lsof /dev/ttyACM0`로 확인 후 kill.

---

## 9. 다음 작업 + 하이엔드 품질 로드맵

### 9.1 즉시 이어갈 것
1. **펌웨어 조향 스무스니스(고급)**:
   - **PWM 디더링**: 저속 구간에서 PWM을 고주파로 on/off(듀티 조절)해 스티션을 깨면서 평균속도를 낮춰 **매끈한 저속 이동** 구현. (friction FF는 떨림 유발이라 폐기)
   - 또는 **속도 피드포워드**: 목표 ADC의 변화율(명령 속도)을 알고 그만큼 기저 PWM 인가 → 추종 오차 최소.
   - 또는 조향 모터에 **전류센싱 추가**해 토크 기반 제어(현재 전류센서 없음 → 스톨감지가 유일 보호).
2. **구동모터 벤치 검증** → MAX_DRIVE_PWM 단계적 상향, 속도PID 튜닝, 구동스톨 확인.
3. **/cmd_vel → serial_bridge → 펌웨어 통합** 실측(프로토콜 정합 확정).
4. **헤딩 반전 확정**: 정방향 10m 캘리브 후 RViz 화살표·미래점 방향 검증. 부호문제면 heading_init/direct_localization의 yaw 부호 수정.

### 9.2 하이엔드 품질 개선 아이디어
- **로컬라이제이션**: RTK가 순간 끊길 때(carr_soln 1/0) 대비 **robot_localization EKF 재도입**(IMU dead-reckoning). 단일 EKF의 map→odom→base_link TF 구성 정리 필요(현재 direct_localization은 RTK 의존적). fix 품질 게이팅(carr_soln 기반) 추가.
- **경로계획**: 곡률을 lookahead 이내 좁은 구간으로 계산(현재 윈도우 전체 평균이라 급커브 완만화). 곡률 기반 **속도 프로파일**(코너 전 감속) 구현. 폐루프 트랙이면 리샘플 `--closed`.
- **횡방향**: pure pursuit + **곡률 피드포워드**(`atan(L·κ)`) 결합. 속도별 lookahead 튜닝. Stanley/MPC 등 상위 제어 검토.
- **안전(대회 무고장)**: 상위 E-stop, 소나 기반 긴급정지, 워치독 다층화, fix 이상 시 감속정지. 펌웨어에 전류/온도 센싱 추가.
- **파이프라인 정리**: 미사용 3갈래 잔재(gps_local_bridge, vehicle_transform, gps_local_realtime, 옛 pure_pursuit_node, ros2-ublox-zedf9p) 제거로 혼선 방지.
- **재현성**: 이 워크스페이스를 git으로 관리(현재 아님). 파라미터 yaml화, 런치 인자화 확대.

### 9.3 파일 위치 요약
- 펌웨어: `arduino/henes_firmware/henes_firmware.ino`, 캘리브: `arduino/steering_calib/`
- NTRIP 도구: `ntrip/`(README, ntrip_test.sh=str2str RTCM, ntrip_auth_test.sh=curl 인증)
- 현장절차: `FIELD_TEST.md`
- 웨이포인트: `src/pure_pursuit_pkg/config/waypoints_recorded*.yaml`
- 마스터 런치: `src/gps_localization/launch/bringup.launch.py`

---

## 10. 빌드/환경
- ROS 2 **Humble**, Ubuntu 22.04. `colcon build && source install/setup.bash`.
- 의존: `ros-humble-ublox-dgnss`, `rtklib`(str2str), python `pyproj`, `pyserial`, `tf_transformations`.
- Arduino IDE 1.8.19(`/usr/bin/arduino`) + avrdude. NewPing 라이브러리(`~/Arduino/libraries/NewPing`).

> 이 문서는 세션 전반을 정리한 것이며, 세부 결정 근거는 프로젝트 메모리
> (`~/.claude/projects/-home-han-racing-ws-src/memory/racing-ws-rebuild.md`)에도 기록됨.
