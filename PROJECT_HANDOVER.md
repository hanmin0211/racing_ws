# 자율주행 대회 프로젝트 — 전체 인계 문서

**차량**: HENES T870 유아용 전동차 개조 · **플랫폼**: ROS 2 Humble
**작성**: 2026-08-25 · **대회**: 2026-08-25 (당일)
**대상**: 이 프로젝트를 처음 보는 사람이 이어받아 바로 작업할 수 있도록 작성

---

# 목차

1. [시스템 전체 구조](#1-시스템-전체-구조)
2. [하드웨어 — 실측값과 함정](#2-하드웨어)
3. [소프트웨어 구성](#3-소프트웨어-구성)
4. [미션별 구현 상태](#4-미션별-구현-상태)
5. [해결한 문제들 (원인·조치·근거)](#5-해결한-문제들)
6. [미해결 문제](#6-미해결-문제)
7. [실행 방법](#7-실행-방법)
8. [진단 방법론](#8-진단-방법론)
9. [절대 규칙](#9-절대-규칙)

---

# 1. 시스템 전체 구조

## 1.1 데이터 흐름

```
[NGII VRS 캐스터]
   ↑ GGA(위치보고)          ↓ RTCM3(보정신호)
[vrs_ntrip_client] ──/ntrip_client/rtcm──→ [ublox_dgnss] ──USB──→ [ZED-F9P]
                                                  │
                                                  ↓ /fix (NavSatFix)
[IMU A9] ──/handsfree/imu──┐                     │
                            ↓                     ↓
                  [gps_heading_init] ──/heading/yaw_offset──┐
                                                             ↓
                                              [direct_localization]
                                                             ↓
                                            /odometry/filtered (map 프레임)
                                                             ↓
                                    [global_path_publisher] → /global_path
                                                             ↓
                                    [local_sliding_window] → /local_path, /curvature
                                                             ↓
                        [local_pure_pursuit] → /steering_cmd
                        [longitudinal_controller] → /target_speed
                                                             ↓
                                          [vehicle_cmd_mux] → /cmd_vel
                                                             ↓
                                    [serial_bridge] ──USB──→ [Arduino Mega]
                                                             ↓
                                                    구동모터 · 조향모터
```

## 1.2 명령 우선순위 (vehicle_cmd_mux)

```
1) E-stop (/e_stop = true)        → 무조건 정지
2) teleop (/teleop/cmd_vel)       → 최근 0.5초 내 수신 시 자율을 덮어씀
3) 자율 (/target_speed + /steering_cmd)
```

- `teleop_timeout = 0.5s` 데드맨: teleop 발행이 멈추면 0.5초 뒤 자율로 복귀
- `min_speed = -0.6` / `max_speed` = launch 인자 → **후진 명령이 통과함**
- **미션 노드(주차)는 이 teleop 채널을 써서 자율을 덮어쓴다** → 완주 코드 무수정

## 1.3 좌표계

- GPS 위경도(WGS84) → **UTM 52N(EPSG:32652)** 투영 → **원점 감산** → 로컬 map 좌표
- 원점은 `config/site_origin.yaml` 하나가 정본. **모든 노드가 이 값을 공유**
- 현재: `(477800.0, 3964400.0)` — 대구권 시험장

---

# 2. 하드웨어

## 2.1 구성

| 부품 | 모델 | 연결 | 비고 |
|---|---|---|---|
| 차량 | HENES T870 | — | 축거 **0.785m** (실측) |
| GNSS | u-blox ZED-F9P | USB (1546:01a9) | RTK, NTRIP 보정 |
| IMU | HandsFree A9 | `/dev/imu` (10c4:ea60, CP2102) | 방위각용 |
| MCU | Arduino Mega 2560 | `/dev/arduino` (2341:0042) | 하위제어 |
| 구동 | 24V 듀얼 DC | MOTOR1 PWM5/ENA6/ENB7, MOTOR2 PWM2/ENA3/ENB4 | 전·후륜 |
| 조향 | DC모터 + 포텐셔미터 | MOTOR3 PWM8/ENA9/ENB10, 센서 **A15** | 서보 아님 |
| 엔코더 | SPI 카운터 LS7366R | CS **22/23** | 보드에 내장 |

## 2.2 ★ 드라이버 보드가 여러 개다 — 개체마다 다르다

**이걸 모르면 이틀을 날린다.** 실제로 그랬다.

| 보드 | 조향센서 | 엔코더 카운터 | VCC | 용도 |
|---|---|---|---|---|
| ms2405 #1 | **A8** | ❌ 없음 | 4630~5068mV | RC 조종기용 |
| ms2405 #2 | **A8** | ❌ 없음 | 4708~4728mV | RC 조종기용 |
| **amap 계열 (현재)** | **A15** | ✅ CS22/23 | 3.9~4.1V | **자율주행용** |
| 불량 개체 | ? | ❌ | 3893mV | 사용 금지 |

- ms2405 는 **D17(Serial2)로 115200bps "Frame Lost or FailSafe"** 를 계속 보낸다 = RC 수신기 채널
- **보드를 바꾸면 반드시 재측정**:
  1. `arduino/adc_scan/adc_scan.ino` → 앞바퀴 돌리며 최대변동 아날로그핀 확인
  2. `arduino/enc_scan/enc_scan.ino` → SPI 카운터 CS 응답 확인
  3. 좌/중/우 ADC 재측정 → 캘리브 갱신
  4. **조향 극성 재검증** (데드밴드 밖 각도 명령해 ADC 방향 확인)
- `arduino/board_id/board_id.ino` 가 ①②를 한 번에 보지만, **판별기보다 실제 동작 실적이 우선**
  (실제로 판별기가 A8 로 오판정한 적 있음)

## 2.3 ★ VCC 3.9~4.1V 는 이 차의 정상대다

- 완주하던 시절에도 4018mV 였다
- ATmega2560 의 BOD 는 퓨즈 설정값(메가는 보통 **2.7V**)이라 3.9V 에서도 정상 동작
- `serial_bridge` 의 "공급전압 강하 VMIN=3893mV" 경고는 **4300mV 기준이라 이 차엔 과한 경고. 무시**
- **단 여유가 적다**: PWM 190 에서 부하 시 3483mV 까지 떨어져 랩 종반 리셋된 이력
- 속도 상향은 VMIN 보며 단계적으로

## 2.4 ★ USB 연결 — 프로젝트 최대 함정

**IMU·라이다 둘 다 CP2102(10c4:ea60)** 라 udev 가 시리얼번호로 구분한다.

```
/etc/udev/rules.d/99-imu.rules
  SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="imu"
```

**허브 경유가 치명적이다** (실측):
```
IMU:      usb 3-2.1.4.1   ← 허브 3단 → 8~24초마다 끊김
아두이노:  usb 3-1        ← 직결 → 멀쩡
```

커널 로그 증거:
```
02:15:49  cp210x attached to ttyUSB0
02:15:57  disconnected          (8초)
02:15:57  attached again
02:16:21  disconnected          (24초)
```

**→ IMU 는 반드시 노트북에 직결. 포트 부족하면 GPS 를 허브로 옮길 것.**

## 2.5 캘리브레이션 값 (전부 실측)

```
조향 센서 A15:  완전좌 936 / 직진중앙 412 / 완전우 0   (각 80~100샘플)
조향 스케일:    좌 26.2 / 우 20.6 counts/도  (링키지 비대칭 — 방향별 분리)
최대 타각:      20° (펌웨어) / 18° (제어 상한, 포화 회피)
최소 회전반경:  2.42m = 0.785/tan(18°)
바퀴 반경:      0.1327m (RTK 대조 보정, 계수 1.0208)
엔코더:         -290 counts/rev (부호: 전진 시 감소)
FF:             PWM = 80 + 95·v   (지상 실측)
속도 PID:       kp30 ki15 kd0.5
```

---

# 3. 소프트웨어 구성

## 3.1 패키지

| 패키지 | 역할 |
|---|---|
| `gps_localization` | bringup launch, `direct_localization` (측위 융합) |
| `gps_heading_init` | 10m 직진 헤딩 캘리브 + 안전게이트 |
| `ngii_ntrip` | VRS NTRIP 클라이언트 (GGA 상향 전송 포함) |
| `waypoint_follower` | 경로 발행, 로컬 슬라이딩 윈도우, 웨이포인트 기록 |
| `pure_pursuit_pkg` | 경로 추종 제어 |
| `velocity_controller` | 종방향 제어, 명령 먹스, 시리얼 브리지, teleop |
| `mission_perception` | 주차·횡단보도·신호등 미션 노드 |
| `lidar_clustering` | 라이다 회피 (미통합) |

## 3.2 핵심 노드 동작

### direct_localization
- `/fix` → UTM 변환 → 원점 감산 → `(x, y)`
- IMU yaw + `yaw_offset` → 지도 정렬 방위각
- **센서 두절 시 발행 중단** (마지막 값 유지 금지):
  ```python
  if (t_now - last_fix_time) > fix_timeout:   fix_lost=True; return
  if (t_now - last_imu_time) > imu_timeout:   imu_lost=True; return
  ```
  → 하류가 타임아웃으로 안전 정지. **의도된 설계**

### gps_heading_init (매 세션 필수)
- IMU yaw 는 부팅마다 리셋 → **매번 10m 직진 캘리브 필요**
- `course = atan2(Δn, Δe)`, `yaw_offset = normalize(course − yaw_imu)`
- **안전게이트 5종** (2026-08-24 추가, 벽 충돌 재발 방지):

| 게이트 | 파라미터 | 동작 | 검증 |
|---|---|---|---|
| ① | `require_rtk` | RTK 수렴(σ≤5cm) 전엔 시작점도 안 잡음 | ✅ 실차 |
| ② | `max_jump_speed` 2.0m/s | 불가능한 이동 = 측위 점프 → 무효화 | ❌ 미검증 |
| ③ | 최소 소요시간 | 거리/(속도×2)보다 빠르면 거부 | ❌ 미검증 |
| ③-b | `max_peak_dev_deg` 25° | peak 편차 하드 상한 | ❌ 미검증 |
| ④ | `require_wheels_straight` | 앞바퀴 ±2° 밖이면 출발 안 함 | ❌ 미검증 |

### local_sliding_window
- `/global_path` (latched, TRANSIENT_LOCAL) 수신 → 차량 최근접점 주변 윈도우
- 3차 다항식 피팅 → `/local_path` (base_link), `/curvature`
- 열린 경로 끝에서 `/goal_reached` 발행

### vehicle_cmd_mux
- 우선순위 조정 + 최종 클램프 (`min_speed`~`max_speed`, `±max_steer`)
- 헤딩 캘리브 전에는 자율 거부 (teleop·E-stop 은 통과)

---

# 4. 미션별 구현 상태

## 4.1 기본 완주 — ✅ 구현 완료, 실차 검증됨

- 실차 190m 완주 실적 (3.6km/h)
- HIL 완주 2회
- 최근 실차: RTK Fixed 94.8%, 수평정확도 1.4cm, 횡오차 중앙값 0.18m

## 4.2 횡단보도 정지 — ✅ 구현 완료

**요구**: 정지선 64cm 이내 정지 → 3초 대기 → 재출발

**구현**: `crosswalk_stop_node`
- `longitudinal_controller` 가 이미 `/stop_line_distance` 하나만 보고 √(2ad) 감속·정지
- **없던 건 "정지 후 3초 뒤 재출발" 하나뿐** → 그 상태기계만 새 노드로
- 상태: `IDLE → APPROACH → HOLD(3초) → CLEARED`

**실차 이슈와 해결**:
```
증상: 앞바퀴가 정지선 30cm 넘어감
원인: longitudinal_controller 가 0.3m 이하에서 하드정지
조치: launch 에 crosswalk_bias 인자 추가 (기본 0.35m)
     → 정지지점을 35cm 앞당겨 봐서 그만큼 뒤에 선다
미세조정: 아직 넘으면 0.40, 너무 뒤면 0.20
```

⚠ `mission:=true`(신호등)와 **동시에 켜지 말 것** — 둘 다 `/stop_line_distance` 발행

## 4.3 후진주차 — ✅ 자세기반 planner 로 3자리 전부 성공

> **2026-09-04 정정.** 아래 내용은 `parking_planner`(Dubins) 도입 후 재측정한 것이다.
> 그 전 판(궤적 재생 방식, slot3 만 성공)은 이 절 끝 "옛 방식" 에 남긴다.

**요구**: 지정된 자리로 후진 진입. **뒷바퀴 걸침만 해도 인정**

**설계 핵심**:
- `pure_pursuit` 는 전진 전용 → **건드리지 않고** 별도 노드가 `/teleop/cmd_vel` 로 우회
- mux 우선순위 덕에 자율을 자동으로 덮어씀 · 20Hz 발행(데드맨 0.5초)
- `parking_node` 는 **`~/parking_pose_N.yaml`(목표 자세 하나)** 만 읽는다.
  궤적은 `parking_planner` 가 **차가 실제로 서 있는 자세에서 그때그때** 만든다:

  ```
  start ─후진직선 d_back─→ ─전진 Dubins(R=3.0)─→ ─직선─→ cusp ─후진 원호─→ goal
  ```

**왜 이렇게 바뀌었나**: 옛 방식은 미리 만든 궤적에서 최근접 점을 찾아 이어 갔는데,
위치는 0.1~0.4m 로 가까워도 그 점의 궤적 자세와 차 자세가 58~78° 어긋나 ABORT 했다.
Dubins 는 임의의 두 자세를 곡률한계 안에서 잇고 해가 항상 존재하므로
**"어긋난 자세를 메운다" 가 아니라 "지금 자세에서 출발하는 경로를 새로 만든다"** 로
문제를 바꿨다. 자리1·2 는 활주로가 모자라서, 사람이 하듯 **먼저 조금 물러서는**
구간(`d_back`)을 두어 풀었다.

**검증 (2026-09-04 재측정, 완주 종점 (-25.605, 103.797) 헤딩 +166.9° 에서)**:

| 자리 | 목표까지 | 뒤로 | 전진 | 후진 | R_rev | 계획 | 폐루프 정차오차 |
|---|---|---|---|---|---|---|---|
| 1 | 2.26m | 2.25m | 2.66m | 3.40m | 3.45m | ✅ | **22cm** |
| 2 | 1.63m | 1.90m | 2.96m | 3.10m | 3.45m | ✅ | **15cm** |
| 3 | 2.80m | 0.00m | 3.36m | 2.73m | 3.10m | ✅ | **15cm** |

```bash
python3 tools/preflight.py                      # 계획 가능 여부 (planner 직접 호출)
python3 tools/test_parking_node.py --slot 1     # 폐루프 시뮬 (--slot 을 반드시 줄 것)
```

⚠ `test_parking_node.py` 의 `--file` 인자는 **무시된다.** 자리는 `--slot` 으로 고른다.
(문서 옛 판의 `--file ~/parking_3.yaml` 은 실제로 slot1 을 시험하고 있었다.)

**옛 방식 (기록용)**: `~/parking_N.yaml` = `make_parking_path.py` 가 미리 만든 궤적.
`parking_direct`(목표점 직접 후진)는 1.7~2.3m 에서 맴돌아 폐기. 두 파일·노드 모두
현재 `parking_node` 는 쓰지 않는다.

## 4.4 라이다 회피 — 미통합
검출은 실HW 검증됨. 조향 융합 실트랙 튜닝 남음.

## 4.5 신호등 — 인프라만
`traffic_light_bridge` 존재. 파이(Hailo)가 `/traffic_light_state` 를 쏴야 의미 있음.

---

# 5. 해결한 문제들

## 5.1 ★ 엔코더 무신호 — 보드에 카운터 칩이 없었다 (2026-08-22)

**증상**: `ENC1` 이 정확히 0 고정. 손으로 굴려도, 모터로 돌려도 안 변함.
**연쇄 증상**: 속도 0 → 스톨가드가 정상 주행을 스톨로 오인 → 700ms 마다 컷

**진단 (전수 배제)**:
1. 전원 순서 가설 → 기각
2. SPI CS 후보 **29핀 전부 0x00 무응답**
3. A/B 펄스 스캔 → 모터핀 포함 전 핀 무전이
   - D17 이 뛰었으나 **정지 상태에서도 동일** → 노이즈로 판명 (대조군이 오판 막음)
4. 아날로그 A0~A15 → 회전에만 반응하는 핀 없음
5. UART 3포트×5보드레이트, I2C 스캔 → RC 수신기 신호만 발견

**원인**: ms2405 보드에 **LS7366R 카운터 IC 가 없다**. amap 에는 있었다.
**조치**: amap 계열 보드로 교체 → CS22/23 에서 즉시 응답, 엔코더 부활

**교훈**: 보드가 센서 인터페이스를 담당한다. 교체하면 핀맵이 통째로 바뀐다.

## 5.2 ★ 조향 좌우 비대칭 — 좌회전이 78%만 꺾였다 (2026-08-22)

**증상**: 커브에서 차선 밖으로 이탈
**원인**:
```
좌반 실측 (936−412)/20° = 26.2 counts/도
우반 실측 (412−0)/20°   = 20.6 counts/도
설정값: 20.6 단일 (작은 쪽)
→ 좌회전 18° 명령에 실제 14° (78%)
```
**조치**: `STEER_CPD_LEFT 26.2` / `STEER_CPD_RIGHT 20.6` 방향별 분리
**검증**: +18°→+17.3° / −18°→−17.1°, **좌우 차이 0.2°**

## 5.3 ★ 경로에 물리적으로 못 도는 커브 (2026-08-22)

**원인**: 최대 타각 18°(최소반경 2.42m)인데 경로에 R=1.73m, 1.91m 구간 존재
→ **제어가 완벽해도 반드시 이탈**

**조치**: `tools/smooth_path.py` — 이탈량 제한 반복 평활화
```
p_i ← p_i + α(p_i⁰ − p_i) + β(p_{i−1} + p_{i+1} − 2p_i)
if |p_i − p_i⁰| > d_max:  경계로 투영
```
**결과**: 최소R 1.73→**2.81m**, 못 도는 구간 **0개**, 평균 이동 0.09m

**부수 발견**: 리샘플 간격 0.5→0.3m 는 **역효과** (못 도는 커브 2→25개).
간격을 줄이면 경로가 급해지는 게 아니라 **GPS 기록 노이즈가 곡률로 드러난다.**

## 5.4 ★ 빌드 함정 — `--symlink-install` 을 믿으면 안 된다 (2026-08-25)

**증상**: 소스를 고쳤는데 옛 동작이 그대로
**원인**: `--symlink-install` 을 써도 **`build/` 에는 복사본이 들어간다**

**실제 발견**:
| 파일 | 소스 | 빌드본 | 결과 |
|---|---|---|---|
| `heading_init_node.py` | 825줄 (게이트 5개) | 577줄 (게이트 **0개**) | 안전장치 미작동 |
| `bringup.launch.py` | crosswalk_bias 있음 | 없음 | 파라미터 무시 |
| `waypoints...0.5.yaml` | 370점 | 옛 버전 | 다른 경로 주행 |

**조치**: 전체 재빌드. 현재 소스=빌드본=install 일치 확인
**규칙**: **소스 고치면 무조건 `colcon build`**

검증 스크립트:
```bash
for f in heading_init_node parking_node crosswalk_stop_node; do
  s=$(find src -name "$f.py" -not -path "*/build/*" -not -path "*/install/*" | head -1)
  i=$(find install -name "$f.py" -path "*site-packages*" | head -1)
  diff -q "$s" "$i" >/dev/null 2>&1 && echo "✅ $f" || echo "❌ $f 재빌드 필요"
done
```

## 5.5 ★ 원점 불일치 — 150km 어긋난 사건 (2026-08-17)

**증상**: 대구권에서 로컬좌표가 `(78019, −127771)` 로 찍힘
**원인**: 원점이 5곳에 하드코딩돼 있었고, 장소 변경 시 일부만 갱신 (충주 값 잔존)
**조치**: `config/site_origin.yaml` 단일 정본 + 공통 로더
**부수**: `parking_recorder` 가 파일에 원점을 함께 저장 → `parking_node` 가 1m 이상 다르면 거부

## 5.6 ★ 연석 충돌 — IMU 미연결 (2026-08-24)

**증상**: 10m 직진 캘리브 중 우측으로 쏠려 연석 충돌
**진단**: 로그 확인 결과 **최근 8개 런 전부에서 IMU 노드 사망**
```
[ERROR] hfi_a9_ros2: process has died, exit code 1
/dev/imu 없음, lsusb 에 CP2102 미검출
```
**원인**: IMU 가 USB 에 물리적으로 연결되지 않은 채 주행
**2차 요인**: GPS 점프 (단독측위→RTK Fixed 스냅, 0.38초에 7.3m)

## 5.7 ★ "경로 끊김" — 원인은 상류에 있었다 (2026-08-25)

**증상**:
```
[longitudinal_controller] ⚠ 경로(/curvature) 끊김 → 정지
[local_pure_pursuit]      정지: 경로 35.1s 끊김
```
헤딩 캘리브는 **성공**했는데(편차 4°) 차가 안 움직임.

**추적 순서** (이 방법론이 핵심):
1. `/curvature` 발행자는 있는데 **echo 타임아웃** → 메시지 안 옴
2. `local_sliding_window` 디버그 인스턴스 별도 기동
   → **전역 경로는 정상 수신(370점)**, curvature 는 안 냄 → `odom_callback` 이 안 불림
3. `/odometry/filtered` echo → **타임아웃. 발행 중단 상태**
4. `direct_localization.publish_odom()` 코드 확인 → `imu_lost` 가드에서 return
5. 커널 로그 → IMU 가 8~24초마다 끊김, `/dev/imu` 가 ttyUSB0→ttyUSB1 로 변경

**결론**: IMU USB 접촉 불량 → `imu_lost` → 측위 발행 중단 → 로컬경로 없음 → 제어기 정지.
**안전 로직은 의도대로 작동한 것. 고장이 아니다.**

**조치**: IMU 를 허브에서 빼서 노트북 직결. 코드 수정 불필요.

## 5.8 완주→주차 자동 연동 (2026-08-25)

`parking_node` 에 `trigger_on_goal_reached` 파라미터 추가 (기본 True).
`/goal_reached` 를 시작 트리거로 사용 → **사람 개입 없이** 완주 후 주차 시작.

## 5.9 parking_node 시작점 로직 수정 (2026-08-25)

**기존**: 첫 구간(전진 접근) 안에서만 커서 탐색 + 궤적 첫 점 거리로 판정
→ 완주 종점에서 5.07m 라 임계 5m 에 걸려 거부. 통과시켜도 지나온 구간을 되밟음

**수정**: **전 구간을 훑어 차량 최근접 점**을 찾고 그 구간부터 재생
→ slot1 은 후진구간 idx45 에서 0.36m, slot2 는 idx52 에서 0.11m 로 정상 진입

## 5.10 ★ 웨이포인트에 원점이 안 적혀 있었다 (2026-09-04)

**증상 (아직 안 터졌지만 반드시 터질 것이었다)**
로컬좌표는 `(UTM − 원점)`이라 **원점을 모르면 해석이 불가능**하다. 그런데
`waypoint_recorder` 는 `{'waypoints': [...]}` 만 저장했고 `global_path_publisher` 는
원점 검증을 전혀 하지 않았다.

```
stop_point_recorder     → origin_x/y 저장 ✅   bringup 이 환산·검증 ✅
parking_pose_recorder   → origin 저장 ✅       parking_node 가 1m 초과 시 거부 ✅
waypoint_recorder       → 원점 없음 ❌         검증 없음 ❌   ← 여기만 뚫려 있었다
```

→ 장소를 옮겨 `site_origin.yaml` 을 갱신하면 옛 경로 파일이 **조용히** 149km
어긋난 채 로드된다. 정지점은 걸러지는데 **주행경로는 안 걸러진다.**

**조치**
1. `site_origin.py` 에 `read_origin_stamp / origin_stamp / reconcile_origin` 추가.
   기존 두 포맷(`origin_x/y`, `origin: {x,y,epsg,site}`)을 모두 읽는다.
2. `waypoint_recorder` · `waypoint_resample` · `smooth_path` 가 원점을 **파일 맨 위**에 저장.
   기록→리샘플→평활화 전 체인에서 스탬프가 살아남는 것을 확인.
3. `global_path_publisher` 에 `origin_check` 파라미터(기본 `strict`):
   - 스탬프 일치 → 그대로 / 소폭 차이 → 자동 환산
   - **1km 초과(=다른 장소) 또는 EPSG 불일치 → 발행 거부.** 경로가 없으면 차는
     안 움직인다 — 엉뚱한 경로를 쫓는 것보다 안전하다.
   - 거부 시 5초마다 에러를 반복한다. 하류의 "경로 끊김" 을 상류로 되짚는
     시간(§8.1)을 없애기 위해서다.
4. `tools/shift_waypoints.py` 신규 — `site_origin.yaml` 주석이 예전부터 이 도구를
   가리켰지만 **실제로는 없었다.** 환산(`--from-origin`)과 스탬프만 찍기
   (`--stamp-only`) 를 지원. 기존 대구 파일 6개에 스탬프를 소급 기록했다.

**검증** — 원점을 용인으로 임시 교체하고 대구 경로를 물림:
```
[WARN]  원점 환산: (477800, 3964400 — 대구권) → (333600, 4127100 — 용인)
[WARN]  ❌ 원점 차이가 217.4km 다 …
[ERROR] ❌ 전역 경로를 발행하지 않는다
```
원점 일치 시에는 경고 없이 정상 로드(370점 184.0m, 범위 x[-45.4, 15.5]).

## 5.11 ★ 현장 데이터 사전검사 — `tools/preflight.py` (2026-09-04)

현장에서 기록한 데이터가 **못 쓰는 것이었다는 사실을 현장을 떠난 뒤 알면**
되돌릴 수 없다. 이 프로젝트에서 실제로 넷 다 겪었고, 넷 다 파일만 보면
미리 알 수 있는 것이었다. 그래서 한 번에 검사한다.

| 검사 | 잡는 사고 |
|---|---|
| A 원점 | 150km 어긋남 (2026-08-17) |
| B 경로 — 원점 스탬프 / 곡률 / **시작 10m run-up 직선성** | 못 도는 커브(2026-08-22), 램프 캘리브 40° 틀어짐(2026-09-02) |
| C 정지점 — 원점 / 경로까지 거리 | 다른 장소 정지점 |
| D 주차 — **planner 직접 호출** | 진입 불가 자세 (2026-08-25) |

D 는 `parking_node` 와 **같은 `parking_planner` 를 부른다.** 판정이 곧 노드의 결과다.

```bash
python3 tools/preflight.py           # ❌ 가 없으면 그 데이터로 주행 가능 (exit 0)
```

---

# 6. 미해결 문제

| # | 항목 | 상태 | 대응 |
|---|---|---|---|
| 1 | **IMU USB 접촉 불량** | 8~24초마다 끊김 | **노트북 직결** (허브 금지) |
| 2 | ~~후진주차 slot1·2~~ | **해결됨** (2026-09-04, parking_planner) | §4.3 |
| 3 | 배터리 방전 | 18초에 879mV 하락 관측 | 완충 + 예비 |
| 4 | 헤딩 게이트 ②③④ | 미검증 | `test_heading_calib_guard.py` 가 ExternalShutdownException (테스트 코드 버그) |
| 5 | `parking_recorder` yaw=0 | 저장 버그 | 현재는 궤적 진행방향으로 대체 계산 중. 재기록 전 수정 권장 |
| 6 | 앞바퀴 좌 6~9° 틀어짐 | 정지 중 조향 스톨 유발 | **출발 전 손으로 정면 맞추기** |
| 7 | 직진 차선밟기 | RTK Float 구간에서 오차 3배 | Fixed 유지가 관건 |
| 8 | lookahead 튜닝 | 2.3m 급커브 이탈 / 1.6m 직진 악화 | 1.9m 시험 여지 |
| 9 | 10km/h | FF 기준 PWM 344 필요 (255 로도 불가) | 원차 설계속도 ~5km/h 로 보임 |

## 6.1 ~~후진주차 slot1·2 — 선택지~~ (해결됨)

`parking_planner` 도입으로 3자리 전부 계획·폐루프 통과(§4.3). "slot3 만 사용" 전략은
더 이상 필요 없다. 남은 것은 실차 확인뿐.

---

# 7. 실행 방법

## 7.1 주행 전 점검 (필수)

```bash
# ① 장치 3종
ls -l /dev/arduino /dev/imu && lsusb | grep -E "2341|1546|10c4"

# ② IMU 안정성 30초 (10번 전부 ✅ 여야 함)
for i in $(seq 1 10); do printf "%2ds " $((i*3)); \
  ls /dev/imu >/dev/null 2>&1 && echo -n "✅ " || echo -n "❌ "; \
  lsusb | grep -q 10c4 && echo "USB✅" || echo "USB❌"; sleep 3; done

# ③ 앞바퀴 손으로 정면 맞추기
# ④ 배터리 충전 확인
```

## 7.2 정상 주행 (완주 + 횡단보도 + 주차 전자동)

```bash
# 터미널1
cd ~/racing_ws && bash tools/ros_cleanup.sh && source install/setup.bash
ros2 launch gps_localization bringup.launch.py \
    control:=true auto_calib:=true rviz:=false max_speed:=1.0 \
    crosswalk:=true parking:=true parking_slot:=3

# 터미널2 — 로그 (★ 반드시 같이)
cd ~/racing_ws && source install/setup.bash && python3 tools/drive_log.py
```

**자동 흐름**: 헤딩 캘리브(10m 자동직진) → 완주 → 횡단보도 정지(3초) → 재출발 → 완주 종점 → 후진주차

## 7.3 좌표 기록 (현장, 장소 바뀌면 필수)

```bash
# 원점부터 확인/수정
cat config/site_origin.yaml

# bringup 띄우고 RTK Fixed + 헤딩 캘리브 완료 후
ros2 run mission_perception parking_recorder --ros-args -p slot:=1
ros2 run mission_perception stop_point_recorder
```
⚠ **캘리브 전에는 recorder 가 기록을 거부**한다 (yaw 가 지도 정렬 안 됨)

## 7.4 하드웨어 없이 검증

```bash
python3 tools/preflight.py                     # ★ 데이터 일관성·주행가능성 일괄 검사
python3 tools/test_parking_node.py --slot 3    # (--file 은 무시된다. --slot 을 쓸 것)
python3 tools/test_parking_node.py --slot 3 --start-pose="-25.605,103.797,166.8"
python3 tools/test_crosswalk_stop.py
python3 tools/smooth_path.py --alpha 0.10 --beta 0.50 --max-dev 0.50 --target-r 2.42
python3 tools/list_waypoints.py --plot
```

## 7.5 주요 launch 인자

| 인자 | 기본 | 설명 |
|---|---|---|
| `control` | false | 제어 체인(모터 구동) |
| `auto_calib` | false | 자동 10m 직진 캘리브 |
| `max_speed` | 2.8 | 목표속도 상한 [m/s] |
| `crosswalk` | false | 횡단보도 정지 |
| `crosswalk_bias` | 0.35 | 정지 위치 보정 [m] |
| `crosswalk_dwell` | 3.0 | 정지 유지 [s] |
| `parking` | false | 후진주차 노드 |
| `parking_slot` | 1 | 자리 번호 |
| `mission` | false | 신호등 (crosswalk 와 배타) |
| `lidar` | false | 라이다 |

---

# 8. 진단 방법론

## 8.1 "경로 끊김" 이 떠도 원인은 상류에 있다

```bash
ros2 topic echo /curvature --once          # 안 나오면 ↓
ros2 topic echo /odometry/filtered --once  # 안 나오면 ↓
ros2 topic echo /fix --once                # GPS
ros2 topic echo /handsfree/imu --once      # IMU
journalctl -k -n 40 | grep -iE "cp210|ttyUSB|disconnect"
```

⚠ **`topic info` 의 Publisher/Subscription count 는 연결만 보여줄 뿐 데이터 흐름을 보장하지 않는다.**
반드시 `echo` 로 실제 수신을 확인할 것.

## 8.2 노드 단독 검증

문제 노드를 **이름만 바꿔 하나 더 띄우면** 같은 입력으로 어떻게 되는지 볼 수 있다:
```bash
ros2 run waypoint_follower local_sliding_window_node \
    --ros-args -r __node:=lsw_debug -r /curvature:=/curvature_debug
```

## 8.3 측정 원칙 (이 프로젝트에서 반복 확인된 것)

- **예측 말고 측정.** 코드 주석·기억으로 넘겨짚으면 반드시 틀린다
- **대조군을 둬라.** D17 노이즈를 엔코더로 오인할 뻔한 것을 정지 대조군이 막았다
- **판별 도구보다 실제 동작 실적이 우선.** board_id 가 A8 로 오판정한 적 있다
- **손으로 굴려 부호를 판단하지 마라.** 방향을 착각한다. 모터로 구동해 확인할 것

---

# 9. 절대 규칙

1. **매 주행 전 `ls -l /dev/imu` 확인** — 연석 충돌의 직접 원인
2. **소스 고치면 `colcon build`** — `--symlink-install` 믿지 말 것
3. **pure_pursuit / 완주 로직 수정 금지** — 미션은 teleop 채널로만 얹는다
4. **원점(`site_origin.yaml`) 확인 먼저** — 장소 바뀌면 이것부터 (150km 전례)
4-b. **좌표 기록 직후 `python3 tools/preflight.py`** — 현장을 떠나면 되돌릴 수 없다
5. **미션 노드는 계속 발행(≥2Hz)** — 멈추면 mux 데드맨 0.5초가 정지시킨다
6. **속도는 mux `min_speed`(−0.6) 안에서** — 후진은 −0.3 권장
7. **매 실차 단계 E-stop 손에** — 무부하(공중) 테스트로 로직 먼저 잡고 실차
8. **매 세션 헤딩 캘리브 필요** — IMU yaw 는 부팅마다 리셋
9. **`mission:=true` 와 `crosswalk:=true` 동시 금지** — 같은 토픽 덮어씀
10. **보드 교체 시 2.2절 절차대로 재측정**

---

# 부록 A. 현재 좌표 데이터

```
원점: (477800.0, 3964400.0) EPSG:32652 — 대구권 시험장

완주 경로 (waypoints_recorded_resampled_0.5.yaml):
  370점 / 184.0m / 시작-끝 10.52m (열린 경로)
  시작 (-20.30, 112.88)  종점 (-25.60, 103.80) 진행방향 +166.8°
  최소R 2.81m → 필요타각 15.6° / 못 도는 구간 0개

주차 자리:
  parking_1: 59점(전진35/후진24) 시작(-20.67,102.62) 정차(-24.45,105.74)
  parking_2: 61점(전진38/후진23) 시작(-20.67,102.73) 정차(-25.30,105.40)
  parking_3: 70점(전진46/후진24) 시작(-21.23,104.58) 정차(-27.70,105.66)

정지선: (-25.09, 78.55)  — 완주경로에서 6cm
```

# 부록 B. 도구 목록

| 도구 | 용도 |
|---|---|
| **`tools/preflight.py`** | **원점·경로·정지점·주차 일괄 사전검사 (기록 직후 필수)** |
| **`tools/shift_waypoints.py`** | **좌표 YAML 원점 환산·스탬프** |
| `tools/set_origin_from_fix.py` | 현장 RTK fix → 원점 자동 세팅 |
| `tools/drive_log.py` | 주행 로그 + 원인 판정 (횡오차/헤딩/RTK 분리) |
| `tools/smooth_path.py` | 곡률 제약 경로 평활화 |
| `tools/list_waypoints.py` | 시스템 전체 웨이포인트 목록·시각화 |
| `tools/test_parking_node.py` | 주차 노드 폐루프 시뮬 |
| `tools/test_crosswalk_stop.py` | 횡단보도 시뮬 |
| `tools/hil_vehicle.py` | HIL 가상차량 (자전거모델) |
| `arduino/board_id/` | 보드 판별 (조향핀·카운터·VCC) |
| `arduino/adc_scan/` | 조향센서 핀 탐색 |
| `arduino/enc_scan/` | SPI 엔코더 카운터 탐색 |

⚠ `tools/ros_cleanup.sh` 는 프로세스를 광범위하게 kill 한다.
자동화 스크립트 안에서 호출하면 자기 셸까지 죽는다.

# 부록 C. git 상태

```
최근 커밋: 140a1ed "진단 도구 추가 — drive_scan"
미커밋 변경: 38개 (heading_init 게이트, 미션 노드, 경로, 문서 등)
```
**미션 노드 전부(`parking_node`, `crosswalk_stop_node`, `parking_recorder` 등)와
heading_init 게이트가 아직 커밋 안 됨.** 인계 시 반드시 함께 전달할 것.

---

**문서 끝. 질문이 생기면 이 문서의 "진단 방법론"(8절)부터 볼 것.**
