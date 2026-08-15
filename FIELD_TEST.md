# 현장 테스트 순서 (RTK + 헤딩초기화 + 웨이포인트)

> 목표: ① RTK Fixed 확인 → ② IMU 헤딩 초기화(10m 직진) → ③ 웨이포인트 찍기(1m) + 0.5m 리샘플
> 장소: **하늘 트인 곳**(건물/나무 없는 개활지). RTK는 하늘이 잘 보여야 잡힘.

---

## 0. 준비 (매번 1회)

하드웨어 연결 확인:
- [ ] F9P GPS → USB 허브 (안테나 하늘 향하게, 그라운드플레인 위)
- [ ] A9 IMU → USB (`/dev/ttyUSB0`)
- [ ] 인터넷 연결(NTRIP용 — 폰 핫스팟 등)

빌드 + 소싱 (코드 바뀐 게 있으면):
```bash
cd /home/han/racing_ws
colcon build
source install/setup.bash
```
> 이후 여는 **모든 터미널에서** 먼저: `cd /home/han/racing_ws && source install/setup.bash`

포트 확인:
```bash
ls /dev/ttyUSB0            # IMU 있는지
lsusb | grep -i u-blox     # GPS 있는지
```

---

## 1. RTK GPS 켜기  → **터미널 1**
```bash
NGII_PW=ngii ros2 launch ngii_ntrip ngii_rtk.launch.py
```
로그 확인:
- `NTRIP 연결 성공: ICY 200 OK`
- `RTCM ...프레임 발행` (계속 증가)
- ublox 쪽에 RXM_RTCM 수신 로그

### ✅ RTK Fixed 확인 (별도 터미널)
```bash
ros2 topic echo /fix --once
```
- `position_covariance` 첫 값이 **0.001 이하(≈cm)** 면 **RTK Fixed** ✅
- 값이 크면(수 m²) 아직 단독측위 → **1~3분 기다리며 하늘 잘 보이는 곳에서 대기**
- 팁: 처음엔 Float(수십 cm)였다가 Fixed(cm)로 내려감

> ⚠️ RTK Fixed 안 되면 이후 정확도가 안 나옵니다. 여기서 꼭 Fixed 확인하고 진행.

---

## 2. IMU 켜기  → **터미널 2**
```bash
ros2 launch handsfree_ros2_imu handsfree_imu.launch.py
```
확인:
```bash
ros2 topic hz handsfree/imu      # 값이 나오는지 (보통 수십~100Hz)
```

---

## 3. 로컬라이제이션(헤딩초기화 + EKF) 켜기  → **터미널 3**
```bash
ros2 launch robot_localization_config gps_imu_fusion.launch.py
```
뜨는 노드: heading_init + ekf + navsat + static TF (5개)
로그: `헤딩 초기화 노드 시작: 10m 직진하면...`, `시작점 기록: (...)`

---

## 4. IMU 헤딩 초기화 — **10m 직진**
차량을 **똑바로 10m 이상 직진**시킨다 (천천히, 곧게).
- 터미널 3 로그에 진행상황: `직진 중... 3.0/10m` ...
- 완료 시:
```
✅ 헤딩 초기화 완료: 10.x m 직진. GPS course=..°, IMU yaw=..°, → yaw_offset=..°
```
- [ ] **yaw_offset 값 기록** (사진/메모)

확인:
```bash
ros2 topic echo /heading/yaw_offset --once      # 오프셋(rad)
ros2 topic echo /odometry/filtered --once       # 융합된 위치+yaw 나오는지
```
> 다시 하려면: `ros2 service call /gps_heading_init/reset std_srvs/srv/Trigger` 후 또 10m 직진

---

## 5. 웨이포인트 찍기  → **터미널 4**
```bash
ros2 run waypoint_follower waypoint_recorder
```
(RTK Fixed일 때만 찍고 싶으면: `... waypoint_recorder --ros-args -p require_rtk:=true`)

- 차량으로 **원하는 경로를 주행** → 1m마다 `점 기록 #N` 로그
- 다 찍었으면 **차량 정지 후 3초 대기** → 자동 저장:
```
✅ 저장 완료: N개 점(원본) → .../waypoints_recorded.yaml
✅ 리샘플 저장: M개 점 @ 0.5m → .../waypoints_recorded_resampled_0.5.yaml
```
- (수동 저장도 가능: `ros2 service call /waypoint_recorder/save std_srvs/srv/Trigger`)
- [ ] 저장된 파일 2개 생겼는지 확인:
```bash
ls -l /home/han/racing_ws/src/pure_pursuit_pkg/config/waypoints_recorded*.yaml
```

---

## 6. (선택) 찍은 경로 확인
```bash
# 점 개수/앞부분 보기
head -20 /home/han/racing_ws/src/pure_pursuit_pkg/config/waypoints_recorded_resampled_0.5.yaml
```

---

## 종료
각 터미널 `Ctrl-C` (레코더는 Ctrl-C 시에도 저장됨).

---

## 빠른 문제 해결
| 증상 | 확인 |
|---|---|
| NTRIP 401 | 비번 `ngii` 맞는지, 인터넷 되는지 |
| RTCM 0프레임 | 인터넷/방화벽, GGA 위치가 트랙과 너무 멀면 lat/lon 인자로 지정 |
| RTK Fixed 안 됨 | 하늘 개활지로, 안테나 위치, 멀티GNSS(이미 켜짐), 1~3분 대기 |
| `/fix` 안 나옴 | ublox_dgnss 로그 에러, USB 연결, `lsusb` |
| 헤딩 완료 안 뜸 | 10m 직진 실제로 했는지, `/fix`·`handsfree/imu` 둘 다 살아있는지 |
| 점 기록 안 됨 | `/fix` 나오는지, require_rtk면 Fixed 됐는지 |

## 확정된 접속 정보
- NTRIP: RTS2.ngii.go.kr:2101 / VRS-RTCM32 / <NGII_ID> / `ngii`
- 원점(UTM52N): x=399848.522, y=4092209.171 (lat 36.970661, lon 127.874852)
