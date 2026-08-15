# pure_pursuit_pkg — HENES T870 Pure Pursuit 경로추종 패키지

## 0. 이 패키지가 하는 일

```
/fix (GPS)  ─┐
             ├─▶ gps_local_realtime_node ─▶ /vehicle_local_pose (Odometry)
/navheading ─┘                                       │
                                                       ▼
   waypoints_local_resampled_0.3.yaml ─▶ pure_pursuit_node ─▶ /drive (AckermannDriveStamped)
                                                       │
                                                       ▼
                                          /pure_pursuit/markers (RViz 시각화)
```

검증 완료 사항 (본 대화에서 순수 파이썬으로 수치 검증):
- 반지름 5m 원형 경로에서 조향각이 이론값 `atan(L/R)`과 0.001도 미만 오차로 일치
- 직선 경로에서 좌/우 이탈 시 조향 방향(부호) 기하학적으로 정확
- GPS→로컬좌표 변환값이 기존 `waypoints_local.yaml`과 일치

**미검증 사항**: 실제 ROS2 환경에서의 빌드/실행 (샌드박스에 ROS2가 없어 실행 테스트 불가). 아래 안전 절차를 반드시 따라주세요.

---

## 1. 설치

```bash
# 필요 패키지 설치
sudo apt install ros-humble-ackermann-msgs
pip3 install pyproj --break-system-packages

# 워크스페이스에 배치
cp -r pure_pursuit_pkg ~/racing_ws/src/
cd ~/racing_ws
colcon build --packages-select pure_pursuit_pkg
source install/setup.bash
```

## 2. 설정 파일 수정 (필수)

`~/racing_ws/src/pure_pursuit_pkg/config/pure_pursuit_params.yaml` 열어서:

1. **`origin_lat`, `origin_lon`**: `gps_to_local_fit.py` 실행 시 로그에 찍힌 원점 값과 **정확히 동일하게** 입력
   (지금까지 쓰신 waypoint 파일들의 첫 점이 원점 `(0,0)`이 되도록 만들어진 원점입니다.
   `waypoints_local.yaml`의 원본이 된 위경도 파일의 **첫 번째 점**의 위경도를 넣으면 됩니다:
   `origin_lat: 36.970661166666666`, `origin_lon: 127.87485216666667`)
2. **`path_file`**: 실제 경로 (`~/gps_converter/waypoints_local_resampled_0.3.yaml`)
3. **`wheelbase`**: 0.88 (이미 반영됨)
4. **`target_speed`**: 처음엔 반드시 낮게 (0.3~0.5 m/s 권장), 튜닝하며 점진적으로 올리기

빌드 후 다시 반영하려면:
```bash
colcon build --packages-select pure_pursuit_pkg
source install/setup.bash
```

## 3. 실행

```bash
ros2 launch pure_pursuit_pkg pure_pursuit.launch.py
```

RViz에서 확인 (선택):
```bash
rviz2
# Fixed Frame: map
# Add -> By topic -> /pure_pursuit/markers -> MarkerArray
```
- 파란 구: 현재 차량 위치
- 초록 구: 가장 가까운 waypoint (closest)
- 빨간 구: lookahead point (목표점)
- 노란 선: 현재 로컬 윈도우(3차 피팅에 쓰인 구간)

## 4. 실차 튜닝 순서 (반드시 이 순서로)

### 4-1. 바퀴 띄운 상태 드라이런
차량을 잭 등으로 들어 바퀴가 지면에 안 닿게 한 뒤 launch 실행.
- 에러 없이 노드가 뜨는지
- `/vehicle_local_pose`가 실제 GPS 이동에 맞게 값이 변하는지 (`ros2 topic echo /vehicle_local_pose`)
- `/drive`의 `steering_angle`이 차량을 옆으로 이동시켜보며 좌우 부호가 맞는지 확인
  (차를 왼쪽 스으로 이동시키면 오른쪽으로 조향 명령이 나와야 정상 — 경로 쪽으로 되돌아오려는 방향)

### 4-2. 저속 직선 구간 테스트
- `target_speed: 0.3` 정도로 짧은 직선 구간에서 실주행
- 차량이 경로 중앙을 잘 유지하는지, 좌우로 심하게 떨지(oscillation) 않는지 확인

**떨림(oscillation) 발생 시**: `min_lookahead`, `fixed_lookahead`를 늘리세요 (너무 짧으면 과민반응해서 떪).
**코너를 너무 크게 도는(cut 안 되는) 경우**: lookahead를 줄이세요.
이건 Pure Pursuit의 잘 알려진 트레이드오프입니다 (lookahead가 클수록 부드럽지만 코너를 자름, 작을수록 정밀하지만 떨림에 민감).

### 4-3. 곡선 구간 확대
직선에서 안정적이면 코너 포함 구간으로 확대. `curvature_speed_gain`으로 코너 감속 강도 조절
(값을 높이면 급커브에서 더 많이 감속).

### 4-4. 목표 속도까지 단계적 증가
`target_speed`를 0.1~0.2 m/s씩 올리면서 매번 안정성 확인. 속도가 오르면 `max_lookahead`도 같이
키워줘야 안정적입니다 (적응형 lookahead가 자동으로 처리하긴 하지만 `k_ld`, `max_lookahead`로 상한 조절).

## 5. 문제 해결

| 증상 | 원인 후보 | 대응 |
|---|---|---|
| 차량이 경로와 무관하게 엉뚱한 방향으로 감 | `origin_lat/lon`이 waypoint 생성시와 다름 | config의 원점 값 재확인 |
| 조향이 항상 한쪽으로 치우침 | heading(`/navheading`) 부호/오프셋 문제 | `ros2 topic echo /navheading`로 원시값 확인, 필요시 yaw에 오프셋 보정 추가 |
| 좌우로 심하게 떪 | lookahead 너무 짧음 | `min_lookahead`, `fixed_lookahead` 증가 |
| 코너를 심하게 자름(안쪽으로 파고듦) | lookahead 너무 긺 | lookahead 감소, `curvature_speed_gain` 증가 |
| `/drive` 안 나옴 | `/vehicle_local_pose` 안 들어옴 | GPS/헤딩 토픽 실제 발행 여부 `ros2 topic hz /fix` 확인 |
