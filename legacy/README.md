# legacy — 초기 개발 도구 (2026-07)

`racing_ws`를 만들기 전, 별도 폴더에서 좌표 변환과 웨이포인트 처리를 실험하던 코드입니다.
여기서 검증한 아이디어가 이후 `waypoint_follower`, `tools/smooth_path.py`, `site_origin` 로더로 발전했습니다.

> `COLCON_IGNORE` 파일이 있으므로 colcon 빌드에서 제외됩니다.

| 폴더 | 원래 위치 | 내용 | 발전한 곳 |
|---|---|---|---|
| `gps_converter/` | `~/gps_converter` | 위경도 → ENU/로컬 변환, 리샘플링, 다항식·윈도우 피팅, 점프 탐지 | `waypoint_follower/local_path_core.py`, `tools/latlon_to_local.py` |
| `waypoints/` | `~/waypoints` | 초기 수집 웨이포인트 (2026-07-13) | `config/*/wp_*.yaml` |
| `waypoint_save_pkg/` | `~/ros2_ws/src` | 최초 웨이포인트 기록 ROS 2 노드 | `waypoint_follower/waypoint_recorder.py` |

## 주요 스크립트

| 파일 | 역할 |
|---|---|
| `gps_to_local_fit.py` | GPS 경로를 로컬 좌표로 변환하고 곡선 피팅 |
| `local_window_fit.py` | 차량 주변 윈도우 구간 3차 피팅 (슬라이딩 윈도우의 원형) |
| `resample_waypoints.py` | 등간격 리샘플링 |
| `find_jump.py`, `check_spacing.py` | GPS 점프·간격 이상 탐지 |
| `world_to_vehicle.py` | 월드 → 차량 좌표 변환 |
