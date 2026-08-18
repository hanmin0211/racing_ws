#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
bringup.launch.py
=================
전체 스택을 한 번에 실행하는 마스터 런치.

  ros2 launch gps_localization bringup.launch.py

실행 구성:
  1. RTK GPS      : ngii_rtk.launch.py (ublox_dgnss + nav_sat_fix + NTRIP)
  2. IMU          : handsfree A9 (/dev/imu)
  3. heading_init : 10m 직진 후 yaw_offset 계산 → 발행 후 자동 종료
  4. direct_localization : /fix + IMU + yaw_offset → /odometry/filtered
  5. global_path_publisher : 웨이포인트 → /global_path
  6. local_sliding_window_node : 3차곡선 → /local_path (앞 10m)
  7. rviz2        : 시각화 (rviz:=false 로 끌 수 있음)

인자:
  rviz         : RViz 실행 여부 (기본 true)
  waypoints    : 웨이포인트 파일 경로
  calib_distance : 헤딩 캘리브 직진 거리[m] (기본 10)
  imu_port     : IMU 시리얼 포트 (기본 /dev/imu)
  control      : 제어 체인(모터 구동) 실행 여부 (기본 false)
  mission      : 신호등 연동 실행 여부 (기본 false).
                 정지지점은 ~/stop_points.yaml 에서 읽는다
                 (STOP_POINTS_FILE 환경변수로 다른 파일 지정 가능)

비밀번호: ngii_rtk가 env NGII_PW(기본 'ngii')를 사용.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

DEFAULT_WAYPOINTS = ('/home/han/racing_ws/src/pure_pursuit_pkg/config/'
                     'waypoints_recorded_resampled_0.5.yaml')
# stop_point_recorder 가 저장하는 기본 경로
DEFAULT_STOP_POINTS = os.path.expanduser('~/stop_points.yaml')


def load_stop_points(path, waypoints_file):
  """정지지점 YAML → traffic_light_bridge 가 받는 평탄 리스트 [x1,y1,x2,y2,...].

  좌표를 사람이 손으로 옮겨 적으면 자리수 하나 틀려도 차가 엉뚱한 데서 선다.
  파일에서 직접 읽어 그 경로를 없앤다.

  ★ 두 가지를 반드시 검증한다 (2026-08-17 실제로 걸린 문제)
    1) 원점 일치 — 파일에 기록된 origin 이 현재 site_origin.yaml 과 다르면
       좌표를 현재 원점으로 환산한다. 안 하면 통째로 평행이동한 위치가 된다.
    2) 코스 위에 있는가 — 환산해도 경로에서 멀면 다른 장소에서 찍은 것이다.
       그대로 쓰면 '조용히 아무 데서도 안 서는' 상태가 되어 대회 중 원인을
       찾기 어렵다. 그럴 땐 아예 비활성화하고 크게 경고한다.

  파일이 없으면 빈 리스트. bridge 는 '정지지점 0개 → 항상 제약 없음'으로
  정상 동작한다(신호등 미션 없는 연습 주행에서 런치가 죽으면 안 되므로).
  """
  try:
    import yaml
  except ImportError:
    return []
  try:
    with open(path) as f:
      d = yaml.safe_load(f) or {}
  except FileNotFoundError:
    print(f'[bringup] ⚠ 정지지점 파일 없음: {path} — 신호등 정지 비활성')
    return []
  except Exception as e:  # noqa: BLE001
    print(f'[bringup] ⚠ 정지지점 로드 실패({e}) — 신호등 정지 비활성')
    return []

  pts = [(float(p['x']), float(p['y'])) for p in (d.get('stop_points') or [])]
  if not pts:
    print(f'[bringup] ⚠ {path} 에 정지지점이 없다 — 신호등 정지 비활성')
    return []

  # 1) 원점 환산
  try:
    with open('/home/han/racing_ws/config/site_origin.yaml') as f:
      site = yaml.safe_load(f) or {}
    cur_x, cur_y = float(site['origin_x']), float(site['origin_y'])
  except Exception:  # noqa: BLE001
    cur_x = cur_y = None
  old_x, old_y = d.get('origin_x'), d.get('origin_y')
  if None not in (cur_x, cur_y, old_x, old_y):
    dx, dy = float(old_x) - cur_x, float(old_y) - cur_y
    if abs(dx) > 0.01 or abs(dy) > 0.01:
      print(f'[bringup] 정지지점 원점 환산: ({old_x}, {old_y}) → ({cur_x}, {cur_y})')
      pts = [(x + dx, y + dy) for (x, y) in pts]

  # 2) 중복 제거 (엔터를 두 번 눌러 같은 점이 들어가는 일이 있다)
  uniq = []
  for p in pts:
    if not any(abs(p[0] - q[0]) < 0.3 and abs(p[1] - q[1]) < 0.3 for q in uniq):
      uniq.append(p)
  if len(uniq) != len(pts):
    print(f'[bringup] 정지지점 중복 {len(pts) - len(uniq)}개 제거')
  pts = uniq

  # 3) 코스 위에 있는지 확인
  try:
    with open(waypoints_file) as f:
      wp = [(float(q['x']), float(q['y']))
            for q in (yaml.safe_load(f) or {}).get('waypoints', [])]
  except Exception:  # noqa: BLE001
    wp = []
  if wp:
    far = []
    for (x, y) in pts:
      dmin = min(((x - a) ** 2 + (y - b) ** 2) ** 0.5 for (a, b) in wp)
      if dmin > 50.0:
        far.append(((x, y), dmin))
    if far:
      print('[bringup] ❌ 정지지점이 경로에서 멀다 — 다른 장소에서 찍은 파일이다:')
      for (p, dm) in far:
        print(f'           ({p[0]:.1f}, {p[1]:.1f}) 경로까지 {dm / 1000:.1f}km')
      print('           신호등 정지를 **비활성화**한다. '
            '이 코스에서 stop_point_recorder 로 다시 찍을 것.')
      return []

  flat = []
  for (x, y) in pts:
    flat += [x, y]
  print(f'[bringup] 정지지점 {len(pts)}개 로드: {path}')
  return flat


def generate_launch_description():
  ngii_share = get_package_share_directory('ngii_ntrip')
  wpf_share = get_package_share_directory('waypoint_follower')
  pp_share = get_package_share_directory('pure_pursuit_pkg')
  rviz_config = os.path.join(wpf_share, 'rviz', 'waypoint_follower.rviz')

  rviz = LaunchConfiguration('rviz')
  waypoints = LaunchConfiguration('waypoints')
  calib_distance = LaunchConfiguration('calib_distance')
  imu_port = LaunchConfiguration('imu_port')
  invert_imu_yaw = LaunchConfiguration('invert_imu_yaw')
  control = LaunchConfiguration('control')
  arduino_port = LaunchConfiguration('arduino_port')
  auto_calib = LaunchConfiguration('auto_calib')
  auto_calib_speed = LaunchConfiguration('auto_calib_speed')
  mission = LaunchConfiguration('mission')
  max_speed = LaunchConfiguration('max_speed')
  # 정지지점은 런치 시점에 파일에서 읽는다(LaunchConfiguration 은 파라미터
  # 배열로 못 넘기므로 여기서 실제 값으로 확정한다).
  stop_pts = load_stop_points(
      os.environ.get('STOP_POINTS_FILE', DEFAULT_STOP_POINTS),
      DEFAULT_WAYPOINTS)

  return LaunchDescription([
      DeclareLaunchArgument('rviz', default_value='true'),
      DeclareLaunchArgument('waypoints', default_value=DEFAULT_WAYPOINTS),
      DeclareLaunchArgument('calib_distance', default_value='10.0'),
      DeclareLaunchArgument('imu_port', default_value='/dev/imu'),
      # 헤딩 반전 진단: 회전 시 화살표가 반대로 돌면 invert_imu_yaw:=true 로 실행.
      # heading_init·direct_localization 둘 다에 같은 값으로 전달됨.
      DeclareLaunchArgument('invert_imu_yaw', default_value='false'),
      # ⚠ control:=true 면 제어 체인(local_pure_pursuit+serial_bridge)까지 켜져
      # 실제 모터가 구동된다. 기본 false(안전). 포트는 auto(자동감지).
      DeclareLaunchArgument('control', default_value='false'),
      DeclareLaunchArgument('arduino_port', default_value='auto'),
      # ⚠ auto_calib:=true 면 헤딩 캘리브 10m를 차가 스스로 직진해서 한다.
      # 헤딩을 모르는 개루프 전진이므로 앞을 비우고 E-stop을 쥔 채로 쓸 것.
      # control:=true 여야 실제로 움직인다(serial_bridge가 있어야 명령이 나감).
      DeclareLaunchArgument('auto_calib', default_value='false'),
      DeclareLaunchArgument('auto_calib_speed', default_value='0.3'),
      # ⚠ mission:=true 면 신호등 연동(traffic_light_bridge)이 켜진다.
      # 파이(Hailo)가 /traffic_light_state 를 쏘고 있어야 의미가 있고,
      # 정지지점은 ~/stop_points.yaml 에서 읽는다(STOP_POINTS_FILE 로 변경 가능).
      # 기본 false — 정지지점이 없는데 켜면 차가 엉뚱한 데서 설 수 있다.
      DeclareLaunchArgument('mission', default_value='false'),
      # 목표 주행속도[m/s]. 직선 순항속도이며 코너는 곡률 감속으로 자동으로
      # 느려진다. 5 km/h = 1.39. 무게중심이 높으면(배터리 뱅크 등) 낮게 시작해
      # 코너 거동을 보고 올릴 것. 먹스·v_max·조향 상한에 함께 전달된다.
      DeclareLaunchArgument('max_speed', default_value='1.0'),

      # 1. RTK GPS (ublox_dgnss + nav_sat_fix + NTRIP)
      IncludeLaunchDescription(
          PythonLaunchDescriptionSource(
              os.path.join(ngii_share, 'launch', 'ngii_rtk.launch.py'))),

      # 2. IMU
      Node(
          package='handsfree_ros2_imu',
          executable='hfi_a9_ros2',
          name='imu',
          output='screen',
          parameters=[{'port': imu_port, 'gra_normalization': True}],
      ),

      # 3. 헤딩 초기화 (10m 직진 후 자동 종료)
      Node(
          package='gps_heading_init',
          executable='heading_init_node',
          name='gps_heading_init',
          output='screen',
          parameters=[{
              'fix_topic': '/fix',
              'imu_topic': 'handsfree/imu',
              'calib_distance': calib_distance,
              'invert_imu_yaw': invert_imu_yaw,
              'auto_drive': auto_calib,
              'auto_speed': auto_calib_speed,
          }],
      ),

      # 4. 직접 로컬라이제이션
      Node(
          package='gps_localization',
          executable='direct_localization_node',
          name='direct_localization',
          output='screen',
          parameters=[{
              'fix_topic': '/fix',
              'imu_topic': 'handsfree/imu',
              'output_topic': '/odometry/filtered',
              # 원점은 지정하지 않는다 — config/site_origin.yaml 이 정본이고
              # 노드가 직접 읽는다. 여기서 덮어쓰면 waypoint_recorder 와
              # 어긋날 수 있다(어긋나면 전역경로가 통째로 평행이동한다).
              'rate': 30.0,
              'publish_tf': True,
              'invert_imu_yaw': invert_imu_yaw,
          }],
      ),

      # 5. 전역경로
      Node(
          package='waypoint_follower',
          executable='global_path_publisher',
          name='global_path_publisher',
          output='screen',
          parameters=[{'path_file': waypoints}],
      ),

      # 6. 로컬 슬라이딩 윈도우
      Node(
          package='waypoint_follower',
          executable='local_sliding_window_node',
          name='local_sliding_window_node',
          output='screen',
      ),

      # 7. 제어 체인 (control:=true 일 때만 — 실제 모터 구동)
      IncludeLaunchDescription(
          PythonLaunchDescriptionSource(
              os.path.join(pp_share, 'launch', 'control.launch.py')),
          condition=IfCondition(control),
          # ★ max_speed 를 제어 체인에 전달한다. 예전엔 arduino_port 만 넘겨서
          # v_max·먹스 상한이 항상 기본 1.0 에 걸렸다(2026-08-18 발견).
          # 목표속도는 여기서 정한다: 5 km/h = 1.39 m/s.
          launch_arguments={
              'arduino_port': arduino_port,
              'max_speed': max_speed,
          }.items(),
      ),

      # 7-b. 신호등 → 정지선거리 (mission:=true 일 때만)
      # 제어 쪽은 한 줄도 안 바뀐다. longitudinal_controller 의 정지선 감속
      # 로직에 /stop_line_distance 입력만 채워주는 어댑터다.
      Node(
          package='mission_perception',
          executable='traffic_light_bridge',
          name='traffic_light_bridge',
          output='screen',
          condition=IfCondition(mission),
          parameters=[{'stop_points': stop_pts}],
      ),

      # 8. RViz (rviz:=false 로 끌 수 있음)
      Node(
          package='rviz2',
          executable='rviz2',
          name='rviz2',
          arguments=['-d', rviz_config],
          output='screen',
          condition=IfCondition(rviz),
      ),
  ])
