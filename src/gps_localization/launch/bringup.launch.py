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
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

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
  lidar = LaunchConfiguration('lidar')
  crosswalk = LaunchConfiguration('crosswalk')
  crosswalk_dwell = LaunchConfiguration('crosswalk_dwell')
  crosswalk_tolerance = LaunchConfiguration('crosswalk_tolerance')
  crosswalk_bias = LaunchConfiguration('crosswalk_bias')
  parking = LaunchConfiguration('parking')
  sequencer = LaunchConfiguration('sequencer')
  sudden_stop = LaunchConfiguration('sudden_stop')
  sudden_stop_dwell = LaunchConfiguration('sudden_stop_dwell')
  mission_plan = LaunchConfiguration('mission_plan')
  parking_slot = LaunchConfiguration('parking_slot')
  parking_dir = LaunchConfiguration('parking_dir')
  # 정지지점은 런치 시점에 파일에서 읽는다(LaunchConfiguration 은 파라미터
  # 배열로 못 넘기므로 여기서 실제 값으로 확정한다).
  # ★ waypoints:= 오버라이드를 sys.argv 에서 직접 읽는다. 안 그러면 정지지점
  #   '코스 위' 검사가 항상 기본(대구) 경로와 비교돼, 다른 장소에서 커스텀
  #   waypoints 를 넘기면 정지점이 149km 밖으로 오판돼 거부된다(2026-09-02 실측).
  wp_for_check = DEFAULT_WAYPOINTS
  for _a in sys.argv:
    if _a.startswith('waypoints:='):
      wp_for_check = _a.split(':=', 1)[1]
  stop_pts = load_stop_points(
      os.environ.get('STOP_POINTS_FILE', DEFAULT_STOP_POINTS),
      wp_for_check)

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
      DeclareLaunchArgument('max_speed', default_value='2.8'),
      # ⚠ lidar:=true 면 라이다 장애물 회피(cluster_plot_node)를 켠다. 별도로
      # sllidar 드라이버가 /scan 을 쏘고 있어야 한다:
      #   ros2 launch sllidar_ros2 sllidar_a1_launch.py \
      #       serial_port:=/dev/ttyUSB0 serial_baudrate:=256000
      # 회피 결과는 /obstacle_distance(정지)+/lidar/avoid_steer(조향 override)로
      # 나가 먹스가 AUTO 중에 반영한다. enable_plot 은 헤드리스라 기본 false.
      DeclareLaunchArgument('lidar', default_value='false'),
      # ⚠ crosswalk:=true 면 횡단보도 정지(정지선 앞 정지 → 3초 → 재출발)를 켠다.
      # 정지지점은 mission 과 같은 ~/stop_points.yaml 을 쓴다.
      # mission:=true 와 함께 켜지 말 것(같은 토픽을 서로 덮어쓴다).
      DeclareLaunchArgument('crosswalk', default_value='false'),
      DeclareLaunchArgument('crosswalk_dwell', default_value='3.0'),
      # 대회 규정: 정지선에서 64cm 이내. 판정만 하고 제어는 바꾸지 않는다.
      DeclareLaunchArgument('crosswalk_tolerance', default_value='0.64'),
      # ★ 2026-08-24 실차: 앞바퀴가 정지선 30cm 넘어감 → stop_bias 로 뒤로 민다.
      # 노드 규약: 양수 = 정지지점을 앞당겨 봐 더 뒤에서 멈춤. 앞바퀴 30cm + 5cm 여유 = 0.35
      # 2026-08-24 2차: 0.35 로도 아직 넘어감 → 0.45.
      DeclareLaunchArgument('crosswalk_bias', default_value='0.45'),
      # ⚠ parking:=true 면 후진주차 노드를 띄운다. 띄우기만 하고 대기하므로
      # 자율주행에 영향이 없다. 실행은 /parking/start 로 사람이 트리거한다.
      # ★ 미션 시퀀서 (2026-09-04)
      #   true 면 mission_sequencer 가 config/mission_plan.yaml 순서대로
      #   미션을 하나씩 arm/disarm 한다. 이때 미션 노드는 arm 을 받기 전엔
      #   아무것도 발행하지 않는다(require_arm). 미션이 8개가 되면
      #   '이건 이거랑 같이 켜지 말 것' 을 사람이 지킬 수 없다.
      DeclareLaunchArgument('sequencer', default_value='false'),
      DeclareLaunchArgument(
          'mission_plan',
          default_value='/home/han/racing_ws/config/mission_plan.yaml'),
      # ⚠ sudden_stop:=true 면 돌발 급정지 미션(라이다 전방 장애물 → 완전정지 →
      #   5초 → 재출발)을 켠다. crosswalk 와 같은 /stop_line_distance 를 쓰므로
      #   **동시에 켜려면 sequencer:=true 로 하나씩만 arm 해야 한다.**
      DeclareLaunchArgument('sudden_stop', default_value='false'),
      DeclareLaunchArgument('sudden_stop_dwell', default_value='5.0'),
      DeclareLaunchArgument('parking', default_value='false'),
      DeclareLaunchArgument('parking_slot', default_value='1'),
      DeclareLaunchArgument('parking_dir',
                            default_value=os.path.expanduser('~')),

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

      # 7-c. 라이다 장애물 회피 (lidar:=true 일 때만)
      # /scan → DBSCAN+트래킹+FollowGap → /obstacle_distance + /lidar/avoid_steer.
      # 종방향이 obstacle_distance 로 감속/정지, 먹스가 avoid_steer 로 조향 회피.
      Node(
          package='lidar_clustering',
          executable='cluster_plot_node',
          name='lidar_clustering',
          output='screen',
          condition=IfCondition(lidar),
          parameters=[{
              'enable_plot': False,
              'require_arm_for_steer': ParameterValue(sequencer, value_type=bool)}],
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

      # 7-c. 횡단보도 정지 (crosswalk:=true 일 때만)
      # 정지선 앞에 서고 → 3초 대기 → 다시 출발. 신호등과 무관하게 무조건 선다.
      # ⚠ mission:=true 와 **동시에 켜지 말 것** — 둘 다 /stop_line_distance 를
      #   발행해 서로 덮어쓴다. 노드가 기동 시 발행자 수를 세어 경고한다.
      Node(
          package='mission_perception',
          executable='crosswalk_stop_node',
          name='crosswalk_stop',
          output='screen',
          condition=IfCondition(crosswalk),
          # ParameterValue 로 타입을 못박는다 — dwell:=3 처럼 정수로 주면
          # 문자열 "3" 이 int 로 추론돼 double 파라미터와 타입이 어긋난다.
          parameters=[{
              'stop_points': stop_pts,
              'dwell': ParameterValue(crosswalk_dwell, value_type=float),
              'stop_tolerance': ParameterValue(crosswalk_tolerance,
                                               value_type=float),
              'stop_bias': ParameterValue(crosswalk_bias,
                                          value_type=float),
              # 시퀀서를 쓰면 arm 을 받기 전엔 발행하지 않는다.
              'require_arm': ParameterValue(sequencer, value_type=bool)}],
      ),

      # 7-d. 후진주차 (parking:=true 일 때만)
      # 띄워도 /parking/start 를 받기 전엔 **아무것도 발행하지 않는다**(자율주행을
      # 덮어쓰지 않기 위해). 완주 뒤 손으로 트리거하는 것이 가장 안전하다:
      #   ros2 topic pub --once /parking/start std_msgs/Bool "{data: true}"
      Node(
          package='mission_perception',
          executable='parking_node',
          name='parking_node',
          output='screen',
          condition=IfCondition(parking),
          parameters=[{
              'slot': ParameterValue(parking_slot, value_type=int),
              'pose_dir': ParameterValue(parking_dir, value_type=str),
              'auto_start': False,
              # ★ 시퀀서가 있으면 /goal_reached 자동 트리거를 끈다.
              #   트리거 주인이 둘이면 시퀀서가 아직 arm 하지 않았는데
              #   완주 신호만으로 주차가 시작된다.
              'trigger_on_goal_reached': ParameterValue(
                  PythonExpression(["'", sequencer, "'.lower() != 'true'"]),
                  value_type=bool)}],
      ),

      # 7-d2. 돌발 급정지 (sudden_stop:=true 일 때만)
      # 라이다 /obstacle_distance 만 보고 판단한다 — 카메라가 필요 없다.
      # 서는 것은 longitudinal 이 이미 하고, 이 노드는 '정지 확인 → 5초 유지 →
      # 재출발' 상태기계와, 안 치워졌을 때 빠져나오는 경로를 담당한다.
      Node(
          package='mission_perception',
          executable='sudden_stop_node',
          name='sudden_stop',
          output='screen',
          condition=IfCondition(sudden_stop),
          parameters=[{
              'dwell': ParameterValue(sudden_stop_dwell, value_type=float),
              'require_arm': ParameterValue(sequencer, value_type=bool)}],
      ),

      # 7-e. 미션 시퀀서 (sequencer:=true 일 때만)
      # 코스 진행거리(s)를 보고 미션을 **한 번에 하나만** arm 한다.
      # 타임아웃이 지나면 강제로 disarm 하고 자율로 복귀시킨다 —
      # 미션 실패는 감점이지만 그 자리에 멈춰 있으면 탈락이기 때문이다.
      Node(
          package='mission_perception',
          executable='mission_sequencer',
          name='mission_sequencer',
          output='screen',
          condition=IfCondition(sequencer),
          parameters=[{
              'plan_file': ParameterValue(mission_plan, value_type=str)}],
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
