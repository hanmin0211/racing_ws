#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ngii_rtk.launch.py
==================
F9P + NGII VRS RTK 전체 파이프라인 (USB 하나로 완결).

  ros2 launch ngii_ntrip ngii_rtk.launch.py

실행 노드:
  1. ublox_dgnss_node        : F9P(USB) 드라이버. 멀티GNSS + RTK rover 설정,
                               /ntrip_client/rtcm 을 구독해 F9P에 RTCM 주입,
                               UBX 고정밀 위치 메시지 발행.
  2. ublox_nav_sat_fix_hp    : 위 UBX를 sensor_msgs/NavSatFix(/fix)로 변환.
  3. vrs_ntrip_client        : NGII VRS 접속(GGA 전송) → RTCM을
                               /ntrip_client/rtcm 으로 발행.

비밀번호는 환경변수로 주는 걸 권장:
  NGII_PW='ngii' ros2 launch ngii_ntrip ngii_rtk.launch.py

트랙 위치가 충주와 많이 다르면 lat/lon 인자로 지정:
  ros2 launch ngii_ntrip ngii_rtk.launch.py lat:=37.1 lon:=127.9
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
  lat = LaunchConfiguration('lat')
  lon = LaunchConfiguration('lon')
  height = LaunchConfiguration('height')
  password = LaunchConfiguration('password')

  # F9P 설정: 멀티GNSS + RTK rover + 차량 동적모델
  ublox_params = [{
      'DEVICE_FAMILY': 'F9P',
      'FRAME_ID': 'gps',
      # 출력 메시지 (고정밀 위치 + 상태 + 공분산 + RTCM 수신확인)
      'CFG_USBOUTPROT_NMEA': False,
      'CFG_RATE_MEAS': 100,          # 측정 주기 100ms = 10Hz
      'CFG_RATE_NAV': 1,
      'CFG_MSGOUT_UBX_NAV_HPPOSLLH_USB': 1,
      'CFG_MSGOUT_UBX_NAV_STATUS_USB': 1,
      'CFG_MSGOUT_UBX_NAV_COV_USB': 1,
      'CFG_MSGOUT_UBX_RXM_RTCM_USB': 1,   # RTCM이 F9P에 실제 들어오는지 확인용
      # 멀티 GNSS 전부 켜기 (RTK Fixed 속도/유지 개선)
      'CFG_SIGNAL_GPS_ENA': True,
      'CFG_SIGNAL_GLO_ENA': True,
      'CFG_SIGNAL_GAL_ENA': True,
      'CFG_SIGNAL_BDS_ENA': True,
      'CFG_SIGNAL_QZSS_ENA': True,
      # RTK rover: DGNSS 모드 3 = RTK fixed(가능하면 fixed, 아니면 float)
      'CFG_NAVHPG_DGNSSMODE': 3,
      # 차량 동적모델(4 = automotive)
      'CFG_NAVSPG_DYNMODEL': 4,
  }]

  return LaunchDescription([
      DeclareLaunchArgument('lat', default_value='36.9706'),
      DeclareLaunchArgument('lon', default_value='127.8748'),
      DeclareLaunchArgument('height', default_value='100.0'),
      DeclareLaunchArgument(
          'password',
          default_value=EnvironmentVariable('NGII_PW', default_value='ngii')),

      Node(
          package='ublox_dgnss_node',
          executable='ublox_dgnss_node',
          name='ublox_dgnss',
          output='screen',
          parameters=ublox_params,
      ),
      Node(
          package='ublox_nav_sat_fix_hp_node',
          executable='ublox_nav_sat_fix_hp',
          name='ublox_nav_sat_fix_hp',
          output='screen',
      ),
      Node(
          package='ngii_ntrip',
          executable='vrs_ntrip_client',
          name='ngii_vrs_ntrip_client',
          output='screen',
          parameters=[{
              'host': 'RTS2.ngii.go.kr',
              'port': 2101,
              'mountpoint': 'VRS-RTCM32',
              'username': '<NGII_ID>',
              'password': password,
              'lat': lat,
              'lon': lon,
              'height': height,
              'gga_interval': 1.0,
              'rtcm_topic': '/ntrip_client/rtcm',
          }],
      ),
  ])
