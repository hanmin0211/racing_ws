#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""check_build.py — 소스 수정이 **실제로 실행되는 코드에 반영됐는지** 본다.

★ 왜 필요한가 (2026-09-14)
  이 워크스페이스는 패키지마다 설치 방식이 다르다. 어떤 것은 소스를 바로
  물고(egg-link), 어떤 것은 **빌드 때 복사**된다. 후자는 소스를 고쳐도
  colcon build 를 안 하면 옛 코드가 그대로 돈다.

  그날 밤 이걸로 두 번 헛돌았다:
    · IMU 자동재연결을 만들어 놓고 "적용됐다" 고 말했는데 안 돌고 있었다
    · 점프 게이트 수정도 몇 시간 동안 반영이 안 된 채였다
  둘 다 현장에서 "왜 아직도 같은 증상이지?" 로 시간을 먹었다.

  고친 내용이 실행 코드에 있는지 **표식 문자열**로 확인한다.

사용:
  python3 tools/check_build.py          # 전부 확인
  (❌ 가 있으면)  colcon build --packages-select <패키지> --symlink-install
"""

import importlib
import inspect
import sys

# 모듈 → (패키지, [있어야 하는 표식])
CHECKS = {
    'gps_heading_init.heading_init_node':
        ('gps_heading_init', ['_rtk_why', 'min_fix_dt', 'header.stamp']),
    'velocity_controller.serial_bridge_node':
        ('velocity_controller', ['ff_breakaway_pwm', 'ff_min_pwm']),
    'handsfree_ros2_imu.hfi_a9_ros2':
        ('handsfree_ros2_imu', ['_reconnect', 'stall_timeout']),
    'mission_perception.mission_sequencer':
        ('mission_perception', ['inhibits', 'allow_full_course']),
    'lidar_clustering.follow_gap_planner':
        ('lidar_clustering', ['prefer_path_gap', 'safety_radius']),
    'lidar_clustering.cluster_plot_node':
        ('lidar_clustering', ['require_arm_for_steer', 'blocked_escape_s']),
}


def main():
  bad = []
  print(f"{'모듈':<45} {'상태'}")
  print('─' * 70)
  for mod, (pkg, marks) in CHECKS.items():
    try:
      m = importlib.import_module(mod)
      src = inspect.getsource(m)
    except Exception as e:  # noqa: BLE001
      print(f'{mod:<45} ⚠ 임포트 실패 ({e})')
      continue
    miss = [k for k in marks if k not in src]
    if miss:
      bad.append(pkg)
      print(f'{mod:<45} ❌ 없음: {", ".join(miss)}')
    else:
      print(f'{mod:<45} ✅')
  print()
  if bad:
    print('소스 수정이 실행 코드에 없다. 빌드할 것:')
    print(f"  colcon build --symlink-install --packages-select {' '.join(sorted(set(bad)))}")
    print('  (빌드 전에 source /opt/ros/humble/setup.bash 를 먼저 할 것)')
    return 1
  print('✅ 전부 반영돼 있다.')
  return 0


if __name__ == '__main__':
  sys.exit(main())
