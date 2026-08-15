#!/usr/bin/env bash
# ROS 2 노드 완전 정리.
#
# ★ `pkill -f ros2` 로는 부족하다.
#   노드 실행파일 경로가 /opt/ros/humble/lib/... 라서 'ros2' 패턴에 안 걸린다.
#   그래서 Ctrl-C 나 `pkill -f ros2` 를 해도 **런처만 죽고 자식 노드가 고아로 남는다.**
#   그게 쌓이면:
#     · NTRIP 클라이언트가 여러 개 → NGII 계정당 1접속 제한 → 401 Unauthorized,
#       RTCM 0프레임 (RTK가 Float/None으로 떨어짐)
#     · ublox_dgnss 가 여러 개 → USB 를 서로 뺏어 LIBUSB_ERROR_ACCESS
#     · global_path_publisher 가 여러 개 → 같은 경로가 중복 발행
#   실제로 2026-08-15 현장에서 NTRIP 3개 / global_path_publisher 4개가 쌓여
#   RTK 가 계속 401 이었다.
#
# 사용:  bash tools/ros_cleanup.sh
set -u

pkill -f "[r]os2 launch"                     2>/dev/null
pkill -f "[/]opt/ros/humble/lib/"            2>/dev/null
pkill -f "[/]home/han/racing_ws/install/"    2>/dev/null
sleep 2
pkill -9 -f "[/]opt/ros/humble/lib/"         2>/dev/null
pkill -9 -f "[/]home/han/racing_ws/install/" 2>/dev/null
sleep 1

echo "정리 후 남은 개수 (전부 0이어야 정상):"
for p in rviz2 vrs_ntrip_client global_path_publisher ublox_dgnss_node \
         ublox_nav_sat_fix_hp direct_localization_node \
         local_sliding_window_node serial_bridge heading_init_node; do
  # -x (프로세스명 정확 매칭) 을 써야 이 스크립트 자신이 오탐되지 않는다
  printf '  %-28s %s\n' "$p" "$(pgrep -xc "$p" 2>/dev/null || echo 0)"
done
