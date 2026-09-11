#!/usr/bin/env bash
# ROS 2 노드 완전 정리 + 잔존 확인.
#
# ★ `pkill -f ros2` 로는 부족하다.
#   노드 실행파일 경로가 /opt/ros/humble/lib/... 라서 'ros2' 패턴에 안 걸린다.
#   그래서 Ctrl-C 나 `pkill -f ros2` 를 해도 **런처만 죽고 자식 노드가 고아로 남는다.**
#   그게 쌓이면:
#     · NTRIP 클라이언트가 여러 개 → NGII 계정당 1접속 제한 → 401 Unauthorized,
#       RTCM 0프레임 (RTK 가 Float/None 으로 떨어짐)
#     · ublox_dgnss 가 여러 개 → USB 를 서로 뺏어 LIBUSB_ERROR_ACCESS
#     · heading_init 이 여러 개 → 서로 다른 yaw_offset 을 덮어씀 (위험)
#   2026-08-15 현장에서 NTRIP 3개 / global_path_publisher 4개 / heading_init 3개가
#   쌓여 RTK 가 계속 401 이었다.
#
# ★ 확인은 `ps -eo comm` 으로 한다.
#   `pgrep -f <패턴>` 은 **이 스크립트 자신의 명령줄이 패턴에 걸려 오탐**하고,
#   `pgrep -x <이름>` 은 리눅스가 프로세스명을 15자로 자르기 때문에 매칭에 실패한다
#   (ublox_dgnss_node → ublox_dgnss_nod). 실제로 둘 다 틀린 값을 줬다.
#
# 사용:  bash tools/ros_cleanup.sh
set -u

pkill -f "[r]os2 launch"                     2>/dev/null
pkill -f "[/]opt/ros/humble/lib/"            2>/dev/null
pkill -f "[/]home/han/racing_ws/install/"    2>/dev/null
# ★ 2026-09-11 추가 — tools/ 에서 직접 돌리는 보조 노드들.
#   위 두 패턴은 install/ · /opt/ros/ 아래만 잡는다. `python3 tools/hil_vehicle.py`
#   는 어느 쪽에도 안 걸려 **살아남았다**(인계문서에는 같이 죽는다고 적혀 있었으나
#   실제로는 반대였다, 2026-09-11 확인).
#   남은 hil_vehicle 이 있는 채로 새로 띄우면 두 대가 같은 /odometry/filtered 에
#   발행해 결과가 통째로 쓰레기가 된다 — 그리고 조용히 그렇게 된다.
pkill -f "[t]ools/hil_vehicle.py"            2>/dev/null
pkill -f "[t]ools/hil_probe.py"              2>/dev/null
sleep 2
pkill -9 -f "[/]opt/ros/humble/lib/"         2>/dev/null
pkill -9 -f "[/]home/han/racing_ws/install/" 2>/dev/null
pkill -9 -f "[t]ools/hil_vehicle.py"         2>/dev/null
sleep 1

echo "정리 후 남은 개수 (전부 0이어야 정상):"
for name in rviz2 ublox_dgnss_node ublox_nav_sat_fix_hp vrs_ntrip_client \
            global_path_publisher direct_localization_node \
            local_sliding_window_node heading_init_node serial_bridge; do
  short="${name:0:15}"                       # 리눅스 comm 은 15자로 잘린다
  n=$(ps -eo comm --no-headers 2>/dev/null | grep -c "^${short}$")
  printf '  %-28s %s\n' "$name" "$n"
done

# 파이썬 노드는 comm 이 'python3' 라 위 방식으로 안 잡히는 경우가 있다 → 경로로 재확인
py=$(ps -eo args --no-headers 2>/dev/null \
     | grep -c "racing_ws/install/.*/lib/")
echo "  (경로 기준 racing_ws 노드 잔존: $py)"
