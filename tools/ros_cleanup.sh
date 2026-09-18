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
# ★ 확인은 `ps -eo args` 의 **실행파일 경로**로 한다 (2026-09-18 개선).
#   전에 쓰던 방법들이 전부 틀렸다:
#     · `pgrep -f <패턴>` — 이 스크립트 자신의 명령줄이 걸려 오탐한다
#     · `pgrep -x <이름>` — 리눅스가 프로세스명을 15자로 자른다
#       (ublox_dgnss_node → ublox_dgnss_nod)
#     · `ps -eo comm` + 고정목록 — 파이썬 노드는 comm 이 전부 'python3' 라
#       **이름별 검사가 아예 작동하지 않았다.** 게다가 목록에 IMU 가 없었다.
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

# ★ 2026-09-18 — 확인 방식을 바꿨다. 예전에는 고정 목록을 `ps -eo comm` 으로
#   셌는데, 두 가지가 틀렸다:
#     ① 파이썬 노드는 comm 이 전부 'python3' 라 **이름별 검사가 아예 안 됐다**
#        (아래 주석이 그걸 인정하면서도 고치지는 않았다).
#     ② 목록에 hfi_a9_ros2(IMU) 가 없었다. 그래서 IMU 좀비가 남아도 보이지
#        않았고, 다음 런치의 IMU 와 /dev/imu 를 나눠 읽다 2분 뒤 exit 1 로
#        죽었다 — 그날 세 세션이 그렇게 죽었다.
#   이제는 **살아남은 프로세스에서 이름을 직접 뽑아 보여준다.** 목록을
#   관리할 필요가 없고, 새 노드가 추가돼도 저절로 잡힌다.
echo "정리 후 잔존 노드 (아무것도 없어야 정상):"
# ⚠ 패키지 이름에 **숫자가 들어간다**(handsfree_ros2_imu, ublox_dgnss).
#   처음에 [a-z_]+ 로 썼다가 하필 IMU 만 못 잡았다 — 고치려던 바로 그 노드를.
left=$(ps -eo pid,etimes,args --no-headers 2>/dev/null \
       | grep -E "racing_ws/install/[a-z_0-9]+/lib/|/opt/ros/humble/lib/[a-z_0-9]+/" \
       | grep -v "[g]rep")
if [ -z "$left" ]; then
  echo "  ✅ 없음"
else
  echo "$left" | while read -r pid secs rest; do
    # 실행파일 경로의 마지막 조각이 노드 이름이다
    node=$(echo "$rest" | grep -oE "/lib/[a-z_0-9]+/[a-z_0-9]+" | head -1 \
           | awk -F/ '{print $NF}')
    printf '  ❌ %-28s pid=%-7s %ss 경과\n' "${node:-?}" "$pid" "$secs"
  done
  echo
  echo "  ⚠ 남아 있으면 다음 런치와 **장치를 두고 싸운다.**"
  echo "    IMU 가 특히 위험하다 — 중복되면 프레임이 깨져 2분쯤 뒤 죽고,"
  echo "    죽으면 경사 보상이 조용히 0 이 된다(GRADE_FRESH_S 0.5s)."
  echo "    한 번 더 돌리거나, 안 죽으면 pid 를 직접 kill -9 할 것."
fi
