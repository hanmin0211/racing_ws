#!/usr/bin/env bash
# lidar_up.sh — 라이다가 붙을 때까지 순차 재시도한다.
#
# ★ 왜 (2026-09-19 용인)
#   허브 3단에 물린 라이다가 시도마다 다른 데서 죽었다:
#     80008002 (info 타임아웃) / health OK 후 scan 실패 / 80008004 (즉시)
#   에러가 매번 다르면 설정이 아니라 **물리적으로 불안정**한 것이다.
#   그런데 한 번은 health 까지 통과했다 — 될 때가 있다는 뜻이다.
#
#   그래서 사람이 계속 올려치는 대신 여기서 자동으로 반복한다.
#   매 시도 사이에 USB 전원을 잠깐 쉬게 두는 것이 핵심이다(칩 리셋).
#
# 사용
#   bash tools/lidar_up.sh           # 최대 8회
#   bash tools/lidar_up.sh 20        # 최대 20회
#
#   성공하면 드라이버를 **띄워 둔 채** 끝난다 (백그라운드).
#   실패하면 종료코드 1 — 그때는 라이다 없이 가라(예선은 장애물이 없다).

set -u
WS=/home/han/racing_ws
TRIES=${1:-8}
LOG=/tmp/lidar_up_$(date +%H%M).log

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash 2>/dev/null
# shellcheck disable=SC1091
source "$WS/install/setup.bash" 2>/dev/null
set -u

pubs() {
  ros2 topic info /scan_front 2>/dev/null \
    | sed -n 's/.*Publisher count: *//p' | head -1
}

if [ "$(pubs || echo 0)" -gt 0 ] 2>/dev/null; then
  echo "✅ /scan_front 이 이미 발행 중이다 (발행자 $(pubs)개)"
  exit 0
fi

echo "라이다 순차 재시도 — 최대 ${TRIES}회"
echo "  포트: $(readlink -f /sys/class/tty/"$(basename "$(readlink /dev/ldlidar_front 2>/dev/null)")"/device 2>/dev/null | grep -oE 'usb[0-9]/[0-9.-]+' | tail -1)"
echo "  로그: $LOG"
echo

for i in $(seq 1 "$TRIES"); do
  printf '  [%d/%d] ' "$i" "$TRIES"

  # 남은 드라이버를 먼저 정리한다 — 포트를 쥔 채면 즉시 실패(80008004)한다.
  pkill -f sllidar_node 2>/dev/null
  sleep 1.5          # ⚠ 칩이 리셋될 시간. 바로 재시도하면 같은 상태로 실패한다.

  ros2 launch lidar_clustering lidar_dual.launch.py rear:=false \
       >> "$LOG" 2>&1 &
  LP=$!

  # 최대 8초 기다린다. 스캔이 나오면 성공.
  ok=0
  for _ in $(seq 1 16); do
    sleep 0.5
    if [ "$(pubs || echo 0)" -gt 0 ] 2>/dev/null; then ok=1; break; fi
    kill -0 $LP 2>/dev/null || break      # 드라이버가 죽었다 — 기다릴 필요 없다
  done

  if [ "$ok" = 1 ]; then
    echo "✅ /scan_front 발행 시작 (발행자 $(pubs)개)"
    echo
    echo "  드라이버를 띄워 둔 채 끝낸다. 이 터미널을 닫지 마라."
    echo "  확인:  ros2 topic hz /scan_front"
    wait $LP
    exit 0
  fi

  kill $LP 2>/dev/null
  pkill -f sllidar_node 2>/dev/null
  echo "실패 — $(tail -3 "$LOG" | grep -oE 'code: [0-9a-fA-F]+|SL_RESULT_[A-Z_]+' | tail -1)"
done

echo
echo "❌ ${TRIES}회 다 실패했다."
echo "   전압을 봐라:  python3 tools/preflight.py --waypoints <경로> --skip-parking | grep 무부하"
echo "   2848mV 같은 값이면 라이다 모터를 못 돌린다 — 케이블을 직결로 옮겨야 한다."
echo
echo "   ⚠ 예선(굴절)은 장애물이 없다. **라이다 없이 나가면 된다.**"
exit 1
