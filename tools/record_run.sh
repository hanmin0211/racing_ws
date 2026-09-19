#!/usr/bin/env bash
# record_run.sh — 주행 하나를 **통째로** 남긴다. 나중에 분석할 수 있게.
#
# ★ 왜 (2026-09-19)
#   drive_record.py 만 돌리면 궤적은 남는데 **어떤 설정으로 돌았는지가 없다.**
#   설정은 /tmp/launch_params_* 에 있고 그건 재부팅하면 사라진다 — 어젯밤
#   주행의 덤프가 실제로 전부 날아갔다. 노드 stdout 은 ~/.ros/log 에 9600개가
#   쌓여 있어 어느 세션이 그 주행인지 찾는 것부터 일이다.
#
#   그래서 한 폴더에 다 모은다. 폴더 하나만 보면 그 주행을 재현할 수 있다.
#
# 남기는 것
#   drive_*.csv        궤적·모드·조향·전압 (20Hz)
#   drive_*_scan.csv   라이다 점을 지도좌표로
#   params/            그 주행의 런치 파라미터 덤프 전부
#   roslog/            그 세션의 노드 stdout
#   run.txt            런치 명령줄 · git 커밋 · 원점 · IMU 영점 · 경로 통계
#
# 사용 (런치가 떠 있는 상태에서)
#   bash tools/record_run.sh <이름>
#   bash tools/record_run.sh yongin_1st
#
#   Ctrl-C 로 끝낸다. 그때 설정 스냅샷을 뜬다.

set -u
WS=/home/han/racing_ws
NAME=${1:-run}
DAY=$(date +%F)
OUT=$WS/data/$DAY/${NAME}_$(date +%H%M)
# 이 주행의 시작 시각 — 어느 ROS 세션이 '내 것' 인지 가르는 기준이다.
# 런치를 먼저 띄우므로 60초 여유를 둔다.
T0=$(( $(date +%s) - 60 ))
mkdir -p "$OUT/params" "$OUT/roslog"

# ⚠ set -u 를 켠 채 ROS setup.bash 를 source 하면 조용히 죽는다 (2026-09-13).
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash 2>/dev/null
# shellcheck disable=SC1091
source "$WS/install/setup.bash" 2>/dev/null
set -u

# ── 주행 전 스냅샷: 지금 돌고 있는 런치가 무엇인가 ────────────────
snapshot() {
  {
    echo "# 주행 기록 — $NAME"
    echo "일시   : $(date '+%F %T %Z')"
    echo "브랜치 : $(cd "$WS" && git rev-parse --abbrev-ref HEAD 2>/dev/null)"
    echo "커밋   : $(cd "$WS" && git rev-parse --short HEAD 2>/dev/null)"
    d=$(cd "$WS" && git status --porcelain 2>/dev/null | wc -l)
    [ "$d" -gt 0 ] && echo "⚠ 커밋 안 된 변경 $d 건 — 이 주행은 커밋 상태와 다르다"
    echo
    echo "## 런치 명령줄"
    for p in $(pgrep -f 'bringup.launch.py' 2>/dev/null); do
      tr '\0' ' ' < "/proc/$p/cmdline" 2>/dev/null; echo
    done
    echo
    echo "## 원점 (config/site_origin.yaml)"
    grep -E '^(site|utm_epsg|origin_x|origin_y):' "$WS/config/site_origin.yaml" 2>/dev/null | sed 's/^/  /'
    echo
    echo "## IMU 영점 (config/imu_pitch_offset.yaml)"
    grep -E '^(pitch_offset_rad|sign):' "$WS/config/imu_pitch_offset.yaml" 2>/dev/null | sed 's/^/  /'
    echo
    echo "## 노드"
    ros2 node list 2>/dev/null | sed 's/^/  /'
  } > "$OUT/run.txt" 2>&1

  # 런치 파라미터 덤프 — /tmp 에 있고 재부팅하면 사라진다. 지금 복사한다.
  #   (돌고 있는 노드의 --params-file 을 /proc 에서 읽으므로 런치가 없으면 0개다)
  n=0
  for p in $(pgrep -f 'install/.*/lib/' 2>/dev/null); do
    f=$(tr '\0' '\n' < "/proc/$p/cmdline" 2>/dev/null \
        | grep -A1 -- '--params-file' | tail -1)
    if [ -n "${f:-}" ] && [ -e "$f" ]; then
      cp "$f" "$OUT/params/$(basename "$f").yaml" 2>/dev/null && n=$((n+1))
    fi
  done
  echo "  파라미터 덤프 ${n}개 복사"

  # 그 세션의 노드 stdout
  # ⚠ **옛 세션을 복사하면 안 된다.** 나중에 엉뚱한 로그를 근거로 삼게 된다.
  #   pgrep -f 로 런치를 찾으면 안 된다 — 검사 문자열이 자기 명령줄에 들어가
  #   **늘 성공**한다(2026-09-19 에 실제로 밟았다). 시각으로 판정한다:
  #   이 주행이 시작되기 전에 만들어진 세션이면 내 것이 아니다.
  S=''
  C=$(ls -dt "$HOME"/.ros/log/2*/ 2>/dev/null | head -1)
  if [ -n "${C:-}" ] && [ "$(stat -c %Y "$C" 2>/dev/null || echo 0)" -ge "$T0" ]; then
    S=$C
  else
    echo "  ⚠ 이 주행 중에 만들어진 ROS 세션이 없다 — 노드 로그를 안 남긴다"
    echo "     (런치를 먼저 띄우고 이 스크립트를 실행할 것)"
  fi
  if [ -n "${S:-}" ]; then
    cp "$S"/launch.log "$OUT/roslog/" 2>/dev/null
    for p in $(pgrep -f 'install/.*/lib/' 2>/dev/null); do
      for g in "$HOME"/.ros/log/*_"$p"_*.log; do
        [ -e "$g" ] && cp "$g" "$OUT/roslog/" 2>/dev/null
      done
    done
    echo "  노드 로그 $(ls "$OUT/roslog" 2>/dev/null | wc -l)개 복사"
  fi
}

trap 'echo; echo "── 설정 스냅샷 ──"; snapshot; echo "✅ $OUT"; ls "$OUT"' EXIT

echo "기록 → $OUT"
echo "  Ctrl-C 로 끝낸다. 그때 설정을 같이 뜬다."
echo

# ⚠ --yaw-offset 은 cluster_plot_node 의 fg_yaw_offset_deg 와 맞춰야 한다.
#   기본 173 으로 두면 라이다 점이 7° 돌아가 찍힌다.
YAW=$(ros2 param get /lidar_clustering fg_yaw_offset_deg 2>/dev/null \
      | grep -oE '[0-9.]+' | tail -1)
YAW=${YAW:-180.0}
echo "  라이다 마운트 보정 ${YAW}° (cluster_plot_node 와 맞춤)"
echo

# ⚠ exec 를 쓰면 안 된다 — 셸 프로세스가 교체돼 위 trap 이 **안 돈다**.
#   그러면 설정 스냅샷이 통째로 안 남는다(2026-09-19 에 실제로 밟았다).
python3 "$WS/tools/drive_record.py" --yaw-offset "$YAW" --out "$OUT/drive.csv"
