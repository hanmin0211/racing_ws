#!/usr/bin/env bash
# drive_school.sh — 학교 트랙 주행을 한 번에. 전압을 재서 ff_min_pwm 을 스스로 정한다.
#
# ★ 왜 이게 필요한가 (2026-09-13)
#   그날 차가 안 움직였다. 명령도 경로도 라이다도 PWM 상수도 전부 맞았는데
#   **공급전압만 전날보다 1V 낮았다.** PWM 은 전압이 아니라 비율이라, 같은
#   PWM 이 전날엔 굴리고 그날엔 모터를 잠갔다(VMIN 2938mV).
#   ff_min_pwm 을 60 으로 올리니 굴렀다 — 출발 8.0초, 그 뒤 1.07 m/s.
#
#   매번 전압 재고 배수 계산하고 인자 붙이는 걸 사람이 하면 또 빠뜨린다.
#   여기서 자동으로 한다. 전원이 정상이면 기본값 50 그대로 간다.
#
# 사용:
#   bash tools/drive_school.sh              # 완주만 (어제 형태)
#   bash tools/drive_school.sh --no-avoid   # 회피 조향 끄고 **경로추종만** 본다
#   bash tools/drive_school.sh --missions   # 회피 + 돌발정지 시퀀서까지
#   bash tools/drive_school.sh --speed 0.5
#   bash tools/drive_school.sh --min-pwm 55   # 크리프 하한을 직접 지정
#
# ★ --min-pwm 을 쓰는 경우 (2026-09-13)
#   breakaway(출발용 60)는 정지마찰을 뚫는 값이고, ff_min_pwm(크리프)은
#   **구르는 중** 을 유지하는 값이다. 후자가 50 으로 충분한지는 아직 실측이
#   없다 — 출발은 뚫었는데 1.2초 뒤 다시 설 수 있다. PWM 55 는 9/12 캘리브
#   에서 1.00 m/s 로 구른 실측이 있으므로, 확실히 굴려야 할 때 지정한다.
#
# ★ --no-avoid 를 먼저 쓰는 이유 (2026-09-13)
#   그날 차가 가드레일을 박고 경로를 못 따라갔다. 로그를 보니 라이다가
#   `Width=7.89m` 짜리 물체(가드레일)를 잡아 AVOID 가 계속 떴고, 회피가
#   뜨면 **먹스가 경로조향을 통째로 버린다**(§2-5). best_angle 이 −18°
#   풀락까지 갔다. 차는 경로가 아니라 회피각을 따라간 것이다.
#   라이다를 범퍼 위 → 차량 정면으로 옮기면서 **스캔 평면이 낮아진** 것이
#   원인으로 보인다. 위에 있을 땐 가드레일 위를 지나갔다.
#   경로추종이 되는지부터 확인하려면 회피 조향을 꺼야 한다.
#   ⚠ 전방 감속·정지(/obstacle_distance)는 안전 기능이라 이 옵션과 무관하게
#     계속 동작한다. 앞에 뭐가 있으면 여전히 선다.
set -u
WS=/home/han/racing_ws
WP=$WS/config/chungju_school/wp_school_track_0.5.yaml
PLAN=$WS/config/chungju_school/mission_plan_school.yaml
MISSIONS=0; SPEED=0.7; YAW=""; NOAVOID=0; FORCE_PWM=""
while [ $# -gt 0 ]; do
  case "$1" in
    --missions) MISSIONS=1; shift;;
    --no-avoid) NOAVOID=1; shift;;
    --min-pwm) FORCE_PWM="$2"; shift 2;;
    --speed) SPEED="$2"; shift 2;;
    --yaw-offset) YAW="$2"; shift 2;;
    *) echo "모르는 인자: $1" >&2; exit 1;;
  esac
done

cd "$WS" || exit 1
# ⚠ set -u 를 켠 채로 ROS setup.bash 를 source 하면 **조용히 죽는다.**
#   setup.bash 가 AMENT_TRACE_SETUP_FILES 같은 미설정 변수를 참조하는데,
#   -u 면 그 순간 셸이 종료된다. 에러 메시지도 안 나와서 '스크립트가 아무것도
#   안 한다' 로 보인다(2026-09-13 에 실제로 이걸로 한참 헤맸다).
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash 2>/dev/null
# shellcheck disable=SC1091
source install/setup.bash 2>/dev/null
set -u

echo "──────────────────────────────────────────────────────────"
echo " 출발 전 점검"
echo "──────────────────────────────────────────────────────────"

fail=0
for n in arduino ldlidar_front imu; do
  if [ -e "/dev/$n" ]; then echo "  ✅ /dev/$n"
  else echo "  ❌ /dev/$n 없음"; [ "$n" = imu ] || fail=1; fi
done
if ! lsusb | grep -q 1546:; then echo "  ❌ GPS(u-blox) 안 보임"; fail=1
else echo "  ✅ GPS(u-blox)"; fi
if [ "$fail" = 1 ]; then
  echo; echo "  장치가 빠졌다. 라이다는 뺐다 꽂으면 살아나는 경우가 많다"
  echo "  (9/13: 같은 포트에 재삽입으로 복구). 꽂고 다시 실행할 것."
  exit 1
fi

# ── 전압을 재서 ff_min_pwm 을 정한다 ──────────────────────────
MV=$(timeout 8 python3 - <<'PY'
import re, sys
try:
    import serial
except ImportError:
    sys.exit(0)
vals = []
try:
    with serial.Serial('/dev/arduino', 57600, timeout=0.5) as sp:
        import time
        t0 = time.time(); buf = b''
        while time.time() - t0 < 4:
            buf += sp.read(256)
            while b'\n' in buf:
                line, buf = buf.split(b'\n', 1)
                m = re.search(rb'VCC=(\d+)', line)
                if m:
                    vals.append(int(m.group(1)))
except Exception:
    sys.exit(0)
if vals:
    print(min(vals))
PY
)

# ★ 전압이 정상이면 **아무것도 바꾸지 않는다.**
#   9/12 에 이 기본값(50 / 0.5)으로 완주했다. 검증된 설정을 전압이 멀쩡한데도
#   건드리면, 문제가 생겼을 때 원인이 하나 더 늘어난다.
PWM=50; CALIB=0.5
if [ -n "${MV:-}" ]; then
  echo
  if [ "$MV" -ge 4700 ]; then
    echo "  ✅ 공급전압 ${MV}mV — 정상. 검증된 기본값으로 간다 (ff_min_pwm=50)"
  else
    read -r PWM CALIB <<< "$(python3 -c "
import math
mv=$MV
pwm=max(50,min(90,int(math.ceil(50*5100/mv))))
# 캘리브 최소시간 게이트: need = 10m ÷ (auto_speed×2).
# 하한 PWM 의 예상속도 v=(pwm-17.2)/38.8 로 10m 가 걸리는 시간이
# need 보다 넉넉히 크도록 auto_speed 를 잡는다(여유 1.6배).
v=(pwm-17.2)/38.8
auto=round(min(1.2, max(0.5, 1.6*v/2)),1)
print(pwm, auto)")"
    echo "  ⚠ 공급전압 ${MV}mV — 낮다 (정상선 4700)"
    echo "     전압이 낮으면 같은 PWM 이 정지마찰을 못 넘어 모터가 잠긴다."
    echo "     잠긴 모터는 전류를 더 빨아 전압을 더 떨어뜨린다 — 그래서"
    echo "     역설적으로 **PWM 을 올리는 쪽이 안전하다.**"
    echo "  → ff_min_pwm=$PWM  auto_calib_speed=$CALIB  (전압 보정, 자동 계산)"
  fi
else
  echo
  echo "  ⚠ 전압을 못 읽었다 (포트 사용 중?). 기본값으로 간다."
fi

# --min-pwm 으로 직접 지정했으면 전압 보정을 덮어쓴다.
if [ -n "$FORCE_PWM" ]; then
  PWM="$FORCE_PWM"
  echo "  → ff_min_pwm=$PWM  (--min-pwm 으로 직접 지정, 전압 보정 무시)"
fi

# ★ 라이다 드라이버를 여기서 띄운다 (2026-09-13)
#   bringup 의 lidar:=true 는 **판단 노드(cluster_plot_node)만** 띄운다.
#   드라이버(sllidar)는 lidar_dual.launch.py 가 따로 띄워야 한다. 이걸
#   빼먹으면 "스캔이 한 번도 안 왔다" 로 끝난다 — 9/13 밤에 실제로 밟았다.
#   (9/12 에는 아예 lidar:=false 로 띄워 의자를 그대로 박았다. 같은 종류의
#    실수가 두 번 났으니 사람이 기억하게 두지 않는다)
if ros2 topic list 2>/dev/null | grep -q '^/scan_front$'; then
  echo "  ✅ /scan_front 이미 발행 중 — 드라이버를 새로 띄우지 않는다"
else
  echo "  · 라이다 드라이버를 띄운다 (lidar_dual.launch.py, 전방만)"
  ros2 launch lidar_clustering lidar_dual.launch.py rear:=false \
      > /tmp/lidar_$(date +%H%M).log 2>&1 &
  LIDAR_PID=$!
  trap 'kill $LIDAR_PID 2>/dev/null' EXIT
  for _ in $(seq 1 20); do
    sleep 0.5
    ros2 topic list 2>/dev/null | grep -q '^/scan_front$' && break
  done
  if ros2 topic list 2>/dev/null | grep -q '^/scan_front$'; then
    echo "  ✅ /scan_front 올라왔다"
  else
    echo "  ❌ /scan_front 이 안 올라온다. 라이다 USB 를 뺐다 꽂을 것."
    echo "     (9/13: 같은 포트에 재삽입만으로 살아난 적이 있다)"
    exit 1
  fi
fi

LOG=/tmp/run_$(date +%H%M).log
echo
echo "  로그: $LOG"
echo "  별 터미널에서 전압 감시:  python3 -u $WS/tools/vcc_watch.py"
echo "──────────────────────────────────────────────────────────"
echo

ARGS=(control:=true lidar:=true auto_calib:=true
      "waypoints:=$WP" "max_speed:=$SPEED"
      "ff_min_pwm:=$PWM" "auto_calib_speed:=$CALIB")
[ -n "$YAW" ] && ARGS+=("fg_yaw_offset_deg:=$YAW")
[ "$NOAVOID" = 1 ] && ARGS+=(no_avoid_steer:=true)
[ "$MISSIONS" = 1 ] && ARGS+=(sequencer:=true sudden_stop:=true "mission_plan:=$PLAN")
if [ "$NOAVOID" = 1 ]; then
  echo "  ⚠ 회피 조향 꺼짐 — 경로추종만 본다."
  echo "    전방 감속·정지는 그대로 동작한다(안전 기능)."
  echo
fi

echo "ros2 launch gps_localization bringup.launch.py ${ARGS[*]}"
echo
exec ros2 launch gps_localization bringup.launch.py "${ARGS[@]}" 2>&1 | tee "$LOG"
