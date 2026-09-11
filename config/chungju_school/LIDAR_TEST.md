# 라이다 회피 실측 절차 (충주 캠퍼스)

> 2026-09-11 작성. 경로: `config/chungju_school/wp_school_track_0.5.yaml` (89m 루프)

## ★ 시작 전에 알아둘 것

**회피는 s 구간 등록이 필요 없다.**
- 전방 감속·정지(`/obstacle_distance`)는 **코스 전체에서 항상** 동작한다(안전 기능).
- 조향 override 만 구간 제한을 받는데, 그 제한은 `require_arm_for_steer` 이고
  이 값은 런치의 `sequencer` 인자에 묶여 있다. **`sequencer:=false` 면 제한 없이 켜진다.**

**오프라인 검증은 이미 실패했다** (`tools/lidar_slalom_test.py`, 2026-09-11):
- 규정 배치(T870 2대, 좌우 랜덤) 잠정 치수에서 **두 배치 모두 실패**
  - 좌→우: 통과하지만 |y| 2.69m 로 **트랙(±1.35m)을 벗어나서** 통과
  - 우→좌: 두 번째 장애물에 갇혀 **BLOCKED → 정지**
- 원인: 회피 중에는 `avoid_steer` 가 경로추종을 **완전히 대체**한다. 차선 개념이
  없어서 첫 장애물만 보고 한쪽으로 계속 호를 그린다.
- 간격 6m 이상 + 오프셋 0.9m 면 통과한다 → **실제 배치 치수가 결정적이다.**

→ 그래서 오늘 실측에서 봐야 할 것은 "피하는가"보다 **"얼마나 과하게 꺾는가"** 와
  **"BLOCKED 로 서는가"** 다. 그리고 장애물 실측 치수를 확보하는 것.

---

## 0. 준비

```bash
ls -l /dev/ldlidar          # 없으면 라이다 USB 미연결 (udev 규칙은 이미 있다)
ls -l /dev/arduino /dev/imu
```
- **차량 배터리 연결** (2026-09-11 에 이게 빠져 하드웨어 검증을 통째로 날렸다)
- E-stop 을 손에 들고 시작할 것

## 1. 원점을 충주로 (지금 용인이다)

```bash
python3 tools/set_origin_from_fix.py            # 미리보기
python3 tools/set_origin_from_fix.py --write    # .bak 백업 후 갱신
```
> ⚠ **대회 전에 반드시 용인으로 되돌릴 것.** 안 되돌리면 용인 경로가 219km 밖으로
> 판정돼 `global_path_publisher` 가 발행을 거부한다(= 차가 안 움직인다).

## 2. 라이다 마운트 방향 확인 ← **가장 중요. 차는 안 움직인다**

```bash
ros2 launch sllidar_ros2 sllidar_a1_launch.py \
    serial_port:=/dev/ldlidar serial_baudrate:=256000
ros2 run lidar_clustering cluster_plot_node          # 다른 터미널
python3 tools/lidar_monitor.py                       # 또 다른 터미널
```

차 **정면 2m** 에 물체를 놓고 `corrected` 각을 본다.
- `~0°` → 정상 (`fg_yaw_offset_deg` 기본 180 이 맞다)
- `~180°` → 라이다가 뒤집혀 있다 → `fg_yaw_offset_deg:=0` 으로 띄울 것

**이게 틀리면 차가 장애물 쪽으로 꺾는다.** 여기서 반드시 확정하고 넘어갈 것.

## 3. HIL 로 회피 체인 확인 ← 차는 안 움직인다, GPS 불필요

실제 라이다 스캔 → `/lidar/avoid_steer` → 먹스 → `/cmd_vel` → **가상 차량이
RViz 에서 피해 간다.** 체인 전체를 안전하게 본다.

```bash
ros2 launch gps_localization hil.launch.py \
    waypoints:=/home/han/racing_ws/config/chungju_school/wp_school_track_0.5.yaml \
    lidar:=true max_speed:=0.8 max_steer_deg:=15 curvature_gain:=3
# 다른 터미널
python3 tools/hil_vehicle.py \
    --wp /home/han/racing_ws/config/chungju_school/wp_school_track_0.5.yaml --start-idx 0
python3 tools/hil_probe.py \
    --wp /home/han/racing_ws/config/chungju_school/wp_school_track_0.5.yaml \
    --max-steer-deg 15 --tag lidar_hil
```

라이다 앞 **1.5~3m, 중심에서 좌/우로 비껴** 물체를 놓는다
(정중앙 0.8m 이내면 `/obstacle_distance` 로 그냥 선다 — 회피가 안 보인다).
RViz 에서 가상 차량이 피해 가면 체인이 살아 있는 것이다.

## 4. 실주행

```bash
python3 tools/preflight.py \
    --waypoints config/chungju_school/wp_school_track_0.5.yaml   # 원점/경로 확인

ros2 launch gps_localization bringup.launch.py \
    waypoints:=/home/han/racing_ws/config/chungju_school/wp_school_track_0.5.yaml \
    control:=true lidar:=true sequencer:=false \
    max_speed:=0.8 max_steer_deg:=15 curvature_gain:=3
# 다른 터미널
python3 tools/hil_probe.py \
    --wp /home/han/racing_ws/config/chungju_school/wp_school_track_0.5.yaml \
    --max-steer-deg 15 --tag lidar_real
```

- `max_speed` 는 **0.8 부터**. 회피가 과하게 꺾이는 걸 보고 나서 올릴 것.
- `sequencer:=false` 여야 회피가 구간 제한 없이 켜진다(§시작 전에 알아둘 것).

## 5. 무엇을 기록할 것인가

`hil_probe` 가 자동으로 낸다:
- **회피 발동 횟수 / 발동 지점 s**
- **회피 중 이탈량** ← 차선을 얼마나 벗어나는지. 오프라인에선 2.69m 였다
- **회피 조향각 최대** ← 과조향 여부
- **BLOCKED 샘플 수** ← 서 버리면 대회에서 탈락 경로
- 전방 최근접 거리

손으로 잴 것 (이게 없으면 시뮬이 계속 추측이다):
- **장애물 실측**: 길이 / 폭 / 두 개의 종방향 간격 / 중심선 오프셋
- 그 값으로 다시:
  ```bash
  python3 tools/lidar_slalom_test.py --length <L> --width <W> \
      --spacing <S> --offset <O>
  ```

## 6. 안전

- 첫 시도는 **HIL(3번)** 로. 차를 움직이는 건 그 다음.
- E-stop 상시 대기. 먹스에서 E-stop > teleop > 자율 순으로 우선한다.
- 라이다 조향 override 가 이상하게 나오면 **`lidar:=false` 로 내리고** 주행만
  확인할 것. 회피는 감점-only(10점)지만 **이탈은 탈락**이다.
