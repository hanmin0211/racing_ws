# 도착하자마자 — 순서대로 (용인 2026-09-19)

> 위에서부터 그대로. 각 단계의 **기대 출력**이 안 나오면 멈추고 다음으로 가지 마라.
> 근거는 `HANDOFF_2026-09-19-night.md`, 증상별 처방은 `START.md` 아래쪽.

---

## ① 원점 — 틀리면 차가 한 발짝도 안 간다

```bash
cd /home/han/racing_ws && bash tools/use_site.sh
```

기대:
```
site: "용인 (2026-09-05)"
origin_x: 332200.0
```

아니면:
```bash
cd /home/han/racing_ws && bash tools/use_site.sh yongin
```

**충주로 남아 있으면 76.8km 차이로 전역 경로 발행이 거부된다.** 캘리브까지
`✅ 완료` 로 끝나고 먹스가 `자율 허용` 까지 가는데 차가 안 간다.

---

## ② 차 연결 → 장치 확인

```bash
ls -l /dev/imu /dev/arduino /dev/ldlidar_front && lsusb | grep -icE "10c4:ea60" && stty -F /dev/imu 921600 raw -echo && timeout 2 head -c 64 /dev/imu | wc -c
```

기대: 심링크 3개 · `2` · `64`

- `/dev/imu` 없음 → 꽂아라 (`ttyUSB` 번호는 무시 — udev 가 시리얼로 잡는다)
- 마지막이 `0` → 뽑고 **10초 뒤** 재삽입 (바로 꽂으면 칩이 리셋 안 된다)
- ⚠ **USB 허브 깊게 쓰지 마라** — 시리얼이 끊긴다

---

## ③ 정리

```bash
cd /home/han/racing_ws && bash tools/ros_cleanup.sh
```

기대: `✅ 없음`

---

## ④ 사전점검

```bash
cd /home/han/racing_ws && source install/setup.bash && python3 tools/preflight.py --waypoints config/yongin_2026-09-05/wp_yongin_drive_0.5.yaml --calib-distance 10.0 --skip-parking
```

기대: **`치명적 문제 없음`**

정상적으로 뜨는 주의 3건 (무시해도 된다):
```
⚠ 빠듯한 커브 19개    → max_speed 1.6 이면 횡가속 0.60 m/s² 로 안전
⚠ 정지점이 0개        → 용인 정지선은 아직 미측정. 의도한 것
⚠ 활성 미션이 0개     → 트랙 주행은 이게 맞다
```

이 단계에서 **공급전압**도 같이 잰다. 낮으면 화면에 뜬다.

---

## ⑤ 라이다 — 드라이버 + 마운트 (건너뛰면 출발하자마자 선다)

**터미널 A** (켜 두고 놔둔다)
```bash
cd /home/han/racing_ws && source install/setup.bash && ros2 launch lidar_clustering lidar_dual.launch.py rear:=false
```

**터미널 B** (30초)
```bash
cd /home/han/racing_ws && source install/setup.bash && python3 tools/lidar_mount_check.py
```

지면을 때리면 전방이 통째로 장애물이 되어 **차가 영원히 선다**(2회 재발).

---

## ⑥ IMU 영점 확인 — 차를 평지에 세우고

```bash
cd /home/han/racing_ws && source install/setup.bash && python3 tools/imu_grade_live.py
```

기대: **`경사 0 ± 1%`**

- ±3% 안이면 그대로 간다 (grade_pwm 오차 15 이내, 여유 28 안)
- ±3% 넘으면 그때만 재영점. ⚠ `imu_grade.py --level` 은 파일을 덮어쓴다 —
  그 자리가 진짜 평지가 아니면 지금보다 나빠진다
- ⚠ **부호(−1.0)는 절대 건드리지 마라.** 뒤집히면 내리막에서 가속한다

`Ctrl-C` 로 끝낸다.

---

## ⑦ ★ 본선 트랙 주행

**터미널 C**
```bash
cd /home/han/racing_ws && bash tools/drive_school.sh --speed 1.6 --min-pwm 0 --waypoints /home/han/racing_ws/config/yongin_2026-09-05/wp_yongin_drive_0.5.yaml --extra ff_mode:=ros --extra ff_static:=46.0 --extra ff_gain:=42.6 --extra ff_breakaway_pwm:=90.0 --extra ff_breakaway_ms:=1200.0 --extra auto_calib_speed:=0.5 --extra calib_distance:=10.0 --extra curvature_gain:=6.0 --extra avoid_steer_rate_deg:=90.0 --extra grade_ff_gain:=1.0 --extra grade_ff_max:=95.0 --extra gov_pwm:=60.0 --extra gov_min_grade:=0.06 --extra gov_gain:=300.0 --extra gov_lead_s:=0.3 --extra gov_deadband:=0.20
```

**터미널 D** (런치가 뜬 뒤)
```bash
cd /home/han/racing_ws && bash tools/record_run.sh yongin_1st
```

### `exec` 직전 출력에서 눈으로 볼 것 다섯

```
waypoints:=...wp_yongin_drive_0.5.yaml
grade_ff_gain:=1.0         ← 없으면 경사로에서 메시지 없이 죽는다
auto_calib_speed:=0.5      ← 없으면 캘리브 무효가 날 수 있다
ff_static:=46.0            ← 17.2 면 다른 차다
max_speed:=1.6
```

### 런치 로그에서 볼 것

```
✅ /scan_front 발행 시작                          ← 라이다가 진짜 돈다
★ 경사 보상 켜짐 — 영점 +3.81° · 부호 -1.         ← grade_ff 살아 있다
전역 경로 로드 완료: (총 1298개 점, 648.3m)       ← 안 뜨면 ①번(원점)이다
✅ 헤딩 초기화 완료: 10.Xm 직진 (최대 편차 N°)
헤딩 캘리브 완료 — 자율 허용
모드: AUTO
```

---

## ⑧ 주행 중 볼 것 — 이 숫자에서 벗어나면 멈춰라

```
완주 시간     약 5.1분 (648.3m, 평균 2.13 m/s)
평지 속도     약 2.4 m/s   (8.9 km/h)
빠듯한 커브   약 1.42 m/s  (5.1 km/h) — 바닥에 걸린다
경사로 s11~54 약 1.8 m/s
종점          1~2m 지나서 정지 (역토크 없음. 정상이다)
```

**⚠ 용인은 AUTO 시작 1m 뒤가 경사로다.** 학교는 44m 조주거리가 있었다.
계산상 PWM 여유 28 로 오르지만 **이 형태는 실차 미검증이다 — 여기를 제일
먼저 봐라.** 속도가 단조 감소하면 `grade_ff_gain` 을 의심해라(메시지가 안 뜬다).

---

## ⑨ 주행 끝나고

```bash
cd /home/han/racing_ws && source install/setup.bash && python3 tools/drive_plot.py data/$(date +%F)/yongin_1st_*/drive.csv --wp config/yongin_2026-09-05/wp_yongin_drive_0.5.yaml
```

기록 폴더에 `run.txt`(설정)·`params/`·`roslog/` 가 같이 있다.

---

# 그다음 — 예선 작업

## ⑩ 구간 s 확정

```bash
cd /home/han/racing_ws && source install/setup.bash && python3 tools/mission_s.py --waypoints config/yongin_2026-09-05/wp_yongin_drive_0.5.yaml --live
```

굴절·S자의 실제 s 를 읽는다. **지금 계획에 든 값(58~140, 176~250)은 추정이다.**

## ⑪ 구간 웨이포인트를 새로 찍는다면

```bash
cd /home/han/racing_ws && source install/setup.bash && ros2 run waypoint_follower waypoint_recorder
```

⚠ **구간 시작보다 10m 앞에서부터 찍어라.** 그 앞 10m 가 캘리브 조주구간이 된다.
구간 경계에서 시작하면 첫 10m 가 커브라 **캘리브가 거부된다**(실측 편차 31.9°,
거부선 20°).

찍은 뒤 반드시:
```bash
cd /home/han/racing_ws && source install/setup.bash && python3 tools/preflight.py --waypoints <찍은파일>.yaml --calib-distance 10.0 --skip-parking
```
기대: **`✅ run-up 직선 — 여기서 캘리브하면 안전`**

## ⑫ 또는 본 코스에서 잘라 쓴다 (이미 만들어 둠)

```
config/yongin_2026-09-05/wp_yongin_prelim_gujeol.yaml    s44~140    95.5m   48초
config/yongin_2026-09-05/wp_yongin_prelim_scurve.yaml    s166~250   83.5m   44초
config/yongin_2026-09-05/wp_yongin_prelim_both.yaml      s44~250   205.4m  104초
```

다른 구간을 자르려면:
```bash
cd /home/han/racing_ws && python3 tools/slice_waypoints.py config/yongin_2026-09-05/wp_yongin_drive_0.5.yaml --from 44 --to 250
```
미리보기만 한다. `--write` 를 줘야 쓴다.

## ⑬ 예선 주행

절차는 `PRELIM.md`. 핵심만:
- **`--extra auto_calib:=false`** — 차가 스스로 10m 가면 출발선을 지나친다
- 손으로 10m 밀어 캘리브
- 출발 신호를 기다려야 하면 `teleop_keyboard` 의 **`E`** 로 E-stop
- ⚠ **teleop 을 Ctrl+C 로 끄면 E-stop 이 풀려 차가 출발한다.** 해제는 `E` 키로

---

# 절대 하지 말 것

- **런치를 Ctrl+C** — 캘리브(`yaw_offset`)가 날아간다
- **`gov_min_grade` 올리기** — 게이트가 안 열리면 내리막에서 못 잡아 탈락
- **`obstacle_stop_dist` 올리기** — 장애물마다 멀리서 서고 8초 뒤 회피를 포기
- **`ff_static`/`ff_gain` 바꾸기** — 양방향 완주한 값이다
- **`--extra waypoints:=`** — 인자가 중복된다. `--waypoints` 를 써라
- **`wasd_teleop`** — 먹스를 우회해 `/cmd_vel` 을 두고 싸운다
- **시험 스위트를 돌린 뒤 정리 없이 주행** — `test_mission_sequencer` 가 좀비를 남긴다
