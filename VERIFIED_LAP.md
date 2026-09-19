# ★ 검증된 본선 주행 명령 — 2026-09-19 15:36 용인

> **이대로 간다.** 이 문서의 명령을 글자 하나 바꾸지 말 것.
> 근거 로그: `data/2026-09-19/mainlap_1536/console.log` (555행)
> 파라미터 덤프: `data/2026-09-19/mainlap_1536/params/` (14개)

---

## 1. 명령 (그대로 복사)

```bash
cd /home/han/racing_ws && bash tools/ros_cleanup.sh
```

`✅ 없음` 확인 후, **차를 코스 출발점 (56.09, 5.39) 에 놓고**:

```bash
cd /home/han/racing_ws && source install/setup.bash && ros2 launch gps_localization bringup.launch.py control:=true sequencer:=false lidar:=false rviz:=true auto_calib:=true waypoints:=/home/han/racing_ws/config/yongin_2026-09-05/wp_yongin_drive_0.5.yaml max_speed:=1.6 curvature_gain:=6.0 calib_distance:=10.0 auto_calib_speed:=0.5 ff_mode:=ros ff_static:=46.0 ff_gain:=42.6 ff_min_pwm:=0.0 ff_breakaway_pwm:=90.0 ff_breakaway_ms:=1200.0 grade_ff_gain:=1.0 grade_ff_max:=95.0 gov_pwm:=60.0 gov_min_grade:=0.06 gov_gain:=300.0 gov_lead_s:=0.3 gov_deadband:=0.20 2>&1 | tee /tmp/calib.log
```

**손 안 댄다.** 5초 카운트다운 후 차가 스스로 10m 직진 → 캘리브 → AUTO.

## 2. 이 줄들이 나오면 정상 (15:36 실측)

```
RTK 수렴 (수평 σ=1.5cm (상한 5cm), status=1) — 헤딩 캘리브를 시작한다.
직진 중... 0.0/10m  →  9.0/10m
✅ 헤딩 초기화 완료: 10.3m 직진 (최대 편차 1°).
   GPS course=159.4°, IMU yaw=120.0°, → yaw_offset=39.5°
헤딩 캘리브 완료 (39.5°) — 자율 허용
모드: AUTO
```

캘리브 주행은 **14.0초에 10.3m = 평균 0.74 m/s** 였다.

## 3. 그날 실제로 일어난 것

```
출발 (56.2, 3.8)  →  마지막 확인 (0.75, −44.26)
AUTO 주행 3초 샘플 32건, 직선거리 합 약 186m
거버너 개입 10회
VMIN 4106 mV (리셋선 3483 — 뚫지 않았다)
```

## 4. ⚠ 알고 쓰는 것 — 고치지 말고 알고만 있을 것

**① 개루프가 명령보다 훨씬 빠르다.**
```
명령 1.50 → 측정 2.37 m/s   (거버너가 제동 −60 으로 잡았다)
구간 최고 측정 3.09 m/s      (max_speed 는 1.6)
```
`ff_static:=46.0` 은 **정지마찰을 넘기려고** 올린 값이라 고속에서 과하다.
내리면 저속에서 차가 안 나간다(`ff-static-cut-breaks-stiction`). 그래서 그대로 둔다.
**거버너가 이 과속을 잡아 주고 있다** — `gov_pwm:=60` 을 끄면 안 된다.

⚠ 이 코스 최소R 2.71m(급커브 19개)에서 3.09 m/s 는 이탈 여유가 없다.
   속도를 낮추려면 `ff_static` 이 아니라 **`max_speed` 를 내릴 것.**

**② 거버너 자세 게이트가 평지에서도 열린다.**
`gov_min_grade:=0.06` 인데 평지 주행 중 10회 걸렸다. 가속 오염 보정
(`48bd0f5`)을 넣었어도 남는다. 지금은 **과속을 잡아 주는 쪽으로 작동**하므로
그대로 둔다. 문턱을 올리면 이 과속을 못 잡는다.

**③ 캘리브 전에는 거버너가 캘리브를 깎는다.**
```
거버너 — 측정 0.60+0.24(선행) > 명령 0.50 (+0.34) → PWM 69 → 28
```
그래도 캘리브는 **편차 1°** 로 통과했다. 건드리지 말 것.

**④ 전압.** 무부하 4092~4137 mV (preflight 문턱 4300). 주행 중 VMIN 4106.
과거 리셋선 3483 을 이번엔 안 뚫었지만 여유가 없다. **주행 전 충전할 것.**

**⑤ 라이다 꺼져 있다.** 본선의 S자 장애물·돌발정지를 하려면 `lidar:=true`.
붙는지 먼저: `bash tools/lidar_up.sh 12`

**⑥ 활성 미션 0개 · 정지점 0개.** 자율 완주만 한다(미션은 전부 감점).
시퀀서를 켜려면 `config/mission_plan.yaml` 에서 s 를 재고 `enabled: true`.

## 5. 실패하면

| 증상 | 원인 | 조치 |
|---|---|---|
| heading_init 이 **로그를 한 줄도 안 낸다** | 재시도를 다 써서 `drive_aborted` — 이 상태에선 영원히 조용하다 | 런치를 죽이고 다시 띄운다 |
| `❌ 헤딩 캘리브 무효 ... 속도상한` | 개루프 과속 | 이미 고쳤다(`283d654`). `ff_mode:=ros` 면 `auto_speed_cap` 이 2.0 으로 자동 전달된다 — `ros2 param get /gps_heading_init auto_speed_cap` 으로 확인 |
| `↩ 차를 시작점으로 되돌리세요` | 실패 후 10m 앞에 서 있다 | 출발점으로 되돌리면 자동 재출발 |
| 스택이 두 벌 | 정리 없이 겹쳐 띄웠다. `/global_path` 에 경로 2종이 번갈아 올라온다 | 반드시 `ros_cleanup.sh` 먼저 |
