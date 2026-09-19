# 예선 — 굴절코스 · S자 (용인)

> 본선 절차는 `START.md`. **§0 원점 확인은 예선에도 똑같이 필요하다.**
> 여기 적힌 것만 다르다.

## 왜 별도 경로가 필요한가

648m 본 코스 파일을 그대로 쓰면 예선 구간을 지나서도 계속 달린다.
그래서 구간만 잘라 냈다 — `tools/slice_waypoints.py`.

## 왜 구간보다 앞에서 잘랐나 — 캘리브 때문이다

`heading_init` 은 웨이포인트를 안 보고 **차가 향한 방향으로 10m 직진**한다.
구간 경계에서 자르면 그 10m 가 커브라 캘리브가 거부된다 (실측: s58 에서 자르면
편차 **31.9°**, 거부선 20°).

그래서 **구간 시작보다 14m 앞**에서 잘랐다. 잘린 경로의 앞 10m 가 그대로
캘리브 조주구간이 되고, 캘리브가 끝나는 지점이 구간 진입 4m 앞이다.

```
경로 s= 0 ─── 10 ─── 14 ──────────── 끝
      │      │      │
      │      │      └ 구간 시작 (굴절 s58 / S자 s176 — 본 코스 기준)
      │      └ 캘리브 완료. 여기서 AUTO 시작
      └ 차를 여기 놓고 손으로 민다
```

## 경로 셋 (전부 preflight `run-up 직선` 통과)

| 파일 | 본 코스 구간 | 길이 | 최소R / 타각 | 예상 |
|---|---|---|---|---|
| `wp_yongin_prelim_gujeol.yaml` | s44~140 | 95.5m | 2.82m / 15.5° | 48s |
| `wp_yongin_prelim_scurve.yaml` | s166~250 | 83.5m | 3.31m / 13.4° | 44s |
| `wp_yongin_prelim_both.yaml` | s44~250 | 205.4m | 2.82m / 15.5° | 104s |

전부 `config/yongin_2026-09-05/` 안에 있다. 예선이 둘을 이어서 하는지 따로
하는지 모르니 셋 다 만들어 뒀다 — 현장에서 고르면 된다.

## 절차

**① 차를 잘린 경로의 시작점에 놓는다** (= 구간 시작선 14m 뒤)

**② 런치** — `auto_calib:=false`. 차가 스스로 안 간다.

```bash
cd /home/han/racing_ws && bash tools/drive_school.sh --speed 1.6 --min-pwm 0 --waypoints /home/han/racing_ws/config/yongin_2026-09-05/wp_yongin_prelim_both.yaml --extra auto_calib:=false --extra ff_mode:=ros --extra ff_static:=46.0 --extra ff_gain:=42.6 --extra ff_breakaway_pwm:=90.0 --extra ff_breakaway_ms:=1200.0 --extra calib_distance:=10.0 --extra curvature_gain:=6.0 --extra avoid_steer_rate_deg:=90.0 --extra grade_ff_gain:=1.0 --extra grade_ff_max:=95.0 --extra gov_pwm:=60.0 --extra gov_min_grade:=0.06 --extra gov_gain:=300.0 --extra gov_lead_s:=0.3 --extra gov_deadband:=0.20
```

경로 파일만 `gujeol` / `scurve` / `both` 로 바꾼다.

**③ teleop** (별 터미널)

```bash
cd /home/han/racing_ws && source install/setup.bash && ros2 run velocity_controller teleop_keyboard
```

`SPACE` = 조향 중앙. **출발 신호를 기다려야 하면 `E` 로 E-stop 을 걸어 둔다.**

**④ 손으로 10m 곧게 민다** → `✅ 헤딩 초기화 완료` → 차가 구간 진입 4m 앞에 선다

**⑤ 출발** — E-stop 을 걸었으면 `E` 로 해제. 안 걸었으면 캘리브 완료 1.4초 뒤 자동으로 출발한다.

## ⚠ 알아 둘 것

- **E-stop 은 모터만 막는다.** 손으로 미는 건 안 막히고, 캘리브는 GPS·IMU 만
  보므로 E-stop 이 걸린 채로도 정상 완료된다.
- ⚠ **teleop 을 Ctrl+C 로 끄면 E-stop 이 풀려 차가 출발한다**
  (종료 시 `/e_stop=False` 를 발행한다). 해제는 반드시 **`E` 키**로.
- ⚠ **`auto_calib:=true` 를 쓰지 마라.** 차가 스스로 10m 가고 **1.4초 뒤**
  AUTO 로 넘어간다(어젯밤 실측: 완료 601.9 → AUTO 603.3). 그 사이에 E 를
  누르는 건 도박이다.
- `preflight` 가 **미션 계획 불일치를 치명으로 잡는다** — 계획이 648m 본 코스
  것이라 그렇다. **시퀀서를 안 켜면(기본) 계획은 로드조차 안 되므로 무시해도
  된다.** 구간 감속(`sequencer:=true`)을 예선에서 쓰려면 예선용 계획을 따로
  만들어야 한다 — 예선 구간은 짧아서 8분 예산 문제가 없으니 굳이 안 해도 된다.

## 다른 구간을 자르려면

```bash
python3 tools/slice_waypoints.py config/yongin_2026-09-05/wp_yongin_drive_0.5.yaml --from 44 --to 250
```

미리보기만 한다. `--write` 를 줘야 파일을 쓴다. 앞 10m 직진성·원점 스탬프·
최소 곡률을 **heading_init·preflight 와 같은 방식으로** 검사하고, 하나라도
실패하면 파일을 안 쓴다.
