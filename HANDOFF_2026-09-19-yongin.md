# 인계 — 2026-09-19 용인 현장 (예선 굴절코스)

> **이 문서만 읽고 이어서 일할 수 있어야 한다.**
> 절차는 `ARRIVE.md`(도착 순서) · `START.md`(본선) · `PRELIM.md`(예선).
> 어젯밤 근거는 `HANDOFF_2026-09-19-night.md`. 여기는 **오늘 현장에서 일어난 것**이다.

**일정**: 9/19(오늘) 현장 점검·예선 · 9/20 본경기 (오후 5시까지)

---

# 1. ★ 지금 상태 — 바로 이어서 할 것

## 1.1 예선 굴절코스 주행 명령 (검증 완료, 바로 실행 가능)

```bash
cd /home/han/racing_ws && bash tools/ros_cleanup.sh
```

**터미널 1 — 주행** (라이다 없이. 예선은 장애물이 없다)
```bash
cd /home/han/racing_ws && source install/setup.bash && ros2 launch gps_localization bringup.launch.py control:=true sequencer:=false lidar:=false rviz:=false auto_calib:=false waypoints:=/home/han/racing_ws/config/yongin_2026-09-05/wp_prelim_gujeol_smooth.yaml max_speed:=1.6 curvature_gain:=6.0 calib_distance:=10.0 auto_calib_speed:=0.5 ff_mode:=ros ff_static:=46.0 ff_gain:=42.6 ff_min_pwm:=0.0 ff_breakaway_pwm:=90.0 ff_breakaway_ms:=1200.0 grade_ff_gain:=1.0 grade_ff_max:=95.0 gov_pwm:=60.0 gov_min_grade:=0.06 gov_gain:=300.0 gov_lead_s:=0.3 gov_deadband:=0.20
```

**터미널 2 — teleop**
```bash
cd /home/han/racing_ws && source install/setup.bash && ros2 run velocity_controller teleop_keyboard
```

**터미널 3 — 기록**
```bash
cd /home/han/racing_ws && bash tools/record_run.sh prelim_gujeol
```

**순서**
```
① 차를 (−4.19, +29.20) — 웨이포인트 찍기 시작한 그 자리에 놓는다
② teleop 에서 SPACE  (조향 중앙)
③ 출발 신호를 기다려야 하면 E  (E-stop)
④ 손으로 10m 곧게 민다  →  ✅ 헤딩 초기화 완료
⑤ E 한 번 더 → 해제 → AUTO 출발
```

**볼 줄**
```
전역 경로 로드 완료: (총 132개 점, 65.0m)
✅ 헤딩 초기화 완료
헤딩 캘리브 완료 — 자율 허용
모드: AUTO
```

⚠ `auto_calib:=false` 다 — **손으로 밀어야** 캘리브가 진행된다. 차는 스스로 안 간다.

⚠ **`preflight` 가 이 경로에 치명 2건을 낸다 — 무시해도 된다.**
```
❌ 미션 계획: 코스 길이 불일치 (계획 648.3m vs 실제 65.0m)
❌ 미션 계획: 웨이포인트 불일치
```
`config/mission_plan.yaml` 이 648m 본 코스 것이라 그렇다. **시퀀서를 안 켜면
(`sequencer:=false`, 기본) 계획은 로드조차 안 된다.** 원점·경로·run-up 은 전부 통과한다:
```
✅ 경로: 원점 일치 (용인)   132점 65.0m   최소R 3.19m → 13.8°/18°
✅ run-up 직선 — 여기서 캘리브하면 안전
```
구간 감속(`sequencer:=true`)을 예선에 쓰려면 **예선용 계획을 따로 만들어야** 한다.
예선 구간은 짧아 8분 예산 문제가 없으니 굳이 안 해도 된다.
⚠ **teleop 을 Ctrl+C 로 끄면 E-stop 이 풀려 차가 출발한다**(종료 시 `/e_stop=False` 발행).
   해제는 반드시 **`E` 키**로.

## 1.2 남은 위험 둘

**① 공급전압** — 허브를 뺀 뒤로 **다시 안 쟀다.** 나가기 전에:
```bash
cd /home/han/racing_ws && bash tools/ros_cleanup.sh && source install/setup.bash && python3 tools/preflight.py --waypoints config/yongin_2026-09-05/wp_prelim_gujeol_smooth.yaml --skip-parking 2>&1 | grep -A2 "무부하"
```
허브 제거 전 **2848~3117mV**(정상 4700~5100, 리셋선 3483). 주행 중 갑자기 서면 이것이다.

**② 라이다** — 아직 안 붙는다. 예선엔 필요 없지만 **본선(S자 장애물·돌발정지)엔 필수다.** §4 참고.

---

# 2. ★ 오늘 현장에서 터진 것 — 전부

## 2.1 USB 허브가 GPS·라이다·아두이노를 전부 죽였다

도착 당시 배선:
```
IMU       3-1          허브 0단 (직결)   ← 유일하게 정상이었다
아두이노   3-2.2        허브 1단
라이다     3-2.1.2.1    허브 3단          ← scan 시작 실패
GPS       3-2.1.4.3    허브 3단          ← ublox USB 초기화 무한 반복
```
`lsusb` 에 허브가 **17개** 잡혔다. 버스파워 허브가 겹겹이 물려 전압을 나눠 먹었다.

**증상 셋이 전부 같은 뿌리였다:**
- 라이다: `health OK` 는 통과하고 `Can not start scan: 80008002` — **모터를 못 돌린다**
- GPS: `Starting USB initialization` 1초마다 무한 반복 — 장치에 못 붙는다
  → `/fix` 미발행 → `direct_localization: 대기중 — fix:X imu:O`
  → NTRIP 이 **`GGA 고정(36.97060,127.87480)`**(충주 좌표)를 보내 VRS 보정도 엉뚱했다
- 아두이노 5V 레일 **무부하 2848~3117mV**

**조치**: 허브를 빼고 넷을 직결. 그 과정에서 **GPS 케이블이 빠져 한동안 `lsusb` 에
u-blox 가 아예 없었다** — `1546` 이 안 보이면 물리적으로 빠진 것이다.

**현재 배선**
```
아두이노   usb3/3-2     허브 0단  ✅
IMU        usb3/3-1     허브 0단  ✅
라이다     usb1/1-2     허브 0단  ✅
u-blox     1-2.4.3      허브 2단  ⚠ 아직 허브 밑 (그래도 fix 8.4Hz 나온다)
```

**GPS 복구 확인** (이 줄들이 정상 상태다)
```
[NTRIP 연결됨] ... GGA 실측(37.28915,127.10695)    ← "고정" 아니고 "실측", 용인 좌표
/fix  8.4 Hz
direct_localization: pose: x=-4.19 y=29.21 yaw=144.7°
```

⚠ `ublox_dgnss` 의 `Proceeding with partial parameter initialization (degraded mode)` 는
정상 동작 중에도 뜬다. fix 가 나오면 무시해도 된다(어젯밤에도 같은 경고로 완주했다).

## 2.2 라이다 — 아직 미해결

시도한 것과 결과:
```
lidar_dual.launch.py (256000)      80008002 timeout / health OK 후 scan 실패 / 80008004 즉시
sllidar_a1_launch.py (115200 기본)  80008002 timeout (2.5초)
sllidar_a1_launch.py 256000         80008000 즉시 (23ms)
```
- **에러가 시도마다 다르다** = 설정이 아니라 물리적으로 불안정
- 보드레이트는 원인이 아니다: **어젯밤 같은 256000 으로 정상 작동**했다
  (근거: `data/2026-09-19-night/*.csv` 에 AVOID/BLOCKED 가 233~1153행)
- 포트를 쥔 프로세스도 없었다
- **S/N 이 읽을 때마다 다르게 나왔다** → 응답이 없어 버퍼 쓰레기를 찍은 것

**결론: 전원.** 허브를 뺀 지금 다시 시도해 볼 것:
```bash
cd /home/han/racing_ws && bash tools/lidar_up.sh 12
```
(실패할 때마다 드라이버를 정리하고 1.5초 쉬었다 재시도한다 — 칩 리셋 시간)

⚠ `sllidar_a1_launch.py` 로 띄우면 토픽이 **`/scan`** 이다. 그때는 주행 런치에
**`scan_topic:=/scan`** 을 줘야 한다. 안 주면 **에러 없이 조용히 라이다가 없는 것처럼** 돈다.

## 2.3 굴절 웨이포인트 실측 — **원본 그대로 쓰면 못 돈다**

13:18 에 손으로 밀며 116점을 찍었다(σ=0.4cm, RTK 양호). 그런데:

```
파일                              점    최소R    필요타각        run-up
wp_prelim_gujeol_raw              116   2.38m   18.2°  ❌ 한계 초과   —
wp_prelim_gujeol_raw_resampled_0.5 131  2.51m   17.4°  여유 0.6°     ⚠ 6.0°
wp_prelim_gujeol_smooth           132   3.19m   13.8°  여유 4.2°     ✅ 3.8° 직선
```

**손으로 밀면 주행선보다 좁게 돈다.** 원본은 필요타각 **18.2°** 로 **차량 한계 18° 를 넘어
물리적으로 못 도는 커브**가 있었다.

`tools/smooth_path.py` 로 폈다:
```bash
python3 tools/smooth_path.py --in config/yongin_2026-09-05/wp_prelim_gujeol_raw.yaml \
  --out config/yongin_2026-09-05/wp_prelim_gujeol_smooth.yaml \
  --step 0.5 --target-r 3.2 --max-dev 0.25 --write
```
원본에서 **최대 0.13m · 평균 0.04m** 만 움직였다 — 민 선 그대로다.
시작·종점이 원본과 동일(`(−4.19,+29.20)` → `(+22.93,+5.56)`, 화면 마지막 점 #116 과 일치).

**교훈: 찍은 경로는 반드시 `preflight` 로 검증하고, 필요타각이 16° 를 넘으면 평활화할 것.**

---

# 3. ★ 어제 "꼭 저장하라" 한 것 — 전부

## 3.1 원점 — 틀리면 차가 한 발짝도 안 간다

`global_path_publisher` 가 웨이포인트 원점 스탬프를 `config/site_origin.yaml` 과 대조해
**1km 이상 차이면 전역 경로를 발행하지 않는다**(`origin_check` 기본 strict).
충주 ↔ 용인 = **76.8km**.

**증상이 고약하다**: 장치 정상, RTK 정상, **캘리브도 `✅ 완료`**, 먹스도 `자율 허용`.
그런데 `/global_path` 가 없어 차가 안 움직인다.
**유일한 단서는 런치 로그에 `전역 경로 로드 완료` 가 안 뜨는 것.**

```bash
bash tools/use_site.sh          # 현재 원점 확인
bash tools/use_site.sh yongin   # 대회장
bash tools/use_site.sh chungju  # 학교
```
**현재: 용인 (332200.0, 4128600.0) ✅**

같은 함정이 `~/stop_points.yaml` 에도 있었다 — 다른 장소(원점 477800/3964400) 것이라
용인 경로에서 **149.7km** 떨어져 있었다. 빈 용인 파일로 교체했고
원본은 `~/stop_points.OTHERSITE.bak.yaml` 에 보관했다.

## 3.2 절대 빠뜨리면 안 되는 런치 인자

| 인자 | 빼면 | 증상 |
|---|---|---|
| **`grade_ff_gain:=1.0`** | 경사 보상 꺼짐(기본 0.0) | 경사로에서 속도가 단조 감소하다 정지. **메시지 없음** |
| `grade_ff_max:=95.0` | 상한 70 으로 잘림 | 급경사에서 보상 부족 |
| **`auto_calib_speed:=0.5`** | `drive_school.sh` 가 전압 낮으면 최대 1.2 로 올림 | 캘리브 게이트 여유 +49%→−3% = **무효** |
| `ff_static:=46.0` | 17.2 로 떨어짐 | 정지마찰 못 넘어 아예 안 움직임 |
| `ff_breakaway_pwm:=90.0` | 60 으로 떨어짐 | 출발 실패 |

2026-09-19 03:2x 에 `grade_ff_gain` 을 빼고 돌려 s=74m 에서 죽었다. **배터리로 오진**했다가
`/tmp/launch_params_*` 덤프로 확정했다. **`exec` 직전 출력에서 눈으로 확인할 것.**

## 3.3 검증된 상수 (양방향 완주한 값)

```
ff_mode:=ros  ff_static:=46.0  ff_gain:=42.6  ff_min_pwm:=0.0
ff_breakaway_pwm:=90.0  ff_breakaway_ms:=1200.0
grade_ff_gain:=1.0  grade_ff_max:=95.0
gov_pwm:=60.0  gov_min_grade:=0.06  gov_gain:=300.0  gov_lead_s:=0.3  gov_deadband:=0.20
max_speed:=1.6  curvature_gain:=6.0  avoid_steer_rate_deg:=90.0
calib_distance:=10.0  auto_calib_speed:=0.5
```

**실측 관계** (어젯밤 위치미분 996표본)
```
실제속도 = 0.819 + 1.0415 × 명령       (개루프)
최저 지속속도 ~1.34 m/s = 4.8 km/h     ← 이 아래는 정지마찰을 못 넘는다
  명령 0.5 → PWM 67.3 → 1.34 m/s
  명령 1.6 → PWM 114  → 2.48 m/s
```
⚠ **4 km/h 는 물리적으로 불가능하다**(PWM 58, 문턱 아래). 5 km/h 가 하한이다.

실측 FF 는 `PWM = 40.9·v + 12.5` 인데 쓰는 건 `42.6·v + 46.0` 이다 — `ff_static` 만
+33.5 과하다. **내리면 캘리브 저속에서 차가 선다**(어젯밤 두 번 확인). 쓰려면
`ff_min_pwm:=67` 이 같이 가야 하고, 그건 **미검증**이다.

## 3.4 캘리브 — 원리와 게이트

`yaw_offset = normalize(GPS course − IMU yaw)` = **IMU 좌표계와 맵 좌표계의 상수 회전**.
`/heading/yaw_offset` 을 **래치(TRANSIENT_LOCAL)** 로 발행하고 노드는 종료한다.
→ **코스 밖에서 캘리브하고 런치를 켠 채 차를 옮겨도 된다.**
⚠ **런치를 Ctrl+C 하면 날아간다.** IMU 가 재열거돼도 날아간다.

**게이트 6개** — 막히면 원인이 이것이다:
```
RTK 수렴 (σ≤5cm)        "RTK 수렴 대기 중" — ublox 로그에 degraded mode 확인
이동속도 ≤ 2 m/s         "측위 점프 감지" → 시작점 재설정
직진성 (편차 20°)        "❌ 무효"
최소 소요시간            "❌ 무효: N m 를 N초 만에"  ← auto_speed_cap 2.0(ff_mode ros)
앞바퀴 중앙 (2°)         시작조차 안 함
재시도                   시작점 2m 안으로 되돌려야 재출발
```

**10m 가 안 나오면** `calib_distance:=5.0` (헤딩 오차 0.23°). 3m 밑 금지. **5m 는 미검증.**

**손으로 밀면 자동직진보다 휜다**: 자동 2~4° vs 손 6°. 그리고 주행으로 재니
**일관되게 −6° 치우쳐** 있었다. **차 정중앙 뒤에서** 밀 것.

## 3.5 E-stop — 출발선에서 출발하려면

`teleop_keyboard` 의 **`E`** 키가 `/e_stop` 을 토글한다. 먹스가 최우선으로 차단한다.
**E-stop 은 모터만 막고 손으로 미는 것·캘리브(GPS·IMU)는 안 막는다.**

⚠ **`auto_calib:=true` 를 예선에 쓰지 마라.** 차가 스스로 10m 가고 캘리브 완료
**1.4초 뒤** AUTO 로 넘어간다(실측: 완료 601.9 → AUTO 603.3). 그 사이에 E 를 누르는 건 도박이다.
⚠ **teleop 을 Ctrl+C 로 끄면 E-stop 이 풀린다**(`finally` 에서 `/e_stop=False` 발행).

## 3.6 주행마다 기록 — `tools/record_run.sh`

```bash
cd /home/han/racing_ws && bash tools/record_run.sh <이름>
```
한 폴더에 `drive.csv` · `drive_scan.csv` · `params/`(런치 파라미터 덤프) ·
`roslog/`(노드 stdout) · `run.txt`(명령줄·커밋·원점·IMU 영점) 를 모은다.

⚠ `drive_record.py` 를 직접 돌리면 **궤적만** 남는다. 설정은 `/tmp/launch_params_*` 에
있고 **재부팅하면 사라진다** — 어젯밤 덤프가 실제로 전부 날아갔다(확인 시점 0개).
⚠ **런치를 먼저 띄우고** 실행할 것. 아니면 "이 주행 중에 만들어진 ROS 세션이 없다" 고
경고하고 로그를 안 남긴다(옛 세션을 이 주행 것으로 복사하지 않기 위해서다).

## 3.7 라이다 회피 — 알아 둘 것

- `fg_obstacle_trigger 3.0m` 에서 AVOID 진입 → 종방향이 `v_obs = 0.8·(d−0.8)/3.2` 로 감속
- **회피 각도 결정은 피드백이 아니다.** follow-gap 이고 **완전 무상태** — 매 프레임 재판정
- 떨림의 원인은 **갭 뒤집힘이 아니라 같은 갭 안의 조준각 지터**이고 **잡음에 정비례**한다
  (재현: 갭 건너뜀 전 조건 **0회**, 요구 조향율 잡음 0→5cm 에서 160→460°/s)
  → **`avoid_steer_rate_deg:=90` 이 증상 차단이 아니라 원인을 직접 치는 것이다**
- **이탈은 속도에 비례한다**: 1.13m @1.8 m/s → **1.91m @2.7 m/s** (실측)
  오프라인 시뮬은 속도 무관이라고 한다 — **그 점에서 시뮬을 믿지 마라**
- ⚠ `obstacle_stop_dist` 를 올리지 마라. AVOID 중에도 `/obstacle_distance` 에 전방거리가
  나가 **장애물마다 멀리서 서고 8초 뒤 회피를 포기**한다(9/12: 400초에 7m 전진)
- 규정상 **바깥으로 도는 것은 위반이 아니다** — "중앙선 없이 좌·우측으로 자유 회피".
  감점은 접촉 10점/회 · 구간 내 차선이탈 최대 10점

## 3.8 종점 정지 — 1~4m 지나서 선다 (정상)

완주 신호 뒤 명령이 0 이 되면 `PWM:0` 만 나가고 **역토크가 없다.**
⚠ **능동 제동(`ff_brake_pwm`)은 켜도 안 걸린다.** 진입 조건이 "직전 명령 ≥ 0.40" 인데
완주 감속이 `max_decel 1.8 × dt 0.05` = 한 스텝 0.09 라 정지명령 순간의 `_last_cmd_v` 가
항상 0.05~0.14 다. **수학적으로 절대 트리거되지 않는다**(실제 `_brake_pwm()` 호출로 확인).
→ 장애물·정지선 감속도 같은 램프라 **자율 주행에서 능동 제동이 걸린 적이 없다.**

## 3.9 절대 하지 말 것

- **런치를 Ctrl+C** — 캘리브가 날아간다
- **`gov_min_grade` 올리기** — 게이트가 안 열리면 내리막에서 못 잡아 **탈락**(fail-open 설계)
- **`obstacle_stop_dist` 올리기** — §3.7
- **`ff_static`/`ff_gain` 바꾸기** — 양방향 완주한 값
- **`--extra waypoints:=`** — `drive_school.sh` 기본값과 중복된다. **`--waypoints`** 를 쓸 것
- **`wasd_teleop`** — `/cmd_vel` 로 직접 쏴 먹스와 싸운다. `teleop_keyboard` 를 쓸 것
- **USB 허브** — 오늘 GPS·라이다·아두이노를 동시에 죽였다
- **시험 스위트 뒤 정리 없이 주행** — `test_mission_sequencer` 가 좀비를 남긴다

---

# 4. 다음에 할 일

1. **전압 재측정** (허브 제거 후 미확인) — 4300mV 넘어야 한다
2. **라이다 복구** — `bash tools/lidar_up.sh 12`. 본선 필수
3. **예선 굴절 주행** — §1.1
4. **S자 웨이포인트 찍기** — 굴절과 같은 절차. ⚠ **구간 시작보다 10m 앞에서부터**
   찍고, 찍은 뒤 반드시 `preflight` + 필요하면 `smooth_path.py`
5. **본선 트랙 주행** — `ARRIVE.md` §7
6. 시간 남으면 **구간 감속**(굴절·S자 5 km/h): `config/mission_plan.yaml` 의
   `ramp_up`/`ramp_down` 슬롯에 s 를 넣고 `enabled:true`, 런치에
   `sequencer:=true ramp_up_arm_topic:=/ramp/up_arm ramp_down_arm_topic:=/ramp/down_arm
   v_ramp_up:=0.55 v_ramp_down:=0.55` (⚠ **소수점 필수**)

---

# 5. 내가 틀렸던 것 (오늘)

1. **`auto_calib_speed:=0.5` 를 "중복이니 뺀다" 고 판단해 뺐다.** `drive_school.sh` 의
   `CALIB=0.5` 는 고정값이 아니라 **전압이 4700mV 밑이면 다시 계산**된다(최대 1.2).
   `--min-pwm 0` 은 `ff_min_pwm` 만 덮어쓴다. 사용자가 어젯밤 명령과 대조해 잡아냈다.
2. **라이다 실패를 보드레이트로 의심했다.** 어젯밤 같은 256000 으로 정상 작동한
   기록(주행 CSV 의 AVOID 행)이 있었는데 그걸 먼저 안 봤다.
3. **`record_run.sh` 에 `exec` 를 썼다** — 셸이 교체돼 `trap EXIT` 이 죽고 설정
   스냅샷이 통째로 안 남았다.
4. **`pgrep -f 'bringup.launch.py'` 로 런치를 찾았다** — 검사 문자열이 자기 명령줄에
   들어가 **늘 성공**한다. 런치가 없는데도 옛 세션 로그를 이 주행 것으로 복사했다.
   시각 비교(세션 mtime ≥ 주행 시작)로 바꿨다.
