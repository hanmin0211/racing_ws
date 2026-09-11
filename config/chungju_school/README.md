# 충주 캠퍼스 — 학교 시험 경로

대회장(용인)에 못 가는 날 **제어·라이다 회피를 실차로 시험**하기 위한 경로.

## 파일

| 파일 | 내용 |
|---|---|
| `wp_school_track_raw.yaml` | 2026-08-15 기록 원본 (82점 89.4m, 1m 간격) |
| **`wp_school_track_0.5.yaml`** | **★ 주행용** — 0.5m 리샘플 (179점 89.0m, 닫힌 루프) |
| `wp_school_straight_raw.yaml` | 캠퍼스 대로 직선 구간 원본 (49점 254.8m) |
| `wp_school_straight_0.5.yaml` | 위를 0.5m 리샘플 (511점 254.6m) |

## ★ 트랙 경로는 git 이력에서 복원한 것이다 (2026-09-11)

`wp_school_track_*` 는 **커밋 `16ded96`(2026-08-15)** 에서 꺼냈다.
2026-08-17 에 대구권 시험장에서 기록한 경로가 **같은 파일명**
(`src/pure_pursuit_pkg/config/waypoints_recorded_resampled_0.5.yaml`)으로
덮어써서, 워킹트리에서는 사라져 있었다.

동일성 확인:
- 논문 `paper/Fig3_trajectory.png`(= Fig.5)의 그 루프다.
- 그 그림의 원본 주행 로그 `logs/drive_20260823_015757.csv` 와 좌표계·모양 일치.
- 복원본 중심 36.970530, 127.874618 ↔ 주행 로그 중심 36.970536, 127.874621 (0.7m)

원점은 당시 쓰던 충주 원점 `(399848.522, 4092209.171)` EPSG:32652 로 스탬프했다
(`waypoint_follower/site_origin.py` 의 FALLBACK 과 같은 값이고, 8/23 주행 로그의
위경도와 로컬좌표에서 역산한 값과도 일치한다). 원본에는 스탬프가 없었다 —
원점 가드는 그 뒤 커밋 `5b87947` 에서 들어왔다.

## ⚠ 주행 전에 원점을 바꿔야 한다

`config/site_origin.yaml` 이 **지금 용인**이다. 그대로 두면 이 경로는 219km
떨어진 것으로 판정돼 `global_path_publisher` 가 발행을 거부한다(origin_check strict).

```bash
# 학교에서 (GPS 올린 뒤)
python3 tools/set_origin_from_fix.py            # 미리보기
python3 tools/set_origin_from_fix.py --write    # site_origin.yaml 갱신 (.bak 백업됨)
```

**★ 대회 전에 반드시 용인으로 되돌릴 것.** `.bak` 가 남는다.
`python3 tools/preflight.py --waypoints config/yongin_2026-09-05/wp_yongin_drive_0.5.yaml`
로 원점이 용인인지 확인하고 나갈 것.

## 폐루프 검증 (tools/tracking_sim.py, 실제 파라미터로)

| 경로 | 완주 | cte 최대 | 조향 포화 | 최대 타각 |
|---|---|---|---|---|
| `wp_school_track_0.5` (89m 루프) | ✅ 99% | 0.107m | 0% | 9.7° |
| `wp_school_straight_0.5` (255m 직선) | ✅ 100% | 0.081m | 0% | 1.8° |

## 라이다 회피 시험에 쓸 때

- **루프(89m)** — 반복 주행에 좋다. 장애물을 놓고 여러 번 돌려보기.
- **직선(255m)** — S코스 흉내. 좌·우 랜덤 배치를 재현하기 좋다.

⚠ 회피 자체는 **s 구간 등록이 필요 없다.** 전방 감속·정지(`/obstacle_distance`)는
코스 전체에서 항상 동작하고, 조향 override 는 `sequencer:=false` 면 구간 제한 없이
켜진다(`require_arm_for_steer` 가 `sequencer` 인자에 묶여 있다).
