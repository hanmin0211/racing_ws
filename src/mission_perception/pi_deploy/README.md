# pi_deploy — 라즈베리파이5 + Hailo-8 쪽 코드

> 이 디렉터리의 파일은 **노트북에서 실행되지 않는다.** 라즈베리파이로 복사해서
> 파이에서 돌리는 원본이다. 여기가 원본이고 파이는 배포본이다.
> (파이 SD카드에만 있으면 카드가 죽을 때 같이 사라지므로 git으로 관리한다)

## 왜 파이에서 도는가

신호등 인식은 Hailo-8 NPU에서 돈다. Hailo-8은 파이5의 PCIe에 M.2로 붙어 있고
(AI HAT+ 26TOPS), 노트북에서는 접근할 수 없다. 그래서 인지만 파이가 맡고,
결과 문자열만 ROS 2 토픽으로 노트북 주행 스택에 넘긴다.

```
[C920] → [파이5 + Hailo-8] → /traffic_light_state (String) ──랜선──→ [노트북 주행 스택]
                                  "RED" / "GREEN" / "NONE"
```

## 파일

| 파일 | 파이에서의 위치 |
|---|---|
| `detector_node.py` | `~/ros2_ws/.../traffic_light_detector/detector_node.py` |
| `coco_detector_node.py` | `~/ros2_ws/.../traffic_light_detector/coco_detector_node.py` |

## 검출기가 두 개인 이유 — 비교용

| | `detector_node` | `coco_detector_node` |
|---|---|---|
| 모델 | `~/yolov8s.hef` (커스텀 2클래스) | `~/models/yolov8s_coco.hef` (COCO 80클래스) |
| 출처 | 누가 언제 학습했는지 **불명**. 데이터셋·`best.pt` 없음 | Hailo Model Zoo v2.13.0 사전컴파일 |
| 색 판별 | 모델이 직접 (`traffic_green`/`traffic_red`) | **세로 위치 + HSV 교차검증** |
| 노랑 | ❌ 없음 | ✅ 있음 |
| 재학습 | **불가** (데이터·DFC 둘 다 없음) | 불필요 |
| 칩 속도 | 185 FPS | 240 FPS |
| 토픽 | `/traffic_light_state` | `/traffic_light_state_coco` |

**칩과 카메라는 한 프로세스만 점유한다.** 동시에 못 켜니 번갈아 실행해서
실제 신호등으로 비교할 것.

```bash
ros2 run traffic_light_detector coco_traffic_light_node
```

### COCO 쪽 상태 판별 방식

COCO 는 `traffic light`(class 9)의 **위치만** 알려주고 색은 안 알려준다.
그래서 박스를 잘라 직접 판별한다. 대회 신호등 도면(전체 116cm, 신호부
24×56cm, 램프 지름 10cm, 간격 16/15/16)에서 램프 중심을 정규화하면

```
빨강 0.29    노랑 0.50    초록 0.71
```

**위치를 주로, 색을 보조로** 쓴다. 위치는 물리적으로 고정이라 조명·노출에
안 흔들리지만 색조는 흔들린다. 특히 한국 신호등의 '초록'은 실제로 청록이라
(도면에서도 아래 램프가 파랗게 그려져 있다) 순수 초록만 찾으면 놓친다.
둘이 어긋나면 `NONE` 으로 떨어뜨려 빨간 간판·초록 나무 오탐을 막는다.

파라미터 `use_color_check:=false` 로 색 검증을 꺼서 위치만으로 볼 수 있다
(색이 안 맞을 때 원인 분리용).

### 검출 거리 (계산값, 실측 필요)

램프가 10cm 뿐이라 거리 한계가 있다. 640px 입력 기준 추정:

| 거리 | 신호부 폭 | 램프 지름 |
|---|---|---|
| 3m | ~49 px | ~20 px |
| 5m | ~29 px | ~12 px |
| 8m | ~18 px | ~7 px |

**5m 까지는 넉넉, 8m 부터 위태롭다.** 차량 0.4m/s 면 5m 는 12초 전이라
충분하다. 화각 추정치라 실물로 반드시 실측할 것.

파이 쪽 나머지(패키지 뼈대, HailoRT, ROS2 Humble 소스빌드, `yolov8s.hef`)는
`~/Downloads/라즈베리파이5_Hailo8_YOLO_구축매뉴얼.docx` 절차로 이미 구축돼 있다.
**재구축할 필요 없다.**

## 배포

```bash
scp src/mission_perception/pi_deploy/detector_node.py \
    pi@192.168.99.8:~/ros2_ws/src/traffic_light_detector/traffic_light_detector/
```

`colcon build --symlink-install` 로 빌드돼 있어서 파이썬 파일은 복사만 하면
바로 반영된다. 재빌드 불필요.

## 실행 (파이에서)

```bash
source ~/ros2_humble/install/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run traffic_light_detector traffic_light_node
```

SSH로 띄우고 세션을 끊을 거면 완전히 분리해야 한다:

```bash
nohup ros2 run traffic_light_detector traffic_light_node > ~/tld.log 2>&1 < /dev/null &
disown
```

종료할 때 `pkill -f traffic_light_node` 는 **SSH 세션 자체를 죽인다**
(pkill 자기 명령줄이 패턴에 걸림). 대괄호 트릭을 쓸 것:

```bash
pkill -f '[t]raffic_light_node'
```

## 파라미터 (재빌드 없이 조정 가능)

| 파라미터 | 기본 | 설명 |
|---|---|---|
| `confirm_frames` | 5 | 이 횟수만큼 연속 같은 상태여야 확정. 30Hz 기준 ~167ms |
| `score_threshold` | 0.4 | 검출 점수 하한 |
| `camera_device` | 0 | `/dev/videoN` 의 N. USB 포트 바뀌면 조정 |

```bash
ros2 run traffic_light_detector traffic_light_node \
    --ros-args -p confirm_frames:=8 -p score_threshold:=0.5
```

## ★ 시간 필터를 반드시 거치는 이유

필터가 없던 초기 버전은 **카메라가 신호등을 보고 있지 않아도** 1프레임짜리
GREEN 오탐을 그대로 발행했다. 2026-08-16 실측:

```
필터 전:  6.6초에 4번 잘못된 상태 변화 (매번 1프레임)
필터 후:  41초에 0번 (내부에서 2회 걸러냄)
```

이 값이 차량 제어로 들어가면 **한 프레임 오탐이 곧 '출발' 명령**이 되고,
RED 쪽으로 튀면 급정거가 된다. 발행 주기(30Hz)는 그대로 두고 확정된 상태만
내보낸다.

주기 로그에 `걸러낸튐=N` 이 찍힌다. **이 숫자가 계속 늘면** score_threshold를
올리거나 카메라 각도·노출을 손봐야 한다는 신호다.

## 지금 안 되는 것

모델은 **신호등 2클래스뿐**이다 (`classes.json`: `traffic_green`, `traffic_red`).
**정지선과 횡단보도는 이 모델에 없다.** 별도 학습+HEF 재컴파일이 필요하거나,
고전 영상처리(시점변환 + 흰선/줄무늬 검출)로 따로 잡아야 한다.

그리고 신호등만으로는 **어디서 멈출지**를 모른다. 정지 지점은 GPS 경로에
미리 기록해두는 쪽이 현실적이다.
