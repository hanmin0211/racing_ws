#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
coco_detector_node.py — COCO 사전학습 YOLOv8s 로 세로 신호등 상태 판별

기존 detector_node.py(커스텀 2클래스 모델)의 대안. 나란히 두고 비교하려고
만들었다. 토픽 이름이 달라서 동시에 켜도 서로 안 부딪힌다 —
단, Hailo 칩과 카메라는 한 프로세스만 점유하므로 실제로는 번갈아 실행한다.

★ 왜 COCO 인가
  COCO 80클래스에 traffic light(class 9)이 있다. 학습·라벨링·DFC 설치가
  전부 불필요하고, 미리 컴파일된 HEF를 받아 쓰면 된다. COCO 신호등 사진은
  대부분 서구권 도로의 **세로 3구**라, 우리 대회 신호등(세로 3구)과 형태가
  맞는다. 2026-08-16 실측: Hailo-8 에서 240 FPS.

  단점: COCO 는 '신호등이 있다'까지만 알려주고 **색은 안 알려준다.**
  그래서 색/상태는 아래 후처리로 직접 뽑는다.

★ 상태 판별 — 위치를 주로, 색을 보조로
  대회 신호등 도면(2026-08-16 수령): 전체 116cm, 신호부 24x56cm,
  램프 지름 10cm(R5), 위에서부터 빨강-노랑-초록. 램프 간격 16/15/16.
  신호부 높이 56 으로 정규화하면 램프 중심이

      빨강 ≈ 0.29    노랑 ≈ 0.50    초록 ≈ 0.71

  즉 **켜진 램프의 세로 위치가 곧 상태**다. 이건 물리적으로 고정이라
  조명·노출에 흔들리지 않는다. 반면 색조는 흔들린다 — 특히 한국 신호등의
  '초록'은 실제로는 청록(靑綠)이라 순수 초록만 찾으면 놓친다(도면에서도
  아래 램프가 파란색으로 그려져 있다).

  그래서 **위치로 먼저 정하고, 색으로 교차 검증**한다. 색이 위치와 맞지
  않으면 신뢰도를 낮춰 NONE 으로 떨어뜨린다. 빨간 간판·초록 나무에 속는
  것을 이 교차 검증이 막는다.

★ 시간 필터는 기존 노드와 같은 이유로 반드시 거친다 (README 참조).

출력: /traffic_light_state_coco  (String: "RED"/"YELLOW"/"GREEN"/"NONE")
"""

import os
import sys
import time

sys.path.append(os.path.expanduser('~/Hailo-Application-Code-Examples/runtime/python'))
sys.path.append(os.path.expanduser(
    '~/Hailo-Application-Code-Examples/runtime/python/object_detection'))

import cv2                                              # noqa: E402
import numpy as np                                      # noqa: E402
import rclpy                                            # noqa: E402
from rclpy.node import Node                             # noqa: E402
from sensor_msgs.msg import CompressedImage              # noqa: E402
from std_msgs.msg import String                         # noqa: E402

from common.hailo_inference import HailoInfer           # noqa: E402
from common.toolbox import default_preprocess           # noqa: E402
from object_detection_post_process import extract_detections   # noqa: E402

HEF_PATH = os.path.expanduser('~/models/yolov8s_coco.hef')

# ★ 2026-09-17 — 디버그 화면용 COCO 80클래스 이름.
#   "신호등박스=0" 이 나왔을 때 **신호등이 없는 것인지 파이프라인이 죽은
#   것인지** 를 가르려면, 신호등 말고 뭐라도 잡히는지 보여야 한다.
#   사람이 화면에 들어왔을 때 person 박스가 뜨면 칩·전처리·후처리가
#   전부 정상이고 신호등만 시야에 없다는 뜻이다.
COCO_NAMES = (
    'person bicycle car motorcycle airplane bus train truck boat '
    'traffic_light fire_hydrant stop_sign parking_meter bench bird cat dog '
    'horse sheep cow elephant bear zebra giraffe backpack umbrella handbag '
    'tie suitcase frisbee skis snowboard sports_ball kite baseball_bat '
    'baseball_glove skateboard surfboard tennis_racket bottle wine_glass cup '
    'fork knife spoon bowl banana apple sandwich orange broccoli carrot '
    'hot_dog pizza donut cake chair couch potted_plant bed dining_table '
    'toilet tv laptop mouse remote keyboard cell_phone microwave oven '
    'toaster sink refrigerator book clock vase scissors teddy_bear '
    'hair_drier toothbrush').split()

# COCO 클래스 인덱스. 0=person, 1=bicycle, ... 9=traffic light
COCO_TRAFFIC_LIGHT = 9

# 도면에서 나온 램프 중심의 정규화 세로 위치 (신호부 상단=0, 하단=1)
# 세로형(신호부 24x56cm, 램프 R5, 간격 16/15/16) 도면 비율에서의 램프 중심
LAMP_POS = {'RED': 0.29, 'YELLOW': 0.50, 'GREEN': 0.71}
# ★ 2026-09-17 실차 — **가로형** 기준. 이게 없어서 실도로에서 계속 거부됐다.
#   한국 도로의 차량 신호등은 대부분 가로형이다(3구: 적/황/녹, 4구: 적/황/좌/녹).
#   가로형은 어느 램프가 켜지든 **세로** 무게중심이 항상 한가운데(~0.5)라,
#   세로 기준(LAMP_POS)으로 읽으면 무조건 YELLOW 가 된다. 그래서 색이 GREEN
#   이면 '불일치' 로 거부됐다.
#   실측(2026-09-17 실도로): "불일치: 위치=YELLOW 색=GREEN(H=86)" — 실제로는
#   가로형에 초록이 켜져 있었다. 일치 여부가 사실상 동전던지기라 검출이
#   0.1~0.8초만 유지되고 계속 NONE 으로 떨어졌다.
#   가로형에서는 **가로** 위치로 읽는다. 3구/4구 둘 다 덮으려고 경계를 넓게 잡는다:
#       3구  적 0.17 / 황 0.50 / 녹 0.83
#       4구  적 0.125 / 황 0.375 / 좌 0.625 / 녹 0.875
#   → 왼쪽 끝이 적, 오른쪽 끝이 녹, 가운데는 황(좌회전 화살표 포함).
LAMP_X_RED_MAX = 0.30      # 이보다 왼쪽이면 RED
LAMP_X_GREEN_MIN = 0.68    # 이보다 오른쪽이면 GREEN


class CocoTrafficLightNode(Node):

    def __init__(self):
        super().__init__('coco_traffic_light_detector')

        self.declare_parameter('confirm_frames', 5)
        self.declare_parameter('score_threshold', 0.3)
        self.declare_parameter('camera_device', 0)
        # 색 교차검증을 끄면 위치만으로 판별한다 (색이 안 맞을 때 진단용)
        self.declare_parameter('use_color_check', True)
        # 켜진 램프로 인정할 최소 밝기 (0~255)
        self.declare_parameter('min_lamp_value', 120)
        # ★ 디버그 화면 — 기본 꺼짐. 켜면 검출 박스를 겹친 JPEG 를
        #   /traffic_light_debug/compressed 로 쏜다. 카메라는 한 프로세스만
        #   점유하므로 **검출기 자신이 내보내는 수밖에 없다.**
        #   image_every 로 솎아 랜선 대역을 아낀다(3 → 10Hz).
        # ★ 2026-09-17 실차 — '우리 차선 신호등' 을 고르는 규칙.
        #   교차로에는 신호등이 4개 이상 잡힌다(우리 차선·반대편·보행자·측면).
        #   예전엔 **점수 최대** 하나만 봤는데, 점수는 '우리 것인지' 와 아무
        #   상관이 없다. 실측(90초 감시): 프레임마다 다른 걸 골라
        #     t+74.8 RED → 76.3 GREEN → 77.8 NONE → 78.8 YELLOW → 80.2 RED
        #   6초에 4번 뒤집혔다. 한 프레임 오탐이 곧 출발/급정거 명령이다.
        #
        #   그래서 기하로 고른다. 우리 차선 신호등의 성질:
        #     · 화면 **위쪽**에 있다 (육교/지주에 매달려 있다)
        #     · 화면 **가로 중앙**에 가깝다 (우리가 향하는 방향)
        #     · 충분히 **크다** (가까울수록 크고, 그래야 램프가 보인다)
        #   min_box_w 미만은 램프가 몇 픽셀이라 색 판별이 무의미하다.
        #   README 기준 3~8m 에서 신호부 폭 18~49px.
        self.declare_parameter('min_box_w', 18)
        self.declare_parameter('max_center_y', 0.60)   # 화면 아래쪽은 버린다
        self.declare_parameter('center_weight', 1.0)   # 가로 중앙 선호 강도
        self.declare_parameter('publish_image', False)
        self.declare_parameter('image_every', 3)
        self.declare_parameter('image_quality', 60)

        g = lambda n: self.get_parameter(n).value       # noqa: E731
        self.confirm_frames = int(g('confirm_frames'))
        self.score_threshold = float(g('score_threshold'))
        self.use_color_check = bool(g('use_color_check'))
        self.min_lamp_value = int(g('min_lamp_value'))
        camera_device = int(g('camera_device'))

        self.config_data = {"visualization_params": {
            "score_thres": self.score_threshold, "max_boxes_to_draw": 20}}

        self.publisher_ = self.create_publisher(
            String, '/traffic_light_state_coco', 10)
        self.min_box_w = int(g('min_box_w'))
        self.max_center_y = float(g('max_center_y'))
        self.center_weight = float(g('center_weight'))
        self.publish_image = bool(g('publish_image'))
        self.image_every = max(1, int(g('image_every')))
        self.image_quality = int(g('image_quality'))
        self.image_pub = self.create_publisher(
            CompressedImage, '/traffic_light_debug/compressed', 2) \
            if self.publish_image else None

        self.get_logger().info("HEF 로드 중...")
        self.hailo = HailoInfer(HEF_PATH, batch_size=1)
        self.height, self.width, _ = self.hailo.get_input_shape()

        self.cap = cv2.VideoCapture(camera_device, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        if not self.cap.isOpened():
            raise RuntimeError(f"카메라 /dev/video{camera_device} 열기 실패")

        self.result_holder = {}
        self.frame_count = 0
        self.prev_time = time.time()

        self.stable_state = "NONE"
        self.last_state = None
        self.cand_state = "NONE"
        self.cand_count = 0
        self.n_suppressed = 0
        self.n_boxes_seen = 0      # 신호등 박스를 몇 번이나 봤나 (진단)

        self.timer = self.create_timer(1.0 / 30.0, self.tick)
        self.get_logger().info(
            f"COCO 신호등 검출 시작 — 시간필터 {self.confirm_frames}프레임, "
            f"score>={self.score_threshold}, 색교차검증="
            f"{'ON' if self.use_color_check else 'OFF'} "
            f"→ /traffic_light_state_coco")

    def _cb(self, completion_info, bindings_list):
        if completion_info.exception:
            self.get_logger().error(f"추론 오류: {completion_info.exception}")
            return
        b = bindings_list[0]
        if len(b._output_names) == 1:
            self.result_holder['result'] = b.output().get_buffer()
        else:
            self.result_holder['result'] = {
                n: np.expand_dims(b.output(n).get_buffer(), axis=0)
                for n in b._output_names}

    def classify_lamp(self, frame, box):
        """신호등 박스 안에서 켜진 램프를 찾아 상태를 돌려준다.

        box = (x1, y1, x2, y2) 픽셀 좌표.
        반환: ("RED"/"YELLOW"/"GREEN"/None, 진단문자열)
        """
        x1, y1, x2, y2 = [int(v) for v in box]
        h, w = frame.shape[:2]
        x1, y1 = max(0, x1), max(0, y1)
        x2, y2 = min(w, x2), min(h, y2)
        if x2 - x1 < 4 or y2 - y1 < 8:
            return None, "박스 너무 작음"

        roi = frame[y1:y2, x1:x2]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        v = hsv[:, :, 2].astype(np.float32)
        s = hsv[:, :, 1].astype(np.float32)

        # 켜진 램프 = 밝고(V) 채도 있는(S) 영역. 흰 배경에 속지 않으려 S도 본다.
        lit = (v > self.min_lamp_value) & (s > 60)
        if lit.sum() < 4:
            return None, "켜진 램프 없음"

        rh, rw = roi.shape[:2]
        ys, xs = np.nonzero(lit)
        # ★ 신호등 방향을 박스 모양으로 판정한다 (위 LAMP_X_* 주석 참고).
        #   가로형이면 가로 위치, 세로형이면 세로 위치로 램프를 고른다.
        horizontal = rw > rh * 1.3
        if horizontal:
            cpos = float(xs.mean()) / max(1, rw - 1)
            if cpos <= LAMP_X_RED_MAX:
                by_pos = 'RED'
            elif cpos >= LAMP_X_GREEN_MIN:
                by_pos = 'GREEN'
            else:
                by_pos = 'YELLOW'
        else:
            cpos = float(ys.mean()) / max(1, rh - 1)
            by_pos = min(LAMP_POS, key=lambda k: abs(LAMP_POS[k] - cpos))
        cy = cpos      # 진단문에 그대로 쓴다

        if not self.use_color_check:
            return by_pos, f"{'가로' if horizontal else '세로'} {cy:.2f}→{by_pos}"

        # 색 교차검증: 켜진 화소들의 대표 색조(H, OpenCV는 0~179)
        hue = hsv[:, :, 0][lit]
        hm = float(np.median(hue))
        # 빨강은 0 근처와 179 근처로 갈리므로 양쪽을 본다.
        if hm <= 12 or hm >= 168:
            by_col = 'RED'
        elif 15 <= hm <= 40:
            by_col = 'YELLOW'
        elif 45 <= hm <= 105:          # 초록~청록 (한국 신호등은 청록 쪽)
            by_col = 'GREEN'
        else:
            by_col = None

        if by_col is None:
            # ★ 2026-09-17 실차 — 예전엔 여기서 by_pos 를 그대로 돌려줬다.
            #   그래서 **안 켜진 신호등**(측면·보행자)의 어두운 화소를 읽고도
            #   위치만 보고 'RED' 를 냈다. 실측: H=123(청보라, 램프색이 아님)
            #   인데 위치 0.23 → RED 출력.
            #   색을 모르면 **모른다고 해야 한다.** 교차검증의 존재 이유다.
            return None, f"색 불명(H={hm:.0f}) — 위치 {cy:.2f} 는 믿지 않는다"
        if by_col != by_pos:
            # 위치와 색이 어긋나면 신뢰하지 않는다 (간판·나무 등 오탐 방지)
            return None, (f"불일치: {'가로' if horizontal else '세로'}"
                          f"={by_pos} 색={by_col}(H={hm:.0f})")
        return by_pos, (f"일치 {by_pos} ({'가로' if horizontal else '세로'} "
                        f"{cy:.2f}, H={hm:.0f})")

    def _publish_debug(self, img, det, best_i, fps, why):
        """검출 박스를 겹친 JPEG 를 쏜다 (publish_image 일 때만).

        ★ **모든 클래스**를 그린다. 신호등만 그리면 '신호등박스=0' 일 때
          화면이 텅 비어서, 신호등이 없는 것인지 파이프라인이 죽은 것인지
          구분이 안 된다. 사람이라도 잡히면 앞단은 정상이라는 뜻이다.
        ★ 그리는 대상은 **레터박스된 입력 그대로**다. 모델이 실제로 보는
          그림이라, 회색 패딩과 비율까지 눈으로 확인된다.
        """
        if self.image_pub is None:
            return
        if self.frame_count % self.image_every:
            return
        vis = img.copy()
        n = det['num_detections']
        classes, scores, boxes = (det['detection_classes'],
                                  det['detection_scores'],
                                  det['detection_boxes'])
        shown = 0
        for i in range(n):
            if scores[i] < self.score_threshold:
                continue
            ci = int(classes[i])
            is_tl = (ci == COCO_TRAFFIC_LIGHT)
            # ★ 2026-09-17 — 좌표 순서 수정. Hailo 후처리는
            #   **[x_min, y_min, x_max, y_max]** 를 준다
            #   (object_detection_post_process.py:169 주석, 그리고 :104 의
            #    denormalize_and_rm_pad 가 [box[1],box[0],box[3],box[2]] 로
            #    xyxy 로 바꿔서 돌려준다).
            #   예전엔 y0,x0,y1,x1 로 받아 **x 와 y 가 뒤바뀐 자리**에 박스를
            #   그렸다. 실측(2026-09-17): 좌측 흰 트럭(x110~200,y370~415)의
            #   박스가 상단 하늘(x370~415,y110~200)에 찍혔다.
            #   ⚠ classify_lamp 는 처음부터 x1,y1,x2,y2 로 옳게 읽고 있었다.
            #     즉 **검출·분류는 정상이고 이 그림만 거짓말을 하고 있었다.**
            #     화면을 보고 카메라를 맞추면 엉뚱한 곳을 향하게 된다.
            x0, y0, x1, y1 = [int(v) for v in boxes[i]]
            # ★ 고른 신호등만 노란 굵은 테두리로 구분한다. 교차로에는 신호등이
            #   여럿이라, 어느 것을 보고 판단했는지 눈에 보여야 한다.
            chosen = (i == best_i)
            if chosen:
                color, th = (0, 255, 255), 3
            elif is_tl:
                color, th = (0, 0, 255), 2
            else:
                color, th = (140, 140, 140), 1
            cv2.rectangle(vis, (x0, y0), (x1, y1), color, th)
            name = COCO_NAMES[ci] if 0 <= ci < len(COCO_NAMES) else str(ci)
            tag = '<<' if chosen else ''
            cv2.putText(vis, f'{name} {scores[i]:.2f}{tag}',
                        (x0, max(12, y0 - 4)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)
            shown += 1
        head = (f'{self.stable_state}  fps={fps:.0f}  '
                f'boxes={shown}  TL={"yes" if best_i >= 0 else "no"}')
        cv2.rectangle(vis, (0, 0), (vis.shape[1], 40), (0, 0, 0), -1)
        cv2.putText(vis, head, (6, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    (255, 255, 255), 1)
        # ⚠ cv2.putText 는 한글을 못 그린다(전부 '?' 가 된다). why 에는 한글
        #   진단문이 들어오므로 ASCII 만 남긴다 — 자세한 것은 노드 로그에 있다.
        ascii_why = ''.join(c if 32 <= ord(c) < 127 else ' ' for c in why)
        cv2.putText(vis, ascii_why[:70], (6, 33), cv2.FONT_HERSHEY_SIMPLEX,
                    0.40, (180, 220, 180), 1)
        okj, buf = cv2.imencode(
            '.jpg', vis, [int(cv2.IMWRITE_JPEG_QUALITY), self.image_quality])
        if not okj:
            return
        m = CompressedImage()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'camera'
        m.format = 'jpeg'
        m.data = buf.tobytes()
        self.image_pub.publish(m)

    def tick(self):
        ok, frame = self.cap.read()
        if not ok:
            return

        # ★ cv2.resize 로 640x480 → 640x640 을 만들면 세로가 1.33배 늘어난다.
        # YOLO 는 왜곡 안 된 이미지로 학습돼 있어 검출률이 떨어진다.
        # default_preprocess 는 비율을 유지하며 회색으로 패딩한다(레터박스).
        # 기존 detector_node.py 도 이걸 쓴다 — 맞춰둔다.
        self.result_holder.clear()
        resized = default_preprocess(frame, self.width, self.height)
        self.hailo.run([resized], self._cb)

        waited = 0.0
        while 'result' not in self.result_holder and waited < 1.0:
            time.sleep(0.005)
            waited += 0.005
        if 'result' not in self.result_holder:
            return

        det = extract_detections(resized, self.result_holder['result'],
                                 self.config_data)
        classes = det['detection_classes']
        scores = det['detection_scores']
        boxes = det['detection_boxes']
        num = det['num_detections']

        self.frame_count += 1
        now = time.time()
        fps = 1.0 / (now - self.prev_time) if now > self.prev_time else 0.0
        self.prev_time = now

        # 신호등(class 9) 중 **우리 차선 것**을 고른다 (위 min_box_w 주석 참고).
        raw_state, why = "NONE", "검출 없음"
        best_i, best_score = -1, -1.0
        H, W = resized.shape[:2]
        n_small, n_low = 0, 0
        for i in range(num):
            if int(classes[i]) != COCO_TRAFFIC_LIGHT:
                continue
            if scores[i] < self.score_threshold:
                continue
            x0, y0, x1, y1 = [float(v) for v in boxes[i]]
            bw = x1 - x0
            if bw < self.min_box_w:
                n_small += 1          # 너무 멀다 — 램프가 몇 픽셀이라 못 읽는다
                continue
            cy = (y0 + y1) / 2.0 / max(1, H)
            if cy > self.max_center_y:
                n_low += 1            # 화면 아래쪽 = 노면 반사·차량 후미등 등
                continue
            cx = (x0 + x1) / 2.0 / max(1, W)
            # 점수 + 크기 + 중앙/위쪽 선호. 크기가 곧 '가깝다' 이므로 크게 본다.
            rank = (float(scores[i])
                    + bw / max(1, W) * 3.0
                    - abs(cx - 0.5) * self.center_weight
                    - cy * 0.5)
            if rank > best_score:
                best_score, best_i = rank, i
        best_s = float(scores[best_i]) if best_i >= 0 else 0.0
        if best_i < 0 and (n_small or n_low):
            why = f"후보 버림(작음 {n_small}, 아래 {n_low})"

        if best_i >= 0:
            self.n_boxes_seen += 1
            # extract_detections 는 리사이즈된 이미지 기준 좌표를 준다
            state, why = self.classify_lamp(resized, boxes[best_i])
            raw_state = state if state else "NONE"
            why = f"score={best_s:.2f} {why}"

        # ---- 시간 필터 (detector_node.py 와 동일 알고리즘) ----
        if raw_state == self.cand_state:
            self.cand_count += 1
        else:
            if self.cand_state != self.stable_state and self.cand_count > 0:
                self.n_suppressed += 1
            self.cand_state = raw_state
            self.cand_count = 1
        if (self.cand_count >= self.confirm_frames
                and self.cand_state != self.stable_state):
            self.stable_state = self.cand_state

        msg = String()
        msg.data = self.stable_state
        self.publisher_.publish(msg)

        if self.stable_state != self.last_state:
            self.get_logger().info(
                f"State changed: {self.last_state} -> {self.stable_state}")
        self.last_state = self.stable_state

        if self.frame_count % 30 == 0:
            self.get_logger().info(
                f"frame={self.frame_count} fps={fps:.1f} "
                f"state={self.stable_state} raw={raw_state} "
                f"신호등박스={self.n_boxes_seen} 걸러낸튐={self.n_suppressed} | {why}")

        self._publish_debug(resized, det, best_i, fps, why)

    def destroy_node(self):
        if hasattr(self, 'cap'):
            self.cap.release()
        if hasattr(self, 'hailo'):
            self.hailo.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CocoTrafficLightNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, RuntimeError) as e:
        if isinstance(e, RuntimeError):
            print(f"[ERROR] {e}")
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
