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
from std_msgs.msg import String                         # noqa: E402

from common.hailo_inference import HailoInfer           # noqa: E402
from object_detection_post_process import extract_detections   # noqa: E402

HEF_PATH = os.path.expanduser('~/models/yolov8s_coco.hef')

# COCO 클래스 인덱스. 0=person, 1=bicycle, ... 9=traffic light
COCO_TRAFFIC_LIGHT = 9

# 도면에서 나온 램프 중심의 정규화 세로 위치 (신호부 상단=0, 하단=1)
LAMP_POS = {'RED': 0.29, 'YELLOW': 0.50, 'GREEN': 0.71}


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

        # 켜진 화소들의 세로 무게중심 → 정규화 위치
        ys = np.nonzero(lit)[0]
        cy = float(ys.mean()) / max(1, roi.shape[0] - 1)

        by_pos = min(LAMP_POS, key=lambda k: abs(LAMP_POS[k] - cy))

        if not self.use_color_check:
            return by_pos, f"위치 {cy:.2f}→{by_pos}"

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
            return by_pos, f"위치 {cy:.2f}→{by_pos}, 색 불명(H={hm:.0f})"
        if by_col != by_pos:
            # 위치와 색이 어긋나면 신뢰하지 않는다 (간판·나무 등 오탐 방지)
            return None, f"불일치: 위치={by_pos} 색={by_col}(H={hm:.0f})"
        return by_pos, f"일치 {by_pos} (위치 {cy:.2f}, H={hm:.0f})"

    def tick(self):
        ok, frame = self.cap.read()
        if not ok:
            return

        # 모델 입력(640x640)에 맞춰 리사이즈. 원본 좌표로 되돌리려 배율을 남긴다.
        self.result_holder.clear()
        resized = cv2.resize(frame, (self.width, self.height))
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

        # 신호등(class 9) 중 가장 점수 높은 것 하나만 본다
        raw_state, why = "NONE", "검출 없음"
        best_i, best_s = -1, 0.0
        for i in range(num):
            if int(classes[i]) == COCO_TRAFFIC_LIGHT and scores[i] >= self.score_threshold:
                if scores[i] > best_s:
                    best_s, best_i = float(scores[i]), i

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
