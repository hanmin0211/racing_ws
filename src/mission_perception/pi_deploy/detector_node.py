import sys
import os
import json
import time

sys.path.append(os.path.expanduser('~/Hailo-Application-Code-Examples/runtime/python'))
sys.path.append(os.path.expanduser('~/Hailo-Application-Code-Examples/runtime/python/object_detection'))

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from common.hailo_inference import HailoInfer
from common.toolbox import default_preprocess
from object_detection_post_process import extract_detections

HEF_PATH = os.path.expanduser('~/yolov8s.hef')
CLASSES_JSON = os.path.expanduser('~/classes.json')
SCORE_THRESHOLD = 0.4


class TrafficLightDetectorNode(Node):
    """신호등 상태를 /traffic_light_state 로 발행.

    ★ 시간 필터(confirm_frames)를 반드시 거쳐서 내보낸다.
      필터가 없던 초기 버전은 카메라가 신호등을 보고 있지 않아도 6.6초에 4번,
      1프레임짜리 GREEN 오탐이 그대로 발행됐다(2026-08-16 실측). 이 값이 차량
      제어로 들어가면 한 프레임 오탐이 곧 '출발' 명령이 되고, RED 쪽으로 튀면
      급정거가 된다. 같은 상태가 confirm_frames 번 연속으로 나와야 확정한다.
      30Hz 에서 5프레임 = 167ms, 차량 0.4m/s 기준 6.7cm 지연 — 무시할 만하다.

    발행 자체는 매 프레임(30Hz) 계속 한다. 바뀌는 건 '확정된' 상태뿐이다.
    """

    def __init__(self):
        super().__init__('traffic_light_detector')

        # 현장 튜닝 노브 — 전부 재빌드 없이 -p 로 바꿀 수 있어야 한다
        self.declare_parameter('confirm_frames', 5)
        self.declare_parameter('score_threshold', SCORE_THRESHOLD)
        self.declare_parameter('camera_device', 0)

        self.confirm_frames = int(self.get_parameter('confirm_frames').value)
        self.score_threshold = float(self.get_parameter('score_threshold').value)
        camera_device = int(self.get_parameter('camera_device').value)

        with open(CLASSES_JSON) as f:
            classes_dict = json.load(f)
        self.labels = [classes_dict[str(i)] for i in range(len(classes_dict))]
        self.get_logger().info(f"Loaded labels: {self.labels}")

        self.config_data = {
            "visualization_params": {
                "score_thres": self.score_threshold,
                "max_boxes_to_draw": 10
            }
        }

        self.publisher_ = self.create_publisher(String, '/traffic_light_state', 10)

        self.get_logger().info("Loading HEF and configuring Hailo device...")
        self.hailo_inference = HailoInfer(HEF_PATH, batch_size=1)
        self.height, self.width, _ = self.hailo_inference.get_input_shape()
        self.get_logger().info(f"Model input shape: {self.width}x{self.height}")

        self.cap = cv2.VideoCapture(camera_device)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        if not self.cap.isOpened():
            self.get_logger().error(
                f"Could not open camera /dev/video{camera_device}")
            raise RuntimeError("camera open failed")

        self.result_holder = {}
        self.frame_count = 0
        self.prev_time = time.time()

        # 시간 필터 상태
        self.stable_state = "NONE"   # 확정되어 실제로 발행되는 값
        self.last_state = None       # 확정값 변화 로깅용
        self.cand_state = "NONE"     # 연속 관측 중인 후보
        self.cand_count = 0
        self.n_suppressed = 0        # 필터가 걸러낸 튐 횟수 (진단용)

        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)
        self.get_logger().info(
            f"시간 필터: {self.confirm_frames}프레임 연속 일치 시 확정 "
            f"(~{self.confirm_frames / 30.0 * 1000:.0f}ms), "
            f"score_threshold={self.score_threshold}")

    def inference_callback(self, completion_info, bindings_list):
        if completion_info.exception:
            self.get_logger().error(f"Inference error: {completion_info.exception}")
        else:
            bindings = bindings_list[0]
            if len(bindings._output_names) == 1:
                self.result_holder['result'] = bindings.output().get_buffer()
            else:
                self.result_holder['result'] = {
                    name: np.expand_dims(bindings.output(name).get_buffer(), axis=0)
                    for name in bindings._output_names
                }

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        preprocessed = default_preprocess(frame, self.width, self.height)
        self.result_holder.clear()
        self.hailo_inference.run([preprocessed], self.inference_callback)

        waited = 0.0
        while 'result' not in self.result_holder and waited < 1.0:
            time.sleep(0.005)
            waited += 0.005

        if 'result' not in self.result_holder:
            return

        detections = extract_detections(frame, self.result_holder['result'], self.config_data)
        classes = detections['detection_classes']
        scores = detections['detection_scores']
        num = detections['num_detections']

        self.frame_count += 1
        now = time.time()
        fps = 1.0 / (now - self.prev_time) if now > self.prev_time else 0.0
        self.prev_time = now

        # ---- 이번 프레임의 원시 관측 ----
        # 새 모델(best_hailo_op11) 3클래스: 0=green, 1=orange(yellow), 2=red.
        # 라벨 문자열이 아니라 **클래스 인덱스**로 매핑한다(문자열이 바뀌어도 안전).
        # orange(황색)은 '정지'로 취급 — 자율주행에서 황색 통과는 위험. RED 로 접는다.
        # (황색을 별도 상태로 쓰려면 여기서 raw_state="YELLOW" 로 바꾸고 bridge 도 확장할 것)
        raw_state = "NONE"
        if num > 0:
            best_idx = int(np.argmax(scores))
            best_cls = int(classes[best_idx])
            best_score = scores[best_idx]

            if best_score >= self.score_threshold:
                if best_cls == 0:      # green traffic light
                    raw_state = "GREEN"
                elif best_cls == 2:    # red traffic light
                    raw_state = "RED"
                elif best_cls == 1:    # orange/yellow → 안전하게 정지
                    raw_state = "RED"

        # ---- 시간 필터: confirm_frames 연속 일치해야 확정 ----
        if raw_state == self.cand_state:
            self.cand_count += 1
        else:
            # 후보가 바뀌었다 = 직전 후보는 끝까지 못 버텼다.
            # 확정값과 다른 후보가 확정 못 되고 사라졌으면 그게 '걸러낸 튐'이다.
            if self.cand_state != self.stable_state and self.cand_count > 0:
                self.n_suppressed += 1
            self.cand_state = raw_state
            self.cand_count = 1

        if (self.cand_count >= self.confirm_frames
                and self.cand_state != self.stable_state):
            self.stable_state = self.cand_state

        # 확정된 상태만 내보낸다 (발행 주기는 그대로 30Hz)
        msg = String()
        msg.data = self.stable_state
        self.publisher_.publish(msg)

        if self.stable_state != self.last_state:
            self.get_logger().info(
                f"State changed: {self.last_state} -> {self.stable_state} "
                f"({self.confirm_frames}프레임 확정)")
        self.last_state = self.stable_state

        if self.frame_count % 30 == 0:
            # 걸러낸 튐 횟수를 같이 남긴다. 이 값이 계속 늘면 score_threshold를
            # 올리거나 카메라 노출/각도를 손봐야 한다는 신호다.
            self.get_logger().info(
                f"frame={self.frame_count} fps={fps:.1f} "
                f"state={self.stable_state} raw={raw_state} "
                f"걸러낸튐={self.n_suppressed}")

    def destroy_node(self):
        self.cap.release()
        self.hailo_inference.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TrafficLightDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
