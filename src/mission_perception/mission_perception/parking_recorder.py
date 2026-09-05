#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
parking_recorder.py — 후진주차 궤적 기록 (전진·후진 구간 함께)

대회장에서 teleop 으로 "주차 접근 → 후진 진입 → 정차" 를 수동 주행하며 궤적을 찍는다.
각 점에 (x, y, gear) 를 저장하며, gear 는 +1(전진) / -1(후진) 이다.

기어 판정 규칙 (자동):
  - 첫 20샘플 동안 speed 방향으로 초기 gear 결정
  - 이후에는 **최근 5샘플 평균 speed 부호**로 기어를 갱신 (0.1m/s 데드밴드)
  - speed 를 못 얻으면 (prev_x,y) → 현재점의 이동방향과 yaw 를 비교해 부호 판정

사용:
  ros2 run mission_perception parking_recorder --ros-args -p slot:=1
  → 결과: ~/parking_1.yaml

★ 조건: GPS + 측위 노드가 이미 떠 있어야 한다.
   (bringup.launch.py 를 auto_calib:=false 로 띄우고 teleop 으로 몰면서 실행)

★ 원점: config/site_origin.yaml 을 다른 노드와 공유. 이 노드는 로컬좌표(map)만 쓴다.

★ 안테나 위치 기준: stop_point_recorder 와 동일. 정차 자세로 마무리하고 Ctrl-C.
"""

import math
import os
import signal
import sys

import rclpy
import yaml
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Float64


def yaw_from_quat(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class ParkingRecorder(Node):

    def __init__(self):
        super().__init__('parking_recorder')
        self.declare_parameter('slot', 1)
        self.declare_parameter('output', '')
        self.declare_parameter('point_spacing', 0.2)
        self.declare_parameter('odom_topic', '/odometry/filtered')

        slot = int(self.get_parameter('slot').value)
        out = str(self.get_parameter('output').value) or \
              os.path.expanduser(f'~/parking_{slot}.yaml')
        self.output = out
        self.spacing = float(self.get_parameter('point_spacing').value)

        self.pts = []          # [(x, y, yaw, speed, gear)]
        self.last_x = None
        self.last_y = None
        self.recent_v = []
        self.gear = 0          # 0 = 미결정

        # ★ 헤딩 캘리브 전에는 기록을 거부한다.
        #   기어 판정이 '이동방향 vs yaw' 인데, 캘리브 전 yaw 는 IMU 원시값이라
        #   방향 의미가 없다. 그 상태로 찍으면 전진/후진이 통째로 틀리게 들어가고
        #   parking_node 가 재생할 때 ABORT 한다. 현장에서 한 바퀴 몰고 나서
        #   알게 되면 늦으므로 아예 시작을 막는다.
        #   IMU yaw 는 세션마다 리셋되므로 **매 세션** 캘리브가 필요하다.
        self.declare_parameter('require_heading_calib', True)
        self.require_calib = bool(
            self.get_parameter('require_heading_calib').value)
        self.heading_ready = not self.require_calib
        latched = QoSProfile(depth=1,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Float64, '/heading/yaw_offset',
                                 self.heading_cb, latched)

        self.create_subscription(
            Odometry, str(self.get_parameter('odom_topic').value),
            self.odom_cb, 10)

        self.get_logger().info(
            f'주차 궤적 기록 시작: slot={slot} → {out}   '
            f'(간격 {self.spacing}m, Ctrl-C 로 저장)')
        if self.heading_ready:
            self.get_logger().info(
                '이제 teleop 으로 접근→후진→정차 를 수동 주행하세요.')
        else:
            self.get_logger().error(
                '⛔ 헤딩 캘리브 대기 — 아직 기록하지 않는다. '
                '10m 직진 캘리브를 먼저 끝낼 것(/heading/yaw_offset). '
                '캘리브 전 yaw 로는 전진/후진 판정이 틀린다.')
            self.create_timer(3.0, self._nag)

    def _nag(self):
        if not self.heading_ready:
            self.get_logger().error(
                '⛔ 여전히 헤딩 캘리브 전 — 몰아도 기록되지 않는다.')

    def heading_cb(self, msg):
        if not self.heading_ready:
            self.heading_ready = True
            self.get_logger().info(
                f'✅ 헤딩 캘리브 확인 ({math.degrees(float(msg.data)):.1f}°) — '
                f'이제 teleop 으로 접근→후진→정차 를 수동 주행하세요.')

    def _classify_gear(self, speed, dx, dy, yaw):
        """이동방향 vs yaw 로 판정한다. speed 는 보조.

        ★ 왜 속도평균을 안 쓰나 (2026-08-24 수정)
          예전엔 '최근 5샘플 평균 speed 부호'가 1순위였다. 그런데 그 평균은
          기어를 바꾼 뒤에도 몇 점 동안 이전 부호를 유지한다. 그 사이 차는 이미
          후진하는데 점은 '전진'으로 찍히고, 그러면 전진 구간의 끝부분이
          **공간상 뒤로 가는 점들**이 된다. parking_node 는 그걸 따라가려다
          '전진인데 목표점이 뒤' 로 중단한다(실제로 왕복 테스트에서 재현됨).

          이동방향과 yaw 의 각도차는 **지연이 없다**. 점은 0.2m 움직여야 찍히므로
          변위 방향도 충분히 안정적이다. 그래서 이쪽을 1순위로 둔다.
        """
        if dx * dx + dy * dy > 1e-4:
            move_dir = math.atan2(dy, dx)
            d = math.atan2(math.sin(move_dir - yaw), math.cos(move_dir - yaw))
            return -1 if abs(d) > math.pi / 2 else 1
        # 변위가 거의 없을 때만 속도 부호로 (제자리 조향 등)
        if speed is not None and abs(speed) > 0.05:
            return 1 if speed > 0 else -1
        return self.gear if self.gear != 0 else 1

    def odom_cb(self, msg: Odometry):
        if not self.heading_ready:
            return                      # 캘리브 전 데이터는 버린다
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = yaw_from_quat(msg.pose.pose.orientation)
        v = msg.twist.twist.linear.x

        if self.last_x is None:
            self.pts.append((x, y, yaw, v, 1))
            self.last_x, self.last_y = x, y
            self.gear = 1
            self.get_logger().info(f'시작점 기록: ({x:.2f},{y:.2f})')
            return

        dx, dy = x - self.last_x, y - self.last_y
        d = math.hypot(dx, dy)
        if d < self.spacing:
            return

        g = self._classify_gear(v, dx, dy, yaw)
        if g != self.gear:
            self.get_logger().info(
                f'★ 기어 전환: {self.gear:+d} → {g:+d}   '
                f'@ ({x:.2f},{y:.2f}) v={v:+.2f}m/s  [{len(self.pts)}점]')
            self.gear = g

        self.pts.append((x, y, yaw, v, g))
        self.last_x, self.last_y = x, y

        # ★ 점이 늘 때마다 저장한다. 종료 시 저장(Ctrl-C)은 종료 방식에 따라
        #   안 탄다 — ros2 run 래퍼가 SIGINT 를 어떻게 넘기느냐에 좌우된다.
        #   실제로 그래서 마지막 구간이 잘렸다. 현장에서 한 번 몰고 온 궤적을
        #   저장 실패로 날리면 다시 몰아야 하므로 매번 디스크에 떨군다.
        #   (점 수백 개짜리 YAML 이고 최대 몇 Hz라 비용은 무시할 수준)
        self.save(quiet=True)

        if len(self.pts) % 10 == 0:
            self.get_logger().info(
                f'  {len(self.pts)}점  현재 ({x:.2f},{y:.2f}) '
                f'v={v:+.2f} gear={g:+d}')

    def save(self, quiet=False):
        if len(self.pts) < 2:
            if not quiet:
                print(f'기록된 점 {len(self.pts)}개 — 저장 안 함.')
            return
        # gear 별 요약
        fwd = sum(1 for _, _, _, _, g in self.pts if g > 0)
        rev = sum(1 for _, _, _, _, g in self.pts if g < 0)
        total_len = sum(math.hypot(self.pts[i+1][0] - self.pts[i][0],
                                   self.pts[i+1][1] - self.pts[i][1])
                        for i in range(len(self.pts) - 1))

        # ★ 원점을 같이 저장한다. 장소가 바뀌면 좌표가 통째로 어긋나는데
        #   (2026-08-17 에 150km 어긋난 전례) 기록에 원점이 없으면 검증할 방법이 없다.
        from waypoint_follower.site_origin import load_site_origin
        _epsg, ox, oy, site = load_site_origin(self.get_logger())

        data = {
            'origin': {'x': ox, 'y': oy, 'epsg': _epsg, 'site': site},
            'points': [
                {'x': float(x), 'y': float(y), 'yaw': float(yaw),
                 'speed': float(sp), 'gear': int(g)}
                for (x, y, yaw, sp, g) in self.pts
            ],
            'stats': {
                'count': len(self.pts),
                'forward': fwd, 'reverse': rev,
                'length_m': round(total_len, 2),
            },
        }
        with open(self.output, 'w', encoding='utf-8') as f:
            yaml.safe_dump(data, f, allow_unicode=True, sort_keys=False)
        if not quiet:
            # ★ print 를 쓴다. 종료 중에는 rosout 발행이 실패해(context invalid)
            #   get_logger() 로 찍은 저장 완료 메시지가 안 보인다.
            print(f'✅ 저장 완료: {len(self.pts)}점 (전진 {fwd} / 후진 {rev}, '
                  f'총 {total_len:.2f}m) → {self.output}', flush=True)


def main():
    rclpy.init()
    node = ParkingRecorder()
    # ★ SIGTERM 으로도 저장되게 한다. ros2 run 래퍼나 런치 종료는 SIGINT 가
    #   아니라 SIGTERM 을 보낼 수 있고, 그때 finally 가 안 타면 궤적이 날아간다.
    def _term(_sig, _frm):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, _term)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except rclpy.executors.ExternalShutdownException:
        # SIGINT 를 rclpy 가 먼저 잡으면 이 예외로 온다. 조용히 저장만 한다.
        pass
    finally:
        try:
            node.save()
        except Exception as e:  # noqa: BLE001
            print(f'⚠ 저장 실패: {e}', flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
