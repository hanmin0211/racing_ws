#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""traffic_light_bridge 검증 — 판단 로직과 전방거리 계산."""
import math
import os
import sys

import rclpy

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from mission_perception.traffic_light_bridge import (   # noqa: E402
    NO_CONSTRAINT, TrafficLightBridge, yaw_from_quat)

ok = True


def check(label, cond):
    global ok
    print(f'{"PASS" if cond else "FAIL"}  {label}')
    ok = ok and cond


class Q:
    def __init__(self, yaw):
        self.w, self.x, self.y = math.cos(yaw / 2), 0.0, 0.0
        self.z = math.sin(yaw / 2)


rclpy.init(args=['prog', '--ros-args',
                 '-p', 'stop_points:=[10.0,0.0, 30.0,0.0]',
                 '-p', 'approach_range:=25.0',
                 '-p', 'yellow_pass_dist:=2.0',
                 '-p', 'none_timeout:=10.0'])
n = TrafficLightBridge()

# ---------- 판단 로직 ----------
d = n.decide('GREEN', 8.0, False, 0)[0]
check(f'GREEN → 제약없음 ({d})', d == NO_CONSTRAINT)

d = n.decide('RED', 8.0, False, 0)[0]
check(f'RED → 거리 그대로 발행 ({d})', d == 8.0)

d = n.decide('YELLOW', 8.0, False, 0)[0]
check(f'YELLOW 멀면 정지 ({d})', d == 8.0)

d = n.decide('YELLOW', 1.5, False, 0)[0]
check(f'YELLOW 코앞이면 통과 ({d})', d == NO_CONSTRAINT)

d = n.decide('RED', None, False, 0)[0]
check(f'전방 정지지점 없으면 RED여도 제약없음 ({d})', d == NO_CONSTRAINT)

# 미검출 → 기본은 정지(페일세이프)
d, why = n.decide('NONE', 8.0, False, 0)
check(f'미검출 → 정지 ({d})', d == 8.0)

# 미검출로 오래 붙잡히면 통과 (완주 못 하는 것 방지)
d, why = n.decide('NONE', 8.0, False, 11.0)
check(f'미검출 10s 초과 → 통과  ({why})', d == NO_CONSTRAINT)

# 파이 링크 끊김 = 미검출과 동일 취급 (GREEN 이 남아있어도 믿지 않는다)
d, why = n.decide('GREEN', 8.0, True, 0)
check(f'링크끊김이면 직전 GREEN 무시하고 정지  ({why})', d == 8.0)

d, why = n.decide('GREEN', 8.0, True, 11.0)
check(f'링크끊김 10s 초과 → 통과  ({why})', d == NO_CONSTRAINT)

# ---------- 전방 거리 계산 ----------
n.pose = (0.0, 0.0, 0.0)                    # 원점, 동쪽 향함
check(f'전방 10m 지점 선택 ({n.forward_distance()[0]})',
      abs(n.forward_distance()[0] - 10.0) < 1e-6)

n.pose = (12.0, 0.0, 0.0)                   # 첫 지점을 지나침
check(f'지나친 지점 무시, 다음 지점 18m ({n.forward_distance()[0]})',
      abs(n.forward_distance()[0] - 18.0) < 1e-6)

n.pose = (0.0, 0.0, math.pi)                # 반대 방향
check('뒤돌아보면 전방 지점 없음', n.forward_distance()[0] is None)

n.pose = (-30.0, 0.0, 0.0)                  # 40m 밖 (approach_range 25)
check('접근범위 밖이면 무시', n.forward_distance()[0] is None)

n.pose = (0.0, 5.0, 0.0)                    # 옆으로 5m 벗어남
fd = n.forward_distance()[0]
check(f'횡방향 오차 있어도 진행방향 성분으로 계산 ({fd:.1f})',
      abs(fd - 10.0) < 1e-6)

# ---------- 쿼터니언 ----------

# ---------- 오버슈트: 지나쳐도 놓지 않아야 한다 ----------
n.pose = (10.5, 0.0, 0.0)                   # 정지점을 0.5m 지나침
fd = n.forward_distance()[0]
check(f'0.5m 지나쳐도 계속 붙잡음 ({fd})', fd is not None and fd < 0)

n.pose = (12.0, 0.0, 0.0)                   # 여유(1.5m)를 넘어감
fd, fi = n.forward_distance()
check(f'여유 넘으면 다음 지점으로 ({fd})', abs(fd - 18.0) < 1e-6 and fi == 1)

n.cleared.add(0)                            # GREEN 통과 확정 처리
n.pose = (5.0, 0.0, 0.0)
fd, fi = n.forward_distance()
check(f'통과 확정된 지점은 되돌아가도 무시 (다음={fd})', fi == 1)

check('yaw_from_quat 90도', abs(yaw_from_quat(Q(math.pi / 2)) - math.pi / 2) < 1e-9)

n.destroy_node()
rclpy.shutdown()
print('\n=== 전체 통과 ===' if ok else '\n=== 실패 있음 ===')
sys.exit(0 if ok else 1)
