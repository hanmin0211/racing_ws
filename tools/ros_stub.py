#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ros_stub.py — rclpy 가 없는 PC 에서 노드 모듈을 **import 만** 되게 한다.

  노드 클래스의 메서드를 Stub self 로 불러 시험할 때 쓴다(test_ramp_section.py 방식).
  로봇처럼 진짜 rclpy·메시지가 있으면 **아무것도 바꾸지 않는다** — 그대로 진짜를 쓴다.

  import ros_stub; ros_stub.install()
"""

import sys
import types


class _Msg:
  """std_msgs 류 — data 필드 하나."""

  def __init__(self, data=None):
    self.data = data


class _Vec:

  def __init__(self):
    self.x = 0.0
    self.y = 0.0
    self.z = 0.0


class _Twist:

  def __init__(self):
    self.linear = _Vec()
    self.angular = _Vec()


class _Any:

  def __init__(self, *a, **kw):
    pass


def _mod(name, **attrs):
  m = types.ModuleType(name)
  for k, v in attrs.items():
    setattr(m, k, v)
  sys.modules[name] = m
  return m


def install():
  """없는 모듈만 가짜로 채운다. 진짜가 하나라도 있으면 그 패키지는 건드리지 않는다."""
  try:
    import rclpy  # noqa: F401
  except ImportError:
    rclpy = _mod('rclpy', init=lambda *a, **k: None, ok=lambda: False,
                 shutdown=lambda: None, spin=lambda n: None)
    rclpy.node = _mod('rclpy.node', Node=type('Node', (), {}))
    rclpy.executors = _mod('rclpy.executors',
                           ExternalShutdownException=type(
                               'ExternalShutdownException', (Exception,), {}))
    rclpy.qos = _mod('rclpy.qos', QoSProfile=_Any, DurabilityPolicy=_Any)
  try:
    import std_msgs.msg  # noqa: F401
  except ImportError:
    _mod('std_msgs')
    _mod('std_msgs.msg', Bool=_Msg, Float64=_Msg, String=_Msg, Int32=_Msg)
  try:
    import geometry_msgs.msg  # noqa: F401
  except ImportError:
    _mod('geometry_msgs')
    _mod('geometry_msgs.msg', Twist=_Twist, PoseStamped=_Any)
  try:
    import nav_msgs.msg  # noqa: F401
  except ImportError:
    _mod('nav_msgs')
    _mod('nav_msgs.msg', Odometry=_Any, Path=_Any)
  try:
    import yaml  # noqa: F401
  except ImportError:
    _mod('yaml', safe_load=lambda f: None)
