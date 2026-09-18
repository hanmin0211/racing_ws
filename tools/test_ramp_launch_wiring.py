#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""test_ramp_launch_wiring.py — 경사로 파라미터가 **런치를 타고 노드까지 가는지** 본다.

★ 왜 이 시험이 생겼나 (2026-09-18 에 실제로 걸린 것)

  거버너(gov_*)와 IMU 경사 보상(grade_ff_*)은 9/17 에 만들어 놓고
  tools/teleop_drive.launch.py 에만 인자를 냈다. 구간 속도
  (ramp_up_arm_topic·v_ramp_up)는 control.launch.py 에 선언했다.

  그런데 **bringup.launch.py 가 그 어느 것도 control 로 안 넘겼다.**
  결과: 웨이포인트 자율 주행으로는 경사로 기능을 **하나도 켤 수 없었다.**

  이 고장은 조용하다. 런치는 정상이고, 인자를 줘도 에러가 안 나고
  (안 받는 인자는 그냥 무시된다), 노드는 잘 뜬다. 차만 경사로에서
  v_max 로 달린다. 현장에서 "구간 속도가 고장났나" 로 반나절이 간다.

  선언(DeclareLaunchArgument)과 전달(launch_arguments)과 적용(parameters)은
  **서로 다른 세 곳**이고, 셋 중 하나만 빠져도 같은 증상이 나온다.
  그래서 셋을 전부 본다.

★ 이 시험은 하드웨어도 ROS 그래프도 안 쓴다. 런치 파일을 파이썬으로 읽어
  LaunchDescription 을 만들고, 그 안의 액션을 직접 훑는다.

  python3 tools/test_ramp_launch_wiring.py
"""

import importlib.util
import os
import sys

from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CONTROL = os.path.join(WS, 'src', 'pure_pursuit_pkg', 'launch',
                       'control.launch.py')
BRINGUP = os.path.join(WS, 'src', 'gps_localization', 'launch',
                       'bringup.launch.py')

# serial_bridge 로 가야 하는 것 / longitudinal_controller 로 가야 하는 것.
# 값은 '꺼짐' 기본값 — 인자를 안 주면 예전 동작 그대로여야 한다(원칙 2).
BRIDGE_ARGS = {
    'gov_pwm': '0.0',          # 0 = 거버너 꺼짐
    'gov_deadband': '0.10',
    'gov_gain': '300.0',
    'gov_lead_s': '0.30',
    'grade_ff_gain': '0.0',    # 0 = 경사 보상 꺼짐
    'grade_ff_max': '70.0',
    'imu_topic': 'handsfree/imu',
}
LON_ARGS = {
    'ramp_up_arm_topic': '',   # '' = 구독조차 안 한다
    'ramp_down_arm_topic': '',
    'v_ramp_up': '0.0',
    'v_ramp_down': '0.0',
}
# bringup 이 control 로 넘겨야 하는 것 — imu_topic 은 bringup 에 IMU 인자가
# 따로 있어(imu_port) 이름이 겹치므로 제외한다. 기본값이 맞으므로 문제없다.
FORWARD = sorted(set(BRIDGE_ARGS) - {'imu_topic'} | set(LON_ARGS))

fails = []


def chk(name, ok, detail=''):
  print(f'  {"OK " if ok else "✗  "} {name}' + (f'   {detail}' if detail else ''))
  if not ok:
    fails.append(name)


def load(path, name):
  """런치 파일을 모듈로 읽어 LaunchDescription 을 만든다."""
  spec = importlib.util.spec_from_file_location(name, path)
  mod = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(mod)
  return mod.generate_launch_description()


def _text(obj):
  """Substitution(또는 그 튜플/리스트)을 문자열로. 못 풀면 None."""
  if isinstance(obj, str):
    return obj
  if isinstance(obj, (list, tuple)):
    out = ''
    for s in obj:
      t = _text(s)
      if t is None:
        return None
      out += t
    return out
  return getattr(obj, 'text', None)


def declared(ld):
  """{인자이름: 기본값문자열}"""
  out = {}
  for e in ld.entities:
    if isinstance(e, DeclareLaunchArgument):
      out[e.name] = _text(e.default_value)
  return out


def node_params(ld, executable):
  """그 실행파일 Node 의 parameters 키 집합."""
  for e in ld.entities:
    if isinstance(e, Node) and getattr(e, '_Node__node_executable',
                                       None) == executable:
      keys = set()
      for blk in (getattr(e, '_Node__parameters', None) or []):
        if isinstance(blk, dict):
          for k in blk:
            t = _text(k)
            if t:
              keys.add(t)
      return keys
  return None


def include_args(ld, needle):
  """포함된 런치(경로에 needle 이 든 것)로 넘기는 인자 이름 집합."""
  for e in ld.entities:
    if not isinstance(e, IncludeLaunchDescription):
      continue
    # ⚠ src.location 은 이 버전에서 **Substitution 객체의 repr 문자열**을
    #   돌려준다 ('<...TextSubstitution object at 0x...>'). 그대로 비교하면
    #   경로가 절대 안 맞아 '포함 안 함' 으로 오판한다. 원본을 직접 푼다.
    src = getattr(e, 'launch_description_source', None)
    loc = _text(getattr(src, '_LaunchDescriptionSource__location', None))
    if loc is None:
      loc = _text(getattr(src, 'location', None)) or ''
    if needle not in str(loc):
      continue
    out = set()
    for k, _v in (getattr(e, 'launch_arguments', None) or []):
      t = _text(k)
      if t:
        out.add(t)
    return out
  return None


# ─────────────────────────────────────────────────────────────────────
print('① control.launch.py — 선언하고, 노드까지 적용하는가')
ctl = load(CONTROL, 'ctl_launch')
ctl_dec = declared(ctl)

for name, default in {**BRIDGE_ARGS, **LON_ARGS}.items():
  chk(f'{name} 선언됨', name in ctl_dec)
  if name in ctl_dec:
    # ★ 기본값이 '꺼짐' 이어야 한다. 이게 어긋나면 인자를 안 줘도
    #   동작이 바뀐다 — 저장소 원칙 2번 위반이고, 되돌릴 수 없다.
    chk(f'{name} 기본값 = {default!r} (안 주면 예전 동작)',
        ctl_dec[name] == default, f'실제 {ctl_dec[name]!r}')

br_keys = node_params(ctl, 'serial_bridge')
chk('serial_bridge Node 를 찾았다', br_keys is not None)
if br_keys is not None:
  for name in BRIDGE_ARGS:
    chk(f'serial_bridge 에 {name} 전달', name in br_keys)

lon_keys = node_params(ctl, 'longitudinal_controller')
chk('longitudinal_controller Node 를 찾았다', lon_keys is not None)
if lon_keys is not None:
  for name in LON_ARGS:
    chk(f'longitudinal_controller 에 {name} 전달', name in lon_keys)

# ─────────────────────────────────────────────────────────────────────
print()
print('② bringup.launch.py — 선언하고, control 로 **넘기는가**')
print('   (여기가 9/18 에 비어 있었다. 선언만 하고 안 넘기면 증상이 같다)')
brg = load(BRINGUP, 'brg_launch')
brg_dec = declared(brg)
fwd = include_args(brg, 'control.launch.py')

chk('control.launch.py 를 포함한다', fwd is not None)
for name in FORWARD:
  chk(f'{name} bringup 에 선언됨', name in brg_dec)
  if fwd is not None:
    chk(f'{name} control 로 전달됨', name in fwd)
  if name in brg_dec and name in ctl_dec:
    # 두 런치의 기본값이 다르면, 어느 쪽으로 띄웠느냐에 따라 차가 달라진다.
    chk(f'{name} 기본값이 control 과 같다',
        brg_dec[name] == ctl_dec[name],
        f'bringup {brg_dec[name]!r} vs control {ctl_dec[name]!r}')

# ─────────────────────────────────────────────────────────────────────
print()
if fails:
  print(f'❌ 실패 {len(fails)}건:')
  for f in fails:
    print(f'   · {f}')
  print('\n   → 이 상태로 현장에 가면 인자를 줘도 **조용히 무시된다.**')
  sys.exit(1)
print('✅ 전부 통과 — 경사로 파라미터가 런치를 타고 노드까지 간다')
sys.exit(0)
