#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""hil_probe.py — 랩 한 번을 통째로 계측한다 (HIL·실차 공용).

★ 왜 도구로 남기나
  2026-09-09 세션에서 같은 측정을 임시 스크립트로 했다가 세션과 함께 잃었다.
  랩은 한 번 돌리는 데 7~28분이라 다시 짜서 다시 도는 비용이 크다.

측정하는 네 가지 (전부 '완주' 와 직결된다)
  1. 추종 품질  — /odometry/filtered 를 웨이포인트에 최근접 투영해 이탈량(cte)과
                  진행거리 s. **구간 스팟이 아니라 전 구간 프로파일**을 남긴다.
                  이탈은 감점이 아니라 탈락이다.
  2. 조향 포화  — /cmd_vel 의 angular.z(명령 조향각[도])가 상한에 붙어 있는 비율.
                  포화가 나면 그 코너는 기하학적으로 못 도는 것이다.
  3. 조향 실현  — 명령[도] vs /steering_angle(실제[도]) vs /steering_adc(원시).
                  **ADC 가 같이 움직여야 진짜 꺾이는 것**이다. 명령만 보면
                  하드웨어가 안 따라와도 완벽해 보인다(2026-09-09 교훈).
  4. 스톨       — /vehicle_stall 상승엣지마다 에피소드로 세고, 직전 3초를 덤프한다.

부수적으로 전원(/vcc_mv, /vcc_min_mv)과 /encoder_count 도 같이 남긴다.

사용:
  # 랩을 돌리기 직전에 켠다. 완주(/goal_reached)하면 자동으로 요약하고 끝난다.
  python3 tools/hil_probe.py --wp config/yongin_2026-09-05/wp_yongin_drive_0.5.yaml
  # Ctrl-C 로도 요약이 나온다.
  python3 tools/hil_probe.py --wp <파일> --tag gain3_v184
  # 저장된 CSV 재분석 (랩을 다시 안 돌려도 된다)
  python3 tools/hil_probe.py --replay logs/hil_probe_....csv
"""

import argparse
import csv
import math
import os
import sys
import time
from collections import deque

import numpy as np
import yaml

WS = '/home/han/racing_ws'


def load_waypoints(path):
  d = yaml.safe_load(open(path, encoding='utf-8'))
  raw = d.get('waypoints', d.get('poses'))
  wp = np.array([[float(p['x']), float(p['y'])] for p in raw])
  s = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(wp, axis=0).T))])
  return wp, s


def project(wp, seg_len, cum_s, x, y):
  """경로(꺾은선)에 수직 투영 → (이탈량, 진행거리 s)."""
  a, b = wp[:-1], wp[1:]
  ab = b - a
  ap = np.array([x, y]) - a
  denom = np.einsum('ij,ij->i', ab, ab)
  t = np.clip(np.einsum('ij,ij->i', ap, ab) / np.maximum(denom, 1e-12), 0.0, 1.0)
  proj = a + t[:, None] * ab
  d = np.hypot(proj[:, 0] - x, proj[:, 1] - y)
  i = int(np.argmin(d))
  return float(d[i]), float(cum_s[i] + t[i] * seg_len[i])


# ----------------------------------------------------------------- 요약/판정
def summarize(rows, max_steer_deg, path_len, tag=''):
  if not rows:
    print('샘플이 없다 — 스택이 안 돌았거나 토픽이 안 왔다.')
    return False
  A = {k: np.array([r[k] for r in rows], dtype=float)
       for k in ('t', 'cte', 's', 'cmd_steer', 'act_steer', 'adc', 'v_cmd',
                 'vcc', 'stall', 'enc')}
  dur = A['t'][-1] - A['t'][0]
  moved = A['s'].max() - A['s'].min()

  print('=' * 74)
  print(f'랩 계측 요약{"  [" + tag + "]" if tag else ""}')
  print('-' * 74)
  print(f'  시간            : {dur:.0f}s ({dur / 60:.2f}분)   샘플 {len(rows)}개')
  print(f'  진행            : s {A["s"].min():.1f} → {A["s"].max():.1f}m '
        f'(경로 {path_len:.1f}m, {moved / path_len * 100:.0f}%)')
  if dur > 0:
    print(f'  평균 진행속도   : {moved / dur:.2f} m/s')

  print('-' * 74)
  # 1) 추종 — 초기 수렴 5m 제외
  ss = A['cte'][A['s'] > A['s'].min() + 5.0]
  if len(ss) == 0:
    ss = A['cte']
  print(f'  이탈량(cte)     : 평균 {ss.mean():.3f}m  RMS '
        f'{math.sqrt((ss ** 2).mean()):.3f}m  최대 {ss.max():.3f}m')
  for thr in (0.3, 0.5, 1.0):
    n = int((ss > thr).sum())
    print(f'     > {thr:.1f}m 초과 : {n}샘플 ({n / len(ss) * 100:.1f}%)')
  # 최악 지점 위치
  k = int(np.argmax(A['cte'] * (A['s'] > A['s'].min() + 5.0)))
  print(f'     최악 지점    : s={A["s"][k]:.1f}m 에서 {A["cte"][k]:.3f}m')

  print('-' * 74)
  # 2) 조향 포화
  sat = np.abs(A['cmd_steer']) >= max_steer_deg - 0.1
  print(f'  조향 명령       : 최대 {np.abs(A["cmd_steer"]).max():.1f}° '
        f'(상한 {max_steer_deg:.1f}°)')
  print(f'  조향 포화       : {int(sat.sum())}/{len(sat)} '
        f'({sat.mean() * 100:.1f}%)')

  # 3) 조향 실현 — 명령 대비 실제
  ok_act = np.isfinite(A['act_steer'])
  if ok_act.any():
    err = A['act_steer'][ok_act] - A['cmd_steer'][ok_act]
    print(f'  조향 추종오차   : 평균 {err.mean():+.2f}°  RMS '
          f'{math.sqrt((err ** 2).mean()):.2f}°  최대 {np.abs(err).max():.2f}°')
    adc = A['adc'][ok_act]
    adc_moved = int((np.abs(np.diff(adc)) > 0).sum())
    print(f'  조향 ADC 변화   : {adc_moved}/{len(adc) - 1} 샘플에서 움직임 '
          f'(범위 {adc.min():.0f}~{adc.max():.0f})')
    if adc_moved < len(adc) * 0.05:
      print('     ⚠ ADC 가 거의 안 움직였다 — 명령만 가고 바퀴는 안 꺾인 것이다.')

  print('-' * 74)
  # 4) 스톨 에피소드
  st = A['stall'] > 0.5
  edges = np.where(np.diff(st.astype(int)) == 1)[0] + 1
  ends = np.where(np.diff(st.astype(int)) == -1)[0] + 1
  eps = []
  for e in edges:
    nxt = ends[ends > e]
    eps.append((A['t'][e], (A['t'][nxt[0]] if len(nxt) else A['t'][-1])
                - A['t'][e], A['s'][e]))
  tot = sum(d for _, d, _ in eps)
  print(f'  스톨 에피소드   : {len(eps)}회, 총 {tot:.1f}s '
        f'({tot / dur * 100:.1f}% of 랩)')
  if eps:
    ds = [d for _, d, _ in eps]
    print(f'     평균/최장    : {np.mean(ds):.1f}s / {max(ds):.1f}s')
    print('     발생 s       : '
          + ', '.join(f'{sp:.0f}m' for _, _, sp in eps[:12])
          + (' …' if len(eps) > 12 else ''))

  print('-' * 74)
  # 전원
  v = A['vcc'][A['vcc'] > 0]
  if len(v):
    print(f'  VCC             : 최저 {v.min():.0f}mV  '
          f'중앙 {np.median(v):.0f}mV')
    for thr in (3500, 3000):
      n = int((v < thr).sum())
      if n:
        print(f'     < {thr}mV     : {n}샘플')
  # 엔코더 — NO_ENCODER 판단 근거
  enc = A['enc']
  if np.isfinite(enc).any():
    e0, e1 = np.nanmin(enc), np.nanmax(enc)
    changed = int((np.abs(np.diff(enc)) > 0).sum())
    print(f'  엔코더 카운트   : {e0:.0f} ~ {e1:.0f} (변화 {e1 - e0:+.0f}, '
          f'{changed} 샘플에서 갱신)')
    if changed > len(enc) * 0.5:
      print('     → 엔코더가 살아 있다. 펌웨어 NO_ENCODER 1 을 재검토할 것')
      print('       (속도 폐루프 + 구동 스톨 보호가 꺼져 있다)')
    elif changed == 0:
      print('     → 갱신 없음. NO_ENCODER 1 이 맞다.')

  print('=' * 74)
  worst = float(ss.max())
  ok = worst < 0.5 and sat.mean() < 0.05
  print('  판정: ' + ('✅ 이탈 최대 50cm 미만 · 조향 포화 5% 미만'
                      if ok else '⚠ 아래 항목 확인'))
  if worst >= 0.5:
    print(f'    · 이탈 {worst:.2f}m — 연석까지 여유를 현장에서 확인할 것')
  if sat.mean() >= 0.05:
    print(f'    · 조향 포화 {sat.mean() * 100:.0f}% — 못 도는 코너가 있다')
  print('=' * 74)
  return ok


def run_live(args):
  import rclpy
  from geometry_msgs.msg import Twist
  from nav_msgs.msg import Odometry
  from rclpy.node import Node
  # /steering_angle 은 Float64 다 (Float32 로 구독하면 타입 불일치로
  # **조용히 아무것도 안 온다** — 2026-09-10 실제로 걸렸다).
  from std_msgs.msg import Bool, Float64, Int32

  wp, cum_s = load_waypoints(args.wp)
  seg = np.hypot(*np.diff(wp, axis=0).T)
  path_len = float(cum_s[-1])

  class Probe(Node):

    def __init__(self):
      super().__init__('hil_probe')
      self.rows = []
      self.ring = deque(maxlen=int(3.0 * args.rate))   # 스톨 직전 3초
      self.cur = dict(cte=0.0, s=0.0, cmd_steer=0.0, act_steer=float('nan'),
                      adc=float('nan'), v_cmd=0.0, vcc=0.0, stall=0.0,
                      enc=float('nan'))
      self.have_odom = False
      self.done = False
      # ★ /goal_reached 는 완주 후 계속 true 로 남는다(래치). 프로브를 나중에
      #   켜면 출발도 하기 전에 '완주' 로 보고 즉시 끝나 버린다(2026-09-10).
      #   실제로 min_progress 만큼 전진한 뒤의 완주 신호만 인정한다.
      self.s0 = None
      self.t0 = time.time()
      self.prev_stall = False
      self.create_subscription(Odometry, '/odometry/filtered', self.odom, 20)
      self.create_subscription(Twist, '/cmd_vel', self.cmd, 20)
      self.create_subscription(Float64, '/steering_angle', self.act, 20)
      self.create_subscription(Int32, '/steering_adc', self.adc, 20)
      self.create_subscription(Bool, '/vehicle_stall', self.stall, 20)
      self.create_subscription(Int32, '/vcc_mv', self.vcc, 20)
      self.create_subscription(Int32, '/encoder_count', self.enc, 20)
      self.create_subscription(Bool, '/goal_reached', self.goal, 10)
      self.create_timer(1.0 / args.rate, self.tick)
      print(f'계측 시작 — 경로 {len(wp)}점 {path_len:.1f}m. '
            'Ctrl-C 또는 완주 시 요약.')

    def odom(self, m):
      p = m.pose.pose.position
      self.cur['cte'], self.cur['s'] = project(wp, seg, cum_s, p.x, p.y)
      if self.s0 is None:
        self.s0 = self.cur['s']
      self.have_odom = True

    def cmd(self, m):
      self.cur['cmd_steer'] = float(m.angular.z)   # ★ rad/s 아님, 조향각[도]
      self.cur['v_cmd'] = float(m.linear.x)

    def act(self, m): self.cur['act_steer'] = float(m.data)

    def adc(self, m): self.cur['adc'] = float(m.data)

    def vcc(self, m): self.cur['vcc'] = float(m.data)

    def enc(self, m): self.cur['enc'] = float(m.data)

    def stall(self, m):
      on = bool(m.data)
      if on and not self.prev_stall:
        t = time.time() - self.t0
        print(f'\n  ⚡ 스톨 상승엣지 @ {t:.1f}s (s={self.cur["s"]:.1f}m) '
              '— 직전 3초:')
        print('     t[s]  명령[°] 실제[°]   ADC  이탈[m]  VCC[mV]')
        for r in list(self.ring)[::max(1, len(self.ring) // 12)]:
          print(f'    {r["t"] - t:+6.2f} {r["cmd_steer"]:7.1f} '
                f'{r["act_steer"]:7.1f} {r["adc"]:6.0f} {r["cte"]:7.3f} '
                f'{r["vcc"]:8.0f}')
      self.prev_stall = on
      self.cur['stall'] = 1.0 if on else 0.0

    def goal(self, m):
      if not m.data or self.done:
        return
      moved = (self.cur['s'] - self.s0) if self.s0 is not None else 0.0
      if moved < args.min_progress:
        return       # 아직 출발 전 — 지난 랩의 래치된 신호다
      self.done = True
      print(f'\n  🏁 완주 신호 수신 (진행 {moved:.1f}m)')

    def tick(self):
      if not self.have_odom:
        return
      r = dict(self.cur)
      r['t'] = time.time() - self.t0
      self.rows.append(r)
      self.ring.append(r)
      if len(self.rows) % (int(args.rate) * 20) == 0:
        print(f'  [{r["t"]:6.0f}s] s={r["s"]:6.1f}m cte={r["cte"]:.3f}m '
              f'조향 {r["cmd_steer"]:+6.1f}°→{r["act_steer"]:+6.1f}° '
              f'v={r["v_cmd"]:.2f} vcc={r["vcc"]:.0f}')

  # ★ Ctrl-C 처리.
  #   rclpy 는 자기 SIGINT 핸들러로 컨텍스트를 먼저 내려버린다. 그러면 진행 중이던
  #   spin_once 가 KeyboardInterrupt 가 아니라 RCLError('context is not valid')로
  #   터져서, 그대로 두면 **랩을 다 돌고도 요약과 CSV 를 못 남긴다**(2026-09-10).
  #   여기서 같이 받아 정상 종료 경로로 보낸다.
  from rclpy.executors import ExternalShutdownException
  from rclpy._rclpy_pybind11 import RCLError

  rclpy.init()
  n = Probe()
  try:
    while rclpy.ok() and not n.done:
      rclpy.spin_once(n, timeout_sec=0.05)
  except (KeyboardInterrupt, ExternalShutdownException, RCLError):
    print('\n중단 — 지금까지 모은 것으로 요약한다.')
  finally:
    rows = n.rows
    try:
      n.destroy_node()
    except Exception:  # noqa: BLE001  (컨텍스트가 이미 내려갔으면 무시)
      pass
    if rclpy.ok():
      rclpy.shutdown()

  os.makedirs(f'{WS}/logs', exist_ok=True)
  tag = f'_{args.tag}' if args.tag else ''
  out = f'{WS}/logs/hil_probe{tag}_{time.strftime("%Y%m%d_%H%M%S")}.csv'
  if rows:
    with open(out, 'w', newline='', encoding='utf-8') as f:
      w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
      w.writeheader()
      w.writerows(rows)
    print(f'\nCSV: {out}')
  ok = summarize(rows, args.max_steer_deg, path_len, args.tag)
  if rows:
    print(f'CSV: {out}')
  return 0 if ok else 1


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('--wp', default=f'{WS}/config/yongin_2026-09-05/'
                                  'wp_yongin_drive_0.5.yaml')
  ap.add_argument('--rate', type=float, default=20.0)
  ap.add_argument('--max-steer-deg', type=float, default=18.0)
  ap.add_argument('--tag', default='')
  ap.add_argument('--min-progress', type=float, default=10.0,
                  help='이만큼 전진하기 전의 완주 신호는 지난 랩의 래치로 본다')
  ap.add_argument('--replay', default=None, help='저장된 CSV 재분석')
  args = ap.parse_args()

  if args.replay:
    rows = []
    with open(args.replay, encoding='utf-8') as f:
      for d in csv.DictReader(f):
        rows.append({k: (float(v) if v not in ('', 'nan') else float('nan'))
                     for k, v in d.items()})
    _, cum_s = load_waypoints(args.wp)
    return summarize(rows, args.max_steer_deg, float(cum_s[-1]),
                     os.path.basename(args.replay))
  return run_live(args)


if __name__ == '__main__':
  sys.exit(main() or 0)
