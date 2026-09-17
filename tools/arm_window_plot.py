#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""arm_window_plot.py — 회피 구간이 **제때 켜지는가**를 한 장으로 본다.

★ 왜 만들었나 (2026-09-17)
  "첫 장애물에 자꾸 박는다" 의 원인이 s 숫자 표로는 안 보였다. 실제로는
  라이다가 진작 보고 회피각까지 정해 뒀는데 **구간 밖이라 조향이 막혀**
  있었고, arm 이 켜지는 순간엔 이미 BLOCKED 영역 안이었다.
  그 관계 — 감지 · arm · BLOCKED 한계 · 실제 장애물 위치 — 를 겹쳐 그린다.

  위: 지도. 경로(곡률 색) · 실제 궤적 · 라이다가 본 장애물 · arm 구간
  아래: 진행거리 s 에 따른 전방거리(obs). BLOCKED 영역과 arm 구간을 띠로.

사용:
  python3 tools/arm_window_plot.py /tmp/drive_0126.csv
  python3 tools/arm_window_plot.py /tmp/drive_0126.csv --arm-was 40.0
"""
import argparse
import csv
import math
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt          # noqa: E402
import numpy as np                       # noqa: E402
import yaml                              # noqa: E402

plt.rcParams['font.family'] = 'Noto Sans CJK JP'
plt.rcParams['axes.unicode_minus'] = False

DEF_WP = ('/home/han/racing_ws/config/chungju_school/'
          'wp_school_track_0.5.yaml')
# 정면 장애물이 이 거리 안에 들면 안전버블이 조준창을 덮어 BLOCKED 가 된다.
# (safety_margin 0.25 / planning_lookahead 1.5 에서 실측 — blocked_r 계산)
BLOCKED_R = 2.8


def col(rows, k):
    out = []
    for r in rows:
        v = r.get(k, '')
        out.append(float(v) if v not in ('', 'nan', 'None', None) else np.nan)
    return np.array(out)


def main():
    p = argparse.ArgumentParser()
    p.add_argument('csv')
    p.add_argument('--wp', default=DEF_WP)
    p.add_argument('--arm-was', type=float, default=None,
                   help='비교용 옛 arm 지점 s [m]')
    p.add_argument('--out', default=None)
    a = p.parse_args()

    rows = list(csv.DictReader(open(a.csv)))
    t, X, Y = col(rows, 't'), col(rows, 'x'), col(rows, 'y')
    obs = col(rows, 'obstacle_m')
    mode = [r.get('mode', '') for r in rows]
    armed = np.array(['조향OFF' not in m and m.split('|')[0] != '-'
                      for m in mode])

    d = yaml.safe_load(open(a.wp))
    P = np.array([[float(q['x']), float(q['y'])] for q in d['waypoints']])
    A, B = P[:-1], P[1:]
    AB = B - A
    L2 = (AB ** 2).sum(1)
    L2[L2 == 0] = 1e-9
    S = np.concatenate([[0.0], np.cumsum(np.hypot(AB[:, 0], AB[:, 1]))])

    def s_of(px, py):
        tt = (((px - A[:, 0]) * AB[:, 0]
               + (py - A[:, 1]) * AB[:, 1]) / L2).clip(0, 1)
        Q = A + tt[:, None] * AB
        i = int(np.argmin(np.hypot(Q[:, 0] - px, Q[:, 1] - py)))
        return S[i] + tt[i] * math.hypot(*AB[i])

    sv = np.array([s_of(X[i], Y[i]) if np.isfinite(X[i]) else np.nan
                   for i in range(len(rows))])

    ai = np.where(armed)[0]
    arm_s = sv[ai[0]] if len(ai) else np.nan
    dis_s = sv[ai[-1]] if len(ai) else np.nan

    # 감지 시작: obs 가 999 밑으로 내려온 첫 지점 (arm 근처만)
    seen = np.where(np.isfinite(obs) & (obs < 900))[0]
    if len(ai):
        seen = seen[(seen < ai[0] + 60) & (sv[seen] > arm_s - 8)]
    det_s = sv[seen[0]] if len(seen) else np.nan

    # 경로 곡률
    kap = np.zeros(len(P))
    for i in range(2, len(P) - 2):
        p0, p1, p2 = P[i - 2], P[i], P[i + 2]
        ar = abs((p1[0] - p0[0]) * (p2[1] - p0[1])
                 - (p2[0] - p0[0]) * (p1[1] - p0[1])) / 2
        s1 = np.hypot(*(p1 - p0)) * np.hypot(*(p2 - p1)) * np.hypot(*(p2 - p0))
        kap[i] = (4 * ar / s1) if s1 > 1e-9 else 0.0
    R = np.where(kap > 1e-6, 1.0 / np.maximum(kap, 1e-6), 999.0)

    # 라이다가 본 것 (지도좌표) — arm 구간 근처만
    scan = a.csv.replace('.csv', '_scan.csv')
    ox = oy = None
    if os.path.exists(scan) and os.path.getsize(scan) > 100:
        sc = list(csv.DictReader(open(scan)))
        if sc:
            st = np.array([float(r['t']) for r in sc])
            sx = np.array([float(r['x']) for r in sc])
            sy = np.array([float(r['y']) for r in sc])
            if len(ai):
                m = (st >= t[ai[0]] - 4) & (st <= t[ai[-1]])
                ox, oy = sx[m], sy[m]

    fig, (ax, bx) = plt.subplots(
        2, 1, figsize=(11, 11), gridspec_kw={'height_ratios': [2.2, 1]})

    # ── 위: 지도 ────────────────────────────────────────────────
    lo = np.argmin(np.abs(S - (min(arm_s, a.arm_was or arm_s) - 10)))
    hi = np.argmin(np.abs(S - (dis_s + 6)))
    seg = slice(max(lo, 0), min(hi, len(P)))
    sc_ = ax.scatter(P[seg, 0], P[seg, 1], c=np.clip(R[seg], 0, 40), s=26,
                     cmap='RdYlGn', vmin=5, vmax=40, zorder=3)
    plt.colorbar(sc_, ax=ax, label='경로 곡률반경 R [m]  (작을수록 급커브)',
                 fraction=0.035, pad=0.02)
    if ox is not None and len(ox):
        ax.scatter(ox, oy, s=1.2, c='0.55', alpha=.35, zorder=1,
                   label='라이다가 본 것')
    dm = np.isfinite(X) & (sv > S[seg.start]) & (sv < S[seg.stop - 1])
    ax.plot(X[dm], Y[dm], '-', c='tab:blue', lw=2.0, zorder=4, label='실제 궤적')
    am = dm & armed
    ax.plot(X[am], Y[am], '-', c='tab:red', lw=3.4, zorder=5,
            label='회피 조향 ON (arm)')

    def mark(s_val, color, label, dy=0.0):
        if not np.isfinite(s_val):
            return
        i = int(np.argmin(np.abs(S - s_val)))
        ax.plot(P[i, 0], P[i, 1], 'o', ms=13, mfc='none', mec=color, mew=2.8,
                zorder=7)
        ax.annotate(f'{label}\ns={s_val:.1f}m', (P[i, 0], P[i, 1]),
                    textcoords='offset points', xytext=(12, 10 + dy),
                    fontsize=10, color=color, fontweight='bold', zorder=8)

    if a.arm_was is not None:
        mark(a.arm_was, 'tab:orange', '옛 arm (늦음)', dy=-34)
    mark(arm_s, 'tab:red', 'arm ON')
    mark(det_s, 'tab:purple', '라이다 감지 시작', dy=26)
    ax.set_aspect('equal')
    ax.grid(alpha=.3)
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.legend(loc='best', fontsize=9, framealpha=.95)
    ax.set_title(f'회피 구간이 제때 켜지는가 — {os.path.basename(a.csv)}',
                 fontsize=13, fontweight='bold')

    # ── 아래: s vs 전방거리 ─────────────────────────────────────
    w = np.isfinite(sv) & np.isfinite(obs)
    w &= (sv > S[seg.start]) & (sv < S[seg.stop - 1])
    bx.axhspan(0, BLOCKED_R, color='tab:red', alpha=.10, zorder=0)
    bx.axhline(BLOCKED_R, color='tab:red', ls='--', lw=1.4,
               label=f'BLOCKED 한계 {BLOCKED_R}m — 이 밑은 갭이 사라진다')
    if np.isfinite(arm_s):
        bx.axvspan(arm_s, dis_s, color='tab:red', alpha=.10, zorder=0)
        bx.axvline(arm_s, color='tab:red', lw=2, label=f'arm ON (s={arm_s:.1f})')
    if a.arm_was is not None:
        bx.axvline(a.arm_was, color='tab:orange', lw=2, ls=':',
                   label=f'옛 arm (s={a.arm_was:.1f})')
    if np.isfinite(det_s):
        bx.axvline(det_s, color='tab:purple', lw=2, ls='-.',
                   label=f'라이다 감지 (s={det_s:.1f})')
    bx.plot(sv[w], np.clip(obs[w], 0, 9), '.', ms=3, c='tab:blue')
    bx.set_ylim(0, 9)
    bx.set_xlabel('경로 진행거리 s [m]')
    bx.set_ylabel('전방 장애물 거리 obs [m]')
    bx.grid(alpha=.3)
    bx.legend(loc='upper right', fontsize=9, framealpha=.95)

    out = a.out or a.csv.replace('.csv', '_armwindow.png')
    fig.tight_layout()
    fig.savefig(out, dpi=130)
    print(f'저장: {out}')
    print(f'  라이다 감지 s={det_s:.1f}  ·  arm ON s={arm_s:.1f}  '
          f'·  차이 {arm_s - det_s:+.1f}m')
    if np.isfinite(det_s) and arm_s > det_s:
        print(f'  ⚠ arm 이 감지보다 {arm_s - det_s:.1f}m 늦다 — '
              '그동안 회피각이 나와도 조향이 막혀 있다')
    return 0


if __name__ == '__main__':
    sys.exit(main())
