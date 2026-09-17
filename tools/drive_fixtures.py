#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""drive_fixtures.py — 실측 런 CSV 를 찾는다. **저장소 사본을 먼저 본다.**

★ 왜 이게 필요한가 (2026-09-17)

  실측 런으로 검증하는 시험이 세 개 있는데(test_avoid_rate·test_avoid_hold·
  test_lidar_mount_check) 전부 `/tmp/drive_XXXX.csv` 를 직접 가리키고 있었고,
  **파일이 없으면 조용히 건너뛰고 ✅ 를 냈다.**

      if not os.path.exists(RUN):
        return None          # ← 그리고 아무 말 없이 통과

  /tmp 는 재부팅하면 사라진다. 즉 대회 당일 아침에 노트북을 껐다 켜면
  시험 세 개의 **실측 검증부가 통째로 안 돌면서 '전부 통과' 가 뜬다.**
  가짜 초록은 시험이 없는 것보다 나쁘다 — 없으면 사람이 확인이라도 한다.

  그래서 (1) 저장소 안 사본(data/<날짜>/)을 먼저 찾고, (2) 그래도 없으면
  **소리내어 실패**한다.

사용:
  from drive_fixtures import find_run, require_run
  csv_path, scan_path = find_run('0337')      # 없으면 (None, None)
  csv_path, scan_path = require_run('0337')   # 없으면 SystemExit
"""

import glob
import os
import sys

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _candidates(tag):
  """탐색 순서: 저장소 data/<날짜>/ (최신 날짜 먼저) → /tmp."""
  for d in sorted(glob.glob(os.path.join(WS, 'data', '*')), reverse=True):
    if os.path.isdir(d):
      yield os.path.join(d, f'drive_{tag}.csv')
  yield f'/tmp/drive_{tag}.csv'


def find_run(tag, need_scan=False):
  """(csv, scan) 을 돌려준다. 못 찾으면 (None, None).

  need_scan=True 면 스캔 파일까지 있는 후보만 받아들인다 — 마운트 검증처럼
  스캔이 본체인 시험이 CSV 만 보고 잘못 고르는 것을 막는다.
  """
  for c in _candidates(tag):
    s = c.replace('.csv', '_scan.csv')
    if not os.path.exists(c):
      continue
    if need_scan and not os.path.exists(s):
      continue
    return c, (s if os.path.exists(s) else None)
  return None, None


def require_run(tag, need_scan=False):
  """못 찾으면 죽는다. 조용히 건너뛰고 ✅ 를 내는 것보다 낫다."""
  c, s = find_run(tag, need_scan)
  if c is None:
    where = ' 또는 '.join(_candidates(tag))
    sys.exit(
        f'❌ 실측 픽스처 drive_{tag} 를 못 찾았다.\n'
        f'   찾아본 곳: {where}\n'
        f'   /tmp 는 재부팅하면 사라진다. data/<날짜>/ 사본을 확인할 것.')
  return c, s
