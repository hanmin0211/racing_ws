#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""imu_link_check.py — IMU 시리얼이 **끊기지 않고** 흐르는지 본다.

★ 왜 필요한가 (2026-09-17)
  이날 주행에서 IMU 가 49회 재연결했고 경로가 76회 끊겨 차가 계속 섰다.
  그런데 **같은 하드웨어로 어젯밤 완주한 런(run_0337)은 재연결 0회** 였다.
  코드가 아니라 물리적으로 새로 생긴 고장이라는 뜻이다.

  스택을 다 띄워 놓고 찾으면 느리고, 노드 로그는 1.5초 문턱을 넘은 것만
  보여준다. 여기서는 **바이트 흐름 자체**를 재서 더 짧은 끊김까지 잡는다.
  케이블·커넥터를 흔들면서 돌리면 어디서 끊기는지 바로 보인다.

  ⚠ bringup 이 떠 있으면 포트를 잡고 있어 열 수 없다. 먼저 내릴 것:
      pgrep -f 'install/[a-z_]+/lib/' | xargs -r kill -9

사용
    python3 tools/imu_link_check.py                 # 30초
    python3 tools/imu_link_check.py --seconds 60
    python3 tools/imu_link_check.py --dev /dev/ttyUSB0 --baud 921600

  판정 (hfi_a9_ros2 는 1.5초 무데이터에서 재연결한다)
    ✅ 최대 끊김 0.2s 미만   정상
    ⚠ 0.2~1.5s              불안정 — 흔들면 재현되는지 볼 것
    ❌ 1.5s 이상             주행 중 재연결이 뜬다 = 경로 끊김 = 정지
"""
import argparse
import os
import select
import sys
import time

# hfi_a9_ros2 가 무데이터로 판정해 재연결하는 시간 [s]
NODE_RECONNECT_S = 1.5
WARN_GAP_S = 0.2


def summarize(gaps, total_bytes, elapsed, reads):
    """끊김 목록 → 사람이 읽을 요약."""
    lens = [g[1] if isinstance(g, (tuple, list)) else g for g in gaps]
    worst = max(lens) if lens else 0.0
    over_warn = [g for g in lens if g >= WARN_GAP_S]
    over_fail = [g for g in lens if g >= NODE_RECONNECT_S]
    return {
        'worst': worst,
        'warn': len(over_warn),
        'fail': len(over_fail),
        'bps': total_bytes / elapsed if elapsed > 0 else 0.0,
        'reads': reads,
    }


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--dev', default='/dev/imu')
    p.add_argument('--baud', type=int, default=921600)
    p.add_argument('--seconds', type=float, default=30.0)
    p.add_argument('--quiet', action='store_true',
                   help='끊김만 찍는다 (진행 표시 없음)')
    a = p.parse_args()

    print()
    print('=' * 60)
    print(' IMU 시리얼 연속성 점검')
    print('=' * 60)
    if not os.path.exists(a.dev):
        print(f' ❌ {a.dev} 가 없다. USB 가 빠졌거나 udev 규칙이 안 걸렸다.')
        print('    ls -l /dev/imu  ·  lsusb  로 확인할 것.')
        return 2

    # 보드레이트를 맞추지 않으면 프레이밍 에러로 다 버려져 **멀쩡한 장치가
    # 0바이트로 보인다** (drive_school.sh 주석의 같은 함정).
    if os.system(f'stty -F {a.dev} {a.baud} raw -echo 2>/dev/null') != 0:
        print(' ⚠ stty 실패 — 포트를 다른 프로세스가 잡고 있을 수 있다.')
        print("    pgrep -f 'install/[a-z_]+/lib/' | xargs -r kill -9")

    try:
        fd = os.open(a.dev, os.O_RDONLY | os.O_NONBLOCK)
    except OSError as e:
        print(f' ❌ 열 수 없다: {e}')
        print("    bringup 을 먼저 내릴 것:")
        print("    pgrep -f 'install/[a-z_]+/lib/' | xargs -r kill -9")
        return 2

    print(f' {a.dev} @ {a.baud}  —  {a.seconds:.0f}초 동안 바이트 흐름을 본다.')
    print(' 케이블·커넥터를 흔들어 보면 어디서 끊기는지 보인다.')
    print(' (끊김 문턱: 노드가 재연결하는 시간 %.1fs)' % NODE_RECONNECT_S)
    print()

    t0 = time.time()
    last_data = t0
    gaps = []               # (시작 t+, 길이) — 진행 중인 것도 끝에서 계상한다
    total = 0
    reads = 0
    next_tick = t0 + 1.0
    last_report = 0.0
    try:
        while True:
            now = time.time()
            if now - t0 >= a.seconds:
                break
            r, _, _ = select.select([fd], [], [], 0.05)
            now = time.time()
            chunk = b''
            if r:
                try:
                    chunk = os.read(fd, 4096)
                except OSError:
                    chunk = b''
            if chunk:
                gap = now - last_data
                if gap >= WARN_GAP_S:
                    gaps.append((last_data - t0, gap))
                    mark = '❌' if gap >= NODE_RECONNECT_S else '⚠ '
                    print(f'  {mark} t+{last_data - t0:6.1f}s 부터 {gap:6.2f}s 끊김 → 복구')
                last_data = now
                total += len(chunk)
                reads += 1
            else:
                # ★ 진행 중인 끊김. 예전엔 여기서 gaps 에 안 넣어서, 끝까지
                #   안 돌아오면 목록이 비어 **'정상' 으로 보고했다**(자기 모순).
                #   화면 도배도 했다. 이제 1초에 한 번만 찍고, 종료 시 계상한다.
                gap = now - last_data
                if gap >= NODE_RECONNECT_S and now - last_report >= 1.0:
                    print(f'  ❌ t+{last_data - t0:6.1f}s 부터 {gap:6.2f}s 째 끊긴 채…')
                    last_report = now
            if not a.quiet and now >= next_tick:
                next_tick += 1.0
                el = now - t0
                print(f'  · {el:4.0f}s  {total / max(el, 1e-9) / 1024:6.1f} KB/s  '
                      f'끊김 {len(gaps)}회', end='\r')
    except KeyboardInterrupt:
        print('\n (중단됨)')
    finally:
        os.close(fd)

    tail = time.time() - last_data
    if tail >= WARN_GAP_S:
        gaps.append((last_data - t0, tail))
        print(f'  ❌ t+{last_data - t0:6.1f}s 부터 {tail:6.2f}s — **끝까지 안 돌아왔다**')

    elapsed = time.time() - t0
    s = summarize(gaps, total, elapsed, reads)
    print(' ' * 60, end='\r')
    print()
    print('=' * 60)
    print(f' {elapsed:.0f}초 · {total} 바이트 · {s["bps"] / 1024:.1f} KB/s '
          f'· read {s["reads"]}회')
    lens = [g[1] if isinstance(g, (tuple, list)) else g for g in gaps]
    dead = sum(lens)
    print(f' 끊김 {WARN_GAP_S}s 이상 {s["warn"]}회 · '
          f'{NODE_RECONNECT_S}s 이상 {s["fail"]}회 · 최대 {s["worst"]:.2f}s')
    print(f' 총 끊긴 시간 {dead:.1f}s / {elapsed:.0f}s '
          f'({100 * dead / max(elapsed, 1e-9):.0f}%)')
    print('=' * 60)

    if total == 0:
        print(' ❌ 바이트가 하나도 안 온다.')
        print('    · 보드레이트가 맞는지 (IMU 921600)')
        print('    · 다른 프로세스가 포트를 잡고 있는지')
        print('    · USB 를 뽑고 **10초 뒤** 다시 꽂을 것 (바로 꽂으면 칩이 리셋 안 된다)')
        return 3
    if s['fail'] > 0:
        print(f' ❌ 고장 — {NODE_RECONNECT_S}s 이상 끊김이 {s["fail"]}회.')
        print('    주행 중이면 노드가 재연결하고, 그동안 /odometry 가 끊겨')
        print('    경로가 사라지고 pure_pursuit 가 차를 세운다(= 정지 → 탈락 위험).')
        print('    → 커넥터·케이블을 흔들며 다시 돌려 볼 것. 재현되면 그 지점이다.')
        print('    → 안 재현되면 USB 를 뽑고 10초 뒤 다른 포트에 꽂아 볼 것.')
        return 3
    if s['warn'] > 0:
        print(f' ⚠ 불안정 — {WARN_GAP_S}s 이상 끊김 {s["warn"]}회 (최대 {s["worst"]:.2f}s).')
        print('    아직 노드 문턱(%.1fs)은 안 넘었지만 주행 진동에서 넘어갈 수 있다.'
              % NODE_RECONNECT_S)
        return 1
    print(' ✅ 정상 — 끊김 없음. IMU 링크는 문제가 아니다.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
