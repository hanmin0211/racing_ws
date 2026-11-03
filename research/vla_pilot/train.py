#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""train.py — 예비 실험: 조향을 '라이다만' / '측위만' / '라이다+측위' 로 맞혀 보고,
측위 오차에 학습 정책이 얼마나 버티는지 본다.

★ 모델
  A  lidar      라이다 36 구간만
  C  loc        측위 특징 12개만 (경로 대비 오프셋·방향오차·앞 경로점)
  B  lidar+loc  깨끗한 측위로만 학습
  B2 lidar+loc  학습 때 백색 잡음판(σ 0.1/0.3/1.0 m)도 섞는다
  B3 lidar+loc  학습 때 FLOAT형 잡음판(Gauss-Markov σ 0.1/0.3/1.0 m)도 섞는다
  B4 B3 와 같되 입력에 '측위 오차 σ' 숫자 하나를 더 준다 (실차의 RTK 공분산 σ 자리).
     B3 와 B4 의 차이 = σ 를 알려 준 효과. 깨끗한 측위는 σ=0.02 m (RTK FIXED 수준).
  D  B 에 정밀지도 특징 8개 추가 (깨끗한 측위로만 학습)
  D4 B4 에 정밀지도 특징 8개 추가. 지도 특징도 측위 자세로 계산하므로 같은 잡음을 받는다.
     B→D, B4→D4 의 차이 = 정밀지도를 넣은 효과.

★ 시험 조건 — B·B2·B3 를 같은 시험 주행에서
  깨끗 / 백색 σ / FLOAT형 σ (0.5 m 는 어느 모델도 학습에 안 본 크기)
  + '오보': 실제론 FLOAT형 1.0 m 인데 σ 는 0.02 m 로 보고된 경우 — σ 를 믿는 B4 의 약점.
  σ 입력은 시험 조건의 참값을 그대로 준다 (실제 공분산 추정의 오차는 아직 안 넣었다).

★ 평가: 주행 하나씩 빼서 시험(leave-one-run-out) × 시드 3개. 표의 값은
  시드별 (주행 평균) 의 평균 ± 표준편차.

★ 해석할 때 주의
  정답 조향(cmd_steer)은 Pure Pursuit 가 '측위+경로' 로 만든 값이다. 측위가
  들어간 모델이 이기는 건 일부 자명하다. 볼 것은 잡음에 대한 버팀과 회피 구간.

사용 (학습 PC, conda env vla):
  python research/vla_pilot/train.py
"""

import csv
import os
import statistics

import torch

HERE = os.path.dirname(os.path.abspath(__file__))
MIN_RUN = 100       # 샘플이 이보다 적은 주행은 뺀다 (drive_0020: 조향 값이 깨져 11개뿐)
EPOCHS = 400
SEEDS = (0, 1, 2)
WHITE = ('0.1', '0.3', '1.0')
GM = ('0.1', '0.3', '0.5', '1.0')
GM_TRAIN = ('0.1', '0.3', '1.0')   # 0.5 는 시험 전용
SIG_CLEAN = 0.02                   # 깨끗한 측위에 붙이는 σ [m]
USES_SIGMA = {'B4', 'D4'}


def load():
  rows = list(csv.DictReader(open(os.path.join(HERE, 'dataset.csv'), encoding='utf-8')))
  cols = rows[0].keys()
  lidar = [c for c in cols if c.startswith('lidar')]
  loc = [c for c in cols if c.startswith('loc') and c[3:].isdigit()]
  hdm = [c for c in cols if c.startswith('map') and c[3:].isdigit()]
  assert len(lidar) == 36 and len(loc) == 12 and len(hdm) == 8, (len(lidar), len(loc), len(hdm))
  assert all(f'map_g{n}_0' in cols for n in GM), 'dataset.csv 가 옛 판이다 — make_dataset.py 를 다시 돌릴 것'
  runs = [r for r in sorted({r['run'] for r in rows})
          if sum(x['run'] == r for x in rows) >= MIN_RUN]
  return rows, runs, lidar, loc, hdm


def mat(rows, cols, sigma=None):
  m = torch.tensor([[float(r[c]) for c in cols] for r in rows])
  return m if sigma is None else torch.cat([m, torch.full((len(rows), 1), sigma)], 1)


def fit(X, y, dev, seed):
  torch.manual_seed(seed)
  mu, sd = X.mean(0), X.std(0) + 1e-6
  net = torch.nn.Sequential(torch.nn.Linear(X.shape[1], 64), torch.nn.ReLU(),
                            torch.nn.Linear(64, 64), torch.nn.ReLU(),
                            torch.nn.Linear(64, 1)).to(dev)
  opt = torch.optim.Adam(net.parameters(), lr=1e-3, weight_decay=1e-4)
  Xn, yd = ((X - mu) / sd).to(dev), y.to(dev)
  for _ in range(EPOCHS):  # 샘플 수천 개라 전체 배치로 충분하다
    opt.zero_grad()
    torch.nn.functional.l1_loss(net(Xn).squeeze(1), yd).backward()
    opt.step()
  return lambda Z: net(((Z - mu) / sd).to(dev)).squeeze(1).detach().cpu()


def mae(p, y, mask=None):
  if mask is not None:
    if not mask.any():
      return float('nan')
    p, y = p[mask], y[mask]
  return (p - y).abs().mean().item()


def fmt(per_seed):
  """per_seed: 시드마다 [주행별 값] → '평균±편차' (시드별 주행평균 기준)."""
  m = [statistics.mean(v for v in fold if v == v) for fold in per_seed]
  sd = statistics.stdev(m) if len(m) > 1 else 0.0
  return f'{statistics.mean(m):5.2f}±{sd:4.2f}'


def main():
  dev = 'cuda' if torch.cuda.is_available() else 'cpu'
  rows, runs, lidar, loc, hdm = load()
  print(f'장치 {dev} · 샘플 {sum(r["run"] in runs for r in rows)} · 주행 {len(runs)}개 · 시드 {len(SEEDS)}개')
  # 시험 조건마다 입력 열을 깨끗판(lidar+loc+map) 기준으로 잡고, 모델이 안 쓰는 map 열은 뒤에서 뺀다
  tag = lambda kind, n: (lidar + [c.replace('loc', f'loc_{kind}{n}_', 1) for c in loc]
                         + [c.replace('map', f'map_{kind}{n}_', 1) for c in hdm])
  # (이름, 입력 열, 그 조건의 σ)
  conds = ([('깨끗', lidar + loc + hdm, SIG_CLEAN)]
           + [(f'백색 σ{n}', tag('n', n), float(n)) for n in WHITE]
           + [(f'FLOAT형 σ{n}' + (' *' if n not in GM_TRAIN else ''), tag('g', n), float(n))
              for n in GM]
           + [('FLOAT1.0 σ오보', tag('g', '1.0'), SIG_CLEAN)])
  clean = lidar + loc + hdm
  train_sets = {'B': [(clean, SIG_CLEAN)],
                'B2': [(clean, SIG_CLEAN)] + [(tag('n', n), float(n)) for n in WHITE],
                'B3': [(clean, SIG_CLEAN)] + [(tag('g', n), float(n)) for n in GM_TRAIN]}
  train_sets['B4'] = train_sets['B3']
  train_sets['D'] = train_sets['B']
  train_sets['D4'] = train_sets['B3']
  n_nomap = len(lidar) + len(loc)
  pick = lambda name, cols: cols if name.startswith('D') else cols[:n_nomap]
  simple = {'평균값 예측': None, 'A lidar': lidar, 'C loc': loc}
  res = {k: {c: [[] for _ in SEEDS] for c, _, _ in conds} for k in train_sets}
  res_s = {k: [[] for _ in SEEDS] for k in simple}
  res_av = {k: [[] for _ in SEEDS] for k in list(simple) + list(train_sets)}

  for si, seed in enumerate(SEEDS):
    for test in runs:
      tr = [r for r in rows if r['run'] in runs and r['run'] != test]
      te = [r for r in rows if r['run'] == test]
      ytr, yte = mat(tr, ['steer']).squeeze(1), mat(te, ['steer']).squeeze(1)
      av = mat(te, ['avoid_active']).squeeze(1) > 0
      for name, cols in simple.items():
        p = (torch.full_like(yte, ytr.mean().item()) if cols is None
             else fit(mat(tr, cols), ytr, dev, seed)(mat(te, cols)))
        res_s[name][si].append(mae(p, yte))
        res_av[name][si].append(mae(p, yte, av))
      for name, sets in train_sets.items():
        use = name in USES_SIGMA
        f = fit(torch.cat([mat(tr, pick(name, c), s if use else None) for c, s in sets]),
                ytr.repeat(len(sets)), dev, seed)
        for cname, cols, s in conds:
          p = f(mat(te, pick(name, cols), s if use else None))
          res[name][cname][si].append(mae(p, yte))
          if cname == '깨끗':
            res_av[name][si].append(mae(p, yte, av))

  print('\n조향 오차 MAE [deg] (낮을수록 좋음) — 평균±편차, 시드 3개')
  print(f'\n{"기준 모델 (깨끗한 측위)":<24}{"전체":>12}{"회피 구간":>14}')
  for k in simple:
    print(f'{k:<24}{fmt(res_s[k]):>12}{fmt(res_av[k]):>14}')
  print(f'\n{"시험 조건":<16}' + ''.join(f'{k:>14}' for k in train_sets))
  for cname, _, _ in conds:
    print(f'{cname:<16}' + ''.join(f'{fmt(res[k][cname]):>14}' for k in train_sets))
  print(f'{"(회피, 깨끗)":<16}' + ''.join(f'{fmt(res_av[k]):>14}' for k in train_sets))
  print('\nB  깨끗한 측위로만 학습 · B2 백색 잡음 섞어 학습 · B3 FLOAT형 잡음 섞어 학습')
  print('B4 B3 + 측위 오차 σ 입력 · D = B + 정밀지도 · D4 = B4 + 정밀지도')
  print('* 어느 모델도 학습 때 안 본 오차 크기')
  print('σ오보: 실제 FLOAT형 1.0 m 인데 σ 는 0.02 m 로 보고 (B4 만 영향을 받는다)')


if __name__ == '__main__':
  main()
