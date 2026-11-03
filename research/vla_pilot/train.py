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

★ 시험 조건 — B·B2·B3 를 같은 시험 주행에서
  깨끗 / 백색 σ / FLOAT형 σ (0.5 m 는 어느 모델도 학습에 안 본 크기)

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


def load():
  rows = list(csv.DictReader(open(os.path.join(HERE, 'dataset.csv'), encoding='utf-8')))
  cols = rows[0].keys()
  lidar = [c for c in cols if c.startswith('lidar')]
  loc = [c for c in cols if c.startswith('loc') and c[3:].isdigit()]
  assert len(lidar) == 36 and len(loc) == 12, (len(lidar), len(loc))
  assert all(f'loc_g{n}_0' in cols for n in GM), 'dataset.csv 가 옛 판이다 — make_dataset.py 를 다시 돌릴 것'
  runs = [r for r in sorted({r['run'] for r in rows})
          if sum(x['run'] == r for x in rows) >= MIN_RUN]
  return rows, runs, lidar, loc


def mat(rows, cols):
  return torch.tensor([[float(r[c]) for c in cols] for r in rows])


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
  rows, runs, lidar, loc = load()
  print(f'장치 {dev} · 샘플 {sum(r["run"] in runs for r in rows)} · 주행 {len(runs)}개 · 시드 {len(SEEDS)}개')
  tag = lambda kind, n: lidar + [c.replace('loc', f'loc_{kind}{n}_', 1) for c in loc]
  conds = ([('깨끗', lidar + loc)] + [(f'백색 σ{n}', tag('n', n)) for n in WHITE]
           + [(f'FLOAT형 σ{n}' + (' *' if n not in GM_TRAIN else ''), tag('g', n)) for n in GM])
  train_sets = {'B': [lidar + loc],
                'B2': [lidar + loc] + [tag('n', n) for n in WHITE],
                'B3': [lidar + loc] + [tag('g', n) for n in GM_TRAIN]}
  simple = {'평균값 예측': None, 'A lidar': lidar, 'C loc': loc}
  res = {k: {c: [[] for _ in SEEDS] for c, _ in conds} for k in train_sets}
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
        f = fit(torch.cat([mat(tr, c) for c in sets]), ytr.repeat(len(sets)), dev, seed)
        for cname, cols in conds:
          p = f(mat(te, cols))
          res[name][cname][si].append(mae(p, yte))
          if cname == '깨끗':
            res_av[name][si].append(mae(p, yte, av))

  print('\n조향 오차 MAE [deg] (낮을수록 좋음) — 평균±편차, 시드 3개')
  print(f'\n{"기준 모델 (깨끗한 측위)":<24}{"전체":>12}{"회피 구간":>14}')
  for k in simple:
    print(f'{k:<24}{fmt(res_s[k]):>12}{fmt(res_av[k]):>14}')
  print(f'\n{"시험 조건":<16}' + ''.join(f'{k:>14}' for k in train_sets))
  for cname, _ in conds:
    print(f'{cname:<16}' + ''.join(f'{fmt(res[k][cname]):>14}' for k in train_sets))
  print(f'{"(회피, 깨끗)":<16}' + ''.join(f'{fmt(res_av[k]):>14}' for k in train_sets))
  print('\nB  깨끗한 측위로만 학습 · B2 백색 잡음 섞어 학습 · B3 FLOAT형 잡음 섞어 학습')
  print('* 어느 모델도 학습 때 안 본 오차 크기')


if __name__ == '__main__':
  main()
