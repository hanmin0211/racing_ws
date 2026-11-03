#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""train.py — 예비 실험: 조향을 '라이다만' / '측위만' / '라이다+측위' 로 맞혀 본다.

★ 무엇을 비교하나
  A  lidar      라이다 36 구간만
  C  loc        측위 특징 12개만 (경로 대비 오프셋·방향오차·앞 경로점)
  B  lidar+loc  둘 다
  + B 를 깨끗한 측위로 학습한 뒤, 시험 때 측위에 σ=0.1/0.3/1.0 m 잡음을 넣으면
    얼마나 나빠지는지 (측위 품질 → 학습 정책 성능 곡선)
  B2 같은 입력이지만 학습 때 잡음판도 섞는다 → 측위가 나빠도 버티는지.
     잡음판은 시험과 같은 σ 3종이라, 학습에 없던 오차 크기에서의 성능은 아직 모른다.

★ 평가: 주행 단위로 하나씩 빼서 시험(leave-one-run-out). 같은 주행의 이웃
  샘플이 학습·시험에 섞이면 점수가 부풀려진다.

★ 해석할 때 주의
  정답 조향(cmd_steer)은 Pure Pursuit 가 '측위+경로' 로 만든 값이다. 그래서
  측위가 들어간 모델이 이기는 건 당연한 쪽에 가깝다. 이 실험의 값은
    · 라이다만으로는 어디서 꺾을지(경로 의도)를 모른다는 걸 수치로 보이는 것
    · 회피 구간(avoid_active)처럼 조향이 라이다에서 나온 곳의 차이
    · 측위 잡음 곡선
  에 있다. 사람이 운전한 데이터나 카메라가 들어오면 다시 해야 한다.

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
NOISE = ('0.1', '0.3', '1.0')


def load():
  rows = list(csv.DictReader(open(os.path.join(HERE, 'dataset.csv'), encoding='utf-8')))
  cols = rows[0].keys()
  lidar = [c for c in cols if c.startswith('lidar')]
  loc = [c for c in cols if c.startswith('loc') and not c.startswith('loc_n')]
  assert len(lidar) == 36 and len(loc) == 12, (len(lidar), len(loc))
  runs = [r for r in sorted({r['run'] for r in rows})
          if sum(x['run'] == r for x in rows) >= MIN_RUN]
  return rows, runs, lidar, loc


def mat(rows, cols):
  return torch.tensor([[float(r[c]) for c in cols] for r in rows])


def fit(X, y, dev):
  torch.manual_seed(0)
  mu, sd = X.mean(0), X.std(0) + 1e-6
  net = torch.nn.Sequential(torch.nn.Linear(X.shape[1], 64), torch.nn.ReLU(),
                            torch.nn.Linear(64, 64), torch.nn.ReLU(),
                            torch.nn.Linear(64, 1)).to(dev)
  opt = torch.optim.Adam(net.parameters(), lr=1e-3, weight_decay=1e-4)
  Xn, yd = ((X - mu) / sd).to(dev), y.to(dev)
  for _ in range(EPOCHS):  # 샘플 1.3천개라 전체 배치로 충분하다
    opt.zero_grad()
    loss = torch.nn.functional.l1_loss(net(Xn).squeeze(1), yd)
    loss.backward()
    opt.step()
  return lambda Z: net(((Z - mu) / sd).to(dev)).squeeze(1).detach().cpu()


def mae(p, y, mask=None):
  if mask is not None:
    if not mask.any():
      return float('nan')
    p, y = p[mask], y[mask]
  return (p - y).abs().mean().item()


def main():
  dev = 'cuda' if torch.cuda.is_available() else 'cpu'
  rows, runs, lidar, loc = load()
  print(f'장치 {dev} · 샘플 {sum(r["run"] in runs for r in rows)} · 주행 {len(runs)}개')
  sets = {'A lidar': lidar, 'C loc': loc, 'B lidar+loc': lidar + loc}
  noisy = lambda n: lidar + [c.replace('loc', f'loc_n{n}_', 1) for c in loc]
  res = {k: [] for k in ['평균값 예측'] + list(sets) + [f'B 잡음σ{n}m' for n in NOISE]
         + ['B2 잡음학습'] + [f'B2 잡음σ{n}m' for n in NOISE]}
  res_av = {k: [] for k in res}
  for test in runs:
    tr = [r for r in rows if r['run'] in runs and r['run'] != test]
    te = [r for r in rows if r['run'] == test]
    ytr, yte = mat(tr, ['steer']).squeeze(1), mat(te, ['steer']).squeeze(1)
    av = mat(te, ['avoid_active']).squeeze(1) > 0
    base = torch.full_like(yte, ytr.mean().item())
    res['평균값 예측'].append(mae(base, yte))
    res_av['평균값 예측'].append(mae(base, yte, av))
    for name, cols in sets.items():
      f = fit(mat(tr, cols), ytr, dev)
      p = f(mat(te, cols))
      res[name].append(mae(p, yte))
      res_av[name].append(mae(p, yte, av))
      if name == 'B lidar+loc':
        for n in NOISE:  # 같은 모델, 시험 측위만 잡음판으로
          pn = f(mat(te, noisy(n)))
          res[f'B 잡음σ{n}m'].append(mae(pn, yte))
          res_av[f'B 잡음σ{n}m'].append(mae(pn, yte, av))
    # B2: 학습 때 깨끗한 측위 + 잡음판 3종을 섞는다 (같은 정답 4번) → 측위가 나빠도 버티나
    f = fit(torch.cat([mat(tr, lidar + loc)] + [mat(tr, noisy(n)) for n in NOISE]),
            ytr.repeat(1 + len(NOISE)), dev)
    for key, cols in [('B2 잡음학습', lidar + loc)] + [(f'B2 잡음σ{n}m', noisy(n)) for n in NOISE]:
      p = f(mat(te, cols))
      res[key].append(mae(p, yte))
      res_av[key].append(mae(p, yte, av))
  print(f'\n조향 오차 MAE [deg] — 주행 {len(runs)}개 각각을 시험으로 뺀 평균')
  print(f'{"모델":<16}{"전체":>8}{"회피 구간":>10}')
  for k in res:
    av_vals = [v for v in res_av[k] if v == v]
    av_s = f'{statistics.mean(av_vals):10.2f}' if av_vals else f'{"-":>10}'
    print(f'{k:<16}{statistics.mean(res[k]):8.2f}{av_s}')


if __name__ == '__main__':
  main()
