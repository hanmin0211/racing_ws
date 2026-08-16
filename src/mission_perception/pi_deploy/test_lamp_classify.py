#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""coco_detector_node.classify_lamp 검증.

대회 신호등 도면(신호부 24x56cm, 램프 R5, 간격 16/15/16) 비율로 합성 이미지를
만들어, 각 램프가 켜졌을 때 올바른 상태가 나오는지 확인한다.
실제 노드 코드를 import 해서 테스트한다(하드웨어 의존 모듈만 스텁).
"""
import os
import sys
import types

import cv2
import numpy as np

# --- 하드웨어 의존 모듈 스텁 (import 만 통과시키면 된다) ---------------
# common 은 패키지로 만들어야 `from common.toolbox import ...` 가 통한다.
for name in ('common', 'common.hailo_inference', 'common.toolbox',
             'object_detection_post_process'):
    sys.modules[name] = types.ModuleType(name)
sys.modules['common'].__path__ = []            # 패키지로 인식시킨다
sys.modules['common.hailo_inference'].HailoInfer = object
sys.modules['common.toolbox'].default_preprocess = lambda img, w, h: img
sys.modules['object_detection_post_process'].extract_detections = lambda *a: None
sys.modules['common'].hailo_inference = sys.modules['common.hailo_inference']
sys.modules['common'].toolbox = sys.modules['common.toolbox']

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import coco_detector_node as M   # noqa: E402

ok = True


def check(label, cond):
    global ok
    print(f'{"PASS" if cond else "FAIL"}  {label}')
    ok = ok and cond


class Dummy:
    """classify_lamp 가 쓰는 속성만 갖춘 가짜 노드."""

    use_color_check = True
    min_lamp_value = 120


def make_signal(lit, w=48, h=112, bgr=None, bg=(30, 30, 30)):
    """세로 3구 신호등 이미지. lit ∈ {'RED','YELLOW','GREEN',None}.

    도면 비율: 신호부 24x56 → 1:2.33. 램프 중심 0.29 / 0.50 / 0.71.
    """
    img = np.full((h, w, 3), bg, np.uint8)
    colors = {'RED': (0, 0, 230), 'YELLOW': (0, 220, 235),
              'GREEN': (140, 210, 60)}      # 초록은 청록 쪽 (BGR)
    if bgr:
        colors = bgr
    r = int(w * 0.21)                        # 램프 반지름 (10/24 ≈ 0.42 지름)
    for name, pos in M.LAMP_POS.items():
        cy = int(pos * h)
        col = colors[name] if name == lit else (18, 18, 18)   # 꺼진 램프는 어둡게
        cv2.circle(img, (w // 2, cy), r, col, -1)
    return img


d = Dummy()
full = (0, 0, 48, 112)

# --- 1) 각 램프가 켜졌을 때 올바르게 판별되는가 -----------------------
for want in ('RED', 'YELLOW', 'GREEN'):
    img = make_signal(want)
    got, why = M.CocoTrafficLightNode.classify_lamp(d, img, full)
    check(f'{want} 점등 → {want} 판별  ({why})', got == want)

# --- 2) 다 꺼져 있으면 판별하지 않아야 한다 ---------------------------
got, why = M.CocoTrafficLightNode.classify_lamp(d, make_signal(None), full)
check(f'전부 소등 → None  ({why})', got is None)

# --- 3) 위치와 색이 어긋나면 거부해야 한다 (간판·나무 오탐 방지) ------
#     아래쪽(초록 자리)에 빨간 불이 켜진 이상한 물체
img = make_signal('GREEN', bgr={'RED': (0, 0, 230), 'YELLOW': (0, 0, 230),
                                'GREEN': (0, 0, 230)})
got, why = M.CocoTrafficLightNode.classify_lamp(d, img, full)
check(f'아래에 빨강 → 불일치로 거부  ({why})', got is None)

# --- 4) 색 검증을 끄면 위치만으로 판별 -------------------------------
d2 = Dummy()
d2.use_color_check = False
got, why = M.CocoTrafficLightNode.classify_lamp(d2, img, full)
check(f'색검증 OFF → 위치만으로 GREEN  ({why})', got == 'GREEN')

# --- 5) 한국식 청록 초록도 잡히는가 (H가 초록보다 높은 쪽) ------------
img = make_signal('GREEN', bgr={'RED': (0, 0, 230), 'YELLOW': (0, 220, 235),
                                'GREEN': (200, 200, 40)})   # 확실한 청록
got, why = M.CocoTrafficLightNode.classify_lamp(d, img, full)
check(f'청록 초록도 GREEN  ({why})', got == 'GREEN')

# --- 6) 너무 작은 박스는 판단 보류 -----------------------------------
got, why = M.CocoTrafficLightNode.classify_lamp(d, make_signal('RED'), (0, 0, 3, 5))
check(f'초소형 박스 → 보류  ({why})', got is None)

# --- 7) 도면 상수가 실제로 반영돼 있는가 -----------------------------
check(f'램프 위치 상수 = 도면값 {M.LAMP_POS}',
      abs(M.LAMP_POS['RED'] - 0.29) < 0.01
      and abs(M.LAMP_POS['GREEN'] - 0.71) < 0.01)
check('COCO traffic light 클래스 = 9', M.COCO_TRAFFIC_LIGHT == 9)

print('\n=== 전체 통과 ===' if ok else '\n=== 실패 있음 ===')
sys.exit(0 if ok else 1)
