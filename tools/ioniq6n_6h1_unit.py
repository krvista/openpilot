#!/usr/bin/env python3
"""Phase 6h-1 unit verification (handoff COMMIT 1 gate).

1. step response: 15 deg step @ 25 km/h reaches 95% within <= 12 frames
   (6g-1 release took 15; alpha 0.3 linear should take ~9).
2. deadband: 0.05 deg change -> hold, 0.15 deg -> pass (DEADBAND = 0.1).
3. tau matching: controlsd lookahead horizon includes tau(v) (0.20 s at 5 m/s).

Standalone re-implementation of the constants' consumers so it runs without the
full opendbc import chain (crcmod etc.); constants are imported from the tree.
"""
import ast
import re
import sys

import numpy as np


def load_constants():
  src = open('opendbc_repo/opendbc/car/hyundai/values.py').read()
  c = {}
  for name in ['SMOOTHING_ANGLE_VEGO_MATRIX', 'SMOOTHING_ANGLE_ALPHA_MATRIX',
               'SMOOTHING_ANGLE_RELEASE_LO_DEG', 'SMOOTHING_ANGLE_RELEASE_HI_DEG',
               'SMOOTHING_ANGLE_RELEASE_MAX', 'SMOOTHING_ANGLE_DEADBAND_DEG',
               'SMOOTHING_ANGLE_DEADBAND_MAX_VEGO']:
    m = re.search(rf'^\s*{name}\s*=\s*([^#\n]+)', src, re.M)
    c[name] = ast.literal_eval(m.group(1).strip())
  csrc = open('selfdrive/controls/controlsd.py').read()
  for name in ['LAT_CMD_SMOOTH_TAU_BP', 'LAT_CMD_SMOOTH_TAU_V']:
    m = re.search(rf'^\s*{name}\s*=\s*([^#\n]+)', csrc, re.M)
    c[name] = ast.literal_eval(m.group(1).strip())
  return c


def sp_smooth_angle(C, v, a, al):
  gap = abs(a - al)
  if v < C['SMOOTHING_ANGLE_DEADBAND_MAX_VEGO'] and gap < C['SMOOTHING_ANGLE_DEADBAND_DEG']:
    return al
  alpha = float(min(float(np.interp(v, C['SMOOTHING_ANGLE_VEGO_MATRIX'],
                                    C['SMOOTHING_ANGLE_ALPHA_MATRIX'])), 1.))
  release = float(np.interp(gap, [C['SMOOTHING_ANGLE_RELEASE_LO_DEG'],
                                  C['SMOOTHING_ANGLE_RELEASE_HI_DEG']], [0.0, 1.0]))
  headroom = max(C['SMOOTHING_ANGLE_RELEASE_MAX'] - alpha, 0.0)
  a_eff = alpha + headroom * release
  return a * a_eff + al * (1 - a_eff)


def main():
  C = load_constants()
  ok = True

  # 1. step response @ 25 km/h (6.94 m/s)
  v = 25 / 3.6
  last = 0.0
  frames = None
  for k in range(1, 40):
    last = sp_smooth_angle(C, v, 15.0, last)
    if last >= 0.95 * 15.0:
      frames = k
      break
  res = frames is not None and frames <= 12
  ok &= res
  print(f"[{'PASS' if res else 'FAIL'}] step 15deg @25km/h: 95% in {frames} frames (gate <=12)")

  # 2. deadband
  hold = sp_smooth_angle(C, v, 2.05, 2.0) == 2.0
  passed = sp_smooth_angle(C, v, 2.15, 2.0) != 2.0
  res = hold and passed
  ok &= res
  print(f"[{'PASS' if res else 'FAIL'}] deadband: 0.05->hold {hold}, 0.15->pass {passed}")

  # 3. tau matching: tau(5 m/s) == 0.20 and lookahead horizon uses it
  tau5 = float(np.interp(5.0, C['LAT_CMD_SMOOTH_TAU_BP'], C['LAT_CMD_SMOOTH_TAU_V']))
  csrc = open('selfdrive/controls/controlsd.py').read()
  wired = ('lookahead_extra_s' in csrc
           and '_lookahead_curvature(model_v2, CS.vEgo, lat_smooth_tau)' in csrc
           and 'base_s + boost_s + lookahead_extra_s' in csrc)
  res = abs(tau5 - 0.20) < 1e-9 and wired
  ok &= res
  print(f"[{'PASS' if res else 'FAIL'}] tau matching: tau(5m/s)={tau5:.2f} (gate 0.20), lead wired={wired}")

  # 4. release effectively off (HI=1e6): big gap must NOT jump alpha
  a_small = sp_smooth_angle(C, v, 2.15, 2.0) - 2.0          # alpha*0.15
  a_big = (sp_smooth_angle(C, v, 12.0, 2.0) - 2.0) / 10.0   # alpha*10/10
  res = abs(a_small / 0.15 - a_big) < 0.02
  ok &= res
  print(f"[{'PASS' if res else 'FAIL'}] release off: gain(0.15deg)={a_small/0.15:.3f} == gain(10deg)={a_big:.3f}")

  sys.exit(0 if ok else 1)


if __name__ == '__main__':
  main()
