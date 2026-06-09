#!/usr/bin/env python3
"""Ioniq 6 N lane-tracking audit (CCNC HDA2-ALT angle-control).

Quantifies the lateral failure class found in POST_6F2_AUDIT §1.D on the
Phase 6f-5 build (ccnc-drivelog 0x40/0x41): op's commanded path under-tracks the
actual lane curvature on corner entry, so the car runs wide to the outside lane
line (worst: 0x40 seg5 KST 07:27:43-49, right-line clearance 0.77 m, driver took
over with +40 deg). Three metrics, one pass over the rlogs:

  1. low-speed steering wobble  -- 3-7 Hz band-limited deg-RMS of the actual wheel
     vs op command, hands-off, shallow curve (verifies the 6f-4 city-wobble win and
     that a smoothing change does NOT regress it).
  2. corner under-response       -- on op-active corner stretches, ratio of the
     lane-implied required curvature (from lane-center geometry ahead) to op's
     commanded curvature. <1.0 ratio of op/required => under-steer / wide run.
  3. near lane-crossing          -- op-active, blinker off, lane-line (prob>0.4)
     clearance < THRESH while laneChangeState==off. Lists worst events per route.

The three named regression fixtures (0x40 seg5/seg9, 0x41 seg22) are checked
explicitly so any controlsd/carcontroller smoothing change (Phase 6g-1) can be
A/B'd before/after on the same logs.

Usage:
  python tools/ioniq6n_lane_tracking_audit.py [DRIVELOG_DIR]
  python tools/ioniq6n_lane_tracking_audit.py --route 0x40
Logs (.zst) are NOT committed; point DRIVELOG_DIR at a local ccnc-drivelog checkout.
"""
import os
import sys
import glob
import argparse
from collections import defaultdict

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'opendbc_repo'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from openpilot.tools.lib.logreader import LogReader  # noqa: E402

DRIVELOG_DIR = '/home/user/openpilot/drivelog'
DONGLE = '99b215d21bbf8735'

# Named regression fixtures (route_id, seg, label) — POST_6F2_AUDIT §1.D.C / §1.E.B
# Same GPS spot (37.5568, 126.96898): 0x40 seg5 (6f-5, under-response) vs 0x42 seg4
# (6g-1, over-response). A good fix lands both near 1.0x op/required, no driver grab.
FIXTURES = [
  ('00000040--eb2be2a919', 5,  '6f-5 출근 seg5  KST 07:27:43-49  좌굽 미추종(0.5x)→우측선 0.77m, 운전자 개입'),
  ('00000040--eb2be2a919', 9,  '6f-5 출근 seg9  ~10min  차선 끊김→재연결, 좌측선 0.53m'),
  ('00000041--3e9e6dbdb8', 22, '6f-5 퇴근 seg22 ~22min  112 km/h, 좌측선 0.33m'),
  ('00000042--8c1f634610', 4,  '6g-1 출근 seg4  KST 07:25:22-25  S역곡선 과조향(2.6x)→좌측선 0.96m, 운전자 -1166Nm 개입'),
]

NEAR_CROSS_CLEARANCE_M = 0.80   # |laneLine.y[0]| below this with prob>0.4 = near crossing
WOBBLE_BAND = (3.0, 7.0)        # Hz; the 6f-4 city-wobble band
CORNER_CURV_MIN = 0.002         # 1/m; treat |desiredCurvature| above this as a corner


def _band_rms(x, dt, lo, hi):
  x = np.asarray(x, dtype=np.float64)
  x = x - x.mean()
  n = len(x)
  if n < 32:
    return float('nan')
  w = np.hanning(n)
  X = np.abs(np.fft.rfft(x * w)) ** 2
  fr = np.fft.rfftfreq(n, dt)
  m = (fr >= lo) & (fr <= hi)
  return float(np.sqrt(2 * X[m].sum() / np.sum(w ** 2) / n))


def _interp_to(master_t, src_t, src_v):
  if len(src_t) < 2:
    return np.full(len(master_t), np.nan)
  return np.interp(master_t.astype(np.float64), np.asarray(src_t, np.float64), np.asarray(src_v, np.float64))


def _nearest(master_t, src_t, src_v):
  if len(src_t) == 0:
    return np.zeros(len(master_t), dtype=getattr(src_v, 'dtype', np.float64))
  idx = np.clip(np.searchsorted(np.asarray(src_t), master_t), 0, len(src_t) - 1)
  return np.asarray(src_v)[idx]


def load_segment(path):
  """Return per-stream arrays from one rlog (standard LogReader)."""
  s = defaultdict(list)
  for msg in LogReader(path):
    w = msg.which()
    t = msg.logMonoTime
    if w == 'carState':
      c = msg.carState
      s['cs_t'].append(t); s['vego'].append(c.vEgo); s['wheel'].append(c.steeringAngleDeg)
      s['pressed'].append(c.steeringPressed); s['lblink'].append(c.leftBlinker); s['rblink'].append(c.rightBlinker)
    elif w == 'carControl':
      s['cc_t'].append(t); s['lat'].append(msg.carControl.latActive)
    elif w == 'carOutput':
      s['co_t'].append(t); s['ao'].append(msg.carOutput.actuatorsOutput.steeringAngleDeg)
    elif w == 'controlsState':
      s['ct_t'].append(t); s['dc'].append(msg.controlsState.desiredCurvature)
    elif w == 'modelV2':
      m = msg.modelV2
      ll = m.laneLines
      def y0(i):
        try:
          return ll[i].y[0]
        except Exception:
          return float('nan')
      def yat(i, xt):
        try:
          x = ll[i].x
          j = int(np.argmin([abs(xx - xt) for xx in x]))
          return ll[i].y[j]
        except Exception:
          return float('nan')
      s['m_t'].append(t)
      probs = list(m.laneLineProbs) + [0, 0, 0, 0]
      s['pL'].append(probs[1]); s['pR'].append(probs[2])
      s['clrL'].append(abs(y0(1))); s['clrR'].append(abs(y0(2)))
      # lane center 50 m ahead (left+right)/2 -> "where the lane goes"
      s['lc50'].append((yat(1, 50.0) + yat(2, 50.0)) / 2.0)
      s['lcs'].append(str(m.meta.laneChangeState))
  return {k: (np.array(v) if k.endswith('_t') else np.array(v)) for k, v in s.items()}


def seg_num(path):
  return int(os.path.basename(path).split('--')[2])


def route_segments(drivelog_dir, route_id):
  g = glob.glob(os.path.join(drivelog_dir, f'{DONGLE}_{route_id}--*--rlog.zst'))
  return sorted(g, key=seg_num)


def analyze_route(drivelog_dir, route_id):
  wob_op, wob_wh, wob_dur = [], [], []
  corner_ratio = []                  # op_curv / required_curv on corners (<1 = under-response)
  near_events = []                   # (seg, rel_s, side, clearance, v_kmh)
  for path in route_segments(drivelog_dir, route_id):
    d = load_segment(path)
    if len(d.get('co_t', [])) < 60 or len(d.get('m_t', [])) < 10:
      continue
    co_t = d['co_t']; t = co_t.astype(np.float64) / 1e9
    dt = float(np.median(np.diff(t)))
    vego = _interp_to(co_t, d['cs_t'], d['vego']) * 3.6
    wheel = _interp_to(co_t, d['cs_t'], d['wheel'])
    pressed = _nearest(co_t, d['cs_t'], d['pressed'])
    lat = _nearest(co_t, d['cc_t'], d['lat'])
    dc = _interp_to(co_t, d['ct_t'], d['dc'])

    # (1) low-speed wobble: hands-off, shallow curve, 20-50 km/h
    cond = lat & (~pressed) & (vego >= 20) & (vego <= 50) & (np.abs(dc) < CORNER_CURV_MIN)
    runs = np.diff(np.concatenate([[0], cond.view(np.int8), [0]]))
    for a, b in zip(np.where(runs == 1)[0], np.where(runs == -1)[0]):
      if (b - a) * dt < 2.0:
        continue
      wob_op.append(_band_rms(d['ao'][a:b], dt, *WOBBLE_BAND))
      wob_wh.append(_band_rms(wheel[a:b], dt, *WOBBLE_BAND))
      wob_dur.append((b - a) * dt)

    # (2) corner under-response on model timeline
    m_t = d['m_t']
    vk_m = _interp_to(m_t, d['cs_t'], d['vego']) * 3.6
    dc_m = _interp_to(m_t, d['ct_t'], d['dc'])
    lat_m = _nearest(m_t, d['cc_t'], d['lat'])
    lc50 = d['lc50']
    v_ms = np.maximum(vk_m / 3.6, 1.0)
    # required curvature to reach lane center offset 'lc50' at 50 m ahead: k ~= 2*y/L^2
    req_curv = 2.0 * lc50 / (50.0 ** 2)
    corner = lat_m & (np.abs(dc_m) > CORNER_CURV_MIN) & (vk_m > 25)
    for i in np.where(corner)[0]:
      if abs(req_curv[i]) > 1e-4:
        corner_ratio.append(abs(dc_m[i]) / abs(req_curv[i]))

    # (3) near lane-crossing
    lbk = _nearest(m_t, d['cs_t'], d['lblink']); rbk = _nearest(m_t, d['cs_t'], d['rblink'])
    off = np.array([s == 'off' for s in d['lcs']])
    base = lat_m & (~lbk) & (~rbk) & (vk_m > 15) & off
    for side, prob, clr in [('L', d['pL'], d['clrL']), ('R', d['pR'], d['clrR'])]:
      hit = base & (prob > 0.4) & (clr < NEAR_CROSS_CLEARANCE_M)
      hruns = np.diff(np.concatenate([[0], hit.view(np.int8), [0]]))
      for a, b in zip(np.where(hruns == 1)[0], np.where(hruns == -1)[0]):
        imn = a + int(np.argmin(clr[a:b]))
        near_events.append((seg_num(path), (m_t[a] - m_t[0]) / 1e9, side, float(clr[imn]), float(vk_m[imn])))

  return dict(wob_op=wob_op, wob_wh=wob_wh, wob_dur=wob_dur,
              corner_ratio=corner_ratio, near_events=near_events)


def _twmean(vals, weights):
  vals = np.array([v for v in vals if not np.isnan(v)])
  if len(vals) == 0:
    return float('nan')
  w = np.array(weights)[:len(vals)] if weights else np.ones(len(vals))
  return float(np.sum(vals * w) / np.sum(w))


def report(route_id, r):
  print(f'\n######## ROUTE {route_id} ########')
  print('-- (1) low-speed wobble (20-50 km/h, hands-off, shallow), 3-7 Hz deg-RMS --')
  print(f'   OP cmd  {_twmean(r["wob_op"], r["wob_dur"]):.3f}°   WHEEL {_twmean(r["wob_wh"], r["wob_dur"]):.3f}°'
        f'   (n={len(r["wob_dur"])} stretches)  [6f-4 win: wheel should stay ~0.05°]')
  cr = np.array([x for x in r['corner_ratio'] if np.isfinite(x)])
  if len(cr):
    print('-- (2) corner under-response: op_curv / required_curv  (<1.0 = under-steer) --')
    print(f'   p50 {np.percentile(cr,50):.2f}  p25 {np.percentile(cr,25):.2f}  '
          f'frac<0.6 {100*np.mean(cr<0.6):.0f}%  (n={len(cr)} corner frames)')
  print('-- (3) near lane-crossings (op-active, blinker off, clearance<0.80 m) worst 8 --')
  for ev in sorted(r['near_events'], key=lambda x: x[3])[:8]:
    seg, rel, side, clr, v = ev
    print(f'   seg{seg:2d} t+{rel:4.1f}s {side}-line  {clr:.2f} m  {v:.0f} km/h')
  print(f'   (total near-crossing events: {len(r["near_events"])})')


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('drivelog_dir', nargs='?', default=DRIVELOG_DIR)
  ap.add_argument('--route', default=None, help='route id e.g. 0x40 / 00000040--eb2be2a919')
  args = ap.parse_args()

  routes = {'00000040--eb2be2a919', '00000041--3e9e6dbdb8'}
  if args.route:
    routes = {next((f[0] for f in FIXTURES if f[0].startswith(args.route.replace('0x', '000000'))), args.route)}

  print('Regression fixtures (POST_6F2_AUDIT §1.D.C):')
  for rid, seg, lbl in FIXTURES:
    print(f'  {rid} seg{seg}: {lbl}')

  for rid in sorted(routes):
    if not route_segments(args.drivelog_dir, rid):
      print(f'\n[skip] no rlogs for {rid} in {args.drivelog_dir}')
      continue
    report(rid, analyze_route(args.drivelog_dir, rid))


if __name__ == '__main__':
  main()
