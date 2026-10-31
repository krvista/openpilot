"""E37: abrupt 'yank' corrections. Event = measured wheel angle moves >= 3 deg within 0.5 s with a peak wheel rate >= 20 deg/s
(0.1 s differences), op active (lat), >= 30 km/h, no blinker / lane change within 3 s, and the driver NOT steering at that
moment (|tq| < 150 and not pressed over the event). Cause, in order:
  recovery   : MDPS authority (ACI gain) rose by >= 0.3 in the 0.7 s before the onset (the wheel catches up to a command
               it had been held off — typically right after a hand touch / grip release)
  release    : a controlsd restraint switched off within 0.5 s before (41 cap / 6g blend / 39 dropout / 7c / BSM / departure)
  model      : the command moved >= 60 % of the wheel move and the model action moved as much (the plan changed)
  controlsd  : the command moved but the model did not (lead, 7c, other shaping)
  catch-up   : the command did not move — the wheel closed an existing gap to the command (|apply - ang| shrank)
Per event: size (deg), peak rate, lateral-acceleration change (v^2 * dk), the lane offset / heading before it (plan lanes,
when both lines > 0.5) and whether the correction was toward the lane centre. Per build: events per op-hour by cause,
plus the gap |apply - ang| at onset for 'recovery' and 'catch-up'."""
import glob
import sys
from collections import defaultdict
import numpy as np

S = sys.argv[1]
SER = {0: "41gate", 1: "41cap", 2: "41rel", 4: "6g", 5: "drop", 6: "dep", 7: "bsm", 8: "7c"}
agg = defaultdict(lambda: dict(h=0.0, ev=[]))
for f in sorted(glob.glob(f"{S}/r50/0*.npz")):
  d = np.load(f, allow_pickle=True); b = str(d['commit']); r = f[-12:-4]
  T = d['T']; n = len(T); v = d['v']
  if n < 3000:
    continue
  ang = d['ang']; ap = d['apply']; g = d['gain']; tq = np.abs(d['tq_s']); pr = d['pressed']
  K = d['k_model']; kc = d['k_ctrl']; sf = d['sf_ctl'].astype(np.int64)
  bl = d['lb'] | d['rb'] | (d['lcs'] > 0)
  c = lambda m: np.concatenate([[0], np.cumsum(m)])
  idx = np.arange(n)
  win = lambda m, w: (c(m)[np.minimum(idx + w, n)] - c(m)[np.maximum(idx - w, 0)]) > 0
  busy = win(bl, 300)
  act = d['lat'] & (v >= 30) & ~busy
  agg[b]['h'] += act.sum() / 360000
  rate = np.zeros(n); rate[10:] = (ang[10:] - ang[:-10]) / np.maximum(T[10:] - T[:-10], 0.05)
  P = None
  try:
    P = np.load(f"{S}/la/plan_{r}.npz")
  except Exception:
    pass
  i = 100
  while i < n - 60:
    if act[i] and abs(rate[i]) >= 20 and abs(ang[min(i + 40, n - 1)] - ang[max(i - 10, 0)]) >= 3:
      a0 = max(i - 10, 0); a1 = min(i + 40, n - 1)
      if (tq[a0:a1] >= 150).any() or pr[a0:a1].any() or not act[a0:a1].all():
        i += 1; continue
      sgn = np.sign(ang[a1] - ang[a0]); dang = abs(ang[a1] - ang[a0])
      dap = (ap[a1] - ap[a0]) * sgn
      dK = abs(K[a1] - K[a0]); dkc = abs(kc[a1] - kc[a0])
      gup = g[a0] - np.min(g[max(a0 - 70, 0):a0 + 1])
      rel = 0
      for bit in SER:
        x = (sf[max(a0 - 50, 0):a0 + 5] >> bit) & 1
        if x[:-5].any() and not x[-5:].all():
          rel |= 1 << bit
      if gup >= 0.3:
        cause = "recovery"
      elif rel:
        cause = "release:" + "+".join(SER[bb] for bb in SER if rel >> bb & 1)
      elif dap >= 0.6 * dang:
        cause = "model" if dK >= 0.6 * dkc else "controlsd"
      else:
        cause = "catch-up"
      off = head = np.nan; toward = None
      if P is not None:
        j = int(np.searchsorted(P['T'], T[a0]))
        if 0 < j < len(P['T']) and min(P['PR'][j, 1], P['PR'][j, 2]) > 0.5:
          lx = P['LX'][j]; lc = 0.5 * (P['L1'][j] + P['L2'][j])
          off = -float(np.interp(0.0, lx, lc)); head = float((np.interp(5.0, lx, lc) - np.interp(0.0, lx, lc)) / 5.0)
          # wheel angle + = left = toward -y; correction toward the centre if it moves the car opposite to its offset
          toward = bool(np.sign(off) == sgn) if abs(off) > 0.05 else None
      vv = v[i] / 3.6
      dlat = vv ** 2 * abs(kc[a1] - kc[a0]) if cause not in ("recovery", "catch-up") else vv ** 2 * dang / max(abs(ap[a0]) if abs(ap[a0]) > 1 else 15.0, 1) * abs(kc[a0])
      agg[b]['ev'].append(dict(r=r, t=T[i], v=v[i], cause=cause, dang=dang, rate=np.max(np.abs(rate[a0:a1])), gap0=abs(ap[a0] - ang[a0]),
                               gain0=g[a0], gmin=np.min(g[max(a0 - 70, 0):a0 + 1]), off=off, toward=toward, dK=dK, dkc=dkc,
                               last_touch=T[a0] - (T[np.flatnonzero(tq[:a0] >= 150)[-1]] if (tq[:a0] >= 150).any() else -1e9)))
      i = a1 + 50
    else:
      i += 1
  print(r, b, flush=True)
for b, a in agg.items():
  E = a['ev']; h = a['h']
  print(f"== build {b}: op active >= 30 km/h {h * 60:.0f} min, yank events {len(E)} ({len(E) / max(h, 1e-9):.1f}/h)")
  by = defaultdict(list)
  for e in E:
    by[e['cause'].split(':')[0]].append(e)
  for cause, es in sorted(by.items(), key=lambda kv: -len(kv[1])):
    sz = np.array([e['dang'] for e in es]); rt = np.array([e['rate'] for e in es]); gp = np.array([e['gap0'] for e in es])
    tw = [e['toward'] for e in es if e['toward'] is not None]
    lt = np.array([e['last_touch'] for e in es])
    sub = defaultdict(int)
    for e in es:
      sub[e['cause']] += 1
    print(f"   {cause:10} {len(es):3d} ({len(es) / max(h, 1e-9):5.1f}/h) size p50 {np.median(sz):4.1f} p90 {np.percentile(sz, 90):4.1f} deg | rate p50 {np.median(rt):4.0f} "
          f"p90 {np.percentile(rt, 90):4.0f} deg/s | gap at onset p50 {np.median(gp):4.1f} deg | toward centre {sum(tw)}/{len(tw)} | "
          f"last touch p50 {np.median(lt[lt < 1e8]) if (lt < 1e8).any() else float('nan'):4.1f} s before")
    if cause == "release":
      print("      " + ", ".join(f"{k.split(':')[1]} {nn}" for k, nn in sorted(sub.items(), key=lambda kv: -kv[1])))
  for e in sorted(E, key=lambda e: -e['rate'])[:12]:
    print(f"     {e['r']} t {e['t']:7.1f} v {e['v']:5.1f} {e['cause']:18} {e['dang']:4.1f} deg @ {e['rate']:3.0f} deg/s gap0 {e['gap0']:4.1f} gain {e['gmin']:.2f}->{e['gain0']:.2f} "
          f"off {e['off']:+.2f} dK {e['dK'] * 1e3:.2f} dcmd {e['dkc'] * 1e3:.2f} touch {e['last_touch']:.1f}s")
