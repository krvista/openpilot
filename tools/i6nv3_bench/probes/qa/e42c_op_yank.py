"""E42: op-initiated yanks (the driver grabs BECAUSE of them). Onset = the measured wheel starts moving >= 15 deg/s (0.1 s
difference) and moves >= 2 deg within 0.3 s, op active (lat), >= 30 km/h, no blinker / lane change within 3 s, and the
driver quiet before it (|tq| < 100 and not pressed for the 1.5 s before the onset). Reaction = |tq| >= 200 within 1.5 s
after the onset (grab). For each onset, what moved FIRST, looking back 1 s:
  model     : the model action K changed by >= 1e-3 in the 0.5 s before the onset
  controlsd : the controlsd command (k_ctrl / its angle 'apply') stepped while K did not (shaping: 7c, lead, release ramps)
  carctl    : the TX angle (wire) moved >= 1.5 deg more than the controlsd command over the 0.3 s (carcontroller shaping:
              trim, boost, recovery) or the ACI gain jumped >= 0.3 just before (authority step)
Per build: onsets per hour, grabbed share, by cause; flags present at onset; speed, lane prob, road curvature; and the
largest examples."""
import glob
import sys
from collections import Counter, defaultdict
import numpy as np

S = sys.argv[1]
SER = {0: "41gate", 1: "41cap", 2: "41rel", 4: "6g", 5: "drop", 6: "dep", 7: "bsm", 8: "7c", 9: "lead", 10: "budget"}
CCN = {12: "fast", 15: "accelY", 16: "boostSup", 17: "blindC", 18: "city39a", 19: "anch35b", 20: "blinkC", 21: "post29", 25: "outrun", 27: "txSat", 28: "hod51"}
agg = defaultdict(lambda: dict(h=0.0, ev=[]))
for f in sorted(glob.glob(f"{S}/r50/0*.npz")):
  d = np.load(f, allow_pickle=True); b = str(d['commit']); r = f[-12:-4]
  T = d['T']; n = len(T); v = d['v']
  if n < 3000:
    continue
  ang = d['ang']; ap = d['apply']; wire = d['wire']; g = d['gain']; tq = np.abs(d['tq_s']); pr = d['pressed']
  K = d['k_model']; kc = d['k_ctrl']; sf = d['sf_ctl'].astype(np.int64); sc = d['sf_cc'].astype(np.int64); lm = d['lane_min']
  bl = d['lb'] | d['rb'] | (d['lcs'] > 0)
  c = lambda m: np.concatenate([[0], np.cumsum(m)]); idx = np.arange(n)
  busy = (c(bl)[np.minimum(idx + 300, n)] - c(bl)[np.maximum(idx - 300, 0)]) > 0
  act = d['lat'] & (v >= 30) & ~busy
  agg[b]['h'] += act.sum() / 360000
  loud = (tq >= 200) | pr; lc = c(loud)
  rate = np.zeros(n); rate[10:] = (ang[10:] - ang[:-10]) / np.maximum(T[10:] - T[:-10], 0.05)
  i = 200
  while i < n - 160:
    if act[i] and abs(rate[i]) >= 15 and abs(ang[min(i + 30, n - 1)] - ang[i - 10]) >= 2 and lc[i] - lc[i - 150] == 0 and act[i - 150:i + 30].all():
      a0 = i - 10; a1 = i + 30
      sgn = np.sign(ang[a1] - ang[a0])
      grab = bool((tq[i:i + 150] >= 300).any())
      t_grab = (np.argmax(tq[i:i + 150] >= 300) * 0.01) if grab else np.nan
      dK = abs(K[i] - K[i - 50]); dkc = abs(kc[i] - kc[i - 50])
      dap = (ap[a1] - ap[a0 - 20]) * sgn; dwire = (wire[a1] - wire[a0 - 20]) * sgn
      gjump = g[i] - np.min(g[i - 50:i + 1])
      fl_ctl = int(np.bitwise_or.reduce(sf[i - 50:i + 5])); fl_cc = int(np.bitwise_or.reduce(sc[i - 50:i + 5]))
      led = (wire[i] - wire[i - 30]) * sgn >= 1.0 or (ap[i] - ap[i - 30]) * sgn >= 1.0
      if not led:
        cause = "wheel first (hand / road)"
      elif dK >= 1e-3:
        cause = "model"
      elif dkc >= 0.8e-3 or dap >= 1.5:
        cause = "controlsd"
      elif dwire - dap >= 1.5 or gjump >= 0.3:
        cause = "carctl"
      else:
        cause = "unexplained"
      tq_s = d['tq_s']
      if grab:
        k = i + int(np.argmax(tq[i:i + 150] >= 300)); against = np.sign(tq_s[k]) != sgn   # torque + = left = angle +
      else:
        against = None
      agg[b]['ev'].append(dict(against=against, r=r, t=T[i], v=v[i], cause=cause, grab=grab, t_grab=t_grab, size=abs(ang[a1] - ang[a0]),
                               rate=np.max(np.abs(rate[a0:a1 + 10])), dK=dK, dkc=dkc, dap=dap, dwire=dwire, gj=gjump, g0=g[i],
                               ctl=fl_ctl, cc=fl_cc, lm=lm[i], K=K[i], away=(abs(ang[a1]) > abs(ang[a0]))))
      i = a1 + 100
    else:
      i += 1
  print(r, b, flush=True)
for b, a in agg.items():
  E = a['ev']; h = a['h']
  G = [e for e in E if e['grab']]
  print(f"== build {b}: op active >= 30 km/h {h * 60:.0f} min | wheel moves with a quiet/resting hand {len(E)} ({len(E) / h:.0f}/h), "
        f"followed by a grab {len(G)} ({len(G) / h:.1f}/h, {len(G) / max(len(E), 1):.0%})")
  for cause in ("model", "controlsd", "carctl", "unexplained", "wheel first (hand / road)"):
    es = [e for e in E if e['cause'] == cause]; gs = [e for e in es if e['grab']]
    if not es:
      continue
    fl = Counter()
    for e in gs:
      for bit, nm in SER.items():
        if e['ctl'] >> bit & 1 and nm != "lead":
          fl[nm] += 1
      for bit, nm in CCN.items():
        if e['cc'] >> bit & 1:
          fl[nm] += 1
    print(f"   {cause:11} {len(es):4d} ({len(es) / h:5.1f}/h) grabbed {len(gs):3d} ({len(gs) / max(len(es), 1):4.0%}) | grabbed: size p50 "
          f"{np.median([e['size'] for e in gs]) if gs else 0:4.1f} deg rate p50 {np.median([e['rate'] for e in gs]) if gs else 0:3.0f} deg/s, grab after "
          f"{np.median([e['t_grab'] for e in gs]) if gs else 0:.2f} s, v p50 {np.median([e['v'] for e in gs]) if gs else 0:.0f}, lane_min p50 {np.median([e['lm'] for e in gs]) if gs else 0:.2f} | "
          + ", ".join(f"{k} {nn}" for k, nn in fl.most_common(6)))
  op = [e for e in G if e['cause'] in ("model", "controlsd", "carctl", "unexplained")]
  print(f"   op-led yanks followed by a grab: {len(op)} | driver pushed AGAINST the yank {sum(1 for e in op if e['against'])}, WITH it {sum(1 for e in op if e['against'] is False)}")
  for e in sorted(G, key=lambda e: -e['rate'])[:12]:
    print(f"     {e['r']} t {e['t']:7.1f} v {e['v']:4.0f} {e['cause']:11} {e['size']:4.1f} deg @ {e['rate']:3.0f} deg/s grab +{e['t_grab']:.2f}s | dK {e['dK'] * 1e3:.2f} "
          f"{'AGAINST' if e['against'] else 'with   '} dcmd {e['dkc'] * 1e3:.2f} dapply {e['dap']:+.1f} dTX {e['dwire']:+.1f} gain {e['g0']:.2f} (+{e['gj']:.2f}) lm {e['lm']:.2f} K {e['K'] * 1e3:+.1f}")
