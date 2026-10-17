"""E28: is holding the command back in LOW-confidence CURVES right or wrong? Frames (all Phase 46+ builds, 100 Hz) where the
Phase 41 cap (bit 1) or the 6g blend (bit 4) acted while the model action |K| >= K_MIN, op active >= 25 km/h, no
blinker/LC. Truth = the car's curvature 0.6 s later from the wheel (k_meas, includes any driver correction). Compare
|K - truth| vs |k_ctrl - truth|: if the model action is closer, the hold was harmful. Also the driver's reaction in the
2 s after: torque >= 200 onsets per hour in such frames vs elsewhere, and the sign of the driver torque vs the model's
direction (with = the driver helped the model's turn)."""
import glob
import sys
import numpy as np

S = sys.argv[1]; K_MIN = float(sys.argv[2]) if len(sys.argv) > 2 else 2e-3
tot = {}
for f in sorted(glob.glob(f"{S}/r3d/0*.npz") + glob.glob(f"{S}/r43/0*.npz") + glob.glob(f"{S}/r46/0*.npz") + glob.glob(f"{S}/r50/0*.npz")):
  d = np.load(f, allow_pickle=True); b = str(d['commit'])
  n = len(d['T']); v = d['v']; sf = d['sf_ctl'].astype(int); K = d['k_model']; kc = d['k_ctrl']; km = d['k_meas']; tq = d['tq_s']
  bl = d['lb'] | d['rb'] | (d['lcs'] > 0)
  c = np.concatenate([[0], np.cumsum(bl)]); idx = np.arange(n)
  busy = (c[np.minimum(idx + 300, n)] - c[np.maximum(idx - 300, 0)]) > 0
  act = d['lat'] & (v >= 25) & ~busy
  held = act & (((sf >> 1) & 1) | ((sf >> 4) & 1)).astype(bool) & (np.abs(K) >= K_MIN) & (np.abs(kc - K) > 0.5e-3)
  fut = np.concatenate([km[60:], np.full(60, np.nan)])
  st = tot.setdefault(b, dict(act=0, held=0, eK=[], eC=[], grab_h=0, grab_o=0, with_=0, against=0))
  st['act'] += act.sum(); st['held'] += held.sum()
  ok = held & np.isfinite(fut)
  st['eK'].append(np.abs(K - fut)[ok]); st['eC'].append(np.abs(kc - fut)[ok])
  big = np.abs(tq) >= 200
  on = np.flatnonzero(big[1:] & ~big[:-1]) + 1
  hc = np.concatenate([[0], np.cumsum(held)])
  for i in on:
    if not act[i]:
      continue
    if hc[i] - hc[max(i - 200, 0)] > 0:        # a held frame in the 2 s before
      st['grab_h'] += 1
      j = i - 200 + int(np.argmax(held[max(i - 200, 0):i])) if i >= 200 else 0
      # angle = -curvature; driver torque > 0 = left = negative curvature direction
      if np.sign(tq[i]) == -np.sign(K[j]):
        st['with_'] += 1
      else:
        st['against'] += 1
    else:
      st['grab_o'] += 1
for b, st in tot.items():
  eK = np.concatenate(st['eK']); eC = np.concatenate(st['eC'])
  hh = st['held'] / 360000; ho = (st['act'] - st['held']) / 360000
  print(f"{b}: held-in-curve frames {st['held'] / 100:.0f} s ({st['held'] / max(st['act'], 1):.2%} of active) | |model - car@+0.6s| p50 {np.median(eK) * 1e3:.2f} "
        f"vs |held cmd - car| {np.median(eC) * 1e3:.2f} e-3 (model closer in {np.mean(eK < eC):.0%}) | driver grabs after a hold {st['grab_h'] / max(hh, 1e-9):.0f}/h "
        f"(with the model {st['with_']}, against {st['against']}) vs elsewhere {st['grab_o'] / max(ho, 1e-9):.0f}/h")
