"""E36: dropout holds followed by a driver takeover (|tq| >= 200 within 1.5 s of the hold start): the human then drives,
so the car's curvature (k_meas) over the 1 s after the takeover onset is a human-chosen path. Compare the model action K
and the held command at the hold start against that path (median |.| over the window), per build and |K - held| band."""
import glob
import sys
import numpy as np

S = sys.argv[1]
out = {}
for f in sorted(glob.glob(f"{S}/r50/0*.npz")):
  d = np.load(f, allow_pickle=True); b = str(d['commit'])
  T = d['T']; n = len(T); v = d['v']
  sf = d['sf_ctl'].astype(np.int64); K = d['k_model']; kc = d['k_ctrl']; km = d['k_meas']; tq = d['tq_s']
  drop = ((sf >> 5) & 1).astype(bool) & d['lat'] & (v >= 25)
  st = np.flatnonzero(drop[1:] & ~drop[:-1]) + 1
  for s0 in st:
    g = np.abs(tq[s0:min(s0 + 150, n)]) >= 200
    if not g.any():
      continue
    t0 = s0 + int(np.argmax(g)); w = slice(t0, min(t0 + 100, n))
    held = kc[s0]
    eK = np.median(np.abs(K[w] - km[w])); eH = np.median(np.abs(held - km[w]))
    out.setdefault(b, []).append((abs(np.median(K[w]) - held), eK, eH, f[-12:-4], T[s0]))
for b, rows in out.items():
  R = np.array([r[:3] for r in rows])
  print(f"== build {b}: holds with a driver takeover {len(R)}")
  for lo, hi in ((0, 1e-3), (1e-3, 3e-3), (3e-3, 1)):
    m = (R[:, 0] >= lo) & (R[:, 0] < hi)
    if m.sum():
      print(f"   |model - held| {lo * 1e3:.0f}-{min(hi, 0.1) * 1e3:.0f}e-3: n {m.sum():2d} | model closer to the human path in {np.mean(R[m, 1] < R[m, 2]):.0%} | "
            f"median error model {np.median(R[m, 1]) * 1e3:.2f} vs held {np.median(R[m, 2]) * 1e3:.2f}e-3")
