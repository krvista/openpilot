"""E65: steering quality by speed band, new build vs previous builds. Op-active frames (lat), no blinker / LC within 3 s.
Two hand states: hands-off (hold-compensated driver torque < 40, not pressed) and all.
Weave ('와리가리') = side-to-side oscillation:
  * band-limited RMS (0.2-0.6 Hz and 0.6-2 Hz, zero-phase FFT mask over 20 s windows) of: model action K, controlsd
    command kc, sent TX (as curvature), wheel (as curvature), yaw curvature (liveLocationKalman), lane-centre offset
  * the dominant weave frequency (spectral peak of the wheel 0.15-2 Hz)
  * layer gains in the weave band: cmd/K, TX/cmd, wheel/TX, yaw/wheel (ratios of band RMS) -> which layer injects it
  * lateral jerk p95 (yaw curvature * v^2, 0.1 s differences)
Builds: 98e912d (Phase 57), a0108a4 (56/55c), 31520b9 (Phase 58-60)."""
import glob
import sys
from collections import defaultdict
import numpy as np
sys.path.insert(0, "."); sys.path.insert(0, "opendbc_repo")
from opendbc.car.hyundai.carcontroller import compute_hold_torque  # noqa: E402
S = sys.argv[1]
BANDS = [(30, 40), (40, 55), (55, 70), (70, 90), (90, 200)]
FB = {"lo": (0.2, 0.6), "hi": (0.6, 2.0)}
W = 2000   # 20 s
acc = defaultdict(lambda: defaultdict(list))
for f in sorted(glob.glob(f"{S}/r50/0*.npz")):
  r = f[-12:-4]
  d = dict(np.load(f, allow_pickle=True)); b = str(d['commit'])[:7]
  if b not in ("98e912d", "a0108a4", "31520b9"):
    continue
  try:
    V = np.load(f"{S}/vm/{r}.npz", allow_pickle=True); lk = V['lk']; P = np.load(f"{S}/la/plan_{r}.npz")
  except Exception:  # noqa: BLE001
    continue
  T = d['T']; n = len(T); v = d['v']; vm = v / 3.6
  ky = np.radians(np.interp(T, lk[:, 0], lk[:, 1])) / np.maximum(vm, 1.0)
  K = d['k_model']; kc = d['k_ctrl']; km = d['k_meas']
  m = d['lat'] & (np.abs(kc) > 0.5e-3)
  sc = np.sum(d['apply'][m] * kc[m]) / np.sum(kc[m] ** 2) if m.sum() > 100 else -3000.0
  ktx = d['wire'] / sc; kw = d['ang'] / sc
  lc = 0.5 * (P['L1'] + P['L2']); off20 = np.array([-np.interp(0.0, x, y) for x, y in zip(P['LX'], lc)])
  pr = np.minimum(P['PR'][:, 1], P['PR'][:, 2])
  off = np.interp(T, P['T'], off20); lane_ok = np.interp(T, P['T'], pr) > 0.6
  la = np.abs(vm ** 2 * km)
  hold = np.array([compute_hold_torque(a, c) for a, c in zip(vm[::10], la[::10])]).repeat(10)[:n]
  dd = np.maximum(0.0, np.abs(d['tq_s']) - hold)
  bl = d['lb'] | d['rb'] | (d['lcs'] > 0)
  cs = lambda x: np.concatenate([[0], np.cumsum(x)]); idx = np.arange(n)
  busy = (cs(bl)[np.minimum(idx + 300, n)] - cs(bl)[np.maximum(idx - 300, 0)]) > 0
  base = d['lat'] & ~busy & (v >= 30)
  hoff = base & (dd < 40) & ~d['pressed']
  freqs = np.fft.rfftfreq(W, 0.01)
  for a in range(0, n - W, W // 2):
    if not base[a:a + W].all():
      continue
    bi = next((i for i, (lo, hi) in enumerate(BANDS) if lo <= np.median(v[a:a + W]) < hi), None)
    if bi is None:
      continue
    hs = "off" if hoff[a:a + W].mean() > 0.9 else "on"
    sig = {"K": K, "cmd": kc, "TX": ktx, "wheel": kw, "yaw": ky}
    rec = {}
    for nm, x in sig.items():
      X = np.fft.rfft(x[a:a + W] - np.mean(x[a:a + W]))
      for fb, (f0, f1) in FB.items():
        msk = (freqs >= f0) & (freqs < f1)
        rec[(nm, fb)] = np.sqrt(2 * np.sum(np.abs(X[msk]) ** 2)) / W
    if lane_ok[a:a + W].mean() > 0.8:
      X = np.fft.rfft(off[a:a + W] - np.mean(off[a:a + W]))
      for fb, (f0, f1) in (("olo", (0.05, 0.2)), ("ohi", (0.2, 0.6))):
        msk = (freqs >= f0) & (freqs < f1); rec[("off", fb)] = np.sqrt(2 * np.sum(np.abs(X[msk]) ** 2)) / W
    Xw = np.abs(np.fft.rfft(kw[a:a + W] - np.mean(kw[a:a + W])))
    msk = (freqs >= 0.15) & (freqs < 2.0); rec["fpk"] = freqs[msk][np.argmax(Xw[msk])]
    j = np.abs(np.diff((vm[a:a + W] ** 2 * ky[a:a + W])[::10])) / 0.1
    rec["jerk"] = np.percentile(j, 95)
    rec["curvy"] = np.percentile(np.abs(K[a:a + W]), 90) > 2e-3
    acc[(b, hs)][bi].append(rec)
order = ["a0108a4", "98e912d", "31520b9"]
for hs in ("off", "all"):
  print(f"===== hands {hs}")
  for bi, (lo, hi) in enumerate(BANDS):
    print(f"[{lo}-{hi} km/h]")
    for b in order:
      R = acc[(b, "off")][bi] + (acc[(b, "on")][bi] if hs == "all" else [])
      if len(R) < 3:
        print(f"   {b}: n {len(R)}"); continue
      g = lambda k: np.median([x[k] for x in R if k in x]) if any(k in x for x in R) else np.nan
      line = (f"   {b}: n {len(R):3d} ({len(R) * 10 / 60:4.0f} min) | weave 0.2-0.6 Hz RMS (1e-6/m) K {g(('K', 'lo')) * 1e6:5.1f} cmd {g(('cmd', 'lo')) * 1e6:5.1f} "
              f"TX {g(('TX', 'lo')) * 1e6:5.1f} wheel {g(('wheel', 'lo')) * 1e6:5.1f} yaw {g(('yaw', 'lo')) * 1e6:5.1f} | 0.6-2 Hz TX {g(('TX', 'hi')) * 1e6:5.1f} "
              f"wheel {g(('wheel', 'hi')) * 1e6:5.1f} yaw {g(('yaw', 'hi')) * 1e6:5.1f} | lane offset 0.05-0.2 Hz {g(('off', 'olo')) * 100:4.1f} cm, 0.2-0.6 Hz {g(('off', 'ohi')) * 100:4.1f} cm "
              f"| peak f {g('fpk'):.2f} Hz | lat jerk p95 {g('jerk'):.2f}")
      print(line)
