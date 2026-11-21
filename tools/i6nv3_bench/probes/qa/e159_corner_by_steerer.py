"""E159 (report §96): corner entries and exits by who steers (classes of E156: stock = factory LFA alone, op = openpilot
relayed, mixed = both active), measured on the road as in E151: where does the car sit in the lane while the curve
builds up and while it ends? Hands off from -1 s to +3 s, the whole window in one class, no blinker / lane change within
3 s, both lane lines seen (prob >= 0.6), >= 30 km/h.
  entry: the model action |K| was < 0.5e-3 one second ago, is now >= 1.0e-3 (t = 0) and stays >= 1.0e-3 for 1.5 s
  exit:  |K| was >= 1.5e-3 one second ago (same sign for 1 s), is now <= 0.7e-3 (t = 0) and stays <= 0.7e-3 for 1.5 s
Lane position relative to its value at t = -1 s, positive toward the OUTSIDE of the curve; per class and speed band:
n, median [25-75 %] at 0 / 0.5 / 1.0 / 1.5 / 2.0 / 3.0 s, the peak outside position within 3 s, and for entries the
time from t = 0 until the wheel has moved 50 % of its 3 s change (how late the wheel turns in). argv: S route [route ...]"""
import sys
import numpy as np
sys.path.insert(0, "."); sys.path.insert(0, "opendbc_repo")
from opendbc.car.hyundai.carcontroller import compute_hold_torque  # noqa: E402
S = sys.argv[1]
BANDS = [(30, 40), (40, 55), (55, 70), (70, 200)]; TS = [0, 50, 100, 150, 200, 300]; CL = ["stock", "op", "mixed"]
E = {c: {i: [] for i in range(len(BANDS))} for c in CL}; X = {c: {i: [] for i in range(len(BANDS))} for c in CL}
PK = {c: {i: [] for i in range(len(BANDS))} for c in CL}; T50 = {c: {i: [] for i in range(len(BANDS))} for c in CL}
for r in sys.argv[2:]:
  d = dict(np.load(f"{S}/r50/{r}.npz", allow_pickle=True)); n = len(d['T'])
  if 'cb_req' not in d or n < 6000:
    continue
  try:
    P = dict(np.load(f"{S}/la/plan_{r}.npz"))
  except Exception:  # noqa: BLE001
    continue
  T = d['T']; v = d['v']; vm = v / 3.6; la = np.abs(vm ** 2 * d['k_meas'])
  hold = np.array([compute_hold_torque(a, c) for a, c in zip(vm[::10], la[::10])]).repeat(10)[:n]
  dd = np.maximum(0.0, np.abs(d['tq_s']) - hold); hand = (dd >= 40) | d['pressed'].astype(bool)
  bl = d['lb'] | d['rb'] | (d['lcs'] > 0)
  cs = np.concatenate([[0], np.cumsum(bl)]); idx = np.arange(n)
  busy = (cs[np.minimum(idx + 300, n)] - cs[np.maximum(idx - 300, 0)]) > 0
  j = np.clip(np.searchsorted(P['T'], T), 0, len(P['T']) - 1)
  l1 = np.array([np.interp(0.0, x, y) for x, y in zip(P['LX'], P['L1'])])[j]
  l2 = np.array([np.interp(0.0, x, y) for x, y in zip(P['LX'], P['L2'])])[j]
  pr = np.minimum(P['PR'][:, 1], P['PR'][:, 2])[j]
  offset = -(l1 + l2) / 2                                   # + = the car sits right of the lane centre
  lat = d['lat'].astype(bool); sym = d['lfa_sym'].astype(int)
  cmd = (d['cb_lvl2'].astype(int) == 2) & (d['mdps_active'].astype(int) == 2)
  cls = {"stock": cmd & ~lat & (sym == 2), "op": cmd & lat & (sym != 2), "mixed": cmd & lat & (sym == 2)}
  K = d['k_model']; aK = np.abs(K); wh = d['mdps_ang']
  base = ~busy & ~hand & (pr >= 0.6) & (v >= 30)
  for c in CL:
    good = cls[c] & base; k = 150
    while k < n - 320:
      if good[k - 100:k + 300].all():
        bi = next((i for i, (lo, hi) in enumerate(BANDS) if lo <= v[k] < hi), None)
        if bi is not None and aK[k] >= 1.0e-3 and aK[k - 1] < 1.0e-3 and aK[k - 100] < 0.5e-3 and (aK[k:k + 150] >= 1.0e-3).all():
          s = np.sign(K[k])                                 # K > 0 = right curve: outside is the left = offset going negative
          out = -(offset[k - 100:k + 301] - offset[k - 100]) * s
          E[c][bi].append([out[100 + t] for t in TS]); PK[c][bi].append(out[100:].max())
          dw = (wh[k - 100:k + 301] - wh[k - 100]) * -s      # wheel change into the curve
          if dw[-1] > 0.5:
            T50[c][bi].append((int(np.argmax(dw >= 0.5 * dw[-1])) - 100) * 10)
          k += 300; continue
        if (bi is not None and aK[k] <= 0.7e-3 and aK[k - 1] > 0.7e-3 and aK[k - 100] >= 1.5e-3
            and np.all(np.sign(K[k - 200:k - 100]) == np.sign(K[k - 100])) and (aK[k:k + 150] <= 0.7e-3).all()):
          s = np.sign(K[k - 100])
          out = -(offset[k - 100:k + 301] - offset[k - 100]) * s
          X[c][bi].append([out[100 + t] for t in TS]); k += 300; continue
      k += 1
print("lane position relative to t = -1 s, + = toward the outside of the curve (cm): median [25-75 %] at " + " / ".join(f"{t / 100:g}" for t in TS) + " s")
for i, (lo, hi) in enumerate(BANDS):
  for c in CL:
    for nm, D in (("entry", E[c][i]), ("exit ", X[c][i])):
      if len(D) < 3:
        print(f"  [{lo}-{hi}] {c:5s} {nm} n={len(D)}"); continue
      A = np.array(D) * 100
      line = f"  [{lo}-{hi}] {c:5s} {nm} n={len(D):3d}: " + "  ".join(f"{np.median(A[:, q]):+5.1f} [{np.percentile(A[:, q], 25):+.0f} {np.percentile(A[:, q], 75):+.0f}]" for q in range(len(TS)))
      if nm == "entry":
        line += f"  | peak outside {np.median(np.array(PK[c][i]) * 100):.1f} cm"
        if T50[c][i]:
          line += f" | wheel 50 % at {np.median(T50[c][i]):+.0f} ms"
      print(line)
