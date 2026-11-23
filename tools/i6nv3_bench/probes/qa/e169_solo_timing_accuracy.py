"""E169 (report §106): when openpilot's lateral control acts and how accurately, per solo drive (route npz v5 / v6 +
la/plan_<route>.npz). 'hands off' = relayed, no steeringPressed within the last 1 s, no blinker.
TIMING   1 launches from a stop: speed / time / distance at the first request   2 authority back after a driver press
         3 request -> wheel lag (cross-correlation of rates) and gain            4 transmitted angle behind the command, by cause
         5 command -> wheel -> yaw total lag in curves
ACCURACY 6 lane position (model lines; + = right of centre) mean / scatter on straights by speed, and inside / outside
           in curves   7 angle and curvature tracking   8 wheel activity on straights (weave)
argv: S route [route ...]"""
import re
import sys
import numpy as np
S = sys.argv[1]
src = open("opendbc_repo/opendbc/car/hyundai/carcontroller.py").read()
SF = {m.group(1): int(m.group(2)) for m in re.finditer(r"^(SF_[A-Z_]+)\s*=\s*1 << (\d+)", src, re.M)}
BANDS = [(30, 45), (45, 60), (60, 75), (75, 200)]

def runs(m):
  e = np.flatnonzero(np.diff(np.r_[0, m.astype(np.int8), 0]))
  return list(zip(e[::2], e[1::2]))

def lag(a, b, lo=-20, hi=80):   # samples by which b lags a (100 Hz)
  a = a - a.mean(); b = b - b.mean(); best = (0, -2)
  for k in range(lo, hi):
    c = np.corrcoef(a[max(0, -k):len(a) - max(k, 0)], b[max(k, 0):len(b) - max(0, -k)])[0, 1]
    if c > best[1]: best = (k, c)
  return best

for r in sys.argv[2:]:
  d = dict(np.load(f"{S}/r50/{r}.npz", allow_pickle=True)); T = d['T']; n = len(T); v = d['v']
  pl = np.load(f"{S}/la/plan_{r}.npz", allow_pickle=True)
  lat = d['lat'].astype(bool); req = d['tx_active'].astype(int) == 2; cmd = d['cb_lvl2'].astype(int) == 2
  pr = d['pressed'].astype(bool); bl = (d['lb'] | d['rb']).astype(bool); sf = d['sf_cc'].astype(np.int64)
  recent = np.convolve(pr | bl, np.ones(100), 'full')[:n] > 0
  ho = req & cmd & ~recent
  print(f"\n===== {r} (commit {str(d['commit'])[:8]}): {T[-1]/60:.1f} min, moving {np.sum(v>=1)/6000:.1f}, requesting {req.sum()/6000:.1f}, hands off {ho.sum()/6000:.1f} min")
  # 1 launches
  rows = []
  for a, b in runs(v < 1.0):
    if b >= n - 500 or b - a < 200: continue
    seg = slice(b, min(b + 3000, n)); k = np.flatnonzero(req[seg])
    lon = np.flatnonzero(lat[seg])
    if len(k) and v[seg].max() > 30:
      i = b + k[0]; rows.append((v[i], (i - b) / 100, np.sum(v[b:i]) / 360, (lon[0] / 100 if len(lon) else np.nan), pr[b:i].mean()))
  if rows:
    q = np.array(rows)
    print(f" 1 launches n={len(q)}: first request at {np.median(q[:,0]):.0f} km/h (p25 {np.percentile(q[:,0],25):.0f}, p75 {np.percentile(q[:,0],75):.0f}), "
          f"{np.median(q[:,1]):.1f} s / {np.median(q[:,2]):.0f} m after the start (max {q[:,2].max():.0f} m); lateral on after {np.nanmedian(q[:,3]):.1f} s; driver pressing {q[:,4].mean()*100:.0f} % of that time")
  # 2 authority back after a press (v >= 30)
  back = []
  for a, b in runs(pr & lat & (v >= 30)):
    if b - a < 30 or b + 500 >= n or bl[a:b + 300].any(): continue
    ok = req[b:b + 500] & (np.abs(d['apply'][b:b + 500] - d['wire'][b:b + 500]) < 1.0) & (d['gain'][b:b + 500] > 0.8)
    k = np.flatnonzero(ok); back.append(k[0] / 100 if len(k) else 5.0)
  if back:
    print(f" 2 after a driver press (n={len(back)}): request within 1 deg of the command at gain > 0.8 after p50 {np.median(back):.2f} s, p90 {np.percentile(back,90):.2f} s")
  # 3 request -> wheel
  for lo, hi in BANDS:
    m = ho & (v >= lo) & (v < hi)
    if m.sum() < 3000: continue
    ks = []
    for a, b in runs(m):
      if b - a >= 500 and np.std(np.diff(d['wire'][a:b])) > 0.003:
        ks.append(lag(np.diff(d['wire'][a:b]), np.diff(d['mdps_ang'][a:b]))[0])
    e = np.abs(d['wire'] - d['mdps_ang'])[m]
    print(f" 3 [{lo}-{hi}] {m.sum()/100:5.0f} s: request -> wheel lag {np.median(ks)*10 if ks else float('nan'):.0f} ms (n={len(ks)}), |request - wheel| p50 {np.median(e):.2f} p90 {np.percentile(e,90):.2f} deg, gain p10 {np.percentile(d['gain'][m],10):.2f}")
  # 4 behind the command
  beh = np.abs(d['apply'] - d['wire'])
  m = ho & (v >= 30)
  if m.sum():
    b1 = m & (beh > 1.0); tot = b1.sum() / 100
    def bit(k): return ((sf >> SF[k]) & 1) > 0 if k in SF else np.zeros(n, bool)
    since_anchor = np.zeros(n, int); c = 10**6
    for i in range(n):
      c = 0 if not req[i] else c + 1; since_anchor[i] = c
    press3 = np.convolve(pr, np.ones(300), 'full')[:n] > 0
    cause = {"TX rate limit": bit('SF_TX_SATURATED'), "within 1.5 s of a request start": since_anchor < 150, "within 3 s of a driver press": press3,
             "lane change / low lane confidence": (d['lcs'] > 0) | (d['lane_min'] < 0.3)}
    left = b1.copy(); parts = []
    for nm, mm in cause.items():
      parts.append(f"{nm} {np.sum(left & mm)/100:.1f}"); left &= ~mm
    print(f" 4 hands off >= 30 km/h {m.sum()/100:.0f} s: transmitted angle > 1 deg behind the command {tot:.1f} s ({tot/m.sum()*10000:.1f} %): " + ", ".join(parts) + f", other {left.sum()/100:.1f} s; TX-saturated flag {np.sum(m & bit('SF_TX_SATURATED'))/100:.1f} s")
  # 5 total lag in curves: desired curvature -> measured curvature
  m = ho & (v >= 40)
  ks = []
  for a, b in runs(m):
    if b - a >= 800 and np.ptp(d['k_ctrl'][a:b]) > 1.5e-3:
      ks.append(lag(d['k_ctrl'][a:b], d['k_meas'][a:b], -20, 120))
  if ks:
    print(f" 5 curves (n={len(ks)}): controlsd curvature -> measured curvature lag p50 {np.median([k for k,_ in ks])*10:.0f} ms (corr p50 {np.median([c for _,c in ks]):.2f})")
  vm = np.load(f"{S}/vm/{r}.npz", allow_pickle=True); ld = vm['ld']
  if len(ld): print(f"   lagd lateralDelay: start {ld[0][1]:.3f} end {ld[-1][1]:.3f} s") if np.ndim(ld) == 2 else None
  # 6 lane position from the model lines at the car (20 Hz)
  Tp = pl['T']; off = -(pl['L1'][:, 0] + pl['L2'][:, 0]) / 2; width = pl['L2'][:, 0] - pl['L1'][:, 0]; prob = np.minimum(pl['PR'][:, 1], pl['PR'][:, 2])
  idx = np.clip(np.searchsorted(T, Tp), 0, n - 1); K = d['k_model'][idx]; ho20 = ho[idx]; v20 = v[idx]
  good = ho20 & (prob > 0.5) & (width > 2.5) & (width < 4.6)
  print(" 6 lane position, model lines, cm (+ = right of centre); straight = |curvature| < 0.5e-3")
  for lo, hi in BANDS:
    m = good & (v20 >= lo) & (v20 < hi)
    st = m & (np.abs(K) < 0.5e-3); lc = m & (K < -1e-3); rc = m & (K > 1e-3)
    if st.sum() < 200: continue
    hp = off[st] - np.convolve(off[st], np.ones(200) / 200, 'same')
    s = f"   [{lo}-{hi}] straights {st.sum()/20:5.0f} s: mean {off[st].mean()*100:+5.1f}, scatter {off[st].std()*100:4.1f} (10 s high-pass {hp.std()*100:4.1f})"
    if lc.sum() > 100: s += f" | left curves {lc.sum()/20:.0f} s: {off[lc].mean()*100:+5.1f}"
    if rc.sum() > 100: s += f" | right curves {rc.sum()/20:.0f} s: {off[rc].mean()*100:+5.1f}"
    if lc.sum() > 100 and rc.sum() > 100: s += f" -> inside cut {(off[rc].mean() - off[lc].mean())*50:+.1f}"
    print(s)
  # 7 tracking, 8 weave
  for lo, hi in BANDS:
    m = ho & (v >= lo) & (v < hi)
    if m.sum() < 3000: continue
    ke = (d['k_meas'] - d['k_ctrl'])[m]; vv = (v[m] / 3.6) ** 2
    st = m & (np.abs(d['k_model']) < 0.5e-3)
    w = d['mdps_ang']; hpw = w - np.convolve(w, np.ones(300) / 300, 'same')
    print(f" 7 [{lo}-{hi}] curvature error (measured - commanded) rms {np.sqrt(np.mean(ke**2))*1e3:.2f}e-3 = {np.sqrt(np.mean((ke*vv)**2)):.2f} m/s2, mean {ke.mean()*1e3:+.2f}e-3 | 8 wheel activity on straights (3 s high-pass std) {hpw[st].std():.2f} deg, rate rms {np.sqrt(np.mean(np.gradient(w, 0.01)[st]**2)):.1f} deg/s")
