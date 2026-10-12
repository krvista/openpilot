"""§46 lookahead calibration from a hands-off night drive (arterial + highway). Inputs: route_extract npz (100 Hz) and
yaw_extract npy (liveLocationKalman yaw) for each route, in one directory. Per speed band, on 10 s hands-off windows
(pressed 0, |tq| mean < 50 Nm, sent gain mean >= 0.9, lat active, no blinker / lane change, curvature range >= 2e-3):
  L_act   : controlsd command (post-LP desired curvature) -> wheel curvature lag (NCC peak)
  L_chas  : wheel -> yaw-rate curvature lag (any hands)
  L_resid : model action -> wheel lag (the model assumes 0.375 s)
Implied base+boost = plan age (0.08 s) + L_act + L_chas, compared with the current tables (base_s(v) + boost(|k|) <= 0.32).
Also prints the window yield per band and the per-window std, so the precision of the implied value is visible.
Usage: night_calib.py <dir>   (dir has 0000XXXX.npz and yaw_0000XXXX.npy)"""
import glob, sys
import numpy as np
S = sys.argv[1]; B = [(20, 40), (40, 60), (60, 80), (80, 100), (100, 200)]
AGE = 0.08; WIN = 1000
def ncc_lag(a, b, maxlag=120):
  a = a - a.mean(); b = b - b.mean()
  if np.linalg.norm(a) < 1e-9 or np.linalg.norm(b) < 1e-9: return None, 0
  cs = np.array([np.dot(a[:len(a) - L], b[L:]) / (np.linalg.norm(a[:len(a) - L]) * np.linalg.norm(b[L:]) + 1e-12) for L in range(maxlag + 1)])
  L = int(np.argmax(np.abs(cs))); return L, abs(cs[L])
def interp_v(v, bp, fp): return float(np.interp(v, bp, fp))
R = {b: dict(act=[], chas=[], resid=[], tahead=[], n=0) for b in B}
for f in sorted(glob.glob(f"{S}/0000*.npz")):
  d = np.load(f, allow_pickle=True); r = f[-12:-4]; T = d['T']; v = d['v'] / 3.6
  try:
    Y = np.load(f"{S}/yaw_{r}.npy"); ok = Y[:, 2] > 0.5
    yaw = np.interp(T, Y[ok, 0], Y[ok, 1]); yvalid = np.interp(T, Y[:, 0], Y[:, 2]) > 0.99
  except FileNotFoundError:
    yaw = np.zeros(len(T)); yvalid = np.zeros(len(T), bool)
  kw = -np.radians(d['ang']); kc = d['k_ctrl']; km = d['k_model']; ky = -np.radians(yaw) / np.maximum(v, 1.0)
  base = d['lat'] & ~(d['lb'] | d['rb']) & (d['lcs'] == 0)
  i = 0
  while i + WIN <= len(T):
    s = slice(i, i + WIN); vv = v[s].mean() * 3.6
    for b in B:
      if not (b[0] <= vv < b[1]) or ((v[s] * 3.6 >= b[0]) & (v[s] * 3.6 < b[1])).mean() < 0.95: continue
      if base[s].mean() > 0.98 and yvalid[s].mean() > 0.98 and (kw[s].max() - kw[s].min()) >= 2e-3:
        L, c = ncc_lag(kw[s], ky[s], 60)
        if L is not None and c > 0.9: R[b]['chas'].append(L * 0.01)
      if (base[s].mean() > 0.98 and d['pressed'][s].mean() == 0 and np.abs(d['tq_s'][s]).mean() < 50 and d['gain'][s].mean() >= 0.9
          and (kc[s].max() - kc[s].min()) >= 2e-3):   # window-mean hands-off (a one-frame gain dip must not drop a 10 s window)
        R[b]['n'] += 1
        La, ca = ncc_lag(kc[s], kw[s]); Lr, cr = ncc_lag(km[s], kw[s])
        if La is not None and ca > 0.9 and 0 < La < 120: R[b]['act'].append(La * 0.01)
        if Lr is not None and cr > 0.9 and 0 < Lr < 120: R[b]['resid'].append(Lr * 0.01)
        ab = np.abs(kc[s]).mean()
        R[b]['tahead'].append(min(interp_v(v[s].mean(), [5.6, 13.9, 27.8, 38.9], [0.08, 0.10, 0.13, 0.18]) + interp_v(ab, [0.0008, 0.005], [0.0, 0.20]), 0.32))
    i += WIN // 2
def q(a):
  a = np.array(a); return (np.median(a), np.percentile(a, 25), np.percentile(a, 75), a.std(), len(a)) if len(a) else (np.nan,) * 4 + (0,)
print(f"{'band':9} {'win':>4} | {'L_act p50 (p25-p75) std n':30} | {'L_chas p50 n':14} | {'L_resid p50 n (model 0.375)':28} | {'implied base+boost':19} | {'current':8}")
for b in B:
  a = q(R[b]['act']); c = q(R[b]['chas']); x = q(R[b]['resid']); cur = np.median(R[b]['tahead']) if R[b]['tahead'] else np.nan
  implied = AGE + a[0] + (c[0] if not np.isnan(c[0]) else 0.08)
  print(f"{b[0]:>3}-{b[1]:<4}  {R[b]['n']:4d} | {a[0]:.2f} ({a[1]:.2f}-{a[2]:.2f}) {a[3]:.2f} {a[4]:3d}       | {c[0]:.2f} {c[4]:4d}      | {x[0]:.2f} {x[4]:3d}                      | {implied:.2f} +-{(a[3]/max(np.sqrt(a[4]),1))*2:.2f}       | {cur:.2f}")
print("implied = plan age 0.08 + L_act + L_chas (chassis 0.08 assumed where unmeasured); +- is 2 SE of the L_act median proxy.")
print("Judge only bands with n >= 20 and std <= 0.12. A clear implied > current by > 0.05 s across bands = raise LOOKAHEAD_T_AHEAD_CAP / base_s one step on-road.")
