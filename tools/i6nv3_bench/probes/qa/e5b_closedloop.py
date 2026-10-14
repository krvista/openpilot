"""E5: closed-loop lane-keeping simulation with model behaviour IDENTIFIED from the drive data.
1) Identification (good frames, lat active, >= 30 km/h, per speed band): each predictor p in {action K, cubic lead sample
   (Phase 41-2 geometry), orient k_or(0.9+tau), orient k_or(0.3)} is regressed on road curvature at the car now and at
   +0.3/0.6/0.9/1.2/1.5 s, the car's offset from lane centre and the lane heading (lane-centre fit 0-20 m: c0, c1).
   Residual std is kept as an AR(1) noise (lag-1 autocorr measured) — the model's non-linear / unexplained part.
2) Simulation on the recorded road: road curvature and speed sequences from the same drives (continuous good stretches
   >= 30 s). States: offset e (m), lane heading c1. Each 0.05 s the "model" emits the predictors from the identified
   map + noise, controlsd forms the command (variants), LP tau(v), actuator = pure delay 0.30 s + first-order 0.10 s with
   steady gain 0.90 (E3), integrated at 100 Hz. e' = -v c1, c1' = v (k_road - k_car).
3) Metrics: offset RMS (straight / curve), p95 |offset|, heading RMS, command 20 Hz change std, lateral jerk proxy,
   offset spectral peak 0.1-1 Hz (weaving)."""
import sys
import numpy as np
sys.path.insert(0, sys.argv[0].rsplit('/', 1)[0])
from common import routes, T_IDXS, bfit, interp_rows

S = sys.argv[1]
FUT = [0.0, 0.3, 0.6, 0.9, 1.2, 1.5]
BANDS = [(30, 55), (55, 80), (80, 200)]
data = []
for r, P, d, T, f in routes(S):
  n = len(T); v = f['v']
  K = P['K'].astype(float)
  OR, VX = P['OR'].astype(float), P['VX'].astype(float)
  kor_t = lambda t: np.array([np.interp(t[i], T_IDXS, OR[i]) / max(np.interp(t[i], T_IDXS, VX[i]), 1.0) for i in range(n)])
  tau = np.interp(v, [8.0, 13.0, 18.0], [0.30, 0.12, 0.08])
  k9 = kor_t(0.9 + tau); k3 = kor_t(np.full(n, 0.3)); k6 = kor_t(0.6 + tau); k5 = kor_t(0.45 + tau)
  C3 = bfit(P['PX'][:, :12].astype(float), P['PY'][:, :12].astype(float), 3)
  ab = np.abs(K)
  t_ah = np.minimum(np.interp(v, [5.6, 13.9, 27.8, 38.9], [0.08, 0.10, 0.13, 0.18]) + np.interp(ab, [0.0008, 0.005], [0.0, 0.20]), 0.32) + tau
  kcub = 6.0 * C3[:, 0] * np.minimum(v * t_ah, 10.0) + 2.0 * C3[:, 1]
  LX = P['LX'].astype(float); LC = 0.5 * (P['L1'].astype(float) + P['L2'].astype(float))
  xs = np.tile(np.linspace(0, 20, 9), (n, 1))
  cf = bfit(xs, interp_rows(xs, LX, LC), 2)                 # [c2, c1, c0]
  kr = 2.0 * cf[:, 0]
  futs = []
  for dt in FUT:
    j = np.minimum(np.searchsorted(T, T + dt), n - 1)
    futs.append(np.where(np.abs(T[j] - (T + dt)) < 0.03, kr[j], np.nan))
  F = np.stack(futs, 1)
  good = f['good'] & f['lat'] & np.all(np.isfinite(F), 1)
  data.append(dict(T=T, v=v, K=K, kcub=kcub, k9=k9, k3=k3, k6=k6, k5=k5, F=F, e=-cf[:, 2], c1=cf[:, 1], kr=kr, good=good, ab=ab,
                   segok=(P['PR'][:, 1] > 0.6) & (P['PR'][:, 2] > 0.6) & (f['width'] > 2.6) & (f['width'] < 4.2) & (v * 3.6 >= 30) & np.isfinite(kr)))
  print(r, flush=True)

# 1) identification
PRED = ("K", "kcub", "k9", "k3", "k6", "k5")
ident = {}
for b in BANDS:
  rows = {p: [] for p in PRED}; X = []; nxt = {p: [] for p in PRED}
  for dd in data:
    s = dd['good'] & (dd['v'] * 3.6 >= b[0]) & (dd['v'] * 3.6 < b[1])
    s1 = s[:-1] & s[1:] & (np.diff(dd['T']) < 0.07)
    X.append(np.column_stack([dd['F'][s], dd['e'][s], dd['c1'][s], np.ones(s.sum())]))
    for p in PRED:
      rows[p].append(dd[p][s])
  X = np.concatenate(X)
  ident[b] = {}
  for p in PRED:
    y = np.concatenate(rows[p])
    co = np.linalg.lstsq(X, y, rcond=None)[0]
    res = y - X @ co
    ident[b][p] = (co, np.std(res))
    print(f"band {b} {p:5} n {len(y)} R2 {1 - np.var(res) / np.var(y):.2f} road(now..1.5s) " +
          " ".join(f"{c * 1:+.2f}" for c in co[:len(FUT)]) + f"  offset {co[len(FUT)] * 1e3:+.2f}e-3/m  heading {co[len(FUT) + 1] * 1e3:+.1f}e-3  res {np.std(res) * 1e3:.3f}e-3")
# residual correlation between predictors (same frame) and lag-1 autocorr, pooled
R = {p: [] for p in PRED}
for dd in data:
  s = dd['good']
  for p in PRED:
    b_idx = np.digitize(dd['v'] * 3.6, [55, 80])
    pr = np.zeros(len(dd['v']))
    for bi, b in enumerate(BANDS):
      m = b_idx == bi
      co = ident[b][p][0]
      Xb = np.column_stack([dd['F'][m], dd['e'][m], dd['c1'][m], np.ones(m.sum())])
      pr[m] = Xb @ co
    R[p].append(np.where(s, dd[p] - pr, np.nan))
Rc = {p: np.concatenate(R[p]) for p in PRED}
ok = np.all([np.isfinite(Rc[p]) for p in PRED], 0)
RES = np.stack([Rc[p][ok] for p in PRED], 1)
COV = np.cov(RES.T)
ok1 = ok[1:] & ok[:-1]
RHO = np.mean([np.corrcoef(Rc[p][1:][ok1], Rc[p][:-1][ok1])[0, 1] for p in PRED])
print("residual corr matrix:\n", np.round(np.corrcoef(RES.T), 2), "\nlag-1 autocorr", round(RHO, 3))
np.savez(f"{S}/qa/ident.npz", COV=COV, RHO=RHO, **{f"{b[0]}_{p}": ident[b][p][0] for b in BANDS for p in PRED})

# real closed-loop offsets (the drives were on the cubic build): RMS about the stretch mean
re_st, re_cv = [], []
for dd in data:
  g = dd['segok'] & dd['good']
  e = dd['e'][g]; kk = np.abs(dd['kr'][g])
  e = e - np.mean(e)
  re_st.append(e[kk < 0.5e-3]); re_cv.append(e[kk >= 1.5e-3])
print(f"REAL op offsets (de-meaned): straight RMS {np.sqrt(np.mean(np.concatenate(re_st) ** 2)):.3f} curve RMS {np.sqrt(np.mean(np.concatenate(re_cv) ** 2)):.3f}")
# 2) simulation
def band_of(vk):
  return BANDS[0] if vk < 55 else BANDS[1] if vk < 80 else BANDS[2]

def cmd_form(name, P, ab, v, tau):
  K = P['K']
  blend = np.clip((ab - 0.8e-3) / 0.7e-3, 0, 1) * (ab >= 0.8e-3)
  if name == "action":
    return K
  if name == "cubic":                 # Phase 41-2 shipped before Phase 55
    t_ah = min(np.interp(v, [5.6, 13.9, 27.8, 38.9], [0.08, 0.10, 0.13, 0.18]) + np.interp(ab, [0.0008, 0.005], [0.0, 0.20]), 0.32) + tau
    dk = 0.7 * t_ah / max(v, 5.0) ** 2
    return K + blend * np.clip(P['kcub'] - K, -dk, dk)
  dk = 0.7 * (0.9 + tau) / max(v, 5.0) ** 2
  if name == "P55":
    return K + blend * np.clip(P['k9'] - K, -dk, dk)
  if name == "delta":
    return K + blend * np.clip(P['k9'] - P['k3'], -dk, dk)
  if name == "P55half":
    return K + 0.5 * blend * np.clip(P['k9'] - K, -dk, dk)
  if name == "P55ts0.6":
    dk6 = 0.7 * (0.6 + tau) / max(v, 5.0) ** 2
    return K + blend * np.clip(P['k6'] - K, -dk6, dk6)
  if name == "P55ts0.45":
    dk5 = 0.7 * (0.45 + tau) / max(v, 5.0) ** 2
    return K + blend * np.clip(P['k5'] - K, -dk5, dk5)
  if name == "P55trans":            # orient lead only while the plan predicts a curvature change
    wch = min(max((abs(P['k9'] - P['k3']) - 0.3e-3) / 0.5e-3, 0.0), 1.0)
    return K + blend * wch * np.clip(P['k9'] - K, -dk, dk)
  if name == "act+delta":           # action keeps its feedback; add the plan's predicted change (k9 - k3), no gate
    return K + np.clip(P['k9'] - P['k3'], -dk, dk)
  if name == "act+delta.5":
    return K + 0.5 * np.clip(P['k9'] - P['k3'], -dk, dk)
  raise ValueError(name)

VARS = ("action", "cubic", "P55", "P55ts0.6", "P55ts0.45", "P55half", "P55trans", "delta", "act+delta", "act+delta.5")
rng_seed = 7
segs = []
for dd in data:
  g = dd['segok']
  i = 0; n = len(g)
  while i < n:
    if not g[i]:
      i += 1; continue
    j = i
    while j + 1 < n and g[j + 1] and dd['T'][j + 1] - dd['T'][j] < 0.25:
      j += 1
    if dd['T'][j] - dd['T'][i] >= 20:
      segs.append((dd['T'][i:j + 1] - dd['T'][i], dd['v'][i:j + 1], dd['kr'][i:j + 1]))
    i = j + 1
print(f"sim segments {len(segs)}, {sum(s[0][-1] for s in segs) / 60:.0f} min")
NS = float(sys.argv[2]) if len(sys.argv) > 2 else 1.0
import pickle
pickle.dump(dict(segs=segs, ident=ident, COV=COV, RHO=RHO), open(f'{S}/qa/sim_cache.pkl', 'wb'))
if len(sys.argv) > 3: sys.exit(0)
L_ch = NS * np.linalg.cholesky(COV + 1e-18 * np.eye(len(PRED)))
print('noise scale', NS)
results = {vn: {"e_st": [], "e_cv": [], "c1": [], "dcmd": [], "jerk": [], "spec": 0} for vn in VARS}
DT = 0.01; DELAY = 0.30; TAU_A = 0.10; GAIN = 0.90
for si, (Ts, vs, krs) in enumerate(segs):
  t_end = Ts[-1]
  tt = np.arange(0, t_end - 1.6, DT)
  v_t = np.interp(tt, Ts, vs); kr_t = np.interp(tt, Ts, krs)
  nfr = int(len(tt) / 5)
  rng = np.random.default_rng(rng_seed + si)
  z = rng.standard_normal((nfr, len(PRED)))
  noise = np.zeros((nfr, len(PRED)))
  for k in range(1, nfr):
    noise[k] = RHO * noise[k - 1] + np.sqrt(1 - RHO ** 2) * (L_ch @ z[k])
  for vn in VARS:
    e, c1, kcar, lp = 0.0, 0.0, kr_t[0], kr_t[0]
    buf = [kr_t[0]] * int(DELAY / DT)
    cmd_hist, e_hist, kc_hist = [], np.zeros(len(tt)), np.zeros(len(tt))
    cmd = kr_t[0]
    for k in range(len(tt)):
      v = v_t[k]
      if k % 5 == 0:
        fi = min(k // 5, nfr - 1)
        F = np.array([np.interp(tt[k] + dt, tt, kr_t) for dt in FUT])
        x = np.concatenate([F, [e, c1, 1.0]])
        b = band_of(v * 3.6)
        P = {p: float(x @ ident[b][p][0]) + noise[fi, pi] for pi, p in enumerate(PRED)}
        tau = float(np.interp(v, [8.0, 13.0, 18.0], [0.30, 0.12, 0.08]))
        cmd = float(cmd_form(vn, P, abs(P['K']), v, tau))
        cmd_hist.append(cmd)
      tau = float(np.interp(v, [8.0, 13.0, 18.0], [0.30, 0.12, 0.08]))
      lp += DT / (tau + DT) * (cmd - lp)
      buf.append(lp); delayed = buf.pop(0)
      kcar += DT / TAU_A * (GAIN * delayed - kcar)
      e += -v * c1 * DT
      c1 += v * (kr_t[k] - kcar) * DT
      e_hist[k] = e; kc_hist[k] = kcar
    burn = int(5 / DT)
    E = e_hist[burn:]; KR = np.abs(kr_t[burn:])
    res = results[vn]
    res["e_st"].append(E[KR < 0.5e-3]); res["e_cv"].append(E[KR >= 1.5e-3])
    res["dcmd"].append(np.diff(cmd_hist[burn // 5:]))
    ay = v_t[burn:] ** 2 * kc_hist[burn:]
    res["jerk"].append(np.diff(ay) / DT)
    Ef = np.abs(np.fft.rfft((E - E.mean()) * np.hanning(len(E)))) ** 2
    fr = np.fft.rfftfreq(len(E), DT)
    res["spec"] = res["spec"] + np.interp(np.linspace(0.05, 1.0, 96), fr, Ef) / np.sum(Ef)
  if si % 10 == 0:
    print("seg", si, flush=True)
print(f"{'variant':8} {'e RMS str':>9} {'e RMS curve':>11} {'p95|e|':>7} {'d cmd std':>9} {'jerk RMS':>8} {'weave peak':>10}")
for vn in VARS:
  r = results[vn]
  est, ecv = np.concatenate(r["e_st"]), np.concatenate(r["e_cv"])
  ea = np.concatenate([est, ecv])
  fq = np.linspace(0.05, 1.0, 96)
  print(f"{vn:8} {np.sqrt(np.mean(est ** 2)):9.3f} {np.sqrt(np.mean(ecv ** 2)):11.3f} {np.percentile(np.abs(ea), 95):7.3f} "
        f"{np.std(np.concatenate(r['dcmd'])) * 1e3:9.4f} {np.sqrt(np.mean(np.concatenate(r['jerk']) ** 2)):8.3f} {fq[np.argmax(r['spec'])]:8.2f} Hz")
