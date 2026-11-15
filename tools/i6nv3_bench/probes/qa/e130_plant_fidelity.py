"""E130 (report §91): which MDPS model reproduces the wheel — on all hands-off driving and on corner entries.
Hands-off windows (>= 10 s, as the chain sim: op active, >= 30 km/h, hold-compensated driver torque < 40, no press, no
blinker), wheel and wire request relative to the learned straight-ahead offset.
 RF  : the chain sim's plant (E89 rate-floor fit: d, Kf, Kp, Kc, r0 per speed), simulated from the logged request.
 HYB : wheel(t) = a * u(t - d) + b * LP_tau(u)(t - d) + c   (a = immediate share of a request change, a + b = steady
       share, tau = how slowly the rest arrives; LAG is the a = 0 case). (tau, d) on a grid, (a, b) least squares per speed
       band on odd-numbered routes, scored on even-numbered ones; c = per-window constant.
Scores: RMS wheel error on the validation routes (all samples / inside corner entries = E128 events -1 .. +2.5 s) and, in
entries, the time the wheel has moved 1 deg (model vs real). argv: S"""
import glob, json, sys
import numpy as np
from scipy.signal import lfilter
sys.path.insert(0, "."); sys.path.insert(0, "opendbc_repo")
from opendbc.car.hyundai.carcontroller import compute_hold_torque  # noqa: E402
S = sys.argv[1]
CENT = [35.0, 47.5, 62.5, 80.0, 100.0]; VB = [(30, 55), (55, 70), (70, 200)]
_rf = json.load(open(f"{S}/qa/e89_params.json"))
RF = np.array([np.mean([_rf.get(f"RF{fo}{i}", _rf.get(f"RF{i}")) for fo in "AB"], axis=0) for i in range(1, 6)])
TAUS = [0.3, 0.5, 0.7, 0.9, 1.2, 1.6, 2.2, 3.0]; DS = [0.05, 0.15, 0.25, 0.35, 0.45]
def lp(u, tau):
  al = 0.01 / (tau + 0.01)
  return lfilter([al], [1.0, al - 1.0], u, zi=[(1.0 - al) * u[0]])[0]
def sim_rf(u, w0, v):
  w = w0; out = np.empty(len(u)); hist = list(u[:1])
  for k in range(len(u)):
    hist.append(u[k]); pt = [np.interp(v[k], CENT, RF[:, j]) for j in range(5)]; nd = int(round(pt[0] / 0.01))
    g = lambda q: hist[-1 - q] if len(hist) > q else hist[0]
    u1, u2 = g(nd), g(nd + 1)
    if pt[4] > 0.0:
      r1 = (u1 - g(nd + 10)) / 0.1; r2 = (u2 - g(nd + 11)) / 0.1
      dz = lambda r: r - pt[4] if r > pt[4] else (r + pt[4] if r < -pt[4] else 0.0)
      du = 0.5 * (dz(r1) + dz(r2)) * 0.01
    else:
      du = u1 - u2
    w = w + 0.01 * (pt[1] * du / 0.01 + pt[2] * (u1 - w) - pt[3] * w); out[k] = w
  return out
W = []     # windows: (band, fold, u, w, v, entry index list)
routes = sorted({f[-12:-4] for f in glob.glob(f"{S}/r50/0*.npz")})
for f in sorted(glob.glob(f"{S}/r50/0*.npz")):
  d = dict(np.load(f, allow_pickle=True)); n = len(d['T']); r = f[-12:-4]
  if n < 2000: continue
  try: lpv = np.load(f"{S}/vm/{r}.npz", allow_pickle=True)['lp']
  except Exception: continue
  if len(lpv) < 10: continue
  off = np.interp(d['T'], lpv[:, 0], lpv[:, 3]); v = d['v']; vm = v / 3.6; la = np.abs(vm ** 2 * d['k_meas'])
  hold = np.array([compute_hold_torque(a, c) for a, c in zip(vm[::10], la[::10])]).repeat(10)[:n]
  dd = np.maximum(0.0, np.abs(d['tq_s']) - hold)
  ok = d['lat'] & (v >= 30) & (dd < 40) & ~d['pressed'] & ~d['lb'] & ~d['rb'] & (d['tx_active'] == 2) & (d['gain'] >= 0.7)
  ap = d['apply']; e = np.flatnonzero(np.diff(np.concatenate([[0], ok.astype(int), [0]])))
  for a0, b0 in zip(e[::2], e[1::2]):
    if b0 - a0 < 1000: continue
    u = d['wire'][a0:b0] - off[a0:b0]; w = d['ang'][a0:b0] - off[a0:b0]; vv = v[a0:b0]; ents = []
    k = a0 + 150
    while k < b0 - 260:
      mv = ap[k] - ap[k - 100]
      if abs(mv) >= 2.0 and abs(ap[k - 1] - ap[k - 101]) < 2.0 and abs(ap[k - 100] - ap[k - 150]) < 1.0 and np.max((ap[k:k + 250] - ap[k - 100]) * np.sign(mv)) >= 5.0 \
         and np.sign(ap[k]) == np.sign(mv) and abs(d['ang'][k - 100] - d['ang'][k - 150]) < 0.5:
        ents.append((k - a0, np.sign(mv))); k += 300; continue
      k += 1
    bi = next((q for q, (lo, hi) in enumerate(VB) if lo <= np.median(vv) < hi), None)
    if bi is not None: W.append((bi, routes.index(r) % 2, u, w, vv, ents))
print(f"hands-off windows {len(W)} ({sum(len(x[2]) for x in W) / 6000:.0f} min), corner entries inside {sum(len(x[5]) for x in W)}")
def design(u, tau, d):
  nd = int(round(d / 0.01)); x1 = np.concatenate([np.full(nd, u[0]), u[:len(u) - nd]]); x2 = lp(x1, tau); return x1, x2
def score(pred_fn, sel):
  se = 0.0; n = 0; se_e = 0.0; n_e = 0; dt = []
  for bi, fold, u, w, vv, ents in sel:
    p = pred_fn(bi, u, w, vv); m = np.arange(len(u)) >= 200; p = p - np.mean(p[m] - w[m])        # per-window constant
    se += np.sum((p[m] - w[m]) ** 2); n += m.sum()
    for k, s in ents:
      a, b = k - 100, k + 250
      pe = p[a:b] - (p[a] - w[a]); se_e += np.sum((pe - w[a:b]) ** 2); n_e += b - a
      tr = np.flatnonzero((w[a:b] - w[a]) * s >= 1.0); tp = np.flatnonzero((pe - pe[0]) * s >= 1.0)
      if len(tr): dt.append(((tp[0] if len(tp) else 350) - tr[0]) / 100.0)
  return np.sqrt(se / max(n, 1)), np.sqrt(se_e / max(n_e, 1)), (np.median(dt) if dt else np.nan), len(dt)
for bi, (lo, hi) in enumerate(VB):
  tr = [x for x in W if x[0] == bi and x[1] == 1]; va = [x for x in W if x[0] == bi and x[1] == 0]
  if not tr or not va: continue
  best = {}
  for kind in ("HYB", "LAG"):
    bb = None
    for tau in TAUS:
      for dl in DS:
        A = []; Y = []
        for _, _, u, w, vv, _ in tr:
          x1, x2 = design(u, tau, dl); m = slice(200, None)
          cols = [x1[m] - x1[m].mean(), x2[m] - x2[m].mean()] if kind == "HYB" else [x2[m] - x2[m].mean()]
          A.append(np.c_[tuple(cols)]); Y.append(w[m] - w[m].mean())
        A = np.concatenate(A); Y = np.concatenate(Y); co = np.linalg.lstsq(A, Y, rcond=None)[0]; r = np.sqrt(np.mean((A @ co - Y) ** 2))
        if bb is None or r < bb[0]: bb = (r, tau, dl, co)
    best[kind] = bb
  def mk(kind):
    _, tau, dl, co = best[kind]
    def fn(b_, u, w, vv):
      x1, x2 = design(u, tau, dl)
      return (co[0] * x1 + co[1] * x2) if kind == "HYB" else co[0] * x2
    return fn
  rf_fn = lambda b_, u, w, vv: sim_rf(u, w[0], vv)
  print(f"[{lo}-{hi} km/h] train {sum(len(x[2]) for x in tr) / 6000:.0f} min, validation {sum(len(x[2]) for x in va) / 6000:.0f} min ({sum(len(x[5]) for x in va)} entries)")
  for nm, fn, desc in (("RF (chain sim)", rf_fn, ""),
                       ("HYB", mk("HYB"), f"immediate a {best['HYB'][3][0]:.2f}, steady a+b {best['HYB'][3].sum():.2f}, tau {best['HYB'][1]} s, delay {best['HYB'][2]} s"),
                       ("LAG", mk("LAG"), f"steady K {best['LAG'][3][0]:.2f}, tau {best['LAG'][1]} s, delay {best['LAG'][2]} s")):
    r_all, r_ent, dt, ne = score(fn, va)
    print(f"   {nm:15s} RMS all {r_all:.2f} deg | in entries {r_ent:.2f} deg | wheel +1 deg: model - real {dt:+.2f} s (n={ne}) | {desc}")
