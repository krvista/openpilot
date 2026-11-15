"""E131 (report §91): the chain sim's rate-floor plant (RF, E89) starts the wheel 0.1-0.4 s too early in corner entries
(E130). RFW = RF + a 'wake' rule for a wheel at rest: while the wheel is at rest (< 0.5 deg/s over 0.3 s) the motion RF
would give is held back as pending; once the request has pushed for T_w seconds the wheel breaks away and the pending part
is released with time constant tau_r on top of the normal RF response. (E128: the start delay does not depend on the
request rate, the TX - wheel gap or the speed -> a time, not a threshold.) T_w, tau_r per speed band by grid search on
odd-numbered routes (entry RMS + overall RMS), scored on even-numbered ones like E130. Writes qa/e131_params.json.
argv: S"""
import glob, json, sys
import numpy as np
sys.path.insert(0, "."); sys.path.insert(0, "opendbc_repo")
from opendbc.car.hyundai.carcontroller import compute_hold_torque  # noqa: E402
S = sys.argv[1]
CENT = [35.0, 47.5, 62.5, 80.0, 100.0]; VB = [(30, 55), (55, 70), (70, 200)]
_rf = json.load(open(f"{S}/qa/e89_params.json"))
RF = np.array([np.mean([_rf.get(f"RF{fo}{i}", _rf.get(f"RF{i}")) for fo in "AB"], axis=0) for i in range(1, 6)])


def sim_rfw(u, w0, v, tw, taur, rf=RF):
  """RF plant with the wake rule; tw = 0 gives RF exactly."""
  w = w0; out = np.empty(len(u)); hist = [u[0]]; pend = 0.0; push = 0.0; stuck = tw > 0.0; rest_n = 0; wh = [w0]
  for k in range(len(u)):
    hist.append(u[k]); pt = [np.interp(v[k], CENT, rf[:, j]) for j in range(5)]; nd = int(round(pt[0] / 0.01))
    g = lambda q: hist[-1 - q] if len(hist) > q else hist[0]
    u1, u2 = g(nd), g(nd + 1)
    if pt[4] > 0.0:
      r1 = (u1 - g(nd + 10)) / 0.1; r2 = (u2 - g(nd + 11)) / 0.1
      dz = lambda r: r - pt[4] if r > pt[4] else (r + pt[4] if r < -pt[4] else 0.0)
      du = 0.5 * (dz(r1) + dz(r2)) * 0.01
    else:
      du = u1 - u2
    inc = 0.01 * (pt[1] * du / 0.01 + pt[2] * (u1 - w)); cen = -0.01 * pt[3] * w
    if tw > 0.0 and stuck:
      if pend * inc < 0.0: push = 0.0                        # the push reversed: start over
      pend += inc
      push = push + 0.01 if abs(pend) >= 0.05 else max(push - 0.01, 0.0)
      w += cen
      if push >= tw: stuck = False; push = 0.0
    else:
      rel = pend * min(0.01 / max(taur, 0.01), 1.0); pend -= rel; w += inc + rel + cen
    wh.append(w); rest = abs(w - wh[-31]) < 0.15 if len(wh) > 31 else True
    rest_n = rest_n + 1 if rest else 0
    if tw > 0.0 and not stuck and rest_n >= 50 and abs(pend) < 0.1: stuck = True; push = 0.0
    out[k] = w
  return out


def load_windows():
  W = []; routes = sorted({f[-12:-4] for f in glob.glob(f"{S}/r50/0*.npz")})
  for f in sorted(glob.glob(f"{S}/r50/0*.npz")):
    d = dict(np.load(f, allow_pickle=True)); n = len(d['T']); r = f[-12:-4]
    if n < 2000: continue
    v = d['v']; vm = v / 3.6; la = np.abs(vm ** 2 * d['k_meas'])
    hold = np.array([compute_hold_torque(a, c) for a, c in zip(vm[::10], la[::10])]).repeat(10)[:n]
    dd = np.maximum(0.0, np.abs(d['tq_s']) - hold)
    ok = d['lat'] & (v >= 30) & (dd < 40) & ~d['pressed'] & ~d['lb'] & ~d['rb'] & (d['tx_active'] == 2) & (d['gain'] >= 0.7)
    ap = d['apply']; e = np.flatnonzero(np.diff(np.concatenate([[0], ok.astype(int), [0]])))
    for a0, b0 in zip(e[::2], e[1::2]):
      if b0 - a0 < 1000: continue
      for c0 in range(a0, b0 - 999, 3000):                  # chunks of <= 30 s
        c1 = min(c0 + 3000, b0)
        if c1 - c0 < 1000: continue
        u = d['wire'][c0:c1]; w = d['ang'][c0:c1]; vv = v[c0:c1]; ents = []; k = c0 + 150
        while k < c1 - 260:
          mv = ap[k] - ap[k - 100]
          if abs(mv) >= 2.0 and abs(ap[k - 1] - ap[k - 101]) < 2.0 and abs(ap[k - 100] - ap[k - 150]) < 1.0 and np.max((ap[k:k + 250] - ap[k - 100]) * np.sign(mv)) >= 5.0 \
             and np.sign(ap[k]) == np.sign(mv) and abs(d['ang'][k - 100] - d['ang'][k - 150]) < 0.5:
            ents.append((k - c0, np.sign(mv))); k += 300; continue
          k += 1
        bi = next((q for q, (lo, hi) in enumerate(VB) if lo <= np.median(vv) < hi), None)
        if bi is not None: W.append((bi, routes.index(r) % 2, u, w, vv, ents))
  return W


def score(sel, tw, taur):
  se = 0.0; n = 0; se_e = 0.0; n_e = 0; dt = []
  for bi, fold, u, w, vv, ents in sel:
    p = sim_rfw(u, w[0], vv, tw, taur); m = np.arange(len(u)) >= 200
    se += np.sum((p[m] - w[m]) ** 2); n += m.sum()
    for k, s in ents:
      a, b = k - 100, k + 250; pe = p[a:b] - (p[a] - w[a]); se_e += np.sum((pe - w[a:b]) ** 2); n_e += b - a
      tr = np.flatnonzero((w[a:b] - w[a]) * s >= 1.0); tp = np.flatnonzero((pe - pe[0]) * s >= 1.0)
      if len(tr): dt.append(((tp[0] if len(tp) else 350) - tr[0]) / 100.0)
  return np.sqrt(se / max(n, 1)), np.sqrt(se_e / max(n_e, 1)), (np.median(dt) if dt else np.nan), len(dt)


if __name__ == "__main__":
  W = load_windows(); print(f"windows {len(W)} ({sum(len(x[2]) for x in W) / 6000:.0f} min), entries {sum(len(x[5]) for x in W)}")
  out = {}
  for bi, (lo, hi) in enumerate(VB):
    tr = [x for x in W if x[0] == bi and x[1] == 1 and x[5]] + [x for x in W if x[0] == bi and x[1] == 1 and not x[5]][:25]
    va = [x for x in W if x[0] == bi and x[1] == 0]
    best = None
    for tw in (0.0, 0.15, 0.3, 0.45, 0.6, 0.8):
      for taur in ((0.3,) if tw == 0.0 else (0.15, 0.3, 0.6)):
        ra, re_, dt, ne = score(tr, tw, taur); J = re_ + ra
        if best is None or J < best[0]: best = (J, tw, taur)
        print(f"   [{lo}-{hi}] train T_w {tw:.2f} tau_r {taur:.2f}: RMS all {ra:.2f} entries {re_:.2f} start {dt:+.2f} s (n={ne})", flush=True)
    _, tw, taur = best; out[f"{lo}-{hi}"] = (tw, taur)
    r0 = score(va, 0.0, 0.3); r1 = score(va, tw, taur)
    print(f"[{lo}-{hi} km/h] best T_w {tw} s, tau_r {taur} s | validation ({sum(len(x[2]) for x in va) / 6000:.0f} min, {r1[3]} entries): RF RMS all {r0[0]:.2f} / entries {r0[1]:.2f} / start {r0[2]:+.2f} s"
          f"  ->  RFW RMS all {r1[0]:.2f} / entries {r1[1]:.2f} / start {r1[2]:+.2f} s", flush=True)
  json.dump(out, open(f"{S}/qa/e131_params.json", "w")); print("saved", out)
