"""E58: closed-loop simulation of TX policies through the identified MDPS plant (E57, per speed band), on the TEST routes.
Reference theta_d = the logged controlsd command angle (carcontroller input). Policies:
  LOG   : TX = the logged TX (today's carcontroller: trim / boost / kick / caps ...)
  RAW   : TX = theta_d (no carcontroller shaping), panda rate limit only
  INV   : rate-domain inverse of the plant with wheel feedback:
            TX' = ( theta_d' + LAM*(theta_d - w_meas) - Kp*(TX_d - w) + Kc*w ) / Kf     (nominal band parameters)
          clamps: |TX - theta_d| <= GMAX, panda/ISO TX rate limit (3.59 m/s^3), w_meas quantised 0.1 deg
  INVP  : INV with the reference previewed by the plant delay (theta_d(t + d), i.e. the model delay setting raised by d)
Metrics: wheel - theta_d RMS, lag theta_d -> wheel (xcorr), steady-curve delivery wheel/theta_d, |TX - wheel| p90/p99 and
time > 6 / 8 deg (stored gap), TX rate p99. Robustness: INV designed on nominal parameters run on a perturbed plant."""
import json
import pickle
import sys
import numpy as np
S = sys.argv[1]; MODEL = sys.argv[2] if len(sys.argv) > 2 else "M1"
DT = 0.02
runs = [R for R in pickle.load(open(f"{S}/qa/e57_runs.pkl", "rb")) if R['test']]
PAR = json.load(open(f"{S}/qa/e57_params.json"))
DEG_PER = ([40, 60, 80, 100], [2.85, 3.12, 3.45, 3.74])


def plant_step(p, w, ud_prev, ud_prev2, model):
  d, Kf, Kp, Kc = p[:4]
  err = ud_prev - w; rate = (ud_prev - ud_prev2) / DT
  if model == "M2":
    D, R0 = p[4], p[5]
    err = np.sign(err) * max(abs(err) - D, 0.0)
    rate = rate if abs(rate) >= R0 else 0.0
  return w + DT * (Kf * rate + Kp * err - Kc * w)


def run_policy(R, pol, pn, pp, lam=2.0, gmax=8.0):
  """pn: nominal params (design), pp: plant params (truth)."""
  th = R['apply']; n = len(th); h0 = R['h0']; v = R['v']
  dp = int(round(pp[0] / DT)); dn = int(round(pn[0] / DT))
  w = np.empty(n); w[:h0 + 1] = R['ang'][:h0 + 1]
  tx = np.empty(n); tx[:h0 + 1] = R['wire'][:h0 + 1]
  for k in range(h0 + 1, n):
    lim = (3.59 / max(v[k] / 3.6, 3.0) ** 2) * 1000 * np.interp(v[k], *DEG_PER) * DT
    if pol == "LOG":
      tx[k] = R['wire'][k]
    else:
      if pol == "RAW":
        target = th[k]
      else:
        kr = min(k + dn, n - 1) if pol == "INVP" else k
        ref = th[kr]; refr = (th[kr] - th[kr - 1]) / DT
        wm = np.round(w[k - 1] * 10) / 10
        txd = tx[k - 1 - dn] if k - 1 - dn >= 0 else tx[0]
        Kf, Kp, Kc = max(pn[1], 0.05), pn[2], pn[3]
        txr = (refr + lam * (ref - wm) - Kp * (txd - wm) + Kc * wm) / Kf
        target = tx[k - 1] + txr * DT
        target = float(np.clip(target, ref - gmax, ref + gmax))
      tx[k] = float(np.clip(target, tx[k - 1] - lim, tx[k - 1] + lim))
    i1 = k - 1 - dp; i2 = k - 2 - dp
    u1 = tx[i1] if i1 >= 0 else tx[0]; u2 = tx[i2] if i2 >= 0 else tx[0]
    w[k] = plant_step(pp, w[k - 1], u1, u2, MODEL)
  return tx, w


def metrics(rs, outs):
  e, gaps, rates, deliv, lags = [], [], [], [], []
  for R, (tx, w) in zip(rs, outs):
    h0 = R['h0']; th = R['apply'][h0:]; ww = w[h0:]; tt = tx[h0:]
    e.append(ww - th); gaps.append(np.abs(tt - ww)); rates.append(np.abs(np.diff(tt)) / DT)
    thr = np.abs(np.gradient(th, DT))
    for k in range(50, len(th), 25):
      if np.all(thr[k - 50:k] < 1.0) and abs(th[k]) >= 3:
        deliv.append(ww[k] / th[k])
    if len(th) >= 300 and np.std(th) > 0.5:
      x = th - th.mean(); best = (-2, 0)
      for L in range(0, 60):
        y = ww[L:] - ww[L:].mean(); xx = x[:len(y)]
        cc = np.dot(xx, y) / (np.linalg.norm(xx) * np.linalg.norm(y) + 1e-9)
        if cc > best[0]:
          best = (cc, L)
      if best[0] > 0.7:
        lags.append(best[1] * DT * 1000)
  e = np.concatenate(e); g = np.concatenate(gaps); r = np.concatenate(rates)
  return (f"wheel-cmd RMS {np.sqrt(np.mean(e ** 2)):.2f} deg | lag p50 {np.median(lags) if lags else np.nan:.0f} ms (n {len(lags)}) | curve delivery p50 {np.median(deliv) if deliv else np.nan:.2f} (n {len(deliv)}) "
          f"| gap p90 {np.percentile(g, 90):.1f} p99 {np.percentile(g, 99):.1f}, >6 {np.mean(g > 6):.1%} >8 {np.mean(g > 8):.1%} | TX rate p99 {np.percentile(r, 99):.0f} deg/s")


def band_params(b):
  return PAR.get(f"{b}:{MODEL}")


def evaluate(pol, scale=None, **kw):
  outs = []; rs = []
  for R in runs:
    pn = band_params(R['band'])
    if pn is None:
      continue
    pp = list(pn)
    if scale:
      for i, s in scale.items():
        pp[i] = pp[i] + s if i == 0 else pp[i] * s
    outs.append(run_policy(R, pol, pn, pp, **kw)); rs.append(R)
  return metrics(rs, outs)


# plant validity on test runs: LOG policy wheel vs measured
se = 0; nn = 0
for R in runs:
  pn = band_params(R['band'])
  if pn is None:
    continue
  tx, w = run_policy(R, "LOG", pn, pn)
  se += np.sum((w[R['h0']:] - R['ang'][R['h0']:]) ** 2); nn += len(w) - R['h0']
print(f"[{MODEL}] test runs {len(runs)} | plant check (LOG through plant vs measured wheel) RMS {np.sqrt(se / nn):.2f} deg")
for b in range(3):
  print(f"   band {b} params {band_params(b)}")
print(f"MEASURED (logged wheel vs cmd, for reference): ", end="")
outs = [(R['wire'], R['ang']) for R in runs if band_params(R['band'])]
print(metrics([R for R in runs if band_params(R['band'])], outs))
for pol, kw in (("LOG", {}), ("RAW", {}), ("INV", dict(lam=1.0)), ("INV", dict(lam=2.0)), ("INV", dict(lam=4.0)), ("INVP", dict(lam=2.0)),
                ("INV", dict(lam=2.0, gmax=5.0))):
  print(f"{pol:4} {str(kw):24}: " + evaluate(pol, **kw), flush=True)
print("robustness (INV lam 2 designed on nominal):")
for nm, sc in (("Kf x0.7", {1: 0.7}), ("Kf x1.4", {1: 1.4}), ("Kp x0.3", {2: 0.3}), ("Kp x3", {2: 3.0}), ("delay +0.1 s", {0: 0.1}), ("Kc x2", {3: 2.0})):
  print(f"   {nm:12}: INV " + evaluate("INV", scale=sc, lam=2.0) + "\n                 LOG " + evaluate("LOG", scale=sc), flush=True)
