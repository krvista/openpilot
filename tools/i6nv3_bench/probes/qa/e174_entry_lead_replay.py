"""E174 (report §109): the real controlsd replayed over a solo drive with and without the Phase 78 entry lead (open
loop). Hands-off curve entries and exits (the model action passes 0.6e-3 with a peak >= 1.2e-3): how much earlier the
command reaches 20 % / 50 % of its change, the extra curvature, how long the term is on, its open-loop lateral effect,
and what it does on straights. argv: S route   env: REPLAY_PROFILE=solo REPLAY_MODEL_DELAY=log"""
import sys
import numpy as np
sys.path.insert(0, "tools/i6nv3_bench/probes/qa")
import replay_state_control as R   # noqa: E402
cd = R.cd
S, r = sys.argv[1], sys.argv[2]; R.load(S, r); d = R.d; T = d['T']; n = len(T); v = d['v']
req = d['tx_active'] == 2; pr = d['pressed'].astype(bool); bl = (d['lb'] | d['rb']).astype(bool)
ho = req & ~(np.convolve(pr | bl, np.ones(100), 'full')[:n] > 0)
K = np.convolve(d['k_model'], np.ones(40) / 40, 'same'); s = np.sign(K); ak = np.abs(K)
ev = []
for kind, cr in (("entry", np.flatnonzero((ak[1:] >= 0.6e-3) & (ak[:-1] < 0.6e-3)) + 1), ("exit", np.flatnonzero((ak[1:] < 0.6e-3) & (ak[:-1] >= 0.6e-3)) + 1)):
  last = -10**9
  for i in cr:
    if i - last < 500 or i < 300 or i + 400 >= n or v[i] < 40 or not ho[i - 200:i + 400].all(): continue
    if kind == "entry" and (ak[i - 250:i - 50].max() > 0.4e-3 or ak[i:i + 400].max() < 1.2e-3): continue
    if kind == "exit" and (ak[i - 300:i].max() < 1.2e-3 or ak[i + 50:i + 300].max() > 0.4e-3): continue
    last = i; ev.append((kind, i))
keep = cd.ENTRY_LEAD_S
def ft(x, t, f):
  a = x[:50].mean(); b = x[-50:].mean()
  if abs(b - a) < 1e-9: return np.nan
  k = np.flatnonzero((x - a) / (b - a) >= f); return t[k[0]] if len(k) else np.nan
res = {"entry": [], "exit": []}
for kind, i in ev:
  out = {}
  for nm, val in (("off", 0.0), ("on", keep)):
    cd.ENTRY_LEAD_S = val
    o = R.replay("new", T[i] - 2.0, T[i] + 4.0); out[nm] = np.array([b for _, b, _, _ in o]); fl = np.array([c for _, _, c, _ in o])
  m = min(len(out["off"]), len(out["on"])); a0 = s[i] * out["off"][:m]; a1 = s[i] * out["on"][:m]; t = np.arange(m) / 100.0 - 2.0
  dd = a1 - a0; vv = (v[i] / 3.6) ** 2
  res[kind].append((ft(a0, t, 0.2) - ft(a1, t, 0.2), ft(a0, t, 0.5) - ft(a1, t, 0.5), dd.max(), dd.min(), np.sum(np.abs(dd) > 2e-5) / 100,
                    np.cumsum(np.cumsum(dd * vv) * 0.01)[-1] * 0.01 * 100, v[i]))
cd.ENTRY_LEAD_S = keep
for kind in ("entry", "exit"):
  q = np.array(res[kind])
  if not len(q): print(f"{r} {kind}: none"); continue
  print(f"{r} {kind} n={len(q)} ({np.median(q[:,6]):.0f} km/h): command earlier by {np.nanmedian(q[:,0])*1000:.0f} ms at 20 %, {np.nanmedian(q[:,1])*1000:.0f} ms at 50 % | extra curvature max p50 {np.median(q[:,2])*1e3:+.3f}e-3 (p90 {np.percentile(q[:,2],90)*1e3:+.3f}), min {np.median(q[:,3])*1e3:+.3f}e-3 "
        f"| term on {np.median(q[:,4]):.2f} s | open-loop shift toward the inside p50 {np.median(q[:,5]):+.1f} cm, p90 {np.percentile(q[:,5],90):+.1f} cm")
# straights
e = np.flatnonzero(np.diff(np.r_[0, (ho & (ak < 0.3e-3) & (v >= 45)).astype(np.int8), 0])); tot = 0; on = 0; mx = 0.0
for a, b in zip(e[::2], e[1::2]):
  if b - a < 1500 or tot > 30000: continue
  o = {}
  for nm, val in (("off", 0.0), ("on", keep)):
    cd.ENTRY_LEAD_S = val; o[nm] = np.array([x[1] for x in R.replay("new", T[a], T[a] + 15.0)])
  m = min(len(o["off"]), len(o["on"])); dd = o["on"][:m] - o["off"][:m]; tot += m; on += np.sum(np.abs(dd) > 2e-5); mx = max(mx, np.abs(dd).max())
cd.ENTRY_LEAD_S = keep
print(f"{r} straights {tot/100:.0f} s: term on {on/max(tot,1)*100:.1f} % of the time, max {mx*1e3:.3f}e-3")
