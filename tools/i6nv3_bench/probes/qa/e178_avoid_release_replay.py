"""E178 (report 115): the real controlsd replayed over the hands-off stretches of a solo drive with and without the
Phase 83 avoidance release: how often the additions change, what is left of the curve correction, and the episodes where
the release acts. argv: S route   env: REPLAY_PROFILE=solo REPLAY_MODEL_DELAY=log"""
import sys
import numpy as np
sys.path.insert(0, "tools/i6nv3_bench/probes/qa")
import replay_state_control as R   # noqa: E402
cd = R.cd
S, r = sys.argv[1], sys.argv[2]; R.load(S, r); d = R.d


def since(m):
  o = np.zeros(len(m)); c = 1e9
  for i, x in enumerate(m):
    c = 0 if x else c + 0.01; o[i] = c
  return o


rel = ((d['gw_flags'] & 8) > 0) if d['gw_flags'].any() else d['lat'].astype(bool)
ho = (since(d['pressed']) > 1.0) & (since(d['lb'] | d['rb']) > 3.0) & rel & d['lat'] & (d['lcs'] == 0) & (d['v'] >= 30)
e = np.flatnonzero(np.diff(np.r_[0, ho.astype(int), 0])); seg = [(d['T'][a], d['T'][b - 1]) for a, b in zip(e[::2], e[1::2]) if b - a >= 800]
out = {}
for nm, flag in (("off", False), ("on", True)):
  cd.LANE_AVOID_RELEASE = flag; rows = []
  for t0, t1 in seg:
    o = np.array([(a, b) for a, b, _, _ in R.replay("new", t0, t1)])[300:]
    j = np.clip(np.searchsorted(d['T'], o[:, 0]), 0, len(d['T']) - 1)
    rows.append(np.c_[o[:, 0], o[:, 1] - d['k_model'][j], d['k_model'][j], d['v'][j]])
  out[nm] = np.concatenate(rows)
A, B = out["off"], out["on"]; K = A[:, 2] * 1e3; a, b = A[:, 1] * 1e3, B[:, 1] * 1e3
Kp = np.r_[np.zeros(100), K[:-100]]; cu = (np.abs(K) >= 1.0) & (K * Kp > 0) & (np.abs(Kp) >= 1.0); st = (np.abs(K) < 0.3) & (np.abs(Kp) < 0.3)
ch = np.abs(a - b) > 0.01
print(f"{r}: {len(seg)} stretches, {len(a)/100:.0f} s | additions changed by > 0.01e-3 in {100*ch.mean():.1f} % of the time (straights {100*ch[st].mean():.1f} %, curves {100*ch[cu].mean():.1f} %)")
print(f"  curves {cu.sum()/100:.0f} s: outward additions {(-a[cu]*np.sign(K[cu])).mean():+.3f} -> {(-b[cu]*np.sign(K[cu])).mean():+.3f}e-3 | straights {st.sum()/100:.0f} s: mean {a[st].mean():+.3f} -> {b[st].mean():+.3f}, sd {a[st].std():.3f} -> {b[st].std():.3f}")
big = np.abs(a - b) > 0.05; ee = np.flatnonzero(np.diff(np.r_[0, big.astype(int), 0]))
for i0, i1 in list(zip(ee[::2], ee[1::2]))[:12]:
  if i1 - i0 >= 30:
    print(f"  t {A[i0,0]:7.1f} s  {(i1-i0)/100:4.1f} s  {A[i0,3]:3.0f} km/h  |K| {np.abs(K[i0:i1]).max():.1f}e-3  additions {a[i0:i1].mean():+.3f} -> {b[i0:i1].mean():+.3f}e-3")

# time actually released (the open-loop integral keeps a difference long after a release: not a measure of it)
rec = []; orig = cd.Controls._lane_center


def spy(self, mf, CS, k_action, usable_ctx):
  out = orig(self, mf, CS, k_action, usable_ctx); rec.append((getattr(self, "_lc_wa", 1.0), abs(k_action), bool(usable_ctx))); return out


cd.Controls._lane_center = spy; cd.LANE_AVOID_RELEASE = True; cd.LANE_CENTER_KI = 0.0
on = np.concatenate([np.array([(a, b) for a, b, _, _ in R.replay("new", t0, t1)])[300:, 1] for t0, t1 in seg])
cd.Controls._lane_center = orig; cd.LANE_AVOID_RELEASE = False
off = np.concatenate([np.array([(a, b) for a, b, _, _ in R.replay("new", t0, t1)])[300:, 1] for t0, t1 in seg])
W = np.array(rec); u = W[:, 2] > 0; cur = W[:, 1] >= 1e-3
print(f"  released: weight < 0.99 in {100*np.mean(W[u,0] < 0.99):.1f} % of usable frames (straight-ish {100*np.mean(W[u & ~cur,0] < 0.99):.1f} %, curves {100*np.mean(W[u & cur,0] < 0.99):.1f} %), fully (< 0.05) {100*np.mean(W[u,0] < 0.05):.2f} %; mean weight {W[u,0].mean():.3f}")
df = (on - off) * 1e3; cu2 = cu[:len(df)]; Ks = np.sign(K[:len(df)])
print(f"  without the integral: curve outward additions change {(-df[cu2]*Ks[cu2]).mean():+.4f}e-3 (mean), |change| p95 {np.percentile(np.abs(df),95):.3f} p99 {np.percentile(np.abs(df),99):.3f}e-3")
