"""E177 (report 114): the real controlsd replayed over the hands-off stretches of a solo drive with the Phase 79 offset
gain and with the Phase 82 lane fit (open loop: the car's path is the logged one): size of the additions on straights
and in curves (outward +), and their roughness. argv: S route   env: REPLAY_PROFILE=solo REPLAY_MODEL_DELAY=log"""
import sys
import numpy as np
sys.path.insert(0, "tools/i6nv3_bench/probes/qa")
import replay_state_control as R   # noqa: E402 - installs the stubs controlsd needs off the device
cd = R.cd
S, r = sys.argv[1], sys.argv[2]; R.load(S, r); d = R.d


def since(m):
  o = np.zeros(len(m)); c = 1e9
  for i, x in enumerate(m):
    c = 0 if x else c + 0.01; o[i] = c
  return o


rel = ((d['gw_flags'] & 8) > 0) if d['gw_flags'].any() else d['lat'].astype(bool)
ho = (since(d['pressed']) > 1.0) & (since(d['lb'] | d['rb']) > 3.0) & rel & d['lat'] & (d['lcs'] == 0) & (d['v'] >= 45)
e = np.flatnonzero(np.diff(np.r_[0, ho.astype(int), 0])); seg = [(d['T'][a], d['T'][b - 1]) for a, b in zip(e[::2], e[1::2]) if b - a >= 800]
V = (("P79", dict(LANE_FIT_GAIN=0.0, LANE_CENTER_KE=[0.0, 0.3e-3, 0.3e-3, 0.15e-3])), ("P82", dict(LANE_FIT_GAIN=cd.LANE_FIT_GAIN, LANE_CENTER_KE=[0.0] * 4)))
acc = {nm: [] for nm, _ in V}
for t0, t1 in seg:
  for nm, c in V:
    for k, v in c.items():
      setattr(cd, k, v)
    o = np.array([(a, b) for a, b, _, _ in R.replay("new", t0, t1)])[300:]      # 3 s to settle the filters
    j = np.clip(np.searchsorted(d['T'], o[:, 0]), 0, len(d['T']) - 1)
    acc[nm].append(np.c_[o[:, 1] - d['k_model'][j], d['k_model'][j], d['v'][j]])
n = sum(len(x) for x in acc["P79"]) / 100
print(f"{r}: {len(seg)} hands-off stretches >= 8 s at >= 45 km/h, {n:.0f} s")
for nm, _ in V:
  A = np.concatenate(acc[nm]); add, K, v = A[:, 0] * 1e3, A[:, 1] * 1e3, A[:, 2]
  Kp = np.r_[np.zeros(100), K[:-100]]
  st = (np.abs(K) < 0.3) & (np.abs(Kp) < 0.3); cu = (np.abs(K) >= 1.0) & (K * Kp > 0) & (np.abs(Kp) >= 1.0)
  d5 = add[50:] - add[:-50]
  print(f"  {nm}: straights {st.sum()/100:4.0f} s additions mean {add[st].mean():+.3f} sd {add[st].std():.3f}e-3, 0.5 s change sd {d5[st[50:] & st[:-50]].std():.3f} | "
        f"curves {cu.sum()/100:4.0f} s outward {(-add[cu] * np.sign(K[cu])).mean():+.3f} sd {(-add[cu] * np.sign(K[cu])).std():.3f}e-3 | |additions| p99 {np.percentile(np.abs(add), 99):.3f} max {np.abs(add).max():.3f}")
