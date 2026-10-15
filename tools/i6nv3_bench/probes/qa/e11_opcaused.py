"""E11: op-caused line approaches, per build. Episode = car centre within GAP m of an inner line (wheel well over it)
for >= 0.3 s, lat active, >= 25 km/h, no blinker/LC within 3 s, |wheel angle| < 30 deg (no intersection turn), driver
NOT steering in the 2 s before onset (|tq| < 200 and not pressed throughout). For each: lane-prob before onset, the
command's added part (k_ctrl - action K) signed toward the line side over the 1.5 s before onset, the 55b lead bits.
Also, over all op-active hands-off frames: |k_ctrl - K| by lane-prob band (what each build adds on top of the model)."""
import glob
import sys
import numpy as np

S = sys.argv[1]; GAP = float(sys.argv[2]) if len(sys.argv) > 2 else 0.7
RT = {}
for f in sorted(glob.glob(f"{S}/la/plan_*.npz")):
  r = f[-12:-4]
  rf = glob.glob(f"{S}/ropm/{r}.npz") + glob.glob(f"{S}/r43/{r}.npz") + glob.glob(f"{S}/r46/{r}.npz") + glob.glob(f"{S}/r3d/{r}.npz") + glob.glob(f"{S}/rx/{r}.npz")
  if rf:
    RT[r] = rf[0]
by_build = {}
for r, rf in RT.items():
  P = dict(np.load(f"{S}/la/plan_{r}.npz")); d = dict(np.load(rf, allow_pickle=True))
  T = P['T']; n = len(T)
  if n < 1000:
    continue
  Tr = d['T']; b = str(d['commit'])
  at = lambda k: np.interp(T, Tr, d[k].astype(float))
  v = at('v'); lat = at('lat') > 0.5; ang = at('ang'); kc = at('k_ctrl')
  K = P['K'].astype(float)
  bl = d['lb'] | d['rb'] | (d['lcs'] > 0)
  drv = d['pressed'] | (np.abs(d['tq_s']) >= 200)
  cb = np.concatenate([[0], np.cumsum(bl)]); cd_ = np.concatenate([[0], np.cumsum(drv)])
  busy = (cb[np.searchsorted(Tr, T + 1.0)] - cb[np.searchsorted(Tr, T - 3.0)]) > 0
  drv2 = (cd_[np.searchsorted(Tr, T)] - cd_[np.searchsorted(Tr, T - 2.0)]) > 0
  L1 = np.array([np.interp(0.0, P['LX'][i], P['L1'][i]) for i in range(n)])
  L2 = np.array([np.interp(0.0, P['LX'][i], P['L2'][i]) for i in range(n)])
  pmin = np.minimum(P['PR'][:, 1], P['PR'][:, 2])
  near = np.minimum(-L1, L2); side = np.where(-L1 < L2, -1.0, 1.0)
  sf = np.interp(T, Tr, d['sf_ctl'].astype(float)).astype(int)
  ok = lat & ~busy & (v >= 25) & (np.abs(ang) < 30)
  st = by_build.setdefault(b, dict(hrs=0.0, eps=[], add=[]))
  st['hrs'] += np.sum(ok & ~drv2) / 20 / 3600
  st['add'].append(np.stack([pmin[ok & ~drv2], np.abs(kc - K)[ok & ~drv2], v[ok & ~drv2]], 1))
  i = 0
  while i < n:
    if ok[i] and near[i] < GAP and not drv2[i]:
      j = i
      while j + 1 < n and ok[j + 1] and near[j + 1] < GAP:
        j += 1
      if T[j] - T[i] >= 0.3:
        w = (T >= T[i] - 1.5) & (T <= T[i])
        toward = np.mean((kc - K)[w] * side[i]) if w.sum() else np.nan
        st['eps'].append((r, T[i], T[j] - T[i], v[i], near[i:j + 1].min(), pmin[max(i - 20, 0)], toward,
                          int(np.bitwise_or.reduce(sf[w])) if w.sum() else 0, 'R' if side[i] > 0 else 'L', np.mean(K[w] * side[i])))
      i = j + 1
    else:
      i += 1
  print(r, b, flush=True)
for b, st in by_build.items():
  E = st['eps']
  print(f"== build {b}: op-active hands-off {st['hrs'] * 60:.0f} min, op-caused line episodes (gap < {GAP} m): {len(E)} "
        f"({len(E) / max(st['hrs'], 1e-9):.1f}/h)")
  A = np.concatenate(st['add'])
  for lo, hi in ((0, 0.3), (0.3, 0.5), (0.5, 0.7), (0.7, 1.01)):
    s = (A[:, 0] >= lo) & (A[:, 0] < hi) & (A[:, 2] >= 30)
    if s.sum() > 100:
      print(f"   |k_ctrl - K| lane {lo:.1f}-{hi:.1f} ({s.mean() if False else s.sum() / 20 / 60:5.1f} min) p50/p95/p99 "
            f"{np.median(A[s,1])*1e3:.2f}/{np.percentile(A[s,1],95)*1e3:.2f}/{np.percentile(A[s,1],99)*1e3:.2f}e-3")
  for e in E:
    print(f"   {e[0]} t {e[1]:7.1f} {e[2]:4.1f}s v {e[3]:5.1f} {e[8]} gap {e[4]:.2f} lane_p(-1s) {e[5]:.2f} "
          f"added toward line {e[6]*1e3:+.2f}e-3 action toward line {e[9]*1e3:+.2f}e-3 sf_ctl {e[7]:#06x}")
