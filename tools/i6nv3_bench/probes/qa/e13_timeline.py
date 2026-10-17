"""E13: 0.25 s timeline around a crossing: offset to the line, lane probs, action K, k_ctrl, 55b lead (plan), steer flags,
wheel angle vs command angle, ACI gain, driver torque. Usage: e13_timeline.py <scratch> <route> <t0> <t1>"""
import glob
import sys
import numpy as np
sys.path.insert(0, sys.argv[0].rsplit('/', 1)[0])
from common import T_IDXS

S, r, t0, t1 = sys.argv[1], sys.argv[2], float(sys.argv[3]), float(sys.argv[4])
P = dict(np.load(f"{S}/la/plan_{r}.npz"))
d = dict(np.load((glob.glob(f"{S}/r46/{r}.npz") + glob.glob(f"{S}/ropm/{r}.npz") + glob.glob(f"{S}/r50/{r}.npz"))[0], allow_pickle=True))
T = P['T']; Tr = d['T']
names = {0: "LCgate", 1: "LCcap", 2: "LCrel", 3: "LAcap", 4: "blend", 5: "drop", 6: "dep", 7: "bsm", 8: "7c", 9: "orient", 10: "budget"}
print(" t      v   L1y   L2y  pL   pR   K      k_ctrl  lead   ang   apply  gain  tq   flags")
for t in np.arange(t0, t1, 0.25):
  i = int(np.argmin(np.abs(T - t))); j = int(np.argmin(np.abs(Tr - t)))
  v = d['v'][j]; tau = np.interp(v / 3.6, [8.0, 13.0, 18.0], [0.30, 0.12, 0.08])
  OR, VX = P['OR'][i].astype(float), P['VX'][i].astype(float)
  kt = lambda tt: np.interp(tt, T_IDXS, OR) / max(np.interp(tt, T_IDXS, VX), 1.0)
  lead = 0.5 * (kt(0.9 + tau) - kt(0.3))
  l1 = np.interp(0.0, P['LX'][i], P['L1'][i]); l2 = np.interp(0.0, P['LX'][i], P['L2'][i])
  f = int(d['sf_ctl'][j])
  fl = ",".join(nm for b, nm in names.items() if f >> b & 1)
  print(f"{t:7.2f} {v:5.1f} {l1:+.2f} {l2:+.2f} {P['PR'][i,1]:.2f} {P['PR'][i,2]:.2f} {P['K'][i]*1e3:+6.2f} {d['k_ctrl'][j]*1e3:+6.2f} "
        f"{lead*1e3:+5.2f} {d['ang'][j]:+6.1f} {d['apply'][j]:+6.1f} {d['gain'][j]:.2f} {d['tq_s'][j]:+5.0f} {fl}")
