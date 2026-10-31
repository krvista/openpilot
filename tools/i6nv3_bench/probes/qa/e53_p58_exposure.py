"""E53: Phase 58 exposure on whole routes (real CarController, open loop). Per route: op-active time >= 15 km/h, share of
time the cap is armed / binding, binding while the driver-domain torque is < 20 (no hand detected now), the TX reduction
when binding (|TX off - wheel| - |TX on - wheel|), and the gap >= 8 deg time OFF vs ON. Also driver-domain torque stats."""
import sys
import numpy as np
sys.path.insert(0, ".")
from phase_tests.harness import Sim  # noqa: E402
from opendbc.car.hyundai.values import CarControllerParams as CCP  # noqa: E402
S = sys.argv[1]; routes = sys.argv[2].split(",")
for kv in sys.argv[3:]:
  k, v = kv.split("="); setattr(CCP, k, type(getattr(CCP, k))(float(v)))
CAP = CCP.HAND_GAP_CAP_DEG


def run(d, cap):
  CCP.HAND_GAP_CAP_DEG = cap
  sim = Sim(); n = len(d['T']); tx = np.full(n, np.nan); arm = np.zeros(n, bool); bind = np.zeros(n, bool); dtq = np.zeros(n)
  for k in range(n):
    sim.step(v=float(d['v'][k]) / 3.6, tq=float(d['tq_s'][k]), wheel=float(d['ang'][k]), cmd=float(d['apply'][k]),
             lat_active=bool(d['lat'][k]), pressed=bool(d['pressed'][k]), blinker=bool(d['lb'][k]), blinker_right=bool(d['rb'][k]),
             a_ego=float(d['a_ego'][k]), gas=bool(d['gas'][k]), lane_prob_min=float(d['lane_min'][k]), bs_l=bool(d['bsl'][k]), bs_r=bool(d['bsr'][k]))
    m = sim.lkas_alt()
    if m:
      tx[k] = m["ADAS_StrAnglReqVal"]
    arm[k] = sim.cc.hand_gap_cap is not None; bind[k] = sim.cc.hand_gap_binding
    dtq[k] = max(0.0, abs(float(d['tq_s'][k])) - sim.cc.hold_comp_last)
  return tx, arm, bind, dtq


for r in routes:
  d = dict(np.load(f"{S}/r50/{r}.npz", allow_pickle=True))
  off, _, _, dtq = run(d, 0.0)
  on, arm, bind, _ = run(d, CAP)
  m = d['lat'] & (d['v'] >= 15) & (d['gain'] >= 0.5)
  ang = d['ang']; go = np.abs(off - ang); gn = np.abs(on - ang)
  red = (go - gn)[m & bind]
  print(f"{r}: op {m.sum() / 6000:.1f} min | fidelity |off - logged TX| p50 {np.nanmedian(np.abs(off - d['wire'])[m]):.2f} | driver-domain tq >= 60 {np.mean(dtq[m] >= 60):.1%} "
        f"| cap armed {np.mean(arm[m]):.1%}, binding {np.mean(bind[m]):.1%} (with no hand now (<20) {np.mean(bind[m] & (dtq[m] < 20)):.1%}) "
        f"| TX cut when binding p50 {np.median(red) if len(red) else 0:.1f} p90 {np.percentile(red, 90) if len(red) else 0:.1f} deg "
        f"| gap >= 8 deg time off {np.mean(go[m] >= 8):.2%} -> on {np.mean(gn[m] >= 8):.2%}", flush=True)
