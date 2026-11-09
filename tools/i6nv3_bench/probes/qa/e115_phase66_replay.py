"""E115 (report §85): Phase 66 verification. Replays the real estimator with the CCNC profile as shipped (wide search,
press-free window gate, seed) on the cached 7a-7f inputs, carrying valid blocks across ignitions like the LiveDelay cache.
Prints per route: accepted estimates, estimates refused only by the press gate, blocks, block mean / std, status, time of
first 'estimated'; the accepted-lag histogram (edge pile-up check); and the final value modeld would receive.
argv: S routes_csv"""
import pickle, sys, types
import numpy as np
for _m in ("openpilot.cereal.messaging", "openpilot.common.params", "openpilot.common.realtime", "openpilot.sunnypilot.livedelay.lagd_toggle"):
  if _m not in sys.modules:
    _s = types.ModuleType(_m)
    _s.Params = object; _s.config_realtime_process = lambda *a: None; _s.LagdToggle = object
    def _nm(name, size=0, valid=True):
      class _M:
        def __init__(s): s.lateralDelay = types.SimpleNamespace(status=None, lateralDelay=0.0, validBlocks=0, lateralDelayEstimate=0.0, lateralDelayEstimateStd=0.0, calPerc=0, points=[], version=0)
        def to_bytes(s): return b""
      return _M()
    _s.new_message = _nm; sys.modules[_m] = _s
from openpilot.cereal import log
import openpilot.selfdrive.locationd.lagd as L
S = sys.argv[1]; ROUTES = sys.argv[2].split(",")
class _CP: steerActuatorDelay = 0.1; carFingerprint = "HYUNDAI_IONIQ_6_N"; brand = "hyundai"; steerControlType = "angle"
orig = L.LateralLagEstimator.actuator_delay; last = {}
def spy(*a, **k):
  r = orig(*a, **k); last["r"] = r; return r
L.LateralLagEstimator.actuator_delay = staticmethod(spy)
prof = L.lagd_profile(_CP()); print("profile:", {k: round(v, 3) if isinstance(v, float) else v for k, v in prof.items()})
vb, lag0 = 0, prof["initial_lag_seed"]; acc_lags = []; inv_n = est_n = 0; first_est = None; tglobal = 0.0
for r in ROUTES:
  ev = pickle.load(open(f"{S}/lagd/{r}.pkl", "rb"))
  e = L.LateralLagEstimator(_CP(), 0.05, **prof); e.reset(lag0, vb); n = 0; t0 = ev[0][0] * 1e-9; n_acc = n_gate = 0
  for ts, which, b in ev:
    t = ts * 1e-9
    with log.Event.from_bytes(b) as m:
      e.handle_log(t, which, getattr(m, which))
    if which == "deviceMotion":
      e.update_points(); n += 1
      if n % 5 == 0:
        last.clear(); b0 = e.block_avg.valid_blocks * e.block_size + e.block_avg.idx
        e.update_estimate()
        if "r" in last:
          lag, corr, conf = last["r"]; took = (e.block_avg.valid_blocks * e.block_size + e.block_avg.idx) != b0
          if took: n_acc += 1; acc_lags.append(lag)
          elif corr >= e.min_ncc and conf >= e.min_confidence and not e.press_free(): n_gate += 1
        msg = e.get_msg(True).lateralDelay
        if e.block_avg.valid_blocks >= 5:
          est_n += 1; inv_n += str(msg.status) == "invalid" or msg.status == log.LateralDelay.Status.invalid
          if first_est is None: first_est = f"{r[-2:]} {(t - t0)/60:.0f} min"
  vm, vs, cm, cs = e.block_avg.get(); msg = e.get_msg(True).lateralDelay
  print(f"  {r}: accepted {n_acc:4d}  gate-refused {n_gate:4d}  blocks {e.block_avg.valid_blocks:2d}  mean {vm:.3f} std {vs:.3f}  status {msg.status}  modeld gets {msg.lateralDelay:.3f}")
  vb = e.block_avg.valid_blocks; lag0 = cm if np.isfinite(cm) else lag0
a = np.array(acc_lags); edge = L.LAGD_MAX_LAG_S - 0.05
print(f"accepted total {len(a)}: median {np.median(a):.3f} IQR {np.percentile(a,25):.3f}-{np.percentile(a,75):.3f}  <0.45 {np.mean(a<0.45)*100:.1f}%  at search edge ({edge:.2f}) {np.mean(np.isclose(a, edge))*100:.1f}%")
h, ed = np.histogram(a, bins=np.arange(0.15, L.LAGD_MAX_LAG_S + 1e-9, 0.05)); print("  hist:", [(round(x, 2), int(c)) for x, c in zip(ed[:-1], h)])
print(f"invalid samples while >= 5 blocks: {inv_n}/{est_n}; first estimated: {first_est}")
