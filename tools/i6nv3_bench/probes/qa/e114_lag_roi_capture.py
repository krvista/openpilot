"""E114 (report §85): capture the lag cross-correlation itself. Replays the real estimator (CCNC profile, MAX_LAG widened
to 1.2 s so nothing is clipped) on the cached 7a-7f inputs and stores, per evaluation: route, t, confidence, the NCC over
lags 0.15..1.15 s (21 values at 0.05 s) and the context (v_ego, mean |desired la|, mean |actual la|, okay fraction) and whether the estimator accepted it
(is_valid: >= 25 s okay in the window, lat-accel range, new okay points).
Everything else (any MAX_LAG <= 1.2, acceptance rules, block statistics) is derived offline from this capture.
argv: S routes_csv out_pkl"""
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
from openpilot.selfdrive.locationd.helpers import fft_next_good_size
L.MAX_LAG = 1.2
S = sys.argv[1]; ROUTES = sys.argv[2].split(","); OUT = sys.argv[3]
class _CP: steerActuatorDelay = 0.1; carFingerprint = "X"
LAGS = np.arange(0.15, 1.1501, 0.05)
orig = L.LateralLagEstimator.actuator_delay; cap = {}
def spy(expected, actual, mask, dt, min_lag, max_lag):
  r = orig(expected, actual, mask, dt, min_lag, max_lag)
  n = len(expected); mx = int(round(1.2 / dt))
  ncc = L.masked_normalized_cross_correlation(expected, actual, mask, fft_next_good_size(n + max(mx, int(round(1.0 / dt)))))
  i0 = n - 1 + int(round(min_lag / dt))
  cap["r"] = (r, ncc[i0:i0 + len(LAGS)].astype(np.float32), float(np.mean(mask)),
              float(np.mean(np.abs(expected[mask]))) if mask.any() else 0.0, float(np.mean(np.abs(actual[mask]))) if mask.any() else 0.0)
  return r
L.LateralLagEstimator.actuator_delay = staticmethod(spy)
rows = []
for r in ROUTES:
  ev = pickle.load(open(f"{S}/lagd/{r}.pkl", "rb"))
  e = L.LateralLagEstimator(_CP(), 0.05, **L.CCNC_PROFILE_KW); n = 0; t0 = ev[0][0] * 1e-9
  for ts, which, b in ev:
    t = ts * 1e-9
    with log.Event.from_bytes(b) as m:
      e.handle_log(t, which, getattr(m, which))
    if which == "deviceMotion":
      e.update_points(); n += 1
      if n % 5 == 0:
        cap.clear(); b0 = e.block_avg.valid_blocks * e.block_size + e.block_avg.idx; e.update_estimate()
        if "r" in cap:
          (lag, corr, conf), ncc, okf, la_d, la_a = cap["r"]
          rows.append((r, t - t0, float(conf), ncc, okf, la_d, la_a, float(e.v_ego),
                       (e.block_avg.valid_blocks * e.block_size + e.block_avg.idx) != b0))
pickle.dump(dict(lags=LAGS, rows=rows), open(OUT, "wb"))
print(f"captured {len(rows)} evaluations ->", OUT)
