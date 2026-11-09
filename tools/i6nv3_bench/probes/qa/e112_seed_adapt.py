"""E112: verify Phase 64d seed + adaptivity. Replays the real estimator with the CCNC profile (seed on) on the cached
7a-7f inputs, carrying valid blocks across routes. Every 30 s it reads get_msg (what modeld receives): status, the
lateralDelay used, validBlocks, the running estimate. Shows: before learning lateralDelay = seed 0.55 (not 0.30); as
blocks accumulate the value adapts to the learned ~0.57; comparison vs seed off (0.30). argv: S routes_csv"""
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
class _CP: steerActuatorDelay = 0.1; carFingerprint = "HYUNDAI_IONIQ_6_N"
def make(seed):
  kw = dict(L.CCNC_PROFILE_KW)
  if not seed: kw.pop("initial_lag_seed", None)
  return L.LateralLagEstimator(_CP(), 0.05, **kw)
def status_name(s):
  return {log.LateralDelay.Status.estimated: "EST", log.LateralDelay.Status.unestimated: "unest", log.LateralDelay.Status.invalid: "INVALID"}.get(s, "?")
for seed in (False, True):
  e = make(seed); vb = 0; lag0 = e.initial_lag
  print(f"\n===== seed {'ON (0.55)' if seed else 'OFF (0.30)'} | initial_lag {e.initial_lag:.2f}")
  tglobal = 0.0; last = -999; printed = 0
  for r in ROUTES:
    ev = pickle.load(open(f"{S}/lagd/{r}.pkl", "rb"))
    e = make(seed); e.reset(lag0, vb); n = 0; t0 = ev[0][0] * 1e-9
    for ts, which, b in ev:
      t = ts * 1e-9
      with log.Event.from_bytes(b) as m:
        e.handle_log(t, which, getattr(m, which))
      if which == "deviceMotion":
        e.update_points(); n += 1
        if n % 5 == 0: e.update_estimate()
        tt = tglobal + (t - t0)
        if tt - last >= 30.0:
          last = tt; msg = e.get_msg(True); ld = msg.lateralDelay
          if printed < 40 and (ld.validBlocks > 0 or tt < 200 or seed):
            print(f"   t {tt/60:5.1f} min [{r}] | status {status_name(ld.status):6s} | lateralDelay(modeld) {ld.lateralDelay:.3f} | blocks {ld.validBlocks}/5 | estimate {ld.lateralDelayEstimate:.3f} +-{ld.lateralDelayEstimateStd:.3f}")
            printed += 1
    tglobal += (ev[-1][0] * 1e-9 - t0); vb = e.block_avg.valid_blocks
    _, _, cm, _ = e.block_avg.get(); lag0 = cm if np.isfinite(cm) else lag0
  msg = e.get_msg(True); ld = msg.lateralDelay
  print(f"   FINAL: status {status_name(ld.status)} | lateralDelay {ld.lateralDelay:.3f} | blocks {ld.validBlocks}/5 | estimate {ld.lateralDelayEstimate:.3f}")
