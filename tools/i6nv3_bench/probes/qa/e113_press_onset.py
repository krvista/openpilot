"""E113 (report §83): Phase 64a min_press_sec onset leak. A press only counts as steering once held >= 0.5 s, so the
points of its first 0.5 s have already entered the window as okay. Counts those leaked okay points over all confirmed
presses and replays the real estimator (CCNC profile, seed on, 7a-7f carried across routes) with and without retroactively
un-okaying them, to see what the leak does to the learned delay. argv: S routes_csv"""
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


def run(retro):
  vb, lag0 = 0, L.LAGD_SEED_S
  presses = leaked = accepted = 0; ests = []
  for r in ROUTES:
    ev = pickle.load(open(f"{S}/lagd/{r}.pkl", "rb"))
    e = L.LateralLagEstimator(_CP(), 0.05, **L.CCNC_PROFILE_KW); e.reset(lag0, vb); n = 0; prev = False
    for ts, which, b in ev:
      t = ts * 1e-9
      with log.Event.from_bytes(b) as m:
        e.handle_log(t, which, getattr(m, which))
      if which == "carState" and e.steering_pressed and not prev:     # press just confirmed (held 0.5 s)
        presses += 1
        okl, tl = e.points.okay, e.points.times
        for i in range(1, len(okl) + 1):
          if tl[-i] < e.press_start_t:
            break
          leaked += okl[-i]
          if retro:
            okl[-i] = False
      prev = e.steering_pressed
      if which == "deviceMotion":
        e.update_points(); n += 1
        if n % 5 == 0:
          b0 = e.block_avg.valid_blocks * e.block_size + e.block_avg.idx
          e.update_estimate()
          accepted += (e.block_avg.valid_blocks * e.block_size + e.block_avg.idx) != b0
    vb = e.block_avg.valid_blocks
    vm, _, cm, _ = e.block_avg.get(); lag0 = cm if np.isfinite(cm) else lag0
    ests.append((r, vb, vm, cm))
  return presses, leaked, accepted, ests


for retro in (False, True):
  p, k, a, ests = run(retro)
  print(f"retro={retro}: confirmed presses {p}, leaked okay points {k} ({k * 0.05:.1f} s), accepted estimates {a}")
  for r, vb, vm, cm in ests:
    print(f"   after {r}: blocks {vb}, valid mean {vm:.4f}, current mean {cm:.4f}")
