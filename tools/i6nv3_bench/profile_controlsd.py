#!/usr/bin/env python3
"""In-process profile of the REAL controlsd (Controls.update/state_control/publish/run_ext) driven by a logged
segment: the log's carParams is seeded, Controls() is built normally, then its SubMaster is replaced by a
log-driven stand-in and one control iteration runs per logged selfdriveState (controlsd's poll service).
Prints wall ms/frame (no profiler) and the cProfile hotspots (with profiler).
usage: PYTHONPATH=$PWD:$PWD/opendbc_repo python3 tools/i6nv3_bench/profile_controlsd.py <rlog.zst> [frames] [--dump out.pkl]"""
import sys, os, time, cProfile, pstats, io, pickle
sys.path.insert(0, os.getcwd()); sys.path.insert(0, os.path.join(os.getcwd(), "opendbc_repo"))
from openpilot.tools.lib.logreader import LogReader
from openpilot.common.params import Params
import openpilot.cereal.messaging as messaging

class LogSM:
  """Minimal SubMaster stand-in fed from a log; every attribute controlsd touches."""
  def __init__(self, services):
    self.services = list(services)
    self.data = {}
    for s in self.services:
      try:
        msg = messaging.new_message(s)
      except Exception:
        msg = messaging.new_message(s, 0)     # list-typed services
      self.data[s] = getattr(msg.as_reader(), s)
    self.updated = dict.fromkeys(self.services, False)
    self.valid = dict.fromkeys(self.services, True)
    self.alive = dict.fromkeys(self.services, True)
    self.freq_ok = dict.fromkeys(self.services, True)
    self.logMonoTime = dict.fromkeys(self.services, 0)
    self.recv_frame = dict.fromkeys(self.services, 0)
    self.frame = 0
  def __getitem__(self, s): return self.data[s]
  def update(self, timeout=None): pass
  def all_checks(self, service_list=None): return True
  def all_alive(self, service_list=None): return True
  def all_valid(self, service_list=None): return True
  def all_freq_ok(self, service_list=None): return True

def main():
  f = sys.argv[1]; n_frames = int(sys.argv[2]) if len(sys.argv) > 2 and not sys.argv[2].startswith("--") else 3000
  dump = sys.argv[sys.argv.index("--dump") + 1] if "--dump" in sys.argv else None
  lr = list(LogReader(f))
  p = Params()
  cp = next(m for m in lr if m.which() == "carParams"); p.put("CarParams", cp.carParams.as_builder().to_bytes())
  cp_sp = next((m for m in lr if m.which() == "carParamsSP"), None)
  if cp_sp is not None: p.put("CarParamsSP", cp_sp.carParamsSP.as_builder().to_bytes())
  import openpilot.selfdrive.controls.controlsd as cd
  c = cd.Controls()
  real_services = list(c.sm.services)
  sm = LogSM(real_services); c.sm = sm
  # find messages of subscribed services; run one iteration per selfdriveState
  subs = set(real_services)
  def frames():
    for m in lr:
      w = m.which()
      if w in subs:
        sm.data[w] = getattr(m, w); sm.updated[w] = True; sm.logMonoTime[w] = m.logMonoTime; sm.recv_frame[w] = sm.frame
        if w == "selfdriveState":
          yield
          for s in subs: sm.updated[s] = False
          sm.frame += 1
  it = frames()
  for _ in range(200):     # warm-up (calibration, filters)
    next(it); c.update(); CC, lac = c.state_control(); c.publish(CC, lac); c.get_params_sp(sm); c.run_ext(sm, c.pm)
  rows = []
  def step():
    c.update(); CC, lac = c.state_control(); c.publish(CC, lac); c.get_params_sp(sm); c.run_ext(sm, c.pm)
    if dump: rows.append((CC.latActive, float(CC.actuators.curvature), float(CC.actuators.steeringAngleDeg), float(c.desired_curvature), bool(c.lane_dropout)))
  # 1) wall time without profiler
  t = []
  for _ in range(n_frames):
    next(it); t0 = time.perf_counter(); step(); t.append(time.perf_counter() - t0)
  import numpy as np
  t = np.array(t) * 1000
  print(f"WALL {n_frames} frames: mean {t.mean():.3f} ms  p50 {np.median(t):.3f}  p99 {np.percentile(t, 99):.3f}  max {t.max():.3f} ms/frame")
  # 2) profiled
  pr = cProfile.Profile(); pr.enable()
  for _ in range(n_frames):
    next(it); step()
  pr.disable()
  st = pstats.Stats(pr); s = io.StringIO(); st.stream = s
  st.sort_stats("tottime").print_stats(30); print(s.getvalue()[:6000])
  s = io.StringIO(); st.stream = s; st.sort_stats("cumulative").print_stats(40); print(s.getvalue()[:8000])
  if dump: pickle.dump(rows, open(dump, "wb")); print("dumped", len(rows), "rows ->", dump)

if __name__ == "__main__":
  main()
