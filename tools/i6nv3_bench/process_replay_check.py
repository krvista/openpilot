#!/usr/bin/env python3
"""Process replay gate: run the REAL daemons (selfdrived, controlsd, card) on a logged segment
through openpilot's process_replay (fake messaging, real Params), one subprocess per daemon with a
hard timeout. A daemon that raises, hangs (the replay framework waits forever on a dead process —
that is what the first i6nv3 road-test crash looks like here), or publishes far fewer messages than
the log's input rate FAILS the gate. This is the leg that would have caught the audioFeedback
KeyError before the flash.

usage: PYTHONPATH=$PWD:$PWD/opendbc_repo python3 tools/i6nv3_bench/process_replay_check.py <rlog.zst> [procs] [timeout_s]
"""
import os, sys, json, subprocess

CHILD = r'''
import sys, os, json, collections
sys.path.insert(0, os.getcwd()); sys.path.insert(0, os.path.join(os.getcwd(), "opendbc_repo"))
from openpilot.tools.lib.logreader import LogReader
from openpilot.selfdrive.test.process_replay.process_replay import replay_process
from tools.i6nv3_bench.process_replay_check import log_carparams_configs
f, proc = sys.argv[1], sys.argv[2]
lr = list(LogReader(f))
n_in = collections.Counter(m.which() for m in lr)
out = replay_process(log_carparams_configs([proc], lr), lr, disable_progress=True)
n_out = collections.Counter(m.which() for m in out)
lat_in = sum(1 for m in lr if m.which() == "carControl" and m.carControl.latActive)
lat_out = sum(1 for m in out if m.which() == "carControl" and m.carControl.latActive)
cp_in = next(m.carParams for m in lr if m.which() == "carParams")
cp_out = next((m.carParams for m in out if m.which() == "carParams"), None)
cp_match = cp_out is None or (cp_out.steerControlType == cp_in.steerControlType and cp_out.flags == cp_in.flags
                              and cp_out.safetyConfigs[0].safetyParam == cp_in.safetyConfigs[0].safetyParam)
print("RESULT " + json.dumps({"proc": proc, "in": {k: n_in[k] for k in ("carState", "can", "selfdriveState", "carControl")}, "out": dict(n_out),
                              "latActive_in": lat_in, "latActive_out": lat_out, "card_cp_match": bool(cp_match)}))
'''

def log_carparams_configs(procs, lr):
  """Process configs whose init writes the LOG's carParams/carParamsSP into Params.
  process_replay's fingerprint path builds CarInterface.get_non_essential_params(fingerprint) — the GENERIC
  Ioniq 6 N params (torque steering, no CCNC / LKA_STEER_MSG_ALT flags, safety param 2089). A replay on those
  never enters the angle-steering lateral path at all (found when a lane-dropout latch that fires on the log
  showed 0 frames in replay). The car's own carParams (angle, CCNC flags, 2225) is in every full segment."""
  import dataclasses
  from openpilot.common.params import Params
  from openpilot.selfdrive.test.process_replay.process_replay import get_process_config
  cp = next((m for m in lr if m.which() == "carParams"), None)
  assert cp is not None, "carParams message not found"
  cp_sp = next((m for m in lr if m.which() == "carParamsSP"), None)
  def cb(rc, pm, msgs, fingerprint):
    params = Params()
    params.put("CarParams", cp.carParams.as_builder().to_bytes(), block=True)
    if cp_sp is not None:
      params.put("CarParamsSP", cp_sp.carParamsSP.as_builder().to_bytes(), block=True)
  def card_cb(rc, pm, msgs, fingerprint):
    # card fingerprints itself: seed the car's own carParams as the FW cache (REPLAY=1 lets the cache be used
    # without a VIN round-trip) so the FW match lands on the same car, and let it derive the CCNC flags from CAN
    os.environ["REPLAY"] = "1"
    params = Params()
    params.remove("CarParams")
    params.put("CarParamsCache", cp.carParams.as_builder().to_bytes(), block=True)
    get_process_config("card").init_callback(rc, pm, msgs, fingerprint)
  return [dataclasses.replace(get_process_config(p), init_callback=(card_cb if p == "card" else cb)) for p in procs]


EXPECT = {  # (output service, input service it should roughly track, min ratio)
  "selfdrived": ("selfdriveState", "carState", 0.8),
  "controlsd": ("carControl", "selfdriveState", 0.8),   # controlsd polls selfdriveState (a crashed on-car selfdrived leaves few of them)
  "card": ("carState", "carState", 0.8),
}
# card also needs the sunnypilot NN-lateral data submodule checked out (openpilot/sunnypilot/neural_network_data) —
# the device has it; a bare worktree does not (card dies in setup_interfaces -> FileNotFoundError)

def main():
  f = sys.argv[1]; procs = (sys.argv[2] if len(sys.argv) > 2 else "selfdrived,controlsd,card").split(",")
  timeout = int(sys.argv[3]) if len(sys.argv) > 3 else 420
  env = dict(os.environ, PYTHONPATH=f"{os.getcwd()}:{os.getcwd()}/opendbc_repo")
  fail = 0
  for p in procs:
    try:
      r = subprocess.run([sys.executable, "-c", CHILD, f, p], capture_output=True, text=True, timeout=timeout, env=env)
      res = [l for l in r.stdout.splitlines() if l.startswith("RESULT ")]
      tb = [l for l in (r.stdout + r.stderr).splitlines() if "Traceback" in l or "Error:" in l and "commIssue" not in l]
      if "carParams message not found" in (r.stdout + r.stderr):
        print(f"SKIP {p}: segment has no carParams message (short trailing segment) — not a daemon failure"); continue
      if r.returncode != 0 or not res:
        print(f"FAIL {p}: exit {r.returncode}; " + (tb[-1] if tb else (r.stderr.strip().splitlines() or ['no output'])[-1][:200])); fail = 1; continue
      d = json.loads(res[-1][7:]); out_s, in_s, ratio = EXPECT.get(p, (None, None, 0))
      got = d["out"].get(out_s, 0); want = d["in"].get(in_s, 0) * ratio
      ok = got >= want
      lat = ""
      if p == "card":
        cp_ok = d.get("card_cp_match", True)
        lat = "" if cp_ok else "; CARD FINGERPRINTED A DIFFERENT CONFIG THAN THE LOG (steer type / safety param / flags)"
        ok = ok and cp_ok
      if p == "controlsd":
        # the lateral path must actually run: replayed latActive frames should track the logged ones
        li, lo = d.get("latActive_in", 0), d.get("latActive_out", 0)
        lat_ok = li == 0 or lo >= 0.8 * li
        lat = f"; latActive frames replay {lo} vs log {li}" + ("" if lat_ok else " (LATERAL PATH NOT EXERCISED)")
        ok = ok and lat_ok
      print(f"{'OK  ' if ok else 'FAIL'} {p}: {out_s} {got} vs input {in_s} {d['in'].get(in_s,0)} (need >= {ratio:.0%}){lat}" + (f"; traceback in output: {tb[-1][:120]}" if tb else ""))
      if not ok or tb: fail = 1
    except subprocess.TimeoutExpired:
      print(f"FAIL {p}: TIMEOUT after {timeout}s — the daemon most likely died mid-replay (the framework then waits forever)"); fail = 1
  print("PROCESS REPLAY " + ("GREEN" if not fail else "FAILED"))
  sys.exit(fail)

if __name__ == "__main__":
  main()
