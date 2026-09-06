#!/usr/bin/env python3
"""Process replay gate: run the REAL daemons (selfdrived, controlsd, card) on a logged segment
through openpilot's process_replay (fake messaging, real Params), one subprocess per daemon with a
hard timeout. A daemon that raises, hangs (the replay framework waits forever on a dead process —
that is what the first i6nv3 road-test crash looks like here), or publishes far fewer messages than
the log's input rate FAILS the gate. This is the leg that would have caught the audioFeedback
KeyError before the flash.

usage: PYTHONPATH=$PWD:$PWD/opendbc_repo python3 tools/i6nv3_bench/process_replay_check.py <rlog.zst> [procs] [timeout_s]
"""
import os, sys, json, subprocess, collections

CHILD = r'''
import sys, os, json, collections
sys.path.insert(0, os.getcwd()); sys.path.insert(0, os.path.join(os.getcwd(), "opendbc_repo"))
from openpilot.tools.lib.logreader import LogReader
from openpilot.selfdrive.test.process_replay.process_replay import replay_process_with_name
f, proc = sys.argv[1], sys.argv[2]
lr = list(LogReader(f))
n_in = collections.Counter(m.which() for m in lr)
out = replay_process_with_name([proc], lr, fingerprint="HYUNDAI_IONIQ_6_N", disable_progress=True)
n_out = collections.Counter(m.which() for m in out)
print("RESULT " + json.dumps({"proc": proc, "in": {k: n_in[k] for k in ("carState", "can", "selfdriveState", "carControl")}, "out": dict(n_out)}))
'''

EXPECT = {  # (output service, input service it should roughly track, min ratio)
  "selfdrived": ("selfdriveState", "carState", 0.8),
  "controlsd": ("carControl", "carState", 0.8),
  "card": ("carState", "carState", 0.8),
}

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
      if r.returncode != 0 or not res:
        print(f"FAIL {p}: exit {r.returncode}; " + (tb[-1] if tb else (r.stderr.strip().splitlines() or ['no output'])[-1][:200])); fail = 1; continue
      d = json.loads(res[-1][7:]); out_s, in_s, ratio = EXPECT.get(p, (None, None, 0))
      got = d["out"].get(out_s, 0); want = d["in"].get(in_s, 0) * ratio
      ok = got >= want
      print(f"{'OK  ' if ok else 'FAIL'} {p}: {out_s} {got} vs input {in_s} {d['in'].get(in_s,0)} (need >= {ratio:.0%})" + (f"; traceback in output: {tb[-1][:120]}" if tb else ""))
      if not ok or tb: fail = 1
    except subprocess.TimeoutExpired:
      print(f"FAIL {p}: TIMEOUT after {timeout}s — the daemon most likely died mid-replay (the framework then waits forever)"); fail = 1
  print("PROCESS REPLAY " + ("GREEN" if not fail else "FAILED"))
  sys.exit(fail)

if __name__ == "__main__":
  main()
