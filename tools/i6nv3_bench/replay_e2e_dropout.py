# End-to-end Phase 39: real controlsd on route 4 seg 8 -> laneDropout frames; feed replayed controlsState into real selfdrived -> alert text
#!/usr/bin/env python3
"""End-to-end Phase 39 check: real controlsd on a segment with a logged lane-line dropout -> laneDropout frames;
the replayed controlsState is spliced into the log and real selfdrived is run on it -> the visual alert text.
usage: PYTHONPATH=$PWD:$PWD/opendbc_repo python3 tools/i6nv3_bench/replay_e2e_dropout.py <rlog.zst> [min_dropout_frames]"""
import sys, os, collections
sys.path.insert(0, os.getcwd()); sys.path.insert(0, os.path.join(os.getcwd(), "opendbc_repo"))
from openpilot.tools.lib.logreader import LogReader
from openpilot.selfdrive.test.process_replay.process_replay import replay_process
from tools.i6nv3_bench.process_replay_check import log_carparams_configs
f=sys.argv[1]; need=int(sys.argv[2]) if len(sys.argv)>2 else 1
lr=list(LogReader(f))
out=replay_process(log_carparams_configs(["controlsd"], lr), lr, disable_progress=True)
cs=[m for m in out if m.which()=="controlsState"]
ld=[m.controlsState.laneDropout for m in cs]
runs=[]; cur=None
for i,v in enumerate(ld):
    if v and cur is None: cur=i
    if not v and cur is not None: runs.append((cur,i)); cur=None
if cur is not None: runs.append((cur,len(ld)))
print("E2E controlsState:", len(cs), "laneDropout frames:", sum(ld), "runs(s):", [(round(a/100,1), round((b-a)/100,2)) for a,b in runs])
# splice replayed controlsState into the log and run selfdrived
lr2=[m for m in lr if m.which()!="controlsState"]+cs
lr2.sort(key=lambda m: m.logMonoTime)
out2=replay_process(log_carparams_configs(["selfdrived"], lr2), lr2, disable_progress=True)
ss=[m.selfdriveState for m in out2 if m.which()=="selfdriveState"]
alerts=collections.Counter((s.alertText1, s.alertText2, str(s.alertStatus)) for s in ss if s.alertText1)
print("E2E selfdriveState:", len(ss))
for k,v in alerts.most_common(12): print("  alert", v, k)
n_alert=sum(1 for s in ss if "Lane" in s.alertText1 and "Lost" in s.alertText1)
print("E2E laneDropout alert frames:", n_alert)
ok = sum(ld) >= need and n_alert >= need
print("E2E " + ("GREEN" if ok else "FAILED") + f" (need >= {need} latch frames and alert frames)")
sys.exit(0 if ok else 1)
