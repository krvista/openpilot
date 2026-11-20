"""E137: panda CAN controller state per safety mode for given rlog files — does the panda disturb the bus while the relay is
released (elm327 / noOutput, the factory camera connected straight to the car)? Prints the first / last pandaStates, every
safety-mode change and, per mode, the growth of the controllers' error counters and the frames received.
usage: e137_can_state_by_mode.py <rlog.zst> [...]"""
import collections
import sys
import zstandard
from openpilot.cereal import log


def events(path):
  it = log.Event.read_multiple_bytes(zstandard.ZstdDecompressor().stream_reader(open(path, "rb")).read())
  while True:
    try:
      yield next(it)
    except StopIteration:
      return
    except Exception as ex:   # a log cut off mid-message (the device lost power)
      print("  log ends mid-message:", str(ex)[:60]); return


def show(r, tag):
  print(f"  {tag:6s} t={r['t']:6.1f} mono={r['mono']:7.1f} {r['sm']:13s} txblk={r['txblk']} rxinv={r['rxinv']} hb_lost={int(r['hb'])} sbu={r['sbu']}")
  for i, b in enumerate(r["bus"]):
    print(f"         bus{i}: err={b['err']} rec={b['rec']} tec={b['tec']} last={b['le']}/{b['lde']} rxlost={b['rxlost']} txlost={b['txlost']} "
          f"busoff={b['boff']} rst={b['rst']} tx={b['tx']} rx={b['rx']} fwd={b['fwd']} fd={int(b['fd'])} brs={int(b['brs'])} {b['spd']}/{b['dspd']}")


for path in sys.argv[1:]:
  t0 = None; rows = []
  for e in events(path):
    if t0 is None: t0 = e.logMonoTime
    if e.which() != "pandaStates" or not len(e.pandaStates): continue
    ps = e.pandaStates[0]
    rows.append(dict(t=(e.logMonoTime - t0) / 1e9, mono=e.logMonoTime / 1e9, sm=str(ps.safetyModel), txblk=ps.safetyTxBlocked,
                     rxinv=ps.safetyRxInvalid, hb=ps.heartbeatLost, sbu=f"{ps.sbu1Voltage:.2f}/{ps.sbu2Voltage:.2f}",
                     bus=[dict(err=c.totalErrorCnt, rec=c.receiveErrorCnt, tec=c.transmitErrorCnt, le=str(c.lastError), lde=str(c.lastDataError),
                               rxlost=c.totalRxLostCnt, txlost=c.totalTxLostCnt, boff=c.busOffCnt, rst=c.canCoreResetCnt, tx=c.totalTxCnt,
                               rx=c.totalRxCnt, fwd=c.totalFwdCnt, fd=c.canfdEnabled, brs=c.brsEnabled, spd=c.canSpeed, dspd=c.canDataSpeed)
                          for c in (ps.canState0, ps.canState1, ps.canState2)]))
  print(f"== {path.split('/')[-1]}")
  if not rows:
    print("  no pandaStates"); continue
  show(rows[0], "first")
  for a, b in zip(rows, rows[1:]):
    if a["sm"] != b["sm"]:
      show(a, "before"); show(b, "after")
  show(rows[-1], "last")
  g = collections.defaultdict(lambda: [0.0, [0, 0, 0], [0, 0, 0]])
  for a, b in zip(rows, rows[1:]):
    if a["sm"] == b["sm"]:
      g[a["sm"]][0] += b["t"] - a["t"]
      for i in range(3):
        g[a["sm"]][1][i] += max(0, b["bus"][i]["err"] - a["bus"][i]["err"])
        g[a["sm"]][2][i] += max(0, b["bus"][i]["rx"] - a["bus"][i]["rx"])
  for k, (dur, errs, rxs) in g.items():
    print(f"  mode {k:13s} {dur:6.1f} s  new errors bus0/1/2 = {errs}  rx frames = {rxs}")
