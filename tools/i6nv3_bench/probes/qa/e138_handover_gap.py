"""E138: the relay handover as the CAR saw it, for given rlog files. Around every safety-mode change: the LKAS_ALT (0x110)
and CAM_0x362 streams on the car side of the harness (bus 0 receptions = the camera's frames through the released relay,
src 128 = our own transmissions, src 192 = our frames the panda refused), the largest gaps between car-side frames, the
growth of the CAN error counters, and when the front camera fault signals (CCNC_0x162 FAULT_*, LFA ELK / LKA, FR_CMR DAW)
are set. With Phase 70b manager keeps loggerd alive until the relay is out, so the stock-LFA capture switch is in the
last ~0.5 s of the route it was made in.
usage: e138_handover_gap.py <rlog.zst> [...]"""
import sys
import zstandard
from opendbc.can.dbc import DBC
from opendbc.can.parser import get_raw_value
from openpilot.cereal import log

IDS = (0x110, 0x362)
dbc = DBC("hyundai_canfd_generated")
WATCH = [("CCNC_0x162", "FAULT_FCA"), ("CCNC_0x162", "FAULT_LSS"), ("CCNC_0x162", "FAULT_DAW"), ("CCNC_0x162", "FAULT_FSS"),
         ("CCNC_0x162", "FAULT_SLA"), ("LFA", "ELK_SysFlrSta"), ("LFA", "LKA_SysWrn"), ("FR_CMR_01_10ms", "DAW_SysSta")]
WATCH = [(m, s) for m, s in WATCH if m in dbc.name_to_msg and s in dbc.name_to_msg[m].sigs]
MSGS = {dbc.name_to_msg[m].address: dbc.name_to_msg[m] for m, _ in WATCH}


def events(path):
  it = log.Event.read_multiple_bytes(zstandard.ZstdDecompressor().stream_reader(open(path, "rb")).read())
  while True:
    try:
      yield next(it)
    except Exception:   # the end, or a log cut off mid-message
      return


for path in sys.argv[1:]:
  frames = {i: [] for i in IDS}; modes = []; faults = {}; state = {}; t0 = None; t = 0.0
  for e in events(path):
    if t0 is None: t0 = e.logMonoTime
    t = (e.logMonoTime - t0) / 1e9
    w = e.which()
    if w == "can":
      for c in e.can:
        if c.address in frames:
          frames[c.address].append((t, c.src))
        elif c.src == 1 and c.address in MSGS:
          msg = MSGS[c.address]
          for mn, sn in WATCH:
            if mn == msg.name:
              v = get_raw_value(bytes(c.dat), msg.sigs[sn])
              bad = v not in (0, 5) if sn == "DAW_SysSta" else v != 0
              if bad != state.get(sn, False):
                state[sn] = bad
                faults.setdefault(sn, []).append((round(t, 2), "SET" if bad else "clear", v))
    elif w == "pandaStates" and len(e.pandaStates):
      ps = e.pandaStates[0]
      modes.append((t, str(ps.safetyModel), ps.safetyTxBlocked,
                    [ps.canState0.totalErrorCnt, ps.canState1.totalErrorCnt, ps.canState2.totalErrorCnt]))
  print(f"== {path.split('/')[-1]}: {t:.1f} s, mono start {t0 / 1e9:.1f} s")
  print("  camera fault signals:", {k: v[:4] for k, v in faults.items()} or "none set")
  changes = [(a, b) for a, b in zip(modes, modes[1:]) if a[1] != b[1]]
  if not changes:
    print(f"  no safety-mode change ({modes[0][1] if modes else '?'} throughout)")
  for a, b in changes:
    tc = b[0]
    print(f"  --- {a[1]} -> {b[1]} reported at t={tc:.2f} s (pandaStates is 10 Hz: the switch is in the 0.1 s before); "
          f"CAN errors bus0/1/2 {a[3]} -> {b[3]}, tx blocked {a[2]} -> {b[2]}")
    for i in IDS:
      win = [(x, s) for x, s in frames[i] if tc - 1.0 <= x <= tc + 1.0]
      car = sorted(x for x, s in win if s in (0, 128))
      gaps = sorted(((x2 - x1, x1) for x1, x2 in zip(car, car[1:])), reverse=True)[:3]
      print(f"    {hex(i)} car-side largest gaps: " + ", ".join(f"{g * 1e3:.0f} ms @ {x:.2f}" for g, x in gaps))
      for s in sorted({s for _, s in win}):
        ts = sorted(x for x, q in win if q == s)
        name = {0: "camera on the car side", 2: "camera", 128: "ours, sent", 192: "ours, refused"}.get(s, f"src {s}")
        print(f"        {name:24s} {ts[0]:.3f} .. {ts[-1]:.3f} s ({len(ts)} frames)")
