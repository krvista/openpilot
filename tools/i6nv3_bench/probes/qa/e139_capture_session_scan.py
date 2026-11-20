"""E139: what the car received during a session, for given rlog files (one line block per file) - made for the stock-LFA
capture sessions. Per file: device uptime at the start, CarParams.passive, the panda safety-mode timeline (a car mode =
the harness relay intercepts), LKAS_ALT (0x110) / CAM_0x362 frame counts by source (bus 0 receptions = what reaches the
car from the camera, src 128 = ours sent, src 192 = ours refused, bus 2 = the camera's own output), the front camera
fault signals (when they are set / cleared), speed range, lateral-active time and the alerts shown.
usage: e139_capture_session_scan.py <rlog.zst> [...]"""
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
  cnt = {i: {} for i in IDS}; modes = []; faults = {}; state = {}; val = {}; t0 = None; t = 0.0
  passive = None; lat = 0; vmin = 1e9; vmax = 0.0; alerts = []; txblk = (None, None); bus_seen = {}
  for e in events(path):
    if t0 is None: t0 = e.logMonoTime
    t = (e.logMonoTime - t0) / 1e9
    w = e.which()
    if w == "can":
      for c in e.can:
        if c.address in cnt:
          cnt[c.address][c.src] = cnt[c.address].get(c.src, 0) + 1
        elif c.address in MSGS:
          msg = MSGS[c.address]
          bus_seen.setdefault(msg.name, set()).add(c.src)
          if c.src != 1:
            continue
          for mn, sn in WATCH:
            if mn == msg.name:
              v = get_raw_value(bytes(c.dat), msg.sigs[sn])
              bad = v not in (0, 5) if sn == "DAW_SysSta" else v != 0
              val[sn] = v
              if bad != state.get(sn):
                state[sn] = bad
                faults.setdefault(sn, []).append(f"{'SET' if bad else 'ok'}({v})@{t:.1f}")
    elif w == "pandaStates" and len(e.pandaStates):
      ps = e.pandaStates[0]
      m = str(ps.safetyModel)
      if not modes or modes[-1][1] != m:
        modes.append((round(t, 1), m))
      txblk = (txblk[0] if txblk[0] is not None else ps.safetyTxBlocked, ps.safetyTxBlocked)
    elif w == "carParams":
      passive = e.carParams.passive
    elif w == "carControl":
      lat += e.carControl.latActive
    elif w == "carState":
      vmin = min(vmin, e.carState.vEgo); vmax = max(vmax, e.carState.vEgo)
    elif w == "selfdriveState":
      a = e.selfdriveState.alertText1
      if a and (not alerts or alerts[-1][1] != a):
        alerts.append((round(t, 1), a))
  print(f"== {path.split('/')[-1]}: uptime {t0 / 1e9:.1f} s, {t:.1f} s, passive={passive}, v {vmin * 3.6:.0f}-{vmax * 3.6:.0f} km/h, latActive {lat / 100:.0f} s, tx refused {txblk[0]}->{txblk[1]}")
  print(f"   safety mode: {modes}")
  for i in IDS:
    c = cnt[i]
    print(f"   {hex(i)}: reaches the car from the camera (bus 0) {c.get(0, 0)}, ours sent {c.get(128, 0)}, ours refused {c.get(192, 0)}, camera's own output (bus 2) {c.get(2, 0)}")
  print("   camera faults: " + ("; ".join(f"{k} {' '.join(v[:5])}" for k, v in faults.items()) or "no signal seen"))
  if alerts:
    print(f"   alerts: {alerts[:6]}")
