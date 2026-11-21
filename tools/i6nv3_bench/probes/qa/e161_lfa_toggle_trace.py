"""E161 (report §96): what switches the factory LFA on and off - a trace of one raw segment (rlog).
Prints, in time order, every change of
  the LFA button bit the camera puts in LKAS_ALT (bit 56, bus 2)        - a press of the steering-wheel LFA button
  the same bit in the LKAS_ALT openpilot transmits (sendcan)             - the 'MADS <-> cluster sync pulse'
  HDA_LFA_SymSta and HDA_ICON of ADAS_CMD_31_50ms (0x1E0, bus 1)         - the factory LFA / HDA state
  MADS enabled (selfdriveStateSP), carControl.latActive, cruiseState.enabled / available
so one can read off what preceded each factory-LFA toggle (a button press, openpilot's pulse, or the cruise engaging).
argv: rlog [t_from t_to]  (seconds of logMonoTime; default all)"""
import sys
from openpilot.tools.lib.logreader import LogReader
f = sys.argv[1]; lo = float(sys.argv[2]) if len(sys.argv) > 2 else -1e18; hi = float(sys.argv[3]) if len(sys.argv) > 3 else 1e18
st = dict(cam=0, tx=0, sym=-1, hda=-1, lat=0, mads=None, cruise=0, main=None); out = []
try:
  for m in LogReader(f):
    t = m.logMonoTime * 1e-9; w = m.which()
    if w == "can":
      for c in m.can:
        if c.address == 272 and c.src == 2 and len(c.dat) >= 8:
          b = c.dat[7] & 1
          if b != st['cam']:
            out.append((t, f"camera LFA button bit {st['cam']}->{b}")); st['cam'] = b
        elif c.address == 480 and c.src == 1 and len(c.dat) >= 8:
          bits = int.from_bytes(bytes(c.dat)[:8], "little"); s = (bits >> 47) & 3; h = (bits >> 31) & 1
          if s != st['sym']:
            out.append((t, f"factory LFA symbol {st['sym']}->{s}")); st['sym'] = s
          if h != st['hda']:
            out.append((t, f"HDA icon {st['hda']}->{h}")); st['hda'] = h
    elif w == "sendcan":
      for c in m.sendcan:
        if c.address == 272 and len(c.dat) >= 8:
          b = c.dat[7] & 1
          if b != st['tx']:
            out.append((t, f"openpilot TX LFA button bit {st['tx']}->{b}")); st['tx'] = b
    elif w == "carControl":
      v = int(m.carControl.latActive)
      if v != st['lat']:
        out.append((t, f"latActive {st['lat']}->{v}")); st['lat'] = v
    elif w == "carState":
      cs = m.carState; v = int(cs.cruiseState.enabled)
      if v != st['cruise']:
        out.append((t, f"cruise enabled {st['cruise']}->{v} ({cs.vEgo * 3.6:.0f} km/h)")); st['cruise'] = v
      v = int(cs.cruiseState.available)
      if v != st['main']:
        out.append((t, f"cruise available {st['main']}->{v}")); st['main'] = v
    elif w == "selfdriveStateSP":
      try:
        v = int(m.selfdriveStateSP.mads.enabled)
      except Exception:  # noqa: BLE001
        continue
      if v != st['mads']:
        out.append((t, f"MADS enabled {st['mads']}->{v}")); st['mads'] = v
except Exception as e:  # noqa: BLE001 - truncated segment
  print(f"({type(e).__name__}: {e})")
for t, s in sorted(out):
  if lo <= t <= hi:
    print(f"{t:10.2f}  {s}")
