"""E74: why did the panda reject LKAS_ALT (0x110) frames? For each route: every sendcan 0x110, its echo (accepted: src
128+bus, rejected: src 192+bus), carState speed / wheel angle, pandaStates controlsAllowed(Lateral). For each REJECTED
frame, re-run the panda's checks (lateral.h steer_angle_cmd_checks_vm, Ioniq 6 N VM: steer ratio 13.7, wheelbase 2.756,
slip -0.000609; jerk 3.6 m/s^3, lat accel 3.6 m/s^2, speed fudge -1 m/s) against the previous frame's angle and classify:
rate (angle step over the jerk allowance) / lat-accel / inactive-angle (inactive frame angle != wheel) / not-allowed /
gain / other (rt counter, cascade after a reject, speed mismatch). Also the time since start (boot window)."""
import glob
import math
import sys
from collections import Counter, defaultdict
import numpy as np
from openpilot.tools.lib.logreader import LogReader
routes = sys.argv[1:]
SR, WB, SLIP = 13.7, 2.756, -0.0006085930193026732
JERK = 3.0 + 9.81 * 0.06; LACC = 3.0 + 9.81 * 0.06


def angle_from_curv(k, v):
  cf = 1.0 / (1.0 - SLIP * v * v) / WB
  return k * SR / cf * 57.29577951308232


def dec(dat):
  ang = (dat[11] << 6) | (dat[10] >> 2)
  if ang >= 1 << 13:
    ang -= 1 << 14
  return ang, (dat[9] >> 4) & 0x3, dat[12]


for r in routes:
  files = sorted(glob.glob(f"/home/user/drivelog/drivelog/*_{r}--*--rlog.zst"), key=lambda f: int(f.split("--")[-2]))
  sent = []; rej = []; acc_cnt = 0; cs = []; ps = []; t0 = None
  for f in files:
    try:
      for m in LogReader(f):
        t = m.logMonoTime * 1e-9; w = m.which()
        if t0 is None:
          t0 = t
        if w == "sendcan":
          for c in m.sendcan:
            if c.address == 0x110:
              sent.append((t - t0, c.src, *dec(bytes(c.dat))))
        elif w == "can":
          for c in m.can:
            if c.address == 0x110 and c.src >= 128:
              if c.src >= 192:
                rej.append((t - t0, c.src - 192, *dec(bytes(c.dat))))
              else:
                acc_cnt += 1
        elif w == "carState":
          cs.append((t - t0, m.carState.vEgoRaw, m.carState.steeringAngleDeg))
        elif w == "pandaStates" and len(m.pandaStates):
          p = m.pandaStates[0]
          ps.append((t - t0, float(p.controlsAllowed), float(getattr(p, "controlsAllowedLateral", False))))
    except Exception as e:  # noqa: BLE001
      print(f.split("/")[-1], type(e).__name__, e)
  CS = np.array(cs); PS = np.array(ps) if ps else np.zeros((1, 3)); S_ = np.array(sent, float)
  cat = Counter(); ex = defaultdict(list)
  for (t, bus, ang, act, gain) in rej:
    k = int(np.searchsorted(S_[:, 0], t)) - 1
    # the sent frame this echo belongs to: same bus and payload, latest before the echo; the panda compared it with the
    # frame sent before it on that bus
    me = None
    for j in range(k, max(k - 12, 0) - 1, -1):
      if S_[j, 1] == bus and S_[j, 2] == ang and S_[j, 3] == act and S_[j, 4] == gain:
        me = j; break
    prev = None
    if me is not None:
      for j in range(me - 1, max(me - 6, 0) - 1, -1):
        if S_[j, 1] == bus:
          prev = S_[j]; break
    i = min(int(np.searchsorted(CS[:, 0], t)), len(CS) - 1)
    v = max(CS[i, 1] - 1.0, 1.0); wheel = CS[i, 2] * 10
    pi = min(int(np.searchsorted(PS[:, 0], t)), len(PS) - 1)
    allowed = PS[pi, 1] > 0.5 or PS[pi, 2] > 0.5
    active = act != 1
    dmax = angle_from_curv(JERK / (v * v), v) / 100 * 10 + 1
    amax = angle_from_curv(LACC / (v * v), v) * 10 + 1
    step = abs(ang - prev[2]) if prev is not None else float("nan")
    if t < 25:
      c = "boot (< 25 s)"
    elif gain > 250 or (not active and gain != 0):
      c = "gain"
    elif active and not allowed:
      c = "active while not allowed"
    elif not active and abs(ang - wheel) > 2:
      c = "inactive angle != wheel"
    elif active and abs(ang) > amax:
      c = "lat accel (angle too large)"
    elif active and step > dmax:
      c = "rate (step over jerk limit)"
    else:
      c = "other (rt / cascade)"
    cat[c] += 1
    ex[c].append((t, v * 3.6 + 3.6, ang / 10, int(prev[3]) if prev is not None else -1, prev[2] / 10 if prev is not None else np.nan, step / 10, dmax / 10, act, gain, wheel / 10, allowed))
  op_h = (CS[-1, 0] - CS[0, 0]) / 3600
  print(f"== {r}: sent {len(sent)}, accepted echoes {acc_cnt}, REJECTED {len(rej)} over {op_h * 60:.0f} min | " + ", ".join(f"{k} {v}" for k, v in cat.most_common()))
  for c, L in ex.items():
    for e in L[:4]:
      print(f"     {c:30} t {e[0]:7.1f} v {e[1]:5.1f} km/h | angle {e[2]:+7.1f} prev-active-field {e[3]} prev {e[4]:+7.1f} step {e[5]:5.2f} (allow {e[6]:4.2f}) deg | active-field {e[7]} gain {e[8]} | wheel {e[9]:+7.1f} | allowed {e[10]}")
