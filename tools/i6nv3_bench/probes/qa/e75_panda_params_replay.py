"""E75: Phase 62 check on the real 31520b9 drives (76-79). Open-loop replay of the REAL CarController (logged speed /
torque / wheel / command), shipped settings, with the carcontroller safety VM either the car's own specs (Phase 62) or the
Sportage baseline (before). Every TX frame goes through a Python mirror of the panda's steer_angle_cmd_checks_vm with the
MATCHING panda table entry (own: SR 14.96 / WB 2.965 / slip -4.919e-4; old: 13.7 / 2.756 / -6.086e-4): predicted rejects per
cause; governor saturation share; and how much faster the request may move (p99 |TX step| per frame) at >= 55 km/h."""
import glob
import sys
import numpy as np
sys.path.insert(0, ".")
S = sys.argv[1]; MODE = sys.argv[2]
PAR = {"own": (14.96, 2.965, -0.0004919077400089671), "old": (13.7, 2.756, -0.0006085930193026732)}[MODE]
import opendbc.car.hyundai.carcontroller as ccmod  # noqa: E402
if MODE == "old":
  ccmod.ANGLE_SAFETY_OWN_SPECS = ()
from phase_tests.harness import Sim  # noqa: E402
J = A = 3.0 + 9.81 * 0.06


def lim(v):
  sr, wb, sl = PAR
  v = max(v - 1.0, 1.0)
  cf = 1.0 / (1.0 - sl * v * v) / wb
  to_ang = lambda k: k * sr / cf * 57.29577951308232
  return to_ang(J / (v * v)) / 100 * 10 + 1, to_ang(A / (v * v)) * 10 + 1     # CAN units (0.1 deg) incl. the +1


tot = dict(frames=0, act=0, rej_rate=0, rej_acc=0, rej_inact=0, sat=0, f55=0, steps=[])
for f in sorted(glob.glob(f"{S}/r50/0*.npz")):
  dz = np.load(f, allow_pickle=True)
  if str(dz['commit'])[:7] != "31520b9":
    continue
  d = {k: dz[k] for k in ('ang', 'v', 'tq_s', 'apply', 'lat', 'pressed', 'lb', 'rb', 'a_ego', 'gas', 'lane_min', 'bsl', 'bsr', 'rate')}
  n = len(d['v']); sim = Sim(); last = None
  for k in range(n):
    sim.step(v=float(d['v'][k]) / 3.6, tq=float(d['tq_s'][k]), wheel=float(d['ang'][k]), cmd=float(d['apply'][k]), lat_active=bool(d['lat'][k]),
             pressed=bool(d['pressed'][k]), blinker=bool(d['lb'][k]), blinker_right=bool(d['rb'][k]), a_ego=float(d['a_ego'][k]),
             gas=bool(d['gas'][k]), lane_prob_min=float(d['lane_min'][k]), bs_l=bool(d['bsl'][k]), bs_r=bool(d['bsr'][k]),
             wheel_rate=float(d['rate'][k]))
    m = sim.lkas_alt()
    if not m or not d['lat'][k] or d['v'][k] < 25:
      last = None; continue
    ang = int(round(m["ADAS_StrAnglReqVal"] * 10)); active = m["LKAS_ANGLE_ACTIVE"] == 2
    meas = int(round(float(d['ang'][k]) * 10))
    v = float(d['v'][k]) / 3.6
    dmax, amax = lim(v)
    tot['frames'] += 1
    viol = None
    if active:
      tot['act'] += 1
      if last is not None and abs(ang - last) > dmax:
        viol = "rate"
      elif abs(ang) > amax:
        viol = "acc"
      if v * 3.6 >= 55 and last is not None:
        tot['steps'].append(abs(ang - last) / 10); tot['f55'] += 1
    elif abs(ang - meas) > 1:
      viol = "inact"
    if viol:
      tot["rej_" + viol] += 1
    last = meas if (viol or not active) else ang
    tot['sat'] += (sim.cc.steer_debug_flags >> 27) & 1
mins = tot['frames'] / 6000
print(f"[{MODE}] {mins:.0f} min replayed (op >= 25 km/h) | predicted panda rejects: rate {tot['rej_rate']} accel {tot['rej_acc']} inactive {tot['rej_inact']} "
      f"({(tot['rej_rate'] + tot['rej_acc'] + tot['rej_inact']) / mins * 60:.1f}/h) | governor saturated {tot['sat'] / max(tot['frames'], 1):.1%} "
      f"| >= 55 km/h TX step per frame p99 {np.percentile(tot['steps'], 99):.2f} max {max(tot['steps']):.2f} deg")
