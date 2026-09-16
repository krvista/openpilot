#!/usr/bin/env python3
"""Attribution replay: drive the REAL Hyundai CarController (phase_tests harness Sim) with a route's logged carState /
carControl and record, per 100 Hz frame, what the on-device log cannot show — the driver-domain torque (raw - hold_comp),
the ACI gain call's gates (post_grip, city_release, anchored_recovery, suppress_error_boost, grip_start), curve_trim,
stall kick and the regenerated apply angle / gain. Validates itself against the logged LKAS_ALT (angle, gain) frame by
frame (parity), then summarises the 39a/39b questions from ROUTE8_REPORT §21.
usage: PYTHONPATH=$PWD:$PWD/opendbc_repo python3 tools/i6nv3_bench/attrib_replay.py <route> [--segs a-b] [--npz out.npz]"""
import argparse
import glob
import os
import sys
import numpy as np
sys.path.insert(0, os.getcwd()); sys.path.insert(0, os.path.join(os.getcwd(), "opendbc_repo"))
from openpilot.tools.lib.logreader import LogReader
from phase_tests.harness import Sim
import opendbc.car.hyundai.carcontroller as ccmod

ap = argparse.ArgumentParser()
ap.add_argument("route"); ap.add_argument("--segs", default=None); ap.add_argument("--npz", default=None)
ap.add_argument("--logdir", default="/home/user/drivelog/drivelog")
a = ap.parse_args()
files = sorted(glob.glob(f"{a.logdir}/*_{a.route}--*--rlog.zst"), key=lambda f: int(f.split("--")[-2]))
if a.segs:
  lo, hi = map(int, a.segs.split("-")); files = [f for f in files if lo <= int(f.split("--")[-2]) <= hi]
assert files, "no segments"

# record the gain call's inputs (driver-domain torque and the gates) without touching the controller
rec = {}
_orig = ccmod.compute_torque_reduction_gain
def _wrapped(steering_torque, v_ego_kph, lat_active, last_gain, steering_error, **kw):
  g = _orig(steering_torque, v_ego_kph, lat_active, last_gain, steering_error, **kw)
  rec.clear()
  rec.update(driver_tq=steering_torque, err=steering_error, gain=g,
             post_grip=bool(kw.get("post_grip", False)), city_release=bool(kw.get("city_release", False)),
             anchored=bool(kw.get("anchored_recovery", False)), boost_off=bool(kw.get("suppress_error_boost", False)),
             grip_start=float(kw.get("grip_start", 30.0)))
  return g
ccmod.compute_torque_reduction_gain = _wrapped

sim = Sim()
cs = None; wire = (None, None, None)   # (t, angle, gain) from the logged LKAS_ALT
lc_active = False; rej_t = -1e9         # model lane-change state and the last rejected LKAS_ALT echo (src >= 192)
rows = []
for f in files:
  for m in LogReader(f):
    w = m.which(); t = m.logMonoTime / 1e9
    if w == "carState":
      cs = m.carState
    elif w == "modelV2":
      lc_active = str(m.modelV2.meta.laneChangeState) not in ("off", "0")
    elif w == "can":
      for c in m.can:
        if c.address == 272 and c.src >= 192: rej_t = t
    elif w == "sendcan":
      for c in m.sendcan:
        if c.address == 272 and len(c.dat) >= 13:
          d = bytes(c.dat); raw = ((d[10] >> 2) | (d[11] << 6)) & 0x3FFF
          if raw & 0x2000: raw -= 0x4000
          wire = (t, raw * 0.1, d[12] * 0.004)
    elif w == "carControl" and cs is not None:
      cc = m.carControl
      rec.clear()
      sim.step(v=cs.vEgo, tq=cs.steeringTorque, wheel=cs.steeringAngleDeg, cmd=cc.actuators.steeringAngleDeg,
               lat_active=cc.latActive, enabled=cc.enabled, pressed=cs.steeringPressed,
               blinker=cs.leftBlinker, blinker_right=cs.rightBlinker, bs_l=cs.leftBlindspot, bs_r=cs.rightBlindspot,
               door=cs.doorOpen, belt=cs.seatbeltUnlatched, standstill=cs.standstill, gear=str(cs.gearShifter),
               cruise_available=cs.cruiseState.available, wheel_rate=cs.steeringRateDeg,
               cc_blinker_left=cc.leftBlinker, cc_blinker_right=cc.rightBlinker,
               cc_lc_active=lc_active, tx_rejected=(t - rej_t) < 0.03)
      s = sim.s
      wt, wa, wg = wire if wire[0] is not None and t - wire[0] < 0.05 else (None, np.nan, np.nan)
      rows.append((t, cs.vEgo * 3.6, abs(cs.steeringTorque), int(cs.steeringPressed), int(cc.latActive),
                   rec.get("driver_tq", np.nan), s.hold_comp_last, s.aci_gain_last, rec.get("err", np.nan),
                   int(rec.get("post_grip", False)), int(rec.get("city_release", False)), int(rec.get("anchored", False)),
                   int(rec.get("boost_off", False)), rec.get("grip_start", np.nan),
                   s.curve_trim, getattr(s, "stall_kick_deg", 0.0), s.apply_angle_last, cc.actuators.steeringAngleDeg,
                   cs.steeringAngleDeg, wa, wg, int(cs.leftBlinker or cs.rightBlinker)))
A = np.array(rows, dtype=float)
cols = "t v_kph raw_tq pressed lat driver_tq hold_comp gain err post_grip city_release anchored boost_off grip_start curve_trim kick apply cmd wheel wire_angle wire_gain blinker".split()
C = {k: i for i, k in enumerate(cols)}
if a.npz:
  np.savez_compressed(a.npz, data=A, cols=np.array(cols))

lat = A[:, C["lat"]] == 1
ok = lat & np.isfinite(A[:, C["wire_angle"]])
da = np.abs(A[ok, C["apply"]] - A[ok, C["wire_angle"]]); dg = np.abs(A[ok, C["gain"]] - A[ok, C["wire_gain"]])
print(f"== {a.route}: frames {len(A)} ({len(A)/6000:.1f} min), latActive {lat.mean()*100:.0f}%")
print(f"   PARITY vs logged LKAS_ALT (n={ok.sum()}): |apply-wire| p50/p90/p99 {np.median(da):.2f}/{np.percentile(da,90):.2f}/{np.percentile(da,99):.2f} deg | "
      f"|gain-wire| p50/p90/p99 {np.median(dg):.3f}/{np.percentile(dg,90):.3f}/{np.percentile(dg,99):.3f} | gain within 0.02 {np.mean(dg<=0.02)*100:.0f}%, angle within 0.5 deg {np.mean(da<=0.5)*100:.0f}%")
b = lat & (A[:, C["v_kph"]] >= 30) & (A[:, C["v_kph"]] < 45) & (A[:, C["blinker"]] == 0)
B = A[b]
if len(B):
  dt = B[:, C["driver_tq"]]; rt = B[:, C["raw_tq"]]
  print(f"   30-45 km/h latActive frames {len(B)} ({len(B)/6000:.1f} min): raw |tq| p25/50/75 {np.percentile(rt,25):.0f}/{np.median(rt):.0f}/{np.percentile(rt,75):.0f} Nm | "
        f"driver_tq p25/50/75 {np.nanpercentile(dt,25):.0f}/{np.nanmedian(dt):.0f}/{np.nanpercentile(dt,75):.0f} | hold_comp p50 {np.median(B[:, C['hold_comp']]):.0f}")
  print(f"   39a-eligible (driver_tq<30) {np.nanmean(dt<30)*100:.0f}% | city_release active {B[:, C['city_release']].mean()*100:.1f}% | post_grip {B[:, C['post_grip']].mean()*100:.0f}% | boost off {B[:, C['boost_off']].mean()*100:.0f}% | "
        f"39b band (driver_tq 30-60, pre-39b would yield) {np.nanmean((dt>=30)&(dt<60))*100:.1f}% | driver_tq >= grip_start {np.nanmean(dt>=B[:, C['grip_start']])*100:.0f}%")
  print(f"   request composition p50 |curve_trim| {np.median(np.abs(B[:, C['curve_trim']])):.2f} deg (at cap>=5.9: {np.mean(np.abs(B[:, C['curve_trim']])>=5.9)*100:.0f}%) | |kick| >0: {np.mean(np.abs(B[:, C['kick']])>0)*100:.1f}%")
  # releases: pressed 1->0 inside the band; was 39a active in the following 1 s? gap>3 share in 0-2 s
  P = A[:, C["pressed"]]; idx = np.where((P[1:] == 0) & (P[:-1] == 1))[0] + 1
  rel = []
  for i in idx:
    if not (lat[i] and 30 <= A[i, C["v_kph"]] < 45): continue
    seg = A[i:i + 200]
    if len(seg) < 200: continue
    gap = np.abs(seg[:, C["apply"]] - seg[:, C["wheel"]])
    rel.append((seg[:100, C["city_release"]].max() > 0, np.mean(gap > 3), np.nanmin(seg[:100, C["driver_tq"]]), seg[50, C["gain"]], seg[199, C["gain"]]))
  if rel:
    R = np.array(rel, dtype=float); on = R[:, 0] == 1
    print(f"   releases in band {len(R)}: 39a active within 1 s in {on.mean()*100:.0f}% | 0-2 s gap>3 share: 39a-active {np.mean(R[on,1])*100 if on.any() else float('nan'):.0f}% (n={on.sum()}) vs inactive {np.mean(R[~on,1])*100 if (~on).any() else float('nan'):.0f}% (n={(~on).sum()}) | "
          f"min driver_tq in 1 s p50 {np.nanmedian(R[:,2]):.0f} Nm | replayed gain p50 @0.5 s {np.median(R[:,3]):.2f}, @2 s {np.median(R[:,4]):.2f}")
