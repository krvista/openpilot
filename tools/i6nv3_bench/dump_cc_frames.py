#!/usr/bin/env python3
"""Dump every CAN frame the REAL CarController produces on a logged segment (harness Sim driven by the log's
carState/carControl, exactly like preflight_replay.py) plus its per-frame internal state, and time
CarController.update. Two dumps (before/after a change) compared byte-for-byte = parity proof.
usage: PYTHONPATH=$PWD:$PWD/opendbc_repo python3 tools/i6nv3_bench/dump_cc_frames.py <route> <seg> <out.pkl>"""
import sys, os, glob, pickle, time
sys.path.insert(0, os.getcwd()); sys.path.insert(0, os.path.join(os.getcwd(), "opendbc_repo")); sys.path.insert(0, "phase_tests")
import numpy as np
from openpilot.tools.lib.logreader import LogReader
from opendbc.can.packer import CANPacker
import harness as H

route, seg, out = sys.argv[1], sys.argv[2], sys.argv[3]
f = sorted(glob.glob(f"/home/user/drivelog/drivelog/*_{route}--*--{seg}--rlog.zst"))[0]
lr = list(LogReader(f))
sim = H.Sim(); sim.cc.packer = CANPacker("hyundai_canfd_generated")
cc_cmd = 0.0; lat = False; en = False; mdps2 = None; rows = []; t = []
STATE = ("apply_angle_last", "tx_angle_last", "aci_gain_last", "rain_w", "wheel_outrun_passive", "tx_sat_frames")
for m in lr:
  w = m.which()
  if w == "can":
    for c in m.can:
      if c.address == 0xea and c.src == 1:
        d = bytes(c.dat); r = (d[17] << 8 | d[16]); r = r - 65536 if r >= 32768 else r; mdps2 = r * 0.1
  elif w == "carControl":
    cc_cmd = m.carControl.actuators.steeringAngleDeg; lat = m.carControl.latActive; en = m.carControl.enabled
  elif w == "carState":
    cs = m.carState
    t0 = time.perf_counter()
    msgs = sim.step(v=cs.vEgo, tq=cs.steeringTorque, wheel=cs.steeringAngleDeg, cmd=cc_cmd, lat_active=lat, enabled=en,
                    pressed=cs.steeringPressed, blinker=cs.leftBlinker, blinker_right=cs.rightBlinker, bs_l=cs.leftBlindspot,
                    bs_r=cs.rightBlindspot, standstill=cs.standstill, wheel_rate=cs.steeringRateDeg, mdps_angle_2=mdps2, v_raw=cs.vEgoRaw)
    t.append(time.perf_counter() - t0)
    frames = tuple((a, bytes(d), b) for a, d, b in msgs)
    st = tuple(getattr(sim.s, k, None) for k in STATE)
    rows.append((frames, st))
t = np.array(t) * 1000
print(f"DUMP {route} seg {seg}: {len(rows)} frames -> {out} | step wall mean {t.mean():.3f} p50 {np.median(t):.3f} p99 {np.percentile(t,99):.3f} ms")
pickle.dump(rows, open(out, "wb"))
