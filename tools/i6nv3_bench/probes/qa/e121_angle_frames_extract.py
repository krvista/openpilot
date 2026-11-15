"""E121 (report §91): the angle signals on the bus, per rlog segment file -> npz at 100 Hz (MDPS 0xEA timestamps):
STEERING_SENSORS.STEERING_ANGLE (what carState / control use), MDPS.STEERING_ANGLE, MDPS.STEERING_ANGLE_2, MDPS column and
output torque, MDPS angle-mode active; the op request (sendcan 0x110 angle / gain / active) and the stock camera's own
0x110 (bus 2: angle / gain / active, bytes 7, 8, 13 and the low nibble of byte 9); vEgo, steeringPressed, latActive.
n_tx = number of op 0x110 frames in the segment (0 in a stock-LFA capture drive, Phase 70). argv: rlog out.npz"""
import sys
import numpy as np
from openpilot.tools.lib.logreader import LogReader
f, out = sys.argv[1], sys.argv[2]
def s16(lo, hi):
  v = lo | (hi << 8)
  return v - 0x10000 if v & 0x8000 else v
def lk(dd):
  raw = ((dd[10] >> 2) | (dd[11] << 6)) & 0x3FFF
  if raw & 0x2000: raw -= 0x4000
  return raw * 0.1, dd[12] * 0.004, (dd[9] >> 4) & 0x3
mdps, sas, tx, cam, cs, cc = [], [], [], [], [], []; bus = [None, None]
for m in LogReader(f):
  w = m.which(); t = m.logMonoTime * 1e-9
  if w == "can":
    for c in m.can:
      if c.src < 128 and c.address == 234 and len(c.dat) >= 19 and (bus[0] is None or bus[0] == c.src):
        bus[0] = c.src
        dd = bytes(c.dat)
        mdps.append((t, s16(dd[12], dd[13]) * 0.1, s16(dd[16], dd[17]) * 0.1, ((dd[8] | (dd[9] << 8)) & 0xFFF) * 0.1 - 204.8,
                     ((dd[10] | (dd[11] << 8)) & 0x1FFF) - 4095, dd[18] & 0x3))
      elif c.src < 128 and c.address == 293 and len(c.dat) >= 6 and (bus[1] is None or bus[1] == c.src):
        bus[1] = c.src
        dd = bytes(c.dat); sas.append((t, s16(dd[3], dd[4]) * 0.1, dd[5] * 4.0))
      elif c.src == 2 and c.address == 272 and len(c.dat) >= 13:
        cam.append((t, *lk(bytes(c.dat)), c.dat[7], c.dat[8], c.dat[13], c.dat[9] & 0xF))
  elif w == "sendcan":
    for c in m.sendcan:
      if c.address == 272 and len(c.dat) >= 13:
        tx.append((t, *lk(bytes(c.dat))))
  elif w == "carState":
    cs.append((t, m.carState.vEgo, m.carState.steeringPressed, m.carState.steeringAngleDeg, m.carState.leftBlinker or m.carState.rightBlinker))
  elif w == "carControl":
    cc.append((t, m.carControl.latActive))
M = np.array(mdps, float); T = M[:, 0]
def res(a, col, hold=False):
  A = np.array(a, float)
  if not len(A): return np.zeros(len(T))
  if hold:
    return A[np.clip(np.searchsorted(A[:, 0], T, side="right") - 1, 0, len(A) - 1), col]
  return np.interp(T, A[:, 0], A[:, col])
np.savez_compressed(out, T=T, mdps1=M[:, 1], mdps2=M[:, 2], out_tq=M[:, 3], col_tq=M[:, 4], mdps_act=M[:, 5],
                    sas=res(sas, 1, True), sas_rate=res(sas, 2, True), tx=res(tx, 1, True), gain=res(tx, 2, True), tx_act=res(tx, 3, True),
                    cam=res(cam, 1, True), cam_gain=res(cam, 2, True), cam_act=res(cam, 3, True), cam_b7=res(cam, 4, True),
                    cam_b8=res(cam, 5, True), cam_b13=res(cam, 6, True), cam_b9lo=res(cam, 7, True), n_tx=np.array(len(tx)),
                    v=res(cs, 1), pressed=res(cs, 2, True), cs_ang=res(cs, 3, True), blinker=res(cs, 4, True), lat=res(cc, 1, True))
print(out, len(T), "buses mdps/sas", bus)
