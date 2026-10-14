"""Lookahead study, step 1: per modelV2 frame (20 Hz), same t0 as route_extract (first message of seg 0):
t, action desiredCurvature, plan position x/y (33), plan orientation-rate z (33), plan velocity x (33),
inner lane lines y (lines 1, 2; 33 points at their own x), lane probs (4), lane-line x grid of line 1."""
import glob, sys
import numpy as np
from openpilot.tools.lib.logreader import LogReader
r, out = sys.argv[1], sys.argv[2]
files = sorted(glob.glob(f"/home/user/drivelog/drivelog/*_{r}--*--rlog.zst"), key=lambda f: int(f.split("--")[-2]))
t0 = None; T, K, PX, PY, OR, VX, L1, L2, LX, PR = [], [], [], [], [], [], [], [], [], []
for f in files:
  try:
    for m in LogReader(f):
      t = m.logMonoTime * 1e-9
      if t0 is None: t0 = t
      if m.which() != "modelV2": continue
      d = m.modelV2
      if len(d.position.x) != 33 or len(d.laneLines) < 4 or len(d.orientationRate.z) != 33: continue
      T.append(t - t0); K.append(d.action.desiredCurvature)
      PX.append(list(d.position.x)); PY.append(list(d.position.y)); OR.append(list(d.orientationRate.z)); VX.append(list(d.velocity.x))
      L1.append(list(d.laneLines[1].y)); L2.append(list(d.laneLines[2].y)); LX.append(list(d.laneLines[1].x)); PR.append(list(d.laneLineProbs))
  except Exception as e:  # noqa: BLE001
    print(f.split('/')[-1], type(e).__name__, e)
f32 = lambda a: np.array(a, np.float32)
np.savez_compressed(out, T=np.array(T), K=f32(K), PX=f32(PX), PY=f32(PY), OR=f32(OR), VX=f32(VX), L1=f32(L1), L2=f32(L2), LX=f32(LX), PR=f32(PR))
print(r, len(T))
