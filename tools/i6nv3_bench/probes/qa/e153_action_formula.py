"""E153 (report §95): what IS the model's action? For given rlog segments, the logged modelV2.action.desiredCurvature
against modeld's plan formula 2 psi(T) / (v T) - psi_rate(0) / v at T = lagd's published lateralDelay + 0.075 s (frame +
action delay), psi = plan orientation z, v = carState.vEgo. Per segment (op steering, > 8 m/s): the told delay, the
formula's relative RMS difference to the logged action at T, the T at which the formula fits best, and - for comparison -
the single plan time tau whose curvature k_plan(tau) = plan yaw rate / plan speed is closest to the action and how well
that fits. A formula error near 0 means the action is not a point of the plan but that heading-matching average; the
plan-slice fit shows how far from 'the plan at T' it then is. argv: rlog.zst [...]"""
import sys
import numpy as np
import zstandard
from openpilot.cereal import log

T_IDXS = np.array([10.0 * (i / 32) ** 2 for i in range(33)])


def events(path):
  it = log.Event.read_multiple_bytes(zstandard.ZstdDecompressor().stream_reader(open(path, "rb")).read())
  while True:
    try:
      yield next(it)
    except Exception:   # the end, or a log cut off mid-message
      return


for path in sys.argv[1:]:
  v = 0.0; ld = None; lat = False; rows = []
  for e in events(path):
    w = e.which()
    if w == "carState":
      v = e.carState.vEgo
    elif w in ("lateralDelay", "liveDelay"):
      ld = getattr(e, w).lateralDelay
    elif w == "carControl":
      lat = e.carControl.latActive
    elif w == "modelV2" and ld is not None:
      m = e.modelV2
      psi = np.array(m.orientation.z); pr = np.array(m.orientationRate.z)
      if len(psi) == 33 and len(pr) == 33:
        rows.append((v, ld, m.action.desiredCurvature, lat, psi, pr, np.array(m.velocity.x)))
  if not rows:
    print(f"{path.split('/')[-1]}: no model frames"); continue
  V = np.maximum(np.array([r[0] for r in rows]), 1.0); LD = np.array([r[1] for r in rows]); K = np.array([r[2] for r in rows])
  ok = np.array([r[3] for r in rows]) & (V > 8)
  PSI = np.stack([r[4] for r in rows]); PR = np.stack([r[5] for r in rows]); VX = np.stack([r[6] for r in rows])
  form = lambda t: 2 * np.array([np.interp(tt, T_IDXS, p) for tt, p in zip(np.broadcast_to(t, len(PSI)), PSI)]) / (V * t) - PR[:, 0] / V
  rk = np.sqrt(np.mean(K[ok] ** 2))
  rms = lambda x: np.sqrt(np.mean((x[ok] - K[ok]) ** 2)) / rk
  T = LD + 0.075
  bt = min((rms(form(t)), t) for t in np.arange(0.2, 1.21, 0.025))
  sl = lambda tau: np.array([np.interp(tau, T_IDXS, p) for p in PR]) / np.maximum(np.array([np.interp(tau, T_IDXS, x) for x in VX]), 1.0)
  bs = min((rms(sl(tau)), tau) for tau in np.arange(0.2, 1.21, 0.05))
  print(f"{path.split('/')[-1]}: {int(ok.sum())} frames, {np.median(V[ok]) * 3.6:.0f} km/h | told {np.median(LD[ok]):.3f} s (T = {np.median(T[ok]):.3f}) | "
        f"formula at T: error {rms(form(T)):.3f} | best formula T {bt[1]:.3f} (error {bt[0]:.3f}) | closest plan slice tau {bs[1]:.2f} s (error {bs[0]:.3f})")
