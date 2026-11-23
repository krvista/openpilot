"""E170 (report §107): the real controlsd replayed over hands-off windows of a solo drive with the Phase 73 lane-centre
constants and with the shipped ones (open loop: the car's path is the logged one). Shows the size and the roughness
the changed term adds to the command. argv: S route t0 t1 [t0 t1 ...]   env: REPLAY_PROFILE=solo REPLAY_MODEL_DELAY=log"""
import sys
import numpy as np
sys.path.insert(0, "tools/i6nv3_bench/probes/qa")
import replay_state_control as R   # noqa: E402 - installs the stubs controlsd needs off the device
cd = R.cd
S, r = sys.argv[1], sys.argv[2]; R.load(S, r); d = R.d
NEW = dict(LANE_CENTER_KI=cd.LANE_CENTER_KI, LANE_CENTER_I_CAP=cd.LANE_CENTER_I_CAP, LANE_CENTER_KE=cd.LANE_CENTER_KE)
OLD = dict(LANE_CENTER_KI=0.03e-3, LANE_CENTER_I_CAP=0.15e-3, LANE_CENTER_KE=[0.0] * len(cd.LANE_CENTER_KE_SPEED_MS))
for t0, t1 in zip(sys.argv[3::2], sys.argv[4::2]):
  t0, t1 = float(t0), float(t1); out = {}
  for nm, c in (("old", OLD), ("new", NEW)):
    for k, v in c.items():
      setattr(cd, k, v)
    o = np.array([(a, b) for a, b, _, _ in R.replay("new", t0, t1)]); out[nm] = o
  T = out["old"][:, 0]; j = np.clip(np.searchsorted(d['T'], T), 0, len(d['T']) - 1)
  km = d['k_model'][j]; v = d['v'][j].mean(); deg = float(np.interp(v, *R.DEG_PER)) * 1e3
  line = f"{t0:.0f}-{t1:.0f} s, {v:.0f} km/h: logged k_ctrl - replay(old) p50 {np.median(np.abs(d['k_ctrl'][j] - out['old'][:, 1]))*1e3:.3f}e-3 |"
  for nm in ("old", "new"):
    lc = out[nm][:, 1] - km; hp = lc - np.convolve(lc, np.ones(200) / 200, 'same')
    line += f" {nm}: term mean {lc.mean()*1e3:+.3f}e-3 ({lc.mean()*deg:+.2f} deg), 2 s high-pass std {hp[200:-200].std()*deg:.3f} deg, rate rms {np.sqrt(np.mean(np.diff(lc)**2))*100*deg:.2f} deg/s |"
  print(line)
