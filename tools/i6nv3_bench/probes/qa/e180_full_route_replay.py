"""e180: continuous replay of the real controlsd.state_control (HEAD) over a whole route, recording the
internals of the lane-centre / entry-lead additions per 100 Hz frame.  argv: route variant [t0 t1]
env REPLAY_SCRATCH (inputs), REPLAY_OUT (output dir)
variants: head | nofit (LANE_FIT_GAIN 0 + Phase 79 KE) | noavoid | nolead | noexempt
| f4560 (lane-fit gate of Phase 82) | f3550 | f4055 (gates tried for Phase 88)"""
import os, sys, types, time
os.environ.setdefault("REPLAY_MODEL_DELAY", "log")
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import replay_state_control as R
cd = R.cd; log = R.log
S = os.environ.get("REPLAY_SCRATCH", ".")   # directory holding r50/ and la/ (route and plan npz)
OUT = os.environ.get("REPLAY_OUT", ".")

LCS = {0: log.LaneChangeState.off, 1: log.LaneChangeState.preLaneChange, 2: log.LaneChangeState.laneChangeStarting,
       3: log.LaneChangeState.laneChangeFinishing}

def model_at(i):
  P, d = R.P, R.d
  ln = lambda y: types.SimpleNamespace(x=P['LX'][i].tolist(), y=y)
  j = min(np.searchsorted(d['T'], P['T'][i]), len(d['T']) - 1)
  return types.SimpleNamespace(
    action=types.SimpleNamespace(desiredCurvature=float(P['K'][i])),
    position=types.SimpleNamespace(x=P['PX'][i].tolist(), y=P['PY'][i].tolist(), yStd=[0.01] * 33),
    orientationRate=types.SimpleNamespace(z=P['OR'][i].tolist()), velocity=types.SimpleNamespace(x=P['VX'][i].tolist()),
    orientation=types.SimpleNamespace(z=P['PSI'][i].tolist()),
    laneLines=[ln((P['L1'][i] - 3.5).tolist()), ln(P['L1'][i].tolist()), ln(P['L2'][i].tolist()), ln((P['L2'][i] + 3.5).tolist())],
    laneLineProbs=P['PR'][i].tolist(),
    meta=types.SimpleNamespace(laneChangeState=LCS[int(d['lcs'][j])], laneChangeDirection=log.LaneChangeDirection.none))

def setvar(variant):
  if variant == "nofit":
    cd.LANE_FIT_GAIN = 0.0; cd.LANE_CENTER_KE = [0.0, 0.3e-3, 0.3e-3, 0.15e-3]
  elif variant == "noavoid":
    cd.LANE_AVOID_RELEASE = False
  elif variant == "nolead":
    cd.ENTRY_LEAD_S = 0.0
  elif variant == "noexempt":
    cd.CONF_BLEND_LC_EXEMPT = False
  elif variant == "f4560":
    cd.LANE_FIT_SPEED_MS = [45.0 / 3.6, 60.0 / 3.6]
  elif variant == "f3550":
    cd.LANE_FIT_SPEED_MS = [35.0 / 3.6, 50.0 / 3.6]
  elif variant == "f4055":
    cd.LANE_FIT_SPEED_MS = [40.0 / 3.6, 55.0 / 3.6]
  elif variant != "head":
    raise SystemExit("variant?")

def run(route, variant, t0=None, t1=None):
  R.load(S, route); d, P, LD = R.d, R.P, R.LD
  setvar(variant)
  import openpilot.selfdrive.controls.lib.lowconf_rate_cap as lrc
  lrc.LOWCONF_CAP_OWN_STATE = True
  s = R.H.mk_controls(); s.VM = R.VM()
  rec = {}
  o_lc, o_el = cd.Controls._lane_center, cd.Controls._entry_lead
  def lc(self, mf, CS, k, ok):
    r = o_lc(self, mf, CS, k, ok); rec['lc'] = (k, float(ok), self._lc_p, self._lc_i, self._lc_e, self._lc_f, self._lc_wa,
                                               self._lc_usual if self._lc_usual is not None else np.nan); return r
  def el(self, mf, CS, k, ok):
    r = o_el(self, mf, CS, k, ok); rec['el'] = r; return r
  cd.Controls._lane_center, cd.Controls._entry_lead = lc, el
  Tr = d['T']; a0 = 0 if t0 is None else np.searchsorted(Tr, t0); a1 = len(Tr) if t1 is None else np.searchsorted(Tr, t1)
  cs = s.sm['carState']; s.desired_curvature = float(d['k_ctrl'][a0])
  out = np.full((a1 - a0, 13), np.nan, dtype=np.float32); cur = -1; m = None
  PT = P['T']; pi = np.maximum(np.searchsorted(PT, Tr, side='right') - 1, 0)
  ldi = np.maximum(np.searchsorted(LD[:, 0], Tr, side='right') - 1, 0) if LD is not None else None
  tic = time.time()
  for j in range(a0, a1):
    cs.vEgo = float(d['v'][j]) / 3.6; cs.steeringAngleDeg = float(d['ang'][j]); cs.steeringPressed = bool(d['pressed'][j])
    cs.steeringTorque = float(d['tq_s'][j]); cs.leftBlinker = bool(d['lb'][j]); cs.rightBlinker = bool(d['rb'][j])
    lat = bool(d['lat'][j]); s.get_lat_active = (lambda sm, a=lat: a)
    s.sm['lateralDelay'].lateralDelay = float(LD[ldi[j], 1]) if LD is not None else 0.3
    i = pi[j]
    if i != cur:
      m = model_at(i); cur = i; s.sm.logMonoTime['modelV2'] = i
    s.sm['modelV2'] = m; rec.clear()
    CC, _ = cd.Controls.state_control(s)
    row = out[j - a0]; row[0] = float(CC.actuators.curvature); row[1] = s.steer_flags; row[2] = float(s.lane_dropout)
    if 'lc' in rec:
      row[3:11] = rec['lc']; row[11] = rec.get('el', 0.0)
    row[12] = float(CC.latActive)
  cd.Controls._lane_center, cd.Controls._entry_lead = o_lc, o_el
  print(f"{route} {variant}: {a1-a0} frames in {time.time()-tic:.0f} s", flush=True)
  return Tr[a0:a1], out

if __name__ == "__main__":
  route, variant = sys.argv[1], sys.argv[2]
  t0 = float(sys.argv[3]) if len(sys.argv) > 3 else None; t1 = float(sys.argv[4]) if len(sys.argv) > 4 else None
  T, out = run(route, variant, t0, t1)
  np.savez_compressed(f"{OUT}/rep_{route}_{variant}{'' if t0 is None else '_part'}.npz", T=T, O=out)
