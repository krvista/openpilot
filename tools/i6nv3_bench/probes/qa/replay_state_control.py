"""Replay the REAL controlsd.state_control over a logged window: carState (100 Hz route npz) + modelV2 rebuilt from the
plan npz (action, plan position/yaw rate/speed, inner lane lines, lane probs, lane-change state). Vehicle model:
curvature <-> wheel angle from the drives' own median ratio per speed (deg per 1e-3/m). Variants: 'logged' = the 55b
build's constants (Phase 56 off) — must reproduce controlsState.desiredCurvature; 'new' = shipped constants.
Usage (repo root): PYTHONPATH=$PWD:$PWD/opendbc_repo python3 .../replay_state_control.py <scratch> <route> <t0> <t1>"""
import math
import sys
import types

import numpy as np

from phase_tests.harness_noncontrol import FakeParams  # noqa: F401
from phase_tests import test_noncontrol_controlsd_state as H
import openpilot.selfdrive.controls.controlsd as cd
from openpilot.cereal import log

P: dict = {}
d: dict = {}


DRIVEN = {"8de98262a": "logged", "a0108a47b": "new"}   # other (older) builds drove the cubic lookahead


def load(S, r):
  global P, d
  import glob
  P = dict(np.load(f"{S}/la/plan_{r}.npz"))
  rf = (glob.glob(f"{S}/r46/{r}.npz") + glob.glob(f"{S}/r43/{r}.npz") + glob.glob(f"{S}/r3d/{r}.npz")
        + glob.glob(f"{S}/ropm/{r}.npz") + glob.glob(f"{S}/rx/{r}.npz") + glob.glob(f"{S}/r50/{r}.npz"))
  d = dict(np.load(rf[0], allow_pickle=True))
DEG_PER = ([40, 60, 80, 100], [2.85, 3.12, 3.45, 3.74])     # wheel deg per 1e-3/m (qa/e9)


class VM:
  def update_params(self, x, sr):
    pass

  def calc_curvature(self, angle, v, roll):
    return math.degrees(angle) / float(np.interp(v * 3.6, *DEG_PER)) * 1e-3

  def get_steer_from_curvature(self, curv, v, roll):
    return math.radians(curv * 1e3 * float(np.interp(v * 3.6, *DEG_PER)))


def model_at(i):
  ln = lambda y: types.SimpleNamespace(x=P['LX'][i].tolist(), y=y)  # noqa: E731
  lcs = log.LaneChangeState.preLaneChange if (d['lb'] | d['rb'] | (d['lcs'] > 0))[np.searchsorted(d['T'], P['T'][i])] else log.LaneChangeState.off
  return types.SimpleNamespace(
    action=types.SimpleNamespace(desiredCurvature=float(P['K'][i])),
    position=types.SimpleNamespace(x=P['PX'][i].tolist(), y=P['PY'][i].tolist(), yStd=[0.01] * 33),
    orientationRate=types.SimpleNamespace(z=P['OR'][i].tolist()), velocity=types.SimpleNamespace(x=P['VX'][i].tolist()),
    laneLines=[ln((P['L1'][i] - 3.5).tolist()), ln(P['L1'][i].tolist()), ln(P['L2'][i].tolist()), ln((P['L2'][i] + 3.5).tolist())],
    laneLineProbs=P['PR'][i].tolist(),
    meta=types.SimpleNamespace(laneChangeState=lcs, laneChangeDirection=log.LaneChangeDirection.none))


def replay(variant, t0, t1):
  saved = {k: getattr(cd, k) for k in ("TURN_EXIT_K", "LANE_DROPOUT_COUNTER_TQ", "LOOKAHEAD_ORIENT_WEIGHT_SHRINK",
                                       "LOOKAHEAD_ORIENT_DEADBAND", "LOOKAHEAD_ORIENT_GATE", "LOOKAHEAD_ORIENT_T_S",
                                       "LOOKAHEAD_ORIENT_W_SLEW_S", "LOOKAHEAD_ORIENT_GROW_SPAN", "LOOKAHEAD_ORIENT_WEIGHT",
                                       "LOOKAHEAD_JERK_BUDGET", "TURN_EXIT_SOFT_EXEMPT", "LOWCONF_CURVE_K")}
  if variant in ("logged", "c55only", "cubic") or variant.startswith("x_"):          # Phase 56 off
    cd.TURN_EXIT_K = cd.LANE_DROPOUT_COUNTER_TQ = 1e9
  if variant == "logged":                                # the 55b build that was driven
    cd.LOOKAHEAD_ORIENT_WEIGHT_SHRINK, cd.LOOKAHEAD_ORIENT_DEADBAND, cd.LOOKAHEAD_ORIENT_GATE = 0.5, 0.0, [0.0, 0.0]
  if variant == "cubic":                                 # Phase 41-2 build (before Phase 55)
    cd.LOOKAHEAD_ORIENT_T_S = 0.0
  if variant == "x_nodb":
    cd.LOOKAHEAD_ORIENT_DEADBAND = 0.0
  if variant == "x_nogate":
    cd.LOOKAHEAD_ORIENT_GATE = [0.0, 0.0]
  if variant == "x_w5":
    cd.LOOKAHEAD_ORIENT_WEIGHT_SHRINK = 0.5
  # sensitivity set (Phase 56 ON): p_<param>=<value>
  if variant.startswith("p_"):
    k, v = variant[2:].split("=")
    setattr(cd, {"shrink": "LOOKAHEAD_ORIENT_WEIGHT_SHRINK", "db": "LOOKAHEAD_ORIENT_DEADBAND", "span": "LOOKAHEAD_ORIENT_GROW_SPAN",
                 "w": "LOOKAHEAD_ORIENT_WEIGHT", "ts": "LOOKAHEAD_ORIENT_T_S", "J": "LOOKAHEAD_JERK_BUDGET",
                 "tek": "TURN_EXIT_K", "ctq": "LANE_DROPOUT_COUNTER_TQ"}[k], float(v))
  import openpilot.selfdrive.controls.lib.lowconf_rate_cap as lrc
  if variant in ("logged", "new", "c55only", "cubic", "x_nosoft", "x_noslew") or variant.startswith(("x_", "p_")):
    cd.LOWCONF_CURVE_K = [1e9, 1e9]          # Phase 57 is not part of any driven build; 'p57' / 'p57_<lo>_<hi>' enables it
    lrc.LOWCONF_CAP_OWN_STATE = False
  else:
    lrc.LOWCONF_CAP_OWN_STATE = True
  if variant.startswith("p57"):
    parts = variant.split("_")
    cd.LOWCONF_CURVE_K = [float(parts[1]), float(parts[2])] if len(parts) == 3 else [1.5e-3, 3.0e-3]
  if variant == "p57capfix":                 # the cap fix alone (no curve fade)
    cd.LOWCONF_CURVE_K = [1e9, 1e9]
  if variant == "x_nosoft":
    cd.TURN_EXIT_SOFT_EXEMPT = False
  if variant.startswith("q_"):                 # Phase 57 ON (cap fix + curve fade) plus 55c shaping variants
    cd.LOWCONF_CURVE_K = [1.5e-3, 3.0e-3]; lrc.LOWCONF_CAP_OWN_STATE = True
    for tok in variant[2:].split("+"):
      if tok == "nogate":
        cd.LOOKAHEAD_ORIENT_GATE = [0.0, 0.0]
      elif tok == "nodb":
        cd.LOOKAHEAD_ORIENT_DEADBAND = 0.0
      elif tok == "noslew":
        cd.LOOKAHEAD_ORIENT_W_SLEW_S = 1e-9
      elif tok.startswith("shrink"):
        cd.LOOKAHEAD_ORIENT_WEIGHT_SHRINK = float(tok[6:])
      elif tok.startswith("gate"):
        cd.LOOKAHEAD_ORIENT_GATE = [float(tok[4:]) * 1e-3, float(tok[4:]) * 1e-3 + 0.7e-3]
  if variant == "x_noslew":
    cd.LOOKAHEAD_ORIENT_W_SLEW_S = 1e-9
  s = H.mk_controls(); s.VM = VM(); s._lookahead_orient_w = 1.0
  Tr = d['T']; a0 = np.searchsorted(Tr, t0 - 8.0); a1 = np.searchsorted(Tr, t1)
  cs = s.sm['carState']; out = []
  s.desired_curvature = float(d['k_ctrl'][a0]); s._lat_cmd_lp = s.desired_curvature
  for j in range(a0, a1):
    cs.vEgo = float(d['v'][j]) / 3.6; cs.steeringAngleDeg = float(d['ang'][j]); cs.steeringPressed = bool(d['pressed'][j])
    cs.steeringTorque = float(d['tq_s'][j]); cs.leftBlinker = bool(d['lb'][j]); cs.rightBlinker = bool(d['rb'][j])
    lat = bool(d['lat'][j]); s.get_lat_active = (lambda sm, a=lat: a)
    i = max(np.searchsorted(P['T'], Tr[j], side='right') - 1, 0)
    if getattr(s, "_cur_i", None) != i:
      s._cur_m = model_at(i); s._cur_i = i; s.sm.logMonoTime['modelV2'] = i
    s.sm['modelV2'] = s._cur_m
    CC, _ = cd.Controls.state_control(s)
    out.append((Tr[j], float(CC.actuators.curvature), int(s.steer_flags), bool(s.lane_dropout)))
  for k, v in saved.items():
    setattr(cd, k, v)
  return [o for o in out if o[0] >= t0]


if __name__ == "__main__":
  S, r, t0, t1 = sys.argv[1], sys.argv[2], float(sys.argv[3]), float(sys.argv[4])
  load(S, r)
  L = replay("logged", t0, t1); N = replay("new", t0, t1)
  Tr = d['T']
  err = [abs(lo[1] - float(d['k_ctrl'][np.searchsorted(Tr, lo[0])])) for lo in L]
  print(f"{r} {t0}-{t1}: replay(logged constants) vs logged k_ctrl |err| p50 {np.median(err) * 1e3:.3f} p95 {np.percentile(err, 95) * 1e3:.3f} e-3/m")
  print("   t     K(model)  logged  replay-old  replay-NEW  drop old/new")
  for a, b in list(zip(L, N))[::25]:
    j = np.searchsorted(Tr, a[0]); i = max(np.searchsorted(P['T'], a[0], side='right') - 1, 0)
    print(f"{a[0]:8.2f} {P['K'][i] * 1e3:+7.2f} {d['k_ctrl'][j] * 1e3:+7.2f} {a[1] * 1e3:+8.2f} {b[1] * 1e3:+9.2f}    {int(a[3])}/{int(b[3])}")
