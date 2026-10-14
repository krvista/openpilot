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


def load(S, r):
  global P, d
  P = dict(np.load(f"{S}/la/plan_{r}.npz")); d = dict(np.load(f"{S}/r46/{r}.npz", allow_pickle=True))
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
                                       "LOOKAHEAD_ORIENT_W_SLEW_S")}
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
