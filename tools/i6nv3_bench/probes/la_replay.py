"""Phase 55b replay: push the recorded OPM10V3 plans (la_plan_extract.py npz) through the REAL controlsd
_lookahead_curvature (shipped constants, then the kill switch) at 100 Hz — 5 calls per 20 Hz model frame, so the
cubic <-> orient cross-fade slews as on the device — and score the command as la_cmd.py does: RMSE vs lane-centre road
curvature at t + 0.45 s by regime, plus straight-road 20 Hz command change. The shipped row must reproduce
qa/e4d_openloop.py's "act+0.5(k9-k3)" and the kill-switch row la_cmd.py's "current" (same frames).
Usage (from the repo root): PYTHONPATH=$PWD:$PWD/opendbc_repo python3 tools/i6nv3_bench/probes/la_replay.py <scratch dir>"""
import glob
import sys
import types

import numpy as np

from phase_tests.harness_noncontrol import FakeParams  # noqa: F401 (stub install side effect)
import openpilot.selfdrive.controls.controlsd as cd

S = sys.argv[1]
L_TGT = 0.45


def bfit(X, Y, deg):
  V = np.stack([X ** p for p in range(deg, -1, -1)], axis=-1)
  A = np.einsum('nmi,nmj->nij', V, V)
  b = np.einsum('nmi,nm->ni', V, Y)
  return np.linalg.solve(A + 1e-12 * np.eye(deg + 1), b[..., None])[..., 0]


def interp_rows(xq, X, Y):
  out = np.empty(xq.shape)
  for i in range(X.shape[0]):
    out[i] = np.interp(xq[i], X[i], Y[i])
  return out


class Stub:
  _model_frame = cd.Controls._model_frame

  def __init__(self):
    self.desired_curvature = 0.0
    self._model_nonfinite_frames = 0
    self._mf = None
    self.sm = types.SimpleNamespace(logMonoTime={'modelV2': 0})


def run(P, v, busy, orient_t, wide):
  cd.LOOKAHEAD_ORIENT_T_S, cd.LOOKAHEAD_ORIENT_WIDE_BUDGET = orient_t, wide
  s = Stub()
  out = np.zeros(len(v))
  flags = np.zeros(len(v), dtype=int)
  for i in range(len(v)):
    lcs = cd.LaneChangeState.preLaneChange if busy[i] else cd.LaneChangeState.off
    m = types.SimpleNamespace(action=types.SimpleNamespace(desiredCurvature=float(P['K'][i])),
                              position=types.SimpleNamespace(x=P['PX'][i].tolist(), y=P['PY'][i].tolist()),
                              orientationRate=types.SimpleNamespace(z=P['OR'][i].tolist()),
                              velocity=types.SimpleNamespace(x=P['VX'][i].tolist()),
                              laneLineProbs=P['PR'][i].tolist(),
                              meta=types.SimpleNamespace(laneChangeState=lcs, laneChangeDirection=0))
    s.sm.logMonoTime['modelV2'] = i
    tau = float(np.interp(v[i], cd.LAT_CMD_SMOOTH_TAU_BP, cd.LAT_CMD_SMOOTH_TAU_V))
    for _ in range(5):
      out[i] = cd.Controls._lookahead_curvature(s, m, float(v[i]), tau)
    flags[i] = s._lookahead_flags
  return out, flags


VARS = {"phase55b": (0.9, True), "narrowJ": (0.9, False), "kill": (0.0, True)}
err, jit, fl = {}, {}, {}
for pf in sorted(glob.glob(f"{S}/la/plan_*.npz")):
  r = pf[-12:-4]
  P = dict(np.load(pf))   # NpzFile decompresses on every key access
  T = P['T']
  n = len(T)
  d = np.load((glob.glob(f"{S}/ropm/{r}.npz") + glob.glob(f"{S}/r43/{r}.npz"))[0], allow_pickle=True)
  v = np.interp(T, d['T'], d['v']) / 3.6
  lat = np.interp(T, d['T'], d['lat'].astype(float)) > 0.5
  busy = np.interp(T, d['T'], (d['lb'] | d['rb'] | (d['lcs'] > 0)).astype(float)) > 0
  PR = P['PR']
  good = (PR[:, 1] > 0.7) & (PR[:, 2] > 0.7) & ~busy & (v * 3.6 >= 30)
  LX = P['LX'].astype(float)
  LC = 0.5 * (P['L1'].astype(float) + P['L2'].astype(float))
  xs0 = np.tile(np.linspace(0, 20, 9), (n, 1))
  truth = 2.0 * bfit(xs0, interp_rows(xs0, LX, LC), 2)[:, 0]
  truth[~good] = np.nan
  j = np.minimum(np.searchsorted(T, T + L_TGT), n - 1)
  ok = good & lat & (np.abs(T[j] - (T + L_TGT)) < 0.03) & ~np.isnan(truth[j])
  tr, tr0 = truth[j], truth
  reg = np.where((np.abs(tr) < 0.5e-3) & (np.abs(tr0) < 0.5e-3), "straight", np.where(np.abs(tr) > np.abs(tr0) + 0.7e-3, "entry",
                 np.where(np.abs(tr0) > np.abs(tr) + 0.7e-3, "exit", np.where(np.abs(tr) >= 1.5e-3, "curve", "mild"))))
  st = good & lat & (np.abs(np.nan_to_num(truth, nan=1.0)) < 0.5e-3)
  cons = st[1:] & st[:-1] & (np.diff(T) < 0.07)
  for vn, (ot, wide) in VARS.items():
    c, f = run(P, v, busy, ot, wide)
    e = (c - tr)[ok]
    for rg in ("straight", "entry", "curve", "exit"):
      err.setdefault((vn, rg), []).append(e[reg[ok] == rg])
    jit.setdefault(vn, []).append(np.diff(c)[cons])
    fl.setdefault(vn, []).append(f[lat & (v * 3.6 >= 30) & ~busy])
  print(r, n, flush=True)

rm = lambda e: np.sqrt(np.mean(e ** 2)) * 1e3
print(f"== real _lookahead_curvature, RMSE (1e-3/m) vs road curvature at t + {L_TGT} s")
for rg in ("straight", "entry", "curve", "exit"):
  print(f"  {rg:9} n {len(np.concatenate(err[('kill', rg)])):6d} " + " ".join(f"{vn} {rm(np.concatenate(err[(vn, rg)])):6.3f}" for vn in VARS))
print("== straight-road 20 Hz command change std (1e-3/m): " + " ".join(f"{vn} {np.std(np.concatenate(jit[vn])) * 1e3:.3f}" for vn in VARS))
for vn in VARS:
  f = np.concatenate(fl[vn])
  print(f"== {vn}: frames {len(f)}, ORIENT {np.mean(f & cd.STEER_FLAG_LOOKAHEAD_ORIENT > 0):.1%}, "
        f"BUDGET {np.mean(f & cd.STEER_FLAG_LOOKAHEAD_BUDGET > 0):.1%}")
