"""E149 (report §95): the REAL controlsd replayed over the logged drives of a build (carState + the logged model output, the
model's told delay as lagd published it: REPLAY_MODEL_DELAY=log), with controlsd constants overridden per variant - which
point of the model's plan does the command come out at? Hands-off stretches (>= 10 s, cut to <= 60 s) are replayed whole,
then cut into 6 s windows (3 s hop, some curvature) and, as E143, the plan time tau whose curvature k_plan(tau) is closest
to the command is fitted per window. Also printed: how well the 'drv' variant (the constants as driven) reproduces the
logged command - the replay's own fidelity.
Variants (argv 3, csv): name=CONST:value;CONST:value ... (lists as a|b|c); the pseudo-constant TOLD:<s> replays the drives
as if the model had been told the delay <s> (the logged action re-timed by modeld's own formula 2 psi(T) / (v T) - psi_rate(0) / v,
exact; TOLD:0.30 is the configuration the chain sim validated Phase 60 / 69a on). 'drv' must be given as the as-driven set.
argv: S build variants [workers=4]"""
import glob
import multiprocessing as mp
import os
import sys
import numpy as np
sys.path.insert(0, "."); sys.path.insert(0, "tools/i6nv3_bench/probes/qa"); sys.path.insert(0, "tools/i6nv3_bench/probes"); sys.path.insert(0, "opendbc_repo")
os.environ.setdefault("REPLAY_MODEL_DELAY", "log")
S = sys.argv[1]; BUILD = sys.argv[2]
VARS = {}
for tok in sys.argv[3].split(","):
  nm, _, body = tok.partition("=")
  VARS[nm] = [kv.split(":") for kv in body.split(";") if kv]
NW = int(sys.argv[4]) if len(sys.argv) > 4 else 4
BANDS = [(30, 40), (40, 55), (55, 70), (70, 90), (90, 200)]
T_IDXS = np.array([10.0 * (i / 32) ** 2 for i in range(33)]); TAUS = np.arange(0.0, 1.61, 0.05); W = 600


def parse(cur, v):
  if isinstance(cur, bool):
    return v in ("1", "True", "true")
  if isinstance(cur, (list, tuple)):
    return [float(x) for x in v.split("|")]
  return float(v)


def best_tau(sig, kp):
  e = np.sqrt(np.mean((kp - sig[None, :]) ** 2, axis=1)); j = int(np.argmin(e))
  if 0 < j < len(TAUS) - 1:
    a, b0, c = e[j - 1], e[j], e[j + 1]; den = a - 2 * b0 + c
    return TAUS[j] + (0.5 * (a - c) / den if den > 1e-12 else 0.0) * 0.05
  return TAUS[j]


def stretches():
  from opendbc.car.hyundai.carcontroller import compute_hold_torque
  out = []
  for f in sorted(glob.glob(f"{S}/r50/0*.npz")):
    d = np.load(f, allow_pickle=True)
    if str(d['commit'])[:7] != BUILD:
      continue
    r = f[-12:-4]; T = d['T']; n = len(T); v = d['v']; vm = v / 3.6; la = np.abs(vm ** 2 * d['k_meas'])
    hold = np.array([compute_hold_torque(a, b) for a, b in zip(vm[::10], la[::10])]).repeat(10)[:n]
    dd = np.maximum(0.0, np.abs(d['tq_s']) - hold); bl = d['lb'] | d['rb'] | (d['lcs'] > 0)
    ok = d['lat'] & (v >= 30) & (dd < 40) & ~d['pressed'] & ~bl; ok[:1200] = False
    e = np.flatnonzero(np.diff(np.concatenate([[0], ok.astype(int), [0]])))
    for a, b in zip(e[::2], e[1::2]):
      k = a
      while b - k >= 1000:
        k1 = min(k + 6000, b)
        if k1 - k >= 1000:
          out.append((r, k, k1))
        k = k1
  return out


def run(args):
  r, a, b = args
  import replay_state_control as R
  import openpilot.selfdrive.controls.controlsd as cd
  R.load(S, r); d = R.d; P = R.P
  T = d['T']; res = {}
  for nm, kvs in VARS.items():
    told = next((v for k, v in kvs if k == "TOLD"), None)          # TOLD:<s> = replay as if the model had been told <s>
    kvs = [(k, v) for k, v in kvs if k != "TOLD"]
    saved = {k: getattr(cd, k) for k, _ in kvs}
    for k, v in kvs:
      setattr(cd, k, parse(saved[k], v))
    env_told = os.environ.pop("REPLAY_FORCE_TOLD", None)
    if told not in (None, "log"):
      os.environ["REPLAY_FORCE_TOLD"] = told
    elif told is None and env_told is not None:
      os.environ["REPLAY_FORCE_TOLD"] = env_told
    out = R.replay("p57", T[a], T[b - 1])
    os.environ.pop("REPLAY_FORCE_TOLD", None)
    if env_told is not None:
      os.environ["REPLAY_FORCE_TOLD"] = env_told
    for k, v in saved.items():
      setattr(cd, k, v)
    res[nm] = np.interp(T[a:b], [o[0] for o in out], [o[1] for o in out])
  j = np.clip(np.searchsorted(P['T'], T[a:b], side="right") - 1, 0, len(P['T']) - 1)
  kp = np.empty((len(TAUS), b - a))
  for q, tau in enumerate(TAUS):
    w = min(max(np.searchsorted(T_IDXS, tau, side="right") - 1, 0), 31); fr = (tau - T_IDXS[w]) / (T_IDXS[w + 1] - T_IDXS[w])
    kp[q] = ((P['OR'][:, w] * (1 - fr) + P['OR'][:, w + 1] * fr) / np.maximum(P['VX'][:, w] * (1 - fr) + P['VX'][:, w + 1] * fr, 1.0))[j]
  K = d['k_model'][a:b]; v = d['v'][a:b]; kc = d['k_ctrl'][a:b]
  rows = []
  for a0 in range(0, b - a - W, W // 2):
    sl = slice(a0, a0 + W)
    bi = next((i for i, (lo, hi) in enumerate(BANDS) if ((v[sl] >= lo) & (v[sl] < hi)).mean() > 0.8), None)
    if bi is None or np.std(K[sl]) < 0.2e-3:
      continue
    rows.append((bi, best_tau(kc[sl], kp[:, sl]), {nm: best_tau(res[nm][sl], kp[:, sl]) for nm in VARS}))
  fid = (float(np.sum((res['drv'] - kc) ** 2)), float(np.sum(kc ** 2))) if 'drv' in res else (0.0, 0.0)
  return rows, fid, (b - a) / 6000.0


if __name__ == "__main__":
  ST = stretches()
  print(f"{BUILD}: {len(ST)} hands-off stretches, {sum(b - a for _, a, b in ST) / 6000:.0f} min", flush=True)
  with mp.Pool(NW) as pool:
    out = pool.map(run, ST, chunksize=1)
  rows = [x for o in out for x in o[0]]
  num = sum(o[1][0] for o in out); den = sum(o[1][1] for o in out)
  if den > 0:
    print(f"replay fidelity: RMS(replayed 'drv' command - logged command) / RMS(logged) = {np.sqrt(num / den):.3f}")
  for i, (lo, hi) in enumerate(BANDS):
    x = [r_ for r_ in rows if r_[0] == i]
    if len(x) < 3:
      print(f"  [{lo}-{hi}] windows {len(x)}"); continue
    line = f"  [{lo}-{hi}] windows {len(x):3d}: logged command at {np.median([r_[1] for r_ in x]) * 1e3:4.0f} ms"
    for nm in VARS:
      t = np.array([r_[2][nm] for r_ in x]) * 1e3
      line += f" | {nm} {np.median(t):4.0f} [{np.percentile(t, 25):.0f}-{np.percentile(t, 75):.0f}]"
    print(line)
