"""E71: gate A — OPEN-LOOP replay of the real CarController on every op-active >= 30 km/h stretch of the 31520b9 drives
(76-79), hands on AND off: logged speed / torque / wheel / command / flags in, TX out. The logged wheel already carries
the real MDPS, the hand and the road, so with the 31520b9 settings the replay must reproduce the logged TX (fidelity) and
each variant shows how the request would have behaved against that same wheel: |TX - cmd|, time at >= 3 / >= 5.5 deg,
windups (|TX - cmd| >= 3 while the wheel is still, >= 1 s), TX snaps not from the command, TX 0.2-1 Hz RMS; by speed band.
Variants: name:KEY=VAL;KEY=VAL ..."""
import glob
import multiprocessing as mp
import sys
import numpy as np
sys.path.insert(0, ".")
S = sys.argv[1]
VARIANTS = [a.split(":", 1) for a in sys.argv[2:]] or [["now", ""]]
BANDS = [(30, 55), (55, 70), (70, 200)]
_DEFAULTS = None


def segments():
  out = []
  for f in sorted(glob.glob(f"{S}/r50/0*.npz")):
    d = np.load(f, allow_pickle=True)
    if str(d['commit'])[:7] != "31520b9":
      continue
    lat = d['lat']; v = d['v']
    ok = lat & (v >= 25)
    e = np.flatnonzero(np.diff(np.concatenate([[0], ok.astype(int), [0]])))
    for a, b in zip(e[::2], e[1::2]):
      if b - a >= 500:
        out.append((f, a, b))
  return out


def run(args):
  f, a, b, var = args
  from phase_tests.harness import Sim
  from opendbc.car.hyundai.values import CarControllerParams as CCP
  global _DEFAULTS
  if _DEFAULTS is None:
    _DEFAULTS = {k: getattr(CCP, k) for k in dir(CCP) if k.startswith("MDPS_COMP_") or k.startswith("HAND_GAP_")}
  for k, val in _DEFAULTS.items():
    setattr(CCP, k, val)
  for kv in [x for x in var.split(";") if x]:
    k_, v_ = kv.split("=")
    cur = getattr(CCP, k_)
    if isinstance(cur, (list, np.ndarray)):
      setattr(CCP, k_, [float(x) for x in v_.split(",")])
    elif isinstance(cur, bool):
      setattr(CCP, k_, v_ in ("1", "True", "true"))
    else:
      setattr(CCP, k_, float(v_))
  dz = np.load(f, allow_pickle=True)
  d = {k: dz[k] for k in ('ang', 'v', 'tq_s', 'apply', 'lat', 'pressed', 'lb', 'rb', 'a_ego', 'gas', 'lane_min', 'bsl', 'bsr', 'wire', 'tx_active', 'rate')}
  a0 = max(a - 500, 0)
  sim = Sim(); rows = []
  for k in range(a0, b):
    sim.step(v=float(d['v'][k]) / 3.6, tq=float(d['tq_s'][k]), wheel=float(d['ang'][k]), cmd=float(d['apply'][k]), lat_active=bool(d['lat'][k]),
             pressed=bool(d['pressed'][k]), blinker=bool(d['lb'][k]), blinker_right=bool(d['rb'][k]), a_ego=float(d['a_ego'][k]),
             gas=bool(d['gas'][k]), lane_prob_min=float(d['lane_min'][k]), bs_l=bool(d['bsl'][k]), bs_r=bool(d['bsr'][k]),
             wheel_rate=float(d['rate'][k]))
    if k >= a:
      m = sim.lkas_alt(); act = bool(m and m["LKAS_ANGLE_ACTIVE"] == 2)
      rows.append((float(d['apply'][k]), float(m["ADAS_StrAnglReqVal"]) if m else np.nan, float(d['ang'][k]), float(d['v'][k]), act,
                   float(d['wire'][k]), bool(d['tx_active'][k] == 2)))
  return var, np.array(rows)


def bp(x, lo, hi, dt=0.01):
  X = np.fft.rfft(x - np.mean(x)); f = np.fft.rfftfreq(len(x), dt)
  X[(f < lo) | (f >= hi)] = 0
  return np.fft.irfft(X, len(x))


def summarize(rows, logged=False):
  agg = {i: dict(n=0, g=[], wu=0, rel=0, txb=[], fid=[]) for i in range(len(BANDS))}
  for X in rows:
    if len(X) < 300:
      continue
    ap, tx, wh, v, act = X[:, 0], (X[:, 5] if logged else X[:, 1]), X[:, 2], X[:, 3], (X[:, 6] if logged else X[:, 4]) > 0.5
    gap = tx - ap
    rate = np.zeros(len(wh)); rate[10:] = (wh[10:] - wh[:-10]) / 0.1
    wu = act & (np.abs(gap) >= 3) & (np.abs(rate) < 2)
    e = np.flatnonzero(np.diff(np.concatenate([[0], wu.astype(int), [0]])))
    ends = [b0 for a0, b0 in zip(e[::2], e[1::2]) if b0 - a0 >= 100]
    dtx = np.zeros(len(tx)); dtx[30:] = np.abs(tx[30:] - tx[:-30]); dap = np.zeros(len(tx)); dap[30:] = np.abs(ap[30:] - ap[:-30])
    rel = act & (dtx >= 3) & (dap < 1)
    txb = bp(np.nan_to_num(tx - ap), 0.2, 1.0)
    for i, (lo, hi) in enumerate(BANDS):
      m = act & (v >= lo) & (v < hi)
      if m.sum() < 100:
        continue
      o = agg[i]; o['n'] += m.sum(); o['g'].append(np.abs(gap[m])); o['wu'] += sum(1 for x in ends if m[x - 1])
      o['rel'] += int(np.sum(np.diff(np.concatenate([[0], rel[m].astype(int)])) == 1)); o['txb'].append(txb[m])
      if not logged:
        o['fid'].append(np.abs(X[:, 1] - X[:, 5])[m & (X[:, 6] > 0.5)])
  return agg


def show(tag, agg):
  parts = []
  for i, (lo, hi) in enumerate(BANDS):
    o = agg[i]
    if o['n'] < 500:
      continue
    mins = o['n'] / 6000; g = np.concatenate(o['g'])
    fid = f" fid {np.median(np.concatenate(o['fid'])):.2f}/{np.percentile(np.concatenate(o['fid']), 90):.2f}" if o['fid'] else ""
    parts.append(f"[{lo}-{hi} {mins:4.1f}m] |TX-cmd| p50 {np.median(g):.2f} p90 {np.percentile(g, 90):.1f} >=3 {np.mean(g >= 3):4.0%} >=5.5 {np.mean(g >= 5.5):4.0%} "
                 f"wu {o['wu'] / mins:4.2f}/m snaps {o['rel'] / mins:5.1f}/m sway {np.sqrt(np.mean(np.concatenate(o['txb']) ** 2)):.2f}{fid}")
  print(f"{tag:10} " + " || ".join(parts), flush=True)


if __name__ == "__main__":
  segs = segments()
  print(f"segments {len(segs)} ({sum(b - a for _, a, b in segs) / 6000:.0f} min op >= 25 km/h on 31520b9)", flush=True)
  jobs = [(f, a, b, v[1]) for v in VARIANTS for f, a, b in segs]
  with mp.Pool(4) as pool:
    res = pool.map(run, jobs, chunksize=1)
  first = True
  for name, var in VARIANTS:
    rows = [X for vv, X in res if vv == var and len(X)]
    if first:
      show("LOGGED", summarize(rows, logged=True)); first = False
    show(name, summarize(rows))
