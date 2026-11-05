"""E70: REPRODUCTION GATE + design bench. Closed-loop replay of the real CarController with a plant in the loop, on hands-off
>= 30 km/h windows (as E60). Plant 'M1' (E59 form) or 'BL' (E69 backlash), truth = the route fold's fit, compensator design
params = values.py (or overrides). With the 31520b9 settings the sim must reproduce what the 76-79 logs show (time at the
GMAX clamp, |TX - cmd|, windups, TX snaps, wheel 0.2-1 Hz swing) — computed the same way on the logged signals.
Usage: python3 e70_repro_gate.py <scratch> <plant M1|BL> <routes: new|all> [workers] [KEY=VAL ...] (CarControllerParams)"""
import glob
import json
import multiprocessing as mp
import sys
import numpy as np
sys.path.insert(0, ".")
S = sys.argv[1]; PLANT = sys.argv[2]; RSEL = sys.argv[3]; NW = int(sys.argv[4]) if len(sys.argv) > 4 else 4
OVR = dict(kv.split("=") for kv in sys.argv[5:])
PAR = json.load(open(f"{S}/qa/e69_params.json"))
CENT = [35.0, 47.5, 62.5, 80.0, 100.0]
BANDS = [(30, 40), (40, 55), (55, 70), (70, 90), (90, 200)]


def truth(fold):
  tag = "A" if fold == 0 else "B"
  rows = []
  for b in range(1, 6):
    p = PAR.get(f"{PLANT}{tag}{b}") or PAR.get(f"{PLANT}{b}")
    rows.append(p + ([0.0] if len(p) == 4 else []))
  return np.array(rows)          # d, Kf, Kp, Kc, B


def windows():
  sys.path.insert(0, "opendbc_repo")
  from opendbc.car.hyundai.carcontroller import compute_hold_torque
  files = sorted(glob.glob(f"{S}/r50/0*.npz") + glob.glob(f"{S}/r_old/0*.npz"), key=lambda f: f[-12:-4])
  routes = sorted({f[-12:-4] for f in files})
  out = []
  for f in files:
    d = np.load(f, allow_pickle=True)
    if RSEL == "new" and str(d['commit'])[:7] != "31520b9":
      continue
    T = d['T']; n = len(T); v = d['v']
    if n < 3000 or 'gain' not in d:
      continue
    vm = v / 3.6; la = np.abs(vm ** 2 * d['k_meas'])
    hold = np.array([compute_hold_torque(a, b) for a, b in zip(vm[::10], la[::10])]).repeat(10)[:n]
    dd = np.maximum(0.0, np.abs(d['tq_s']) - hold)
    bl = d['lb'] | d['rb'] | (d['lcs'] > 0)
    ok = d['lat'] & (v >= 30) & (dd < 40) & ~d['pressed'] & ~bl & np.isfinite(d['apply'])
    e = np.flatnonzero(np.diff(np.concatenate([[0], ok.astype(int), [0]])))
    for a, b in zip(e[::2], e[1::2]):
      k = a
      while b - k >= 1000:
        k1 = min(k + 6000, b)
        if k1 - k >= 1000:
          out.append((f, k, k1, routes.index(f[-12:-4]) % 2))
        k = k1
  return out


def run_window(args):
  f, a, b, fold = args
  from phase_tests.harness import Sim
  from opendbc.car.hyundai.values import CarControllerParams as CCP
  for k_, v_ in OVR.items():
    cur = getattr(CCP, k_)
    if isinstance(cur, (list, np.ndarray)):
      setattr(CCP, k_, [float(x) for x in v_.split(",")])
    else:
      setattr(CCP, k_, type(cur)(float(v_)) if not isinstance(cur, bool) else v_ in ("1", "True", "true"))
  tr = truth(fold)
  dz = np.load(f, allow_pickle=True)
  d = {k: dz[k] for k in ('ang', 'v', 'tq_s', 'apply', 'lat', 'pressed', 'lb', 'rb', 'a_ego', 'gas', 'lane_min', 'bsl', 'bsr', 'wire', 'tx_active')}
  a0 = max(a - 300, 0)
  sim = Sim(); w = float(d['ang'][a0]); hist = []; y = None
  rows = []
  for k in range(a0, b):
    vk = float(d['v'][k]); closed = k >= a
    wheel = w if closed else float(d['ang'][k])
    sim.step(v=vk / 3.6, tq=float(d['tq_s'][k]), wheel=wheel, cmd=float(d['apply'][k]), lat_active=bool(d['lat'][k]),
             pressed=bool(d['pressed'][k]), blinker=bool(d['lb'][k]), blinker_right=bool(d['rb'][k]), a_ego=float(d['a_ego'][k]),
             gas=bool(d['gas'][k]), lane_prob_min=float(d['lane_min'][k]), bs_l=bool(d['bsl'][k]), bs_r=bool(d['bsr'][k]))
    m = sim.lkas_alt(); act = bool(m and m["LKAS_ANGLE_ACTIVE"] == 2)
    tx = float(m["ADAS_StrAnglReqVal"]) if act else wheel
    g = min(max((float(m["ADAS_ACIAnglTqRedcGainVal"]) - 0.2) / 0.5, 0.0), 1.0) if act else 0.0
    hist.append(tx)
    pt = [np.interp(vk, CENT, tr[:, j]) for j in range(5)]
    nd = int(round(pt[0] / 0.01))
    u1 = hist[-1 - nd] if len(hist) > nd else hist[0]
    if y is None:
      y = u1
    if closed:
      yp = y
      if u1 - y > pt[4]:
        y = u1 - pt[4]
      elif y - u1 > pt[4]:
        y = u1 + pt[4]
      w = w + 0.01 * (g * (pt[1] * (y - yp) / 0.01 + pt[2] * (y - w)) - pt[3] * w)
      rows.append((float(d['apply'][k]), tx, w, vk, act, float(d['wire'][k]), float(d['ang'][k]), bool(d['tx_active'][k] == 2)))
    else:
      w = float(d['ang'][k]); y = u1
  return f[-12:-4], a, np.array(rows)


def bp(x, lo, hi, dt=0.01):
  X = np.fft.rfft(x - np.mean(x)); f = np.fft.rfftfreq(len(x), dt)
  X[(f < lo) | (f >= hi)] = 0
  return np.fft.irfft(X, len(x))


def stats(rows, which):
  """which = 'sim' (TX, wheel from the sim) or 'log' (logged TX / wheel)."""
  out = {i: dict(n=0, g=[], clamp=0, wu=0, rel=0, wb=[], err=[]) for i in range(5)}
  for X in rows:
    if len(X) < 300:
      continue
    ap = X[:, 0]; v = X[:, 3]
    tx, wh, act = (X[:, 1], X[:, 2], X[:, 4] > 0.5) if which == "sim" else (X[:, 5], X[:, 6], X[:, 7] > 0.5)
    gap = tx - ap
    rate = np.zeros(len(wh)); rate[10:] = (wh[10:] - wh[:-10]) / 0.1
    wu = act & (np.abs(gap) >= 3) & (np.abs(rate) < 2)
    e = np.flatnonzero(np.diff(np.concatenate([[0], wu.astype(int), [0]])))
    ends = [b0 for a0, b0 in zip(e[::2], e[1::2]) if b0 - a0 >= 100]
    dtx = np.zeros(len(tx)); dtx[30:] = np.abs(tx[30:] - tx[:-30]); dap = np.zeros(len(tx)); dap[30:] = np.abs(ap[30:] - ap[:-30])
    rel = act & (dtx >= 3) & (dap < 1)
    wb = bp(wh, 0.2, 1.0)
    for i, (lo, hi) in enumerate(BANDS):
      m = act & (v >= lo) & (v < hi)
      if m.sum() < 100:
        continue
      O = out[i]
      O['n'] += m.sum(); O['g'].append(np.abs(gap[m])); O['clamp'] += (np.abs(gap[m]) >= 5.5).sum()
      O['wu'] += sum(1 for x in ends if m[x - 1]); O['rel'] += int(np.sum(np.diff(np.concatenate([[0], rel[m].astype(int)])) == 1))
      O['wb'].append(wb[m]); O['err'].append(wh[m] - ap[m])
  return out


def show(tag, O):
  for i, (lo, hi) in enumerate(BANDS):
    o = O[i]
    if o['n'] < 500:
      continue
    mins = o['n'] / 6000; g = np.concatenate(o['g'])
    print(f"   {tag:10} [{lo}-{hi}] {mins:5.1f} min | |TX-cmd| p50 {np.median(g):.2f} p90 {np.percentile(g, 90):.2f} | at GMAX {o['clamp'] / o['n']:5.1%} | windup {o['wu'] / mins:4.2f}/min "
          f"| TX snaps {o['rel'] / mins:5.2f}/min | wheel 0.2-1 Hz {np.sqrt(np.mean(np.concatenate(o['wb']) ** 2)):.2f} deg | wheel-cmd RMS {np.sqrt(np.mean(np.concatenate(o['err']) ** 2)):.2f}")


if __name__ == "__main__":
  W = windows()
  print(f"plant {PLANT} routes {RSEL}: windows {len(W)} ({sum(b - a for _, a, b, _ in W) / 6000:.0f} min) overrides {OVR}", flush=True)
  with mp.Pool(NW) as pool:
    res = pool.map(run_window, W, chunksize=1)
  rows = [X for _, _, X in res if len(X)]
  show("SIM", stats(rows, "sim"))
  if RSEL == "new":
    show("LOGGED", stats(rows, "log"))
