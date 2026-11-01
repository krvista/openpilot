"""E60: CLOSED-LOOP replay of the REAL CarController with the identified MDPS plant in the loop. Windows: op active, >= 30
km/h, hold-compensated driver torque < 40 and not pressed, no blinker, >= 10 s (chunked to 60 s), from every extracted route.
Per frame: Sim.step(logged speed / torque / command / flags, wheel = plant state); the plant integrates the sent TX:
  d wheel/dt = g * (Kf * du/dt + Kp * (u - wheel)) - Kc * wheel,  u = TX delayed by d, g = 1 over sent gain 0.7-1 (E59b), fading to 0 at 0.2 (no data below 0.7)
'Truth' plant per route = the fit on that route's own fold; the compensator (Phase 59) uses the OTHER fold's parameters
(2-fold cross validation). Policies: OFF (MDPS_COMP_LAMBDA = 0: today's trim / kick / ... layers) vs ON.
The first 3 s of each window run open loop (logged wheel) to settle the controller state.
Usage: PYTHONPATH=$PWD:$PWD/opendbc_repo python3 e60_cl_replay.py <scratch> [max_windows] [workers]"""
import glob
import json
import multiprocessing as mp
import sys
import numpy as np
sys.path.insert(0, ".")
S = sys.argv[1]; MAXW = int(sys.argv[2]) if len(sys.argv) > 2 else 100000; NW = int(sys.argv[3]) if len(sys.argv) > 3 else 4
OVR = dict(kv.split("=") for kv in sys.argv[4:])   # CarControllerParams overrides for the ON policy, e.g. MDPS_COMP_LAMBDA=3
PAR = json.load(open(f"{S}/qa/e59_params.json"))
BANDS = PAR["bands"]
CENT = [35.0, 47.5, 62.5, 80.0, 100.0]          # band centres 30-40 .. 90+ (index 1..5 in BANDS)


def band_params(prefix):
  rows = [PAR.get(f"{prefix}{b}") for b in range(1, 6)]
  if any(r is None for r in rows):
    return None
  return np.array(rows)                            # [d, Kf, Kp, Kc] per band


def windows():
  sys.path.insert(0, "opendbc_repo")
  from opendbc.car.hyundai.carcontroller import compute_hold_torque
  files = sorted(glob.glob(f"{S}/r50/0*.npz") + glob.glob(f"{S}/r_old/0*.npz"), key=lambda f: f[-12:-4])
  routes = sorted({f[-12:-4] for f in files})
  out = []
  for f in files:
    d = np.load(f, allow_pickle=True)
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
  f, a, b, fold, comp_on = args
  from phase_tests.harness import Sim
  from opendbc.car.hyundai.values import CarControllerParams as CCP
  truth = band_params("A" if fold == 0 else "B"); design = band_params("B" if fold == 0 else "A")
  CCP.MDPS_COMP_LAMBDA = 2.0 if comp_on else 0.0
  CCP.MDPS_COMP_SPEEDS_KPH = CENT
  CCP.MDPS_COMP_DELAY_S = design[:, 0].tolist(); CCP.MDPS_COMP_KF = design[:, 1].tolist()
  CCP.MDPS_COMP_KP = design[:, 2].tolist(); CCP.MDPS_COMP_KC = design[:, 3].tolist()
  if comp_on:
    for k_, v_ in OVR.items():
      setattr(CCP, k_, [float(x) for x in v_.split(",")] if "," in v_ else float(v_))
  dz = np.load(f, allow_pickle=True)
  d = {k: dz[k] for k in ('ang', 'v', 'tq_s', 'apply', 'lat', 'pressed', 'lb', 'rb', 'a_ego', 'gas', 'lane_min', 'bsl', 'bsr', 'rate')}
  a0 = max(a - 300, 0)
  sim = Sim(); w = float(d['ang'][a0]); hist = []
  rows = []
  for k in range(a0, b):
    vk = float(d['v'][k])
    closed = k >= a
    wheel = w if closed else float(d['ang'][k])
    sim.step(v=vk / 3.6, tq=float(d['tq_s'][k]), wheel=wheel, cmd=float(d['apply'][k]), lat_active=bool(d['lat'][k]),
             pressed=bool(d['pressed'][k]), blinker=bool(d['lb'][k]), blinker_right=bool(d['rb'][k]), a_ego=float(d['a_ego'][k]),
             gas=bool(d['gas'][k]), lane_prob_min=float(d['lane_min'][k]), bs_l=bool(d['bsl'][k]), bs_r=bool(d['bsr'][k]),
             wheel_rate=float(d['rate'][k]) if not closed else (rows[-1][3] if rows else 0.0))
    m = sim.lkas_alt()
    tx = float(m["ADAS_StrAnglReqVal"]) if m else wheel
    g_raw = float(m["ADAS_ACIAnglTqRedcGainVal"]) if (m and m["LKAS_ANGLE_ACTIVE"] == 2) else 0.0
    g = min(max((g_raw - 0.2) / 0.5, 0.0), 1.0)    # E59b: the response does not change over gain 0.7-1; fades out below
    if not (m and m["LKAS_ANGLE_ACTIVE"] == 2):
      tx = wheel
    hist.append(tx)
    pt = np.array([np.interp(vk, CENT, truth[:, j]) for j in range(4)])
    nd = int(round(pt[0] / 0.01))
    u1 = hist[-1 - nd] if len(hist) > nd else hist[0]; u2 = hist[-2 - nd] if len(hist) > nd + 1 else hist[0]
    if closed:
      w_new = w + 0.01 * (g * (pt[1] * (u1 - u2) / 0.01 + pt[2] * (u1 - w)) - pt[3] * w)
      rate = (w_new - w) / 0.01
      w = w_new
    else:
      w = float(d['ang'][k]); rate = 0.0
    if closed:
      rows.append((float(d['apply'][k]), tx, w, rate, g_raw, vk, int(sim.cc.steer_debug_flags)))
  return f[-12:-4], a, comp_on, np.array(rows)


def metrics(R):
  e, gap, txr, deliv, lags, gains, flags = [], [], [], [], [], [], []
  for X in R:
    th, tx, w, g = X[:, 0], X[:, 1], X[:, 2], X[:, 4]
    e.append(w - th); gap.append(np.abs(tx - w)); txr.append(np.abs(np.diff(tx)) / 0.01); gains.append(g); flags.append(X[:, 6].astype(np.int64))
    thr = np.abs(np.gradient(th, 0.01))
    for k in range(60, len(th), 30):
      if np.all(thr[k - 60:k] < 2.0) and abs(th[k]) >= 2:
        deliv.append(w[k] / th[k])
    if len(th) >= 1000 and np.std(th) > 0.5:
      x = th - th.mean(); best = (-2, 0)
      for L in range(0, 100, 2):
        y = w[L:] - w[L:].mean(); xx = x[:len(y)]
        cc = np.dot(xx, y) / (np.linalg.norm(xx) * np.linalg.norm(y) + 1e-9)
        if cc > best[0]:
          best = (cc, L)
      if best[0] > 0.7:
        lags.append(best[1] * 10)
  e = np.concatenate(e); gap = np.concatenate(gap); txr = np.concatenate(txr); gains = np.concatenate(gains); fl = np.concatenate(flags)
  outrun = np.mean((fl >> 25) & 1); passive = np.mean(gains == 0)
  return (f"wheel-cmd RMS {np.sqrt(np.mean(e ** 2)):.2f} deg (p95 |e| {np.percentile(np.abs(e), 95):.1f}) | lag p50 {np.median(lags) if lags else np.nan:.0f} ms (n {len(lags)}) "
          f"| curve delivery p50 {np.median(deliv) if deliv else np.nan:.2f} (n {len(deliv)}) | gap >6 {np.mean(gap > 6):.1%} >8 {np.mean(gap > 8):.1%} "
          f"| TX rate p99 {np.percentile(txr, 99):.0f} max {txr.max():.0f} deg/s | gain p10 {np.percentile(gains, 10):.2f}, passive {passive:.1%}, outrun {outrun:.2%}")


if __name__ == "__main__":
  W = windows()
  rng = np.random.default_rng(1); W = [W[i] for i in rng.permutation(len(W))[:MAXW]]
  print(f"windows {len(W)} ({sum(b - a for _, a, b, _ in W) / 6000:.0f} min closed loop)", flush=True)
  jobs = [(f, a, b, fold, on) for f, a, b, fold in W for on in (False, True)]
  with mp.Pool(NW) as pool:
    res = pool.map(run_window, jobs, chunksize=1)
  for on in (False, True):
    R = [X for _, _, c, X in res if c == on and len(X)]
    print(f"{'ON ' if on else 'OFF'}: " + metrics(R))
  # per speed band
  for lo, hi in ((30, 40), (40, 55), (55, 70), (70, 90), (90, 200)):
    for on in (False, True):
      R = []
      for _, _, c, X in res:
        if c == on and len(X):
          m = (X[:, 5] >= lo) & (X[:, 5] < hi)
          if m.sum() > 200:
            R.append(X[m])
      if R:
        print(f"   [{lo}-{hi}] {'ON ' if on else 'OFF'}: " + metrics(R))
  np.save(f"{S}/qa/e60_res.npy", np.array([(r, a, c) for r, a, c, _ in res], dtype=object), allow_pickle=True)
