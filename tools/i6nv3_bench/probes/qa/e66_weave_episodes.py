"""E66: weave EPISODES (side-to-side) at >= 50 km/h, op active, no blinker / LC within 3 s. Yaw curvature band-passed
0.2-1.0 Hz (FFT mask on the whole route); envelope = 2 s RMS. Episode = envelope >= THR (lateral accel equivalent
v^2 * env >= 0.15 m/s^2) for >= 3 s. Per episode: hands (driver-domain torque >= 40 or pressed share), SF_MDPS_COMP
(feedback) share and its on/off toggles per s, SF_HAND_GAP share, ACI gain swing (p90-p10), and which signal carries the
oscillation (band RMS of K, cmd, TX, wheel) -> source. Rates per op-hour at >= 50 km/h, by build."""
import glob
import sys
from collections import defaultdict
import numpy as np
sys.path.insert(0, "."); sys.path.insert(0, "opendbc_repo")
from opendbc.car.hyundai.carcontroller import compute_hold_torque  # noqa: E402
S = sys.argv[1]
SF_HAND_GAP = 1 << 29; SF_MDPS_COMP = 1 << 30


def bp(x, lo, hi, dt=0.01):
  X = np.fft.rfft(x - np.mean(x)); f = np.fft.rfftfreq(len(x), dt)
  X[(f < lo) | (f >= hi)] = 0
  return np.fft.irfft(X, len(x))


agg = defaultdict(lambda: dict(h=0.0, hoff=0.0, ev=[]))
for f in sorted(glob.glob(f"{S}/r50/0*.npz")):
  r = f[-12:-4]
  d = dict(np.load(f, allow_pickle=True)); b = str(d['commit'])[:7]
  if b not in ("98e912d", "a0108a4", "31520b9"):
    continue
  try:
    V = np.load(f"{S}/vm/{r}.npz", allow_pickle=True); lk = V['lk']
  except Exception:  # noqa: BLE001
    continue
  T = d['T']; n = len(T); v = d['v']; vm = v / 3.6
  ky = np.radians(np.interp(T, lk[:, 0], lk[:, 1])) / np.maximum(vm, 1.0)
  K = d['k_model']; kc = d['k_ctrl']
  m = d['lat'] & (np.abs(kc) > 0.5e-3)
  sc = np.sum(d['apply'][m] * kc[m]) / np.sum(kc[m] ** 2) if m.sum() > 100 else -3000.0
  ktx = d['wire'] / sc; kw = d['ang'] / sc
  la = np.abs(vm ** 2 * d['k_meas'])
  hold = np.array([compute_hold_torque(a, c) for a, c in zip(vm[::10], la[::10])]).repeat(10)[:n]
  dd = np.maximum(0.0, np.abs(d['tq_s']) - hold)
  hand = (dd >= 40) | d['pressed']
  bl = d['lb'] | d['rb'] | (d['lcs'] > 0)
  cs = lambda x: np.concatenate([[0], np.cumsum(x)]); idx = np.arange(n)
  busy = (cs(bl)[np.minimum(idx + 300, n)] - cs(bl)[np.maximum(idx - 300, 0)]) > 0
  act = d['lat'] & ~busy & (v >= 50)
  agg[b]['h'] += act.sum() / 360000; agg[b]['hoff'] += (act & ~hand).sum() / 360000
  sig = {nm: bp(np.nan_to_num(x), 0.2, 1.0) for nm, x in (("K", K), ("cmd", kc), ("TX", ktx), ("wheel", kw), ("yaw", ky))}
  env = np.sqrt(np.convolve(sig["yaw"] ** 2, np.ones(200) / 200, "same"))
  hit = act & (vm ** 2 * env >= 0.15)
  e = np.flatnonzero(np.diff(np.concatenate([[0], hit.astype(int), [0]])))
  sc_cc = d['sf_cc'].astype(np.int64)
  for a0, b0 in zip(e[::2], e[1::2]):
    if b0 - a0 < 300:
      continue
    w = slice(a0, b0)
    comp = ((sc_cc[w] & SF_MDPS_COMP) > 0)
    rms = {nm: np.sqrt(np.mean(x[w] ** 2)) for nm, x in sig.items()}
    agg[b]['ev'].append(dict(r=r, t=T[a0], dur=(b0 - a0) / 100, v=np.median(v[w]), amp=np.median(vm[w] ** 2 * env[w]),
                             hand=hand[w].mean(), comp=comp.mean(), tog=np.sum(np.abs(np.diff(comp.astype(int)))) / ((b0 - a0) / 100),
                             gap=((sc_cc[w] & SF_HAND_GAP) > 0).mean(), gsw=np.percentile(d['gain'][w], 90) - np.percentile(d['gain'][w], 10),
                             **{f"r_{k}": x for k, x in rms.items()}))
for b in ("a0108a4", "98e912d", "31520b9"):
  A = agg[b]; E = A['ev']
  if not E:
    continue
  tot = sum(e['dur'] for e in E)
  src = lambda e: "model" if e['r_cmd'] >= 0.6 * e['r_wheel'] and e['r_K'] >= 0.6 * e['r_cmd'] else ("carctl/TX" if e['r_TX'] >= 1.5 * e['r_cmd'] else "other")
  print(f"== {b}: op >= 50 km/h {A['h'] * 60:.0f} min (hands-off {A['hoff'] * 60:.0f}) | weave episodes {len(E)} ({len(E) / A['h']:.1f}/h), "
        f"time in weave {tot / 60:.1f} min ({tot / 36 / A['h']:.1f} %)")
  for nm, sel in (("hands-off (<20 % hand)", lambda e: e['hand'] < 0.2), ("hand on (>= 20 %)", lambda e: e['hand'] >= 0.2)):
    es = [e for e in E if sel(e)]
    if not es:
      continue
    srcs = {s: sum(1 for e in es if src(e) == s) for s in ("model", "carctl/TX", "other")}
    print(f"   {nm:22}: {len(es):3d} ({len(es) / A['h']:.1f}/h) amp p50 {np.median([e['amp'] for e in es]):.2f} m/s2 dur p50 {np.median([e['dur'] for e in es]):.1f}s | "
          f"comp feedback share {np.median([e['comp'] for e in es]):.2f} toggles {np.median([e['tog'] for e in es]):.2f}/s | hand-gap {np.median([e['gap'] for e in es]):.2f} | gain swing p50 {np.median([e['gsw'] for e in es]):.2f} | "
          f"source " + ", ".join(f"{k} {v}" for k, v in srcs.items()) + f" | TX/cmd p50 {np.median([e['r_TX'] / max(e['r_cmd'], 1e-9) for e in es]):.1f} wheel/TX {np.median([e['r_wheel'] / max(e['r_TX'], 1e-9) for e in es]):.2f}")
  for e in sorted(E, key=lambda e: -e['amp'])[:6]:
    print(f"      {e['r']} t {e['t']:7.1f} {e['dur']:4.1f}s v {e['v']:3.0f} amp {e['amp']:.2f} hand {e['hand']:.2f} comp {e['comp']:.2f} tog {e['tog']:.1f}/s gap {e['gap']:.2f} gsw {e['gsw']:.2f} "
          f"| K {e['r_K'] * 1e6:.0f} cmd {e['r_cmd'] * 1e6:.0f} TX {e['r_TX'] * 1e6:.0f} wheel {e['r_wheel'] * 1e6:.0f} yaw {e['r_yaw'] * 1e6:.0f}")
