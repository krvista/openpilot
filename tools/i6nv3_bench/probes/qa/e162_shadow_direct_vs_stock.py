"""E162 (report §97-98): on the stock-LFA capture drives, what would openpilot's DIRECT path (Phase 72) have asked of the
MDPS at the same moment on the same road - against what the factory LFA asked and did? Open loop ("shadow"): the real
controlsd.state_control is replayed over the logged model outputs with lateral forced active (REPLAY_PROFILE=solo), the
action re-timed to the solo delay (REPLAY_FORCE_TOLD, modeld's own formula), curvature -> wheel angle by the drives' own
ratio; that command then runs through the REAL car controller in the solo profile (phase_tests harness: hysteresis band,
low-speed smoothing, VM limits, TX governor) closed around the relayed MDPS as measured (request -> wheel: pure delay
MDPS_DELAY_S + first-order MDPS_TAU_S) - 'shadow TX' / 'shadow wheel' are that chain's output.
The plan is the logged one - it was made while the FACTORY LFA held its own line (11-13 cm left of the model's centre),
so the shadow request carries a standing pull toward the model's centre; smoothness and timing are what this compares.
Windows: factory LFA steering, hands off, no blinker within 3 s, >= 20 s long. Per speed band:
  band RMS (deg) at 0.1-0.3 / 0.3-0.6 / 0.6-1.2 / 1.2-2.5 / 2.5-5 Hz of: factory request (0xCB), factory wheel, shadow
  command (controlsd direct path, as an angle), shadow TX, shadow wheel, and the legacy controlsd chain's command (told
  delay as logged);
  request rate p50 / p95 / p99 (deg/s); the shadow wheel's lead over the factory wheel (xcorr, 6 s windows with
  movement; + = openpilot earlier); mean (shadow request - factory wheel).
argv: S route [route ...]   env: E162_TOLD (default 0.32)"""
import os
import sys
import numpy as np
sys.path.insert(0, "."); sys.path.insert(0, "opendbc_repo"); sys.path.insert(0, "tools/i6nv3_bench/probes/qa")
from opendbc.car.hyundai.carcontroller import compute_hold_torque  # noqa: E402
import replay_state_control as R  # noqa: E402
from phase_tests.harness import Sim  # noqa: E402
S = sys.argv[1]; TOLD = os.environ.get("E162_TOLD", "0.32")
MDPS_DELAY_S, MDPS_TAU_S = 0.05, 0.13
BANDS = [(30, 55), (55, 70), (70, 90), (90, 200)]; FB = [(0.1, 0.3), (0.3, 0.6), (0.6, 1.2), (1.2, 2.5), (2.5, 5.0)]
SIG = ["factory request", "factory wheel", "shadow command", "shadow TX", "shadow wheel", "legacy command"]


def band_power(x, lo, hi, W=1024):
  out = []
  for a in range(0, len(x) - W + 1, W // 2):
    seg = (x[a:a + W] - x[a:a + W].mean()) * np.hanning(W)
    X = np.abs(np.fft.rfft(seg)) ** 2; f = np.fft.rfftfreq(W, 0.01)
    out.append(X[(f >= lo) & (f < hi)].sum() / (W * W / 4) * 2)
  return out


def chain(cmd, v_kph, w0):
  """the real car controller (solo profile, factory LFA off) closed around the relayed MDPS -> (TX, wheel)"""
  sim = Sim(solo=True); n = len(cmd); tx = np.empty(n); wh = np.empty(n); w = float(w0)
  a = 0.01 / (MDPS_TAU_S + 0.01); k = int(round(MDPS_DELAY_S / 0.01)); hist = [float(w0)] * (k + 1)
  for i in range(n):
    sim.step(v=float(v_kph[i]) / 3.6, tq=0.0, wheel=w, cmd=float(cmd[i]), lat_active=True, lfa_state=0, gw_req_angle=hist[-1])
    t = float(sim.tx_angle()) if sim.effective_lat_active() else w
    hist.append(t); w += a * (hist[-1 - k] - w)
    tx[i] = t; wh[i] = w
  return tx, wh


acc = {b: dict(n=0, pw={s: [[] for _ in FB] for s in SIG}, rate={s: [] for s in SIG}, lead=[], bias=[]) for b in BANDS}
for r in sys.argv[2:]:
  R.load(S, r); d = R.d; T = d['T']; n = len(T)
  if 'cb_req' not in d:
    continue
  v = d['v']; vm = v / 3.6; la = np.abs(vm ** 2 * d['k_meas'])
  hold = np.array([compute_hold_torque(a, c) for a, c in zip(vm[::10], la[::10])]).repeat(10)[:n]
  hand = (np.maximum(0.0, np.abs(d['tq_s']) - hold) >= 40) | d['pressed'].astype(bool)
  bl = d['lb'] | d['rb'] | (d['lcs'] > 0); cs = np.concatenate([[0], np.cumsum(bl)]); idx = np.arange(n)
  busy = (cs[np.minimum(idx + 300, n)] - cs[np.maximum(idx - 300, 0)]) > 0
  stock = (d['cb_lvl2'].astype(int) == 2) & (d['mdps_active'].astype(int) == 2) & (d['lfa_sym'].astype(int) == 2) & ~d['lat'].astype(bool)
  ok = stock & ~hand & ~busy & (v >= 30)
  runs = []; a = None
  for i in range(n):
    if ok[i] and a is None:
      a = i
    if a is not None and (not ok[i] or i == n - 1):
      if i - a >= 2000:
        runs.append((a, i))
      a = None
  for a, b in runs:
    os.environ['REPLAY_FORCE_LAT'] = '1'
    os.environ['REPLAY_PROFILE'] = 'solo'; os.environ['REPLAY_FORCE_TOLD'] = TOLD
    D = R.replay("new", T[a], T[b - 1])
    os.environ['REPLAY_PROFILE'] = 'legacy'; os.environ.pop('REPLAY_FORCE_TOLD'); os.environ['REPLAY_MODEL_DELAY'] = 'log'
    L = R.replay("new", T[a], T[b - 1])
    m = min(len(D), len(L), b - a)
    sl = slice(a, a + m); dp = np.interp(v[sl], *R.DEG_PER) * 1e3
    off = float(np.median(d['mdps_ang'][sl] + dp * d['k_meas'][sl]))          # wheel = -dp * k + offset
    req_d = -np.array([x[1] for x in D[:m]]) * dp + off; req_l = -np.array([x[1] for x in L[:m]]) * dp + off
    tx_d, wh_d = chain(req_d, v[sl], d['mdps_ang'][a])
    sig = {"factory request": d['cb_req'][sl], "factory wheel": d['mdps_ang'][sl], "shadow command": req_d,
           "shadow TX": tx_d, "shadow wheel": wh_d, "legacy command": req_l}
    for bi, (lo, hi) in enumerate(BANDS):
      W = 600
      for w0 in range(0, m - 1024, 512):
        ws = slice(w0, w0 + 1024)
        if not ((v[sl][ws] >= lo) & (v[sl][ws] < hi)).mean() > 0.8:
          continue
        A = acc[(lo, hi)]; A['n'] += 512
        for s_ in SIG:
          for fi, (f0, f1) in enumerate(FB):
            A['pw'][s_][fi] += band_power(sig[s_][ws], f0, f1)
          A['rate'][s_].append(np.abs(sig[s_][ws][10:] - sig[s_][ws][:-10]) / 0.1)
        A['bias'].append(float(np.mean(req_d[ws] - d['mdps_ang'][sl][ws])))
        x = sig["shadow wheel"][ws][100:700]; y = sig["factory wheel"][ws]
        if np.std(x) >= 0.2:
          xs = x - x.mean(); best = (-2.0, 0)
          for Lg in range(-60, 61, 2):
            ys = y[100 + Lg:700 + Lg] - y[100 + Lg:700 + Lg].mean()
            c = float(np.dot(xs, ys) / (np.linalg.norm(xs) * np.linalg.norm(ys) + 1e-12))
            if c > best[0]:
              best = (c, Lg)
          if best[0] > 0.7:
            A['lead'].append(best[1] * 10)
  print(f"{r}: {len(runs)} windows, {sum(b - a for a, b in runs) / 6000:.1f} min replayed", flush=True)
for b in BANDS:
  A = acc[b]
  if A['n'] < 3000:
    continue
  print(f"\n[{b[0]}-{b[1]} km/h] {A['n'] / 6000:.1f} min | told delay {TOLD} s")
  print(f"  {'band RMS deg':18s}" + "".join(f"{f0}-{f1} Hz".rjust(12) for f0, f1 in FB) + "   | rate p50 / p95 / p99 deg/s")
  for s_ in SIG:
    rt = np.concatenate(A['rate'][s_])
    print(f"  {s_:18s}" + "".join(f"{np.sqrt(np.mean(A['pw'][s_][fi])):12.3f}" for fi in range(len(FB))) +
          f"   | {np.percentile(rt, 50):.1f} / {np.percentile(rt, 95):.1f} / {np.percentile(rt, 99):.1f}")
  if A['lead']:
    print(f"  shadow wheel leads the factory wheel by {np.median(A['lead']):+.0f} ms (median of {len(A['lead'])}, 25-75 %: "
          f"{np.percentile(A['lead'], 25):+.0f} .. {np.percentile(A['lead'], 75):+.0f})")
  print(f"  shadow command - factory wheel: mean {np.mean(A['bias']):+.2f} deg")
