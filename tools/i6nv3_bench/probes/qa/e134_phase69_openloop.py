"""E134 (report §92): Phase 69b / 69c on the real drives, OPEN-LOOP (as E93: the current CarController replayed on the
logged speed / torque / wheel / command; the logged wheel does not react, so request sizes are an upper bound, but the
exit lead's gate — the wheel measured still out beyond the command — sees the wheel as it really lagged).
Variants: now (Phase 69 on) vs k69 (EXIT_LEAD_GAIN = 0, CURVE_TRIM_FAST_RATE_DPS = 0).
Per speed band: E93's request statistics, how often and how much the exit lead acts, the request at real exits (command
steady in a curve, then unwinding >= 5 deg within 2.5 s) with and without it, centre crossings, and the trim.
argv: S builds_csv"""
import glob
import multiprocessing as mp
import sys
import numpy as np
sys.path.insert(0, "."); sys.path.insert(0, "tools/i6nv3_bench/probes/qa")
import e93_openloop_gate_bf36 as e93
S = sys.argv[1]; BUILDS = sys.argv[2].split(",") if len(sys.argv) > 2 else ["bf36aa2"]
e93.S = S
K69 = "EXIT_LEAD_GAIN=0;CURVE_TRIM_FAST_RATE_DPS=0"
BANDS = [(30, 55), (55, 70), (70, 90), (90, 200)]


def segments():
  out = []
  for f in sorted(glob.glob(f"{S}/r50/0*.npz")):
    d = np.load(f, allow_pickle=True)
    if str(d['commit'])[:7] not in BUILDS: continue
    ok = d['lat'] & (d['v'] >= 25)
    e = np.flatnonzero(np.diff(np.concatenate([[0], ok.astype(int), [0]])))
    out += [(f, a, b) for a, b in zip(e[::2], e[1::2]) if b - a >= 500]
  return out


def run(args):
  f, a, b, var = args
  from phase_tests.harness import Sim
  from opendbc.car.hyundai.values import CarControllerParams as CCP
  if e93._DEFAULTS is None:
    e93._DEFAULTS = {k: getattr(CCP, k) for k in dir(CCP) if k.startswith(("MDPS_COMP_", "HAND_GAP_", "CURVE_TRIM_", "ENTRY_BOOST_", "EXIT_LEAD_"))}
  for k, val in e93._DEFAULTS.items(): setattr(CCP, k, val)
  for kv in [x for x in var.split(";") if x]:
    k_, v_ = kv.split("="); cur = getattr(CCP, k_)
    setattr(CCP, k_, [float(x) for x in v_.split(",")] if isinstance(cur, (list, np.ndarray)) else (v_ in ("1", "True") if isinstance(cur, bool) else float(v_)))
  dz = np.load(f, allow_pickle=True)
  d = {k: dz[k] for k in ('ang', 'v', 'tq_s', 'apply', 'lat', 'pressed', 'lb', 'rb', 'a_ego', 'gas', 'lane_min', 'bsl', 'bsr', 'wire', 'tx_active', 'rate')}
  a0 = max(a - 500, 0); sim = Sim(); rows = []
  for k in range(a0, b):
    sim.step(v=float(d['v'][k]) / 3.6, tq=float(d['tq_s'][k]), wheel=float(d['ang'][k]), cmd=float(d['apply'][k]), lat_active=bool(d['lat'][k]),
             pressed=bool(d['pressed'][k]), blinker=bool(d['lb'][k]), blinker_right=bool(d['rb'][k]), a_ego=float(d['a_ego'][k]),
             gas=bool(d['gas'][k]), lane_prob_min=float(d['lane_min'][k]), bs_l=bool(d['bsl'][k]), bs_r=bool(d['bsr'][k]), wheel_rate=float(d['rate'][k]))
    if k >= a:
      m = sim.lkas_alt(); act = bool(m and m["LKAS_ANGLE_ACTIVE"] == 2)
      rows.append((float(d['apply'][k]), float(m["ADAS_StrAnglReqVal"]) if m else np.nan, float(d['ang'][k]), float(d['v'][k]), act,
                   float(d['wire'][k]), bool(d['tx_active'][k] == 2), sim.cc.eb_off, sim.cc.curve_trim, float(d['tq_s'][k]), sim.cc.xl_off))
  return var, np.array(rows)


if __name__ == "__main__":
  segs = segments(); print(f"segments {len(segs)} ({sum(b - a for _, a, b in segs) / 6000:.0f} min op >= 25 km/h, builds {BUILDS})", flush=True)
  with mp.Pool(4) as pool:
    res = pool.map(run, [(f, a, b, var) for var in ("", K69) for f, a, b in segs], chunksize=1)
  R = {nm: [X for vv, X in res if vv == var and len(X) > 300] for nm, var in (("now", ""), ("k69", K69))}
  for nm in ("k69", "now"):
    e93.show(nm, e93.summarize(R[nm]))
  N = np.concatenate(R["now"]); K = np.concatenate(R["k69"])
  cmd, tx, wh, v, act, eb, tr, xl = N[:, 0], N[:, 1], N[:, 2], N[:, 3], N[:, 4] > 0.5, N[:, 7], N[:, 8], N[:, 10]
  txk, trk, ebk = K[:, 1], K[:, 8], K[:, 7]
  for lo, hi in BANDS:
    m = act & (v >= lo) & (v < hi)
    if m.sum() < 500: continue
    on = m & (np.abs(xl) > 0.01); cross = on & (np.abs(cmd) > 0.5) & (np.sign(tx) != np.sign(cmd))
    fast = m & (np.abs(tr) > 0.05) & (np.abs(np.gradient(tr)) * 100 > 2.2)
    print(f"[{lo}-{hi} km/h, {m.sum() / 6000:.1f} min] exit lead on {on.sum() / m.sum():5.1%} of op, |lead| p50/p95/max {np.median(np.abs(xl[on])) if on.any() else 0:.2f}/"
          f"{np.percentile(np.abs(xl[on]), 95) if on.any() else 0:.2f}/{np.abs(xl[m]).max():.2f} deg, request on the wrong side of centre {cross.sum()} frames"
          f" | entry offset time {np.mean(np.abs(ebk[m]) > 0.05):5.1%} -> {np.mean(np.abs(eb[m]) > 0.05):5.1%}"
          f" | trim on {np.mean(np.abs(trk[m]) > 0.05):5.1%} -> {np.mean(np.abs(tr[m]) > 0.05):5.1%}, |trim| p50 {np.median(np.abs(trk[m & (np.abs(trk) > 0.05)])) if (m & (np.abs(trk) > 0.05)).any() else 0:.2f} -> "
          f"{np.median(np.abs(tr[m & (np.abs(tr) > 0.05)])) if (m & (np.abs(tr) > 0.05)).any() else 0:.2f}, ramping faster than 2.2 deg/s {fast.sum() / max(m.sum(), 1):5.2%} of op")
  # real exits: command steady in a curve, then unwinding
  TS = [0.0, 0.5, 1.0, 1.5, 2.0]; E = {i: [] for i in range(len(BANDS))}
  off = 0
  for X, Xk in zip(R["now"], R["k69"]):
    ap, t1, t0, w, vv, xl1 = X[:, 0], X[:, 1], Xk[:, 1], X[:, 2], X[:, 3], X[:, 10]; n = len(ap); k = 150
    while k < n - 260:
      s = np.sign(ap[k - 100]); back = (ap[k - 100] - ap[k]) * s
      if abs(ap[k - 100]) >= 5.0 and back >= 2.0 and (ap[k - 101] - ap[k - 1]) * s < 2.0 and abs(ap[k - 100] - ap[k - 150]) < 1.0 and np.max((ap[k - 100] - ap[k:k + 250]) * s) >= 5.0:
        i = next((q for q, (lo, hi) in enumerate(BANDS) if lo <= vv[k] < hi), None)
        if i is not None:
          E[i].append([np.max(np.abs(xl1[k - 50:k + 200]))] + [x for t in TS for x in ((ap[k - 100] - ap[k + int(t * 100)]) * s, (t0[k - 100] - t0[k + int(t * 100)]) * s,
                                                                                         (t1[k - 100] - t1[k + int(t * 100)]) * s, (w[k - 100] - w[k + int(t * 100)]) * s)])
        k += 300; continue
      k += 1
  print("real exits (all, hands on or off): unwinding since -1 s, median — command / request without / request with Phase 69 / logged wheel")
  for i, (lo, hi) in enumerate(BANDS):
    Y = np.array(E[i])
    if len(Y) < 4: continue
    print(f"  [{lo}-{hi}] n={len(Y)}, lead acted (> 0.1 deg) in {np.mean(Y[:, 0] > 0.1):4.0%}, peak lead p50 {np.median(Y[:, 0]):.2f}: " +
          " | ".join(f"{t:.1f}s {np.median(Y[:, 1 + 4 * q]):4.1f}/{np.median(Y[:, 2 + 4 * q]):4.1f}/{np.median(Y[:, 3 + 4 * q]):4.1f}/{np.median(Y[:, 4 + 4 * q]):4.1f}" for q, t in enumerate(TS)))
