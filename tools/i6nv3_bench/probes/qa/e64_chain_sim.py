"""E64: FULL-CHAIN closed-loop sim on the routes with plan data (r50): real controlsd state_control (replayed from the logged
model outputs, Phase 60 delay match E) -> angle (per-route fit apply = sc * k + off) -> real CarController (Phase 59 on/off,
preview on/off) -> identified MDPS plant (truth = that route fold's free-delay fit; compensator = the OTHER fold's common-0.2 s
fit) -> wheel -> vehicle curvature (+50 ms, measured wheel -> yaw). Windows: op active, >= 30 km/h, hands off (hold-
compensated driver torque < 40, not pressed), no blinker, 10-60 s.
Metric: INTENT error = k_car(t) - K(t - 0.375) (the model's action is the curvature it wants 0.375 s later), RMS and lag;
command 20 Hz change RMS; vehicle lateral jerk p95.
Variants: base (E 0, Phase 59 off) | p59 (E 0, Phase 59, preview on) | P0e<e> (Phase 59 preview off, E = e) | P1e<e> (preview on, E = e).
Phase 71 (report §95) additions for the 'Ce' variants, all by environment:
  E64_CD="NAME=value;NAME=a,b,c"  controlsd constants for the replay (as E64_CCP does for CarControllerParams)
  REPLAY_MODEL_DELAY=log          the model's told delay as lagd published it on that drive (builds from Phase 64d on)
  E64_REF=a0                      the reference 'K' returned in the rows is the action RE-TIMED to T = 0.375 s
                                  (K + (2 / v) (psi(0.375) / 0.375 - psi(T) / T), T = told delay + 0.075; needs PSI in the
                                  plan npz), so the intent error / entry / exit tools keep their 0.375 s yardstick on drives
                                  whose logged action was computed for a longer delay."""
import glob
import json
import multiprocessing as mp
import sys
import numpy as np
sys.path.insert(0, "."); sys.path.insert(0, "tools/i6nv3_bench/probes/qa"); sys.path.insert(0, "tools/i6nv3_bench/probes")
S = sys.argv[1]; NW = int(sys.argv[2]) if len(sys.argv) > 2 else 4; MAXW = int(sys.argv[3]) if len(sys.argv) > 3 else 100000
EGRID = [-0.2, -0.1, 0.0, 0.1, 0.2, 0.3]
import os
VARS = os.environ.get("E64_VARS", "").split(",") if os.environ.get("E64_VARS") else (["base", "p59"] + [f"P0e{e:+.1f}" for e in EGRID] + [f"P1e{e:+.1f}" for e in EGRID])
FINAL_E = [0.2, 0.1, 0.0, 0.0, 0.0]
TRUTH = json.load(open(f"{S}/qa/e59_params.json")); COMP = json.load(open(f"{S}/qa/e64_common_fold.json"))
# E64_PLANT=rf: truth = E89 request-rate-floor plant (d, Kf, Kp, Kc, r0) per fold; default m1 = E59 (r0 = 0)
PLANT = os.environ.get("E64_PLANT", "m1")
if PLANT in ("rf", "rfw"):
  _rf = json.load(open(f"{S}/qa/e89_params.json"))
  TRUTH = {f"{fo}{i}": _rf.get(f"RF{fo}{i}", _rf.get(f"RF{i}")) for fo in "AB" for i in range(1, 6)}
# E64_PLANT=rfw (E131, report §91.6): RF + the wake rule for a wheel at rest — the motion RF would give is held back as
# pending until the request has pushed for T_w seconds, then released with time constant tau_r (T_w, tau_r per speed band)
WAKE = None
if PLANT == "rfw":
  _w = json.load(open(f"{S}/qa/e131_params.json"))
  WAKE = [(55.0, *_w["30-55"]), (70.0, *_w["55-70"]), (1e9, *_w["70-200"])]
COMMITS = tuple(os.environ.get("E64_COMMITS", "a0108a47b,98e912d3d,8de98262a").split(","))
CENT = [35.0, 47.5, 62.5, 80.0, 100.0]
BANDS = [(30, 40), (40, 55), (55, 70), (70, 90), (90, 200)]


def windows():
  sys.path.insert(0, "opendbc_repo")
  from opendbc.car.hyundai.carcontroller import compute_hold_torque
  files = sorted(glob.glob(f"{S}/r50/0*.npz"))
  allr = sorted({f[-12:-4] for f in glob.glob(f"{S}/r50/0*.npz") + glob.glob(f"{S}/r_old/0*.npz")})
  out = []
  for f in files:
    r = f[-12:-4]
    d = np.load(f, allow_pickle=True)
    if str(d['commit']) not in COMMITS:
      continue
    T = d['T']; n = len(T); v = d['v']
    vm = v / 3.6; la = np.abs(vm ** 2 * d['k_meas'])
    hold = np.array([compute_hold_torque(a, b) for a, b in zip(vm[::10], la[::10])]).repeat(10)[:n]
    dd = np.maximum(0.0, np.abs(d['tq_s']) - hold)
    bl = d['lb'] | d['rb'] | (d['lcs'] > 0)
    ok = d['lat'] & (v >= 30) & (dd < 40) & ~d['pressed'] & ~bl
    ok[:1200] = False
    e = np.flatnonzero(np.diff(np.concatenate([[0], ok.astype(int), [0]])))
    for a, b in zip(e[::2], e[1::2]):
      k = a
      while b - k >= 1000:
        k1 = min(k + 6000, b)
        if k1 - k >= 1000:
          out.append((r, k, k1, allr.index(r) % 2))
        k = k1
  return out


def run(args):
  r, a, b, fold = args
  import replay_state_control as R
  import openpilot.selfdrive.controls.controlsd as cd
  from phase_tests.harness import Sim
  from opendbc.car.hyundai.values import CarControllerParams as CCP
  R.load(S, r); d = R.d
  T = d['T']
  m = d['lat'] & np.isfinite(d['apply'])
  A = np.stack([d['k_ctrl'][m], np.ones(m.sum())], 1); sc, off = np.linalg.lstsq(A, d['apply'][m], rcond=None)[0]
  truth = np.array([(list(TRUTH[f"{'A' if fold == 0 else 'B'}{i}"]) + [0.0])[:5] for i in range(1, 6)])
  comp = np.array([COMP[f"{'B' if fold == 0 else 'A'}{i}"] for i in range(1, 6)])
  res = {}
  for var in VARS:
    e = float(var[3:]) if (var.startswith("P") or var.startswith("B0")) else 0.0
    if var.startswith("Ce"):
      eh = float(var[2:]); cd.LOOKAHEAD_DELAY_EXTRA_S = ([float(x) for x in os.environ["E64_ETAB"].split(",")] if os.environ.get("E64_ETAB") else [0.2, 0.1, eh, eh, eh])
    else:
      cd.LOOKAHEAD_DELAY_EXTRA_S = list(FINAL_E) if var == "final" else [e] * 5
    cd_saved = {}
    for kv in [x for x in os.environ.get("E64_CD", "").split(";") if x] if var.startswith("Ce") else []:
      k_, v_ = kv.split("=")
      cur = getattr(cd, k_); cd_saved[k_] = cur
      setattr(cd, k_, [float(x) for x in v_.split(",")] if isinstance(cur, (list, tuple)) else (v_ in ("1", "True") if isinstance(cur, bool) else float(v_)))
    out = R.replay("p57", T[a], T[b - 1])
    for k_, v_ in cd_saved.items():
      setattr(cd, k_, v_)
    cd.LOOKAHEAD_DELAY_EXTRA_S = [0.0] * 5
    kcmd = np.interp(T[a:b], [o[0] for o in out], [o[1] for o in out])
    cmd = sc * kcmd + off
    CCP.MDPS_COMP_LAMBDA = 0.0 if (var == "base" or var.startswith("B0")) else 2.0
    CCP.MDPS_COMP_PREVIEW = 0.0 if (var.startswith("P0") or var == "final") else 1.0
    CCP.MDPS_COMP_SPEEDS_KPH = CENT; CCP.MDPS_COMP_DELAY_S = comp[:, 0].tolist(); CCP.MDPS_COMP_KF = comp[:, 1].tolist()
    CCP.MDPS_COMP_KP = comp[:, 2].tolist(); CCP.MDPS_COMP_KC = comp[:, 3].tolist()
    if var.startswith("Ce") or var == "final":
      CCP.MDPS_COMP_PREVIEW = 0.0
    for kv in [x for x in os.environ.get("E64_CCP", "").split(";") if x] if var.startswith("Ce") else []:
      k_, v_ = kv.split("=")
      cur = getattr(CCP, k_)
      setattr(CCP, k_, [float(x) for x in v_.split(",")] if isinstance(cur, (list, np.ndarray)) else (v_ in ("1", "True") if isinstance(cur, bool) else float(v_)))
    if not var.startswith("Ce"):
      CCP.MDPS_COMP_LEAK_TAU_S = 1e9; CCP.MDPS_COMP_FREEZE = False; CCP.MDPS_COMP_GMAX_V = [6.0] * 5
    a0 = max(a - 300, 0)
    sim = Sim(); w = float(d['ang'][a0]); hist = []; W = []
    pend = 0.0; push = 0.0; stuck = WAKE is not None; rest_n = 0; wh = [w]      # rfw wake-rule state
    for k in range(a0, b):
      vk = float(d['v'][k]); closed = k >= a
      wheel = w if closed else float(d['ang'][k])
      ck = float(cmd[k - a]) if closed else float(d['apply'][k])
      sim.step(v=vk / 3.6, tq=float(d['tq_s'][k]), wheel=wheel, cmd=ck, lat_active=bool(d['lat'][k]), pressed=bool(d['pressed'][k]),
               blinker=bool(d['lb'][k]), blinker_right=bool(d['rb'][k]), a_ego=float(d['a_ego'][k]), gas=bool(d['gas'][k]),
               lane_prob_min=float(d['lane_min'][k]), bs_l=bool(d['bsl'][k]), bs_r=bool(d['bsr'][k]))
      mm = sim.lkas_alt(); act = bool(mm and mm["LKAS_ANGLE_ACTIVE"] == 2)
      tx = float(mm["ADAS_StrAnglReqVal"]) if act else wheel
      g = min(max((float(mm["ADAS_ACIAnglTqRedcGainVal"]) - 0.2) / 0.5, 0.0), 1.0) if act else 0.0
      hist.append(tx)
      pt = [np.interp(vk, CENT, truth[:, j]) for j in range(5)]
      nd = int(round(pt[0] / 0.01))
      u1 = hist[-1 - nd] if len(hist) > nd else hist[0]; u2 = hist[-2 - nd] if len(hist) > nd + 1 else hist[0]
      if closed:
        if pt[4] > 0.0:   # rate floor: only the request rate (0.1 s difference) above r0 drives the wheel
          ua = hist[-11 - nd] if len(hist) > nd + 10 else hist[0]; ub = hist[-12 - nd] if len(hist) > nd + 11 else hist[0]
          r1 = (u1 - ua) / 0.1; r2 = (u2 - ub) / 0.1
          dz = lambda r: r - pt[4] if r > pt[4] else (r + pt[4] if r < -pt[4] else 0.0)
          du = 0.5 * (dz(r1) + dz(r2)) * 0.01   # increment of the rate-floored target over this 10 ms step
        else:
          du = u1 - u2
        inc = 0.01 * g * (pt[1] * du / 0.01 + pt[2] * (u1 - w)); cen = -0.01 * pt[3] * w
        if WAKE is None:
          w = w + inc + cen
        else:
          tw, taur = next((x[1], x[2]) for x in WAKE if vk < x[0])
          if tw > 0.0 and stuck:
            if pend * inc < 0.0:
              push = 0.0
            pend += inc
            push = push + 0.01 if abs(pend) >= 0.05 else max(push - 0.01, 0.0)
            w += cen
            if push >= tw:
              stuck = False; push = 0.0
          else:
            rel = pend * min(0.01 / max(taur, 0.01), 1.0); pend -= rel; w += inc + rel + cen
          wh.append(w); rest = abs(w - wh[-31]) < 0.15 if len(wh) > 31 else True
          rest_n = rest_n + 1 if rest else 0
          if tw > 0.0 and not stuck and rest_n >= 50 and abs(pend) < 0.1:
            stuck = True; push = 0.0
        W.append(w)
      else:
        w = float(d['ang'][k])
    W = np.array(W)
    kcar = (np.concatenate([np.full(5, W[0]), W[:-5]]) - off) / sc          # + 50 ms wheel -> yaw
    res[var] = (kcmd, kcar)
  K = d['k_model'][a:b]; v = d['v'][a:b]
  if os.environ.get("E64_REF") == "a0":
    P = R.P; TI = np.array([10.0 * (i / 32) ** 2 for i in range(33)])
    j = np.clip(np.searchsorted(P['T'], T[a:b], side="right") - 1, 0, len(P['T']) - 1)
    told = np.array([float(R.LD[max(np.searchsorted(R.LD[:, 0], t, side='right') - 1, 0), 1]) for t in P['T'][j]]) if R.LD is not None else np.full(b - a, 0.3)
    tt = told + 0.075; vv = np.maximum(v / 3.6, 1.0)
    psi_t = np.array([np.interp(t_, TI, P['PSI'][i_]) for t_, i_ in zip(tt, j)]); psi_0 = np.array([np.interp(0.375, TI, P['PSI'][i_]) for i_ in j])
    K = P['K'][j] + (2.0 / vv) * (psi_0 / 0.375 - psi_t / tt)
  return r, a, fold, K, v, res


def summarize(rows, sel=None):
  out = {}
  for var in VARS:
    se = []; lags = []; dch = []; jerk = []; band_se = {i: [] for i in range(5)}
    for r, a, fold, K, v, res in rows:
      kcmd, kcar = res[var]
      e = kcar[38:] - K[:-38]; vv = v[38:]
      se.append(e)
      for i, (lo, hi) in enumerate(BANDS):
        mm = (vv >= lo) & (vv < hi)
        if mm.any():
          band_se[i].append(e[mm])
      dch.append(kcmd[5:] - kcmd[:-5])
      jerk.append(np.abs(np.diff(((v / 3.6) ** 2 * kcar)[::10])) / 0.1)
      if len(K) >= 1000 and np.std(K) > 0.3e-3:
        x = K - K.mean(); best = (-2, 0)
        for L in range(0, 100, 2):
          y = kcar[L:] - kcar[L:].mean(); xx = x[:len(y)]
          cc = np.dot(xx, y) / (np.linalg.norm(xx) * np.linalg.norm(y) + 1e-12)
          if cc > best[0]:
            best = (cc, L)
        if best[0] > 0.7:
          lags.append(best[1] * 10)
    e = np.concatenate(se)
    out[var] = dict(rms=np.sqrt(np.mean(e ** 2)), lag=np.median(lags) if lags else np.nan,
                    band={i: np.sqrt(np.mean(np.concatenate(x) ** 2)) if x else np.nan for i, x in band_se.items()},
                    dch=np.sqrt(np.mean(np.concatenate(dch) ** 2)), jerk=np.percentile(np.concatenate(jerk), 95))
  return out


if __name__ == "__main__":
  W = windows()
  rng = np.random.default_rng(3); W = [W[i] for i in rng.permutation(len(W))[:MAXW]]
  print(f"windows {len(W)} ({sum(b - a for _, a, b, _ in W) / 6000:.0f} min) from {len({w[0] for w in W})} routes", flush=True)
  with mp.Pool(NW) as pool:
    rows = pool.map(run, W, chunksize=1)
  import pickle
  pickle.dump(rows, open(f"{S}/qa/e64_rows{os.environ.get('E64_TAG', '')}.pkl", "wb"))
  allm = summarize(rows)
  for var in VARS:
    m = allm[var]
    print(f"{var:8} intent RMS {m['rms'] * 1e3:.3f}e-3 | lag K->car {m['lag']:.0f} ms | by band " + " ".join(f"{m['band'][i] * 1e3:.3f}" for i in range(5))
          + f" | cmd 20 Hz change RMS {m['dch'] * 1e6:.1f}e-6 | lat jerk p95 {m['jerk']:.2f}", flush=True)
