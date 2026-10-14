"""E5c: closed-loop sim from the cached identification (e5b), with calibration knobs: noise scale NS, feedback multiplier
FB (scales every predictor's offset/heading coefficients — corrects regression attenuation from noisy e/c1), actuator
steady gain GAIN. Usage: e5c_sim.py <scratch> NS FB GAIN variant[,variant...]"""
import pickle
import sys
import numpy as np

S = sys.argv[1]; NS, FB, GAIN = float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4]); VARS = tuple(sys.argv[5].split(","))
FUT = [0.0, 0.3, 0.6, 0.9, 1.2, 1.5]
BANDS = [(30, 55), (55, 80), (80, 200)]
PRED = ("K", "kcub", "k9", "k3", "k6", "k5")
c = pickle.load(open(f"{S}/qa/sim_cache.pkl", "rb"))
segs, ident, COV, RHO = c['segs'], c['ident'], c['COV'], c['RHO']
for b in BANDS:
  for p in PRED:
    co, sd = ident[b][p]
    co = co.copy(); co[len(FUT)] *= FB; co[len(FUT) + 1] *= FB
    ident[b][p] = (co, sd)
rng_seed = 7
L_ch = NS * np.linalg.cholesky(COV + 1e-18 * np.eye(len(PRED)))
def band_of(vk):
  return BANDS[0] if vk < 55 else BANDS[1] if vk < 80 else BANDS[2]

def cmd_form(name, P, ab, v, tau):
  K = P['K']
  blend = np.clip((ab - 0.8e-3) / 0.7e-3, 0, 1) * (ab >= 0.8e-3)
  if name == "action":
    return K
  if name == "cubic":                 # Phase 41-2 shipped before Phase 55
    t_ah = min(np.interp(v, [5.6, 13.9, 27.8, 38.9], [0.08, 0.10, 0.13, 0.18]) + np.interp(ab, [0.0008, 0.005], [0.0, 0.20]), 0.32) + tau
    dk = 0.7 * t_ah / max(v, 5.0) ** 2
    return K + blend * np.clip(P['kcub'] - K, -dk, dk)
  dk = 0.7 * (0.9 + tau) / max(v, 5.0) ** 2
  if name == "P55":
    return K + blend * np.clip(P['k9'] - K, -dk, dk)
  if name == "delta":
    return K + blend * np.clip(P['k9'] - P['k3'], -dk, dk)
  if name == "P55half":
    return K + 0.5 * blend * np.clip(P['k9'] - K, -dk, dk)
  if name == "P55ts0.6":
    dk6 = 0.7 * (0.6 + tau) / max(v, 5.0) ** 2
    return K + blend * np.clip(P['k6'] - K, -dk6, dk6)
  if name == "P55ts0.45":
    dk5 = 0.7 * (0.45 + tau) / max(v, 5.0) ** 2
    return K + blend * np.clip(P['k5'] - K, -dk5, dk5)
  if name == "P55trans":            # orient lead only while the plan predicts a curvature change
    wch = min(max((abs(P['k9'] - P['k3']) - 0.3e-3) / 0.5e-3, 0.0), 1.0)
    return K + blend * wch * np.clip(P['k9'] - K, -dk, dk)
  if name == "act+delta":           # action keeps its feedback; add the plan's predicted change (k9 - k3), no gate
    return K + np.clip(P['k9'] - P['k3'], -dk, dk)
  if name == "act+delta.5":
    return K + 0.5 * np.clip(P['k9'] - P['k3'], -dk, dk)
  if name.startswith("ad:"):          # ad:<weight>:<ref pred>[:J]  -> K + w * clip(k9 - ref, +-dk(J))
    parts = name.split(":")
    w, ref = float(parts[1]), parts[2]
    J = float(parts[3]) if len(parts) > 3 else 0.7
    dkj = J * (0.9 + tau) / max(v, 5.0) ** 2
    return K + w * np.clip(P['k9'] - P[ref], -dkj, dkj)
  if name.startswith("ad6:"):         # lead from the 0.6 s sample instead
    w = float(name.split(":")[1])
    return K + w * np.clip(P['k6'] - P['k3'], -dk, dk)
  raise ValueError(name)

results = {vn: {"e_st": [], "e_cv": [], "c1": [], "dcmd": [], "jerk": [], "spec": 0} for vn in VARS}
DT = 0.01; DELAY = 0.30; TAU_A = 0.10
for si, (Ts, vs, krs) in enumerate(segs):
  t_end = Ts[-1]
  tt = np.arange(0, t_end - 1.6, DT)
  v_t = np.interp(tt, Ts, vs); kr_t = np.interp(tt, Ts, krs)
  nfr = int(len(tt) / 5)
  rng = np.random.default_rng(rng_seed + si)
  z = rng.standard_normal((nfr, len(PRED)))
  noise = np.zeros((nfr, len(PRED)))
  for k in range(1, nfr):
    noise[k] = RHO * noise[k - 1] + np.sqrt(1 - RHO ** 2) * (L_ch @ z[k])
  for vn in VARS:
    e, c1, kcar, lp = 0.0, 0.0, kr_t[0], kr_t[0]
    buf = [kr_t[0]] * int(DELAY / DT)
    cmd_hist, e_hist, kc_hist = [], np.zeros(len(tt)), np.zeros(len(tt))
    cmd = kr_t[0]
    for k in range(len(tt)):
      v = v_t[k]
      if k % 5 == 0:
        fi = min(k // 5, nfr - 1)
        F = np.array([np.interp(tt[k] + dt, tt, kr_t) for dt in FUT])
        x = np.concatenate([F, [e, c1, 1.0]])
        b = band_of(v * 3.6)
        P = {p: float(x @ ident[b][p][0]) + noise[fi, pi] for pi, p in enumerate(PRED)}
        tau = float(np.interp(v, [8.0, 13.0, 18.0], [0.30, 0.12, 0.08]))
        cmd = float(cmd_form(vn, P, abs(P['K']), v, tau))
        cmd_hist.append(cmd)
      tau = float(np.interp(v, [8.0, 13.0, 18.0], [0.30, 0.12, 0.08]))
      lp += DT / (tau + DT) * (cmd - lp)
      buf.append(lp); delayed = buf.pop(0)
      kcar += DT / TAU_A * (GAIN * delayed - kcar)
      e += -v * c1 * DT
      c1 += v * (kr_t[k] - kcar) * DT
      e_hist[k] = e; kc_hist[k] = kcar
    burn = int(5 / DT)
    E = e_hist[burn:]; KR = np.abs(kr_t[burn:])
    res = results[vn]
    res["e_st"].append(E[KR < 0.5e-3]); res["e_cv"].append(E[KR >= 1.5e-3])
    res["dcmd"].append(np.diff(cmd_hist[burn // 5:]))
    ay = v_t[burn:] ** 2 * kc_hist[burn:]
    res["jerk"].append(np.diff(ay) / DT)
    Ef = np.abs(np.fft.rfft((E - E.mean()) * np.hanning(len(E)))) ** 2
    fr = np.fft.rfftfreq(len(E), DT)
    res["spec"] = res["spec"] + np.interp(np.linspace(0.05, 1.0, 96), fr, Ef) / np.sum(Ef)
print(f"NS {NS} FB {FB} GAIN {GAIN}")
print(f"{'variant':8} {'e RMS str':>9} {'e RMS curve':>11} {'p95|e|':>7} {'d cmd std':>9} {'jerk RMS':>8} {'weave peak':>10}")
for vn in VARS:
  r = results[vn]
  est, ecv = np.concatenate(r["e_st"]), np.concatenate(r["e_cv"])
  ea = np.concatenate([est, ecv])
  fq = np.linspace(0.05, 1.0, 96)
  print(f"{vn:8} {np.sqrt(np.mean(est ** 2)):9.3f} {np.sqrt(np.mean(ecv ** 2)):11.3f} {np.percentile(np.abs(ea), 95):7.3f} "
        f"{np.std(np.concatenate(r['dcmd'])) * 1e3:9.4f} {np.sqrt(np.mean(np.concatenate(r['jerk']) ** 2)):8.3f} {fq[np.argmax(r['spec'])]:8.2f} Hz")
