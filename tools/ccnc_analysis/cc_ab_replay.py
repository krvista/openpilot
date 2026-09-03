#!/usr/bin/env python3
"""A/B replay: feed IDENTICAL logged inputs (plan cc, CS) through the real CarController of a given
tree (REPO env), collect the TX angle (LKAS_ALT ADAS_StrAnglReqVal), and report 0.8-2.5 Hz band RMS
in hands-off eff-active 1 s windows per speed regime (>= 6 m/s only: outside the low-speed zone, so
the replay's lead-less passthrough logic cannot diverge from the car).
usage: REPO=/home/user/op_base python3 cc_ab_replay.py <prefix>...   (npz2 in scratchpad)"""
import glob, os, sys, inspect
import numpy as np
REPO = os.environ.get("REPO", "/home/user/openpilot")
sys.path.insert(0, REPO); sys.path.insert(0, os.path.join(REPO, "opendbc_repo"))
from phase_tests.harness import Sim  # noqa
D = os.path.join(os.path.dirname(os.path.abspath(__file__)), "npz2")
HAS_RATE = "wheel_rate" in inspect.signature(Sim.step).parameters
def bp(x, fs=100.0, lo=0.8, hi=2.5):
    n=len(x); F=np.fft.rfft(x-x.mean()); fr=np.fft.rfftfreq(n,1/fs); F[(fr<lo)|(fr>hi)]=0; return np.fft.irfft(F,n)
regs = {"6-8.3": [], "8.3-13": []}; nseg = 0
for f in sorted(glob.glob(os.path.join(D, "*.npz"))):
    if f.split("--")[0].split("_")[-1] not in set(sys.argv[1:]): continue
    z = np.load(f, allow_pickle=True); t = z["cs_t"]
    if len(t) < 500 or len(z["cc_t"]) < 2 or len(z["sp_t"]) < 2: continue
    v = np.nan_to_num(z["cs_v"]); ang = np.nan_to_num(z["cs_ang"]); rate = np.nan_to_num(z["cs_rate"]); tq = np.nan_to_num(z["cs_tq"])
    pr = z["cs_pr"] > 0.5; blink = z["cs_blink"] > 0.5; bsl = z["cs_bsl"] > 0.5; bsr = z["cs_bsr"] > 0.5; stand = z["cs_stand"] > 0.5
    lat = np.interp(t, z["cc_t"], z["cc_lat"]) > 0.5; paused = np.interp(t, z["sp_t"], z["sp_paused"]) > 0.5
    cc = np.interp(t, z["cc_t"], z["cc_ang"]); eff = lat & ~paused
    if not (eff & (v >= 6)).any(): continue
    sim = Sim(); tx = np.zeros(len(t))
    for i in range(len(t)):
        kw = dict(v=float(v[i]), tq=float(tq[i]), wheel=float(ang[i]), cmd=float(cc[i]), lat_active=bool(lat[i]),
                  pressed=bool(pr[i]), blinker=bool(blink[i]), bs_l=bool(bsl[i]), bs_r=bool(bsr[i]), standstill=bool(stand[i]))
        if HAS_RATE: kw["wheel_rate"] = float(rate[i])
        try:
            sim.step(**kw); m = sim.lkas_alt(); tx[i] = m["ADAS_StrAnglReqVal"] if m else ang[i]
        except Exception:
            tx[i] = ang[i]
    nseg += 1
    ftx = bp(tx); fcc = bp(cc)
    for s in range(0, len(t) - 100, 100):
        w = slice(s, s + 100)
        if not (eff[w].all() and not pr[w].any()): continue
        if ang[w].std() < 0.4: continue
        vm = v[w].mean(); key = "6-8.3" if 6 <= vm < 8.3 else ("8.3-13" if vm < 13 else None)
        if key: regs[key].append((np.sqrt(np.mean(fcc[w] ** 2)), np.sqrt(np.mean(ftx[w] ** 2))))
print(f"REPO={REPO} segments replayed={nseg} wheel_rate_supported={HAS_RATE}")
for k, vals in regs.items():
    a = np.array(vals) if vals else np.zeros((0, 2))
    print(f"  {k:7s} n={len(a):3d}  plan-in {a[:,0].mean() if len(a) else 0:.3f}  TX-out {a[:,1].mean() if len(a) else 0:.3f}  CC-stage added ~{np.sqrt(max((a[:,1].mean())**2-(a[:,0].mean())**2,0)) if len(a) else 0:.3f}")
