#!/usr/bin/env python3
"""Exposure of post_grip (reanchor_arm>=30) and yield depth on hands-off, eff-active frames at v>=6 m/s
(replay valid outside the low-speed zone). Reports how often op sits in a yielded state without a real grip
while the plan diverges from the wheel."""
import glob, os, sys, numpy as np
REPO=os.environ.get("REPO","/home/user/openpilot"); sys.path.insert(0,REPO); sys.path.insert(0,os.path.join(REPO,"opendbc_repo"))
from phase_tests.harness import Sim
D=os.path.join(os.path.dirname(os.path.abspath(__file__)),"npz2")
tot=dict(n=0, arm30=0, arm30_gain=[], noarm_gain=[], div3=0, div3_low=0, div3_low_arm=0, dtq_ge100=0, pressed=0)
for f in sorted(glob.glob(os.path.join(D,"*.npz"))):
    if f.split("--")[0].split("_")[-1] not in set(sys.argv[1:]): continue
    z=np.load(f,allow_pickle=True); t=z["cs_t"]
    if len(t)<500 or len(z["cc_t"])<2 or len(z["sp_t"])<2: continue
    v=np.nan_to_num(z["cs_v"]); ang=np.nan_to_num(z["cs_ang"]); rate=np.nan_to_num(z["cs_rate"]); tq=np.nan_to_num(z["cs_tq"])
    pr=z["cs_pr"]>0.5; blink=z["cs_blink"]>0.5; bsl=z["cs_bsl"]>0.5; bsr=z["cs_bsr"]>0.5; stand=z["cs_stand"]>0.5
    lat=np.interp(t,z["cc_t"],z["cc_lat"])>0.5; paused=np.interp(t,z["sp_t"],z["sp_paused"])>0.5; cc=np.interp(t,z["cc_t"],z["cc_ang"])
    if not (lat & ~paused & (v>=6)).any(): continue
    sim=Sim()
    for i in range(len(t)):
        sim.step(v=float(v[i]),tq=float(tq[i]),wheel=float(ang[i]),cmd=float(cc[i]),lat_active=bool(lat[i]),pressed=bool(pr[i]),
                 blinker=bool(blink[i]),bs_l=bool(bsl[i]),bs_r=bool(bsr[i]),standstill=bool(stand[i]),wheel_rate=float(rate[i]))
        if not (lat[i] and not paused[i] and v[i]>=6 and not pr[i]): continue
        s=sim.s; gain=float(getattr(s,"aci_gain_last",0.0)); arm=int(getattr(s,"reanchor_arm",0)); dtq=max(0.0,abs(tq[i])-float(getattr(s,"hold_comp_last",0.0)))
        tot["n"]+=1
        if getattr(s,"driver_pressed",False): tot["pressed"]+=1
        if dtq>=100: tot["dtq_ge100"]+=1
        if arm>=30: tot["arm30"]+=1; tot["arm30_gain"].append(gain)
        else: tot["noarm_gain"].append(gain)
        if abs(cc[i]-ang[i])>=3.0:
            tot["div3"]+=1
            if gain<0.5:
                tot["div3_low"]+=1
                if arm>=30: tot["div3_low_arm"]+=1
n=max(tot["n"],1)
print(f"hands-off(EPS) eff-active frames v>=6: {n} ({n/100/60:.1f} min)")
print(f"  driver_pressed(machine) share {100*tot['pressed']/n:.2f}% | driver_tq>=100 share {100*tot['dtq_ge100']/n:.1f}% | arm>=30 (post_grip) share {100*tot['arm30']/n:.1f}%")
print(f"  mean gain: arm>=30 {np.mean(tot['arm30_gain']) if tot['arm30_gain'] else 0:.3f} vs no-arm {np.mean(tot['noarm_gain']) if tot['noarm_gain'] else 0:.3f}")
print(f"  plan-wheel div>=3deg frames {tot['div3']} ({100*tot['div3']/n:.1f}%): gain<0.5 in {100*tot['div3_low']/max(tot['div3'],1):.1f}% of them, of which arm>=30 {100*tot['div3_low_arm']/max(tot['div3_low'],1):.0f}%")
