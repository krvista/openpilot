#!/usr/bin/env python3
"""Pre-quantification for a HIGH-SPEED comfort jerk cap on the commanded angle (candidate lever for
'slower, longer error correction at speed'). For each candidate cap J (m/s^3) the per-frame angle step
cap is J/v^2 * SR*wb (rad->deg, per 10 ms). Counts how often the cap would bind on (a) OP-DRIVEN frames
(hands-off, driver_tq<30, no pressed within +/-2 s: planned driving) vs (b) RECOVERY frames (hands-off
within 3 s after a pressed release). Uses the logged apply stream (carOutput) — contaminated by the
wheel only during anchors, which (a)/(b) exclude by construction (b: post-release, no re-press)."""
import glob, os, sys, numpy as np
REPO=os.environ.get("REPO","/home/user/openpilot"); sys.path.insert(0,REPO); sys.path.insert(0,os.path.join(REPO,"opendbc_repo"))
from phase_tests.harness import Sim, make_cp
CP=make_cp(); SRWB=CP.steerRatio*CP.wheelbase
D=os.path.join(os.path.dirname(os.path.abspath(__file__)),os.environ.get("NPZ","npz2"))
J=[3.59,2.5,2.0,1.5,1.0]; BINS=[(16.7,22.2,"60-80"),(22.2,27.8,"80-100"),(27.8,99,"100+")]
res={b[2]:{j:dict(op=0,rec=0) for j in J} for b in BINS}; nn={b[2]:dict(op=0,rec=0) for b in BINS}
for f in sorted(glob.glob(os.path.join(D,"*.npz"))):
    if sys.argv[1:] and f.split("--")[0].split("_")[-1] not in set(sys.argv[1:]): continue
    z=np.load(f,allow_pickle=True); t=z["cs_t"]
    if len(t)<500 or len(z["cc_t"])<2 or len(z["sp_t"])<2 or len(z["co_t"])<2: continue
    v=np.nan_to_num(z["cs_v"]); ang=np.nan_to_num(z["cs_ang"]); rate=np.nan_to_num(z["cs_rate"]); tq=np.nan_to_num(z["cs_tq"])
    pr=z["cs_pr"]>0.5; blink=z["cs_blink"]>0.5; bsl=z["cs_bsl"]>0.5; bsr=z["cs_bsr"]>0.5; stand=z["cs_stand"]>0.5
    lat=np.interp(t,z["cc_t"],z["cc_lat"])>0.5; paused=np.interp(t,z["sp_t"],z["sp_paused"])>0.5; cc=np.interp(t,z["cc_t"],z["cc_ang"]); co=np.interp(t,z["co_t"],z["co_ang"])
    if not (lat&~paused&(v>=16.7)).any(): continue
    sim=Sim(); dtq=np.zeros(len(t))
    for i in range(len(t)):
        sim.step(v=float(v[i]),tq=float(tq[i]),wheel=float(ang[i]),cmd=float(cc[i]),lat_active=bool(lat[i]),pressed=bool(pr[i]),blinker=bool(blink[i]),bs_l=bool(bsl[i]),bs_r=bool(bsr[i]),standstill=bool(stand[i]),wheel_rate=float(rate[i]))
        dtq[i]=max(0.0,abs(tq[i])-float(getattr(sim.s,"hold_comp_last",0.0)))
    eff=lat&~paused; ho=eff&~pr
    near=np.convolve(pr.astype(float),np.ones(401),mode='same')>0
    opd=ho&(dtq<30)&~near&~blink
    rec=np.zeros(len(t),bool)
    for i in np.flatnonzero(np.diff(pr.astype(np.int8))==-1)+1:
        w=slice(i,min(i+300,len(t)))
        if not pr[w].any() and not blink[w].any(): rec[w]=True
    rec&=ho
    dco=np.abs(np.diff(co,prepend=co[0]))
    for lo,hi,name in BINS:
        sp=(v>=lo)&(v<hi)
        for key,m in (("op",opd&sp),("rec",rec&sp)):
            nn[name][key]+=int(m.sum())
            for j in J:
                cap=np.degrees(j/np.maximum(v[m],1.0)**2*SRWB)*0.01
                res[name][j][key]+=int((dco[m]>cap).sum())
for _,_,name in BINS:
    print(f"{name} km/h: op-driven n={nn[name]['op']} recovery n={nn[name]['rec']}")
    for j in J:
        print(f"   cap {j:.2f} m/s3: binds on op-driven {100*res[name][j]['op']/max(nn[name]['op'],1):.2f}% | recovery {100*res[name][j]['rec']/max(nn[name]['rec'],1):.2f}%")
