#!/usr/bin/env python3
"""Grip-onset gain drop latency, gain during grip, and post-release recovery (replayed CarController), v>=12 m/s."""
import glob, os, sys, numpy as np
REPO=os.environ.get("REPO","/home/user/openpilot"); sys.path.insert(0,REPO); sys.path.insert(0,os.path.join(REPO,"opendbc_repo"))
from phase_tests.harness import Sim
D=os.path.join(os.path.dirname(os.path.abspath(__file__)),"npz2")
drop=[]; grip_gain=[]; rec=[]; rec_div=[]; rel_deliv=[]
for f in sorted(glob.glob(os.path.join(D,"*.npz"))):
    if f.split("--")[0].split("_")[-1] not in set(sys.argv[1:]): continue
    z=np.load(f,allow_pickle=True); t=z["cs_t"]
    if len(t)<500 or len(z["cc_t"])<2 or len(z["sp_t"])<2: continue
    v=np.nan_to_num(z["cs_v"]); ang=np.nan_to_num(z["cs_ang"]); rate=np.nan_to_num(z["cs_rate"]); tq=np.nan_to_num(z["cs_tq"])
    pr=z["cs_pr"]>0.5; blink=z["cs_blink"]>0.5; bsl=z["cs_bsl"]>0.5; bsr=z["cs_bsr"]>0.5; stand=z["cs_stand"]>0.5
    lat=np.interp(t,z["cc_t"],z["cc_lat"])>0.5; paused=np.interp(t,z["sp_t"],z["sp_paused"])>0.5; cc=np.interp(t,z["cc_t"],z["cc_ang"])
    if not (lat&~paused&(v>=12)&pr).any(): continue
    sim=Sim(); gain=np.zeros(len(t))
    for i in range(len(t)):
        sim.step(v=float(v[i]),tq=float(tq[i]),wheel=float(ang[i]),cmd=float(cc[i]),lat_active=bool(lat[i]),pressed=bool(pr[i]),blinker=bool(blink[i]),bs_l=bool(bsl[i]),bs_r=bool(bsr[i]),standstill=bool(stand[i]),wheel_rate=float(rate[i]))
        gain[i]=float(getattr(sim.s,"aci_gain_last",0.0))
    eff=lat&~paused; wr=np.abs(np.gradient(np.convolve(ang,np.ones(5)/5,"same")))*100
    on=np.flatnonzero(np.diff(pr.astype(np.int8))==1)+1; off=np.flatnonzero(np.diff(pr.astype(np.int8))==-1)+1
    for i in on:
        if v[i]<12 or not eff[i] or i+150>=len(t): continue
        k=np.flatnonzero(gain[i:i+150]<0.2); drop.append(k[0]/100 if len(k) else 1.5)
        j=i
        while j<len(t) and pr[j]: j+=1
        if j-i>=30: grip_gain.append(gain[i+20:j].mean())
    for i in off:
        if v[i]<12 or not eff[i] or i+300>=len(t) or pr[i:i+300].any(): continue
        k=np.flatnonzero(gain[i:i+300]>=0.6); rec.append(k[0]/100 if len(k) else 3.0)
        rec_div.append(np.abs(cc[i:i+150]-ang[i:i+150]).max()); rel_deliv.append(wr[i:i+150].max())
print(f"grip onsets (v>=12): {len(drop)} | gain<0.2 latency p50 {np.median(drop):.2f}s p90 {np.percentile(drop,90):.2f}s | mean gain during grip {np.mean(grip_gain):.3f} (n={len(grip_gain)})")
print(f"clean releases: {len(rec)} | time to gain>=0.6 p50 {np.median(rec):.2f}s p90 {np.percentile(rec,90):.2f}s | max plan-wheel div in 1.5s p50 {np.median(rec_div):.1f} p90 {np.percentile(rec_div,90):.1f} deg | wheel delivery rate p50 {np.median(rel_deliv):.0f} p90 {np.percentile(rel_deliv,90):.0f} deg/s")
