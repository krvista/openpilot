#!/usr/bin/env python3
"""Verifier metrics at >=60 km/h for the 35a descent gate. GATE_NM env: 100 / 160 / 1e9 (pressed-only)."""
import glob, os, sys, numpy as np
REPO=os.environ.get("REPO","/home/user/openpilot"); sys.path.insert(0,REPO); sys.path.insert(0,os.path.join(REPO,"opendbc_repo"))
from opendbc.car.hyundai.values import CarControllerParams as P
P.ACIGAIN_GRIP_RATE_DN_GATE_NM=float(os.environ.get("GATE_NM",P.ACIGAIN_GRIP_RATE_DN_GATE_NM))
if os.environ.get("KILL35A"):
    P.ACIGAIN_GRIP_RATE_DN_FLOOR_V=[0.0,0.0]; P.ACIGAIN_GRIP_FLOOR35_V=[0.08,0.08,0.08,0.05]; P.ACIGAIN_GRIP_FULL35_V=[110.0,102.5,80.0]
    P.ANCHORED_RECOVERY_FRAMES=0
from phase_tests.harness import Sim
D=os.path.join(os.path.dirname(os.path.abspath(__file__)),"npz2")
ho_g=[]; ho_div_low=0; ho_n=0; rest_g=[]; grip_g=[]; onset=[]
for f in sorted(glob.glob(os.path.join(D,"*.npz"))):
    if f.split("--")[0].split("_")[-1] not in set(sys.argv[1:]): continue
    z=np.load(f,allow_pickle=True); t=z["cs_t"]
    if len(t)<500 or len(z["cc_t"])<2 or len(z["sp_t"])<2: continue
    v=np.nan_to_num(z["cs_v"]); ang=np.nan_to_num(z["cs_ang"]); rate=np.nan_to_num(z["cs_rate"]); tq=np.nan_to_num(z["cs_tq"])
    pr=z["cs_pr"]>0.5; blink=z["cs_blink"]>0.5; bsl=z["cs_bsl"]>0.5; bsr=z["cs_bsr"]>0.5; stand=z["cs_stand"]>0.5
    lat=np.interp(t,z["cc_t"],z["cc_lat"])>0.5; paused=np.interp(t,z["sp_t"],z["sp_paused"])>0.5; cc=np.interp(t,z["cc_t"],z["cc_ang"])
    if not (lat&~paused&(v>=16.7)).any(): continue
    sim=Sim(); gain=np.zeros(len(t)); dtq=np.zeros(len(t))
    for i in range(len(t)):
        sim.step(v=float(v[i]),tq=float(tq[i]),wheel=float(ang[i]),cmd=float(cc[i]),lat_active=bool(lat[i]),pressed=bool(pr[i]),blinker=bool(blink[i]),bs_l=bool(bsl[i]),bs_r=bool(bsr[i]),standstill=bool(stand[i]),wheel_rate=float(rate[i]))
        gain[i]=float(getattr(sim.s,"aci_gain_last",0.0)); dtq[i]=max(0.0,abs(tq[i])-float(getattr(sim.s,"hold_comp_last",0.0)))
    eff=lat&~paused&(v>=16.7); ho=eff&~pr
    ho_g+=list(gain[ho]); ho_n+=int(ho.sum()); ho_div_low+=int((ho&(np.abs(cc-ang)>2.0)&(gain<0.5)).sum())
    rest_g+=list(gain[ho&(dtq>=100)&(dtq<150)]); grip_g+=list(gain[eff&pr])
    on=np.flatnonzero(np.diff(pr.astype(np.int8))==1)+1
    for i in on:
        if v[i]<16.7 or not eff[i] or i+150>=len(t): continue
        k=np.flatnonzero(gain[i:i+150]<0.2); onset.append(k[0]/100 if len(k) else 1.5)
print(f"GATE_NM={P.ACIGAIN_GRIP_RATE_DN_GATE_NM:.0f} | HO mean g {np.mean(ho_g):.3f} | HO div>2&g<0.5 {100*ho_div_low/max(ho_n,1):.1f}% | rest-band(100-150) g {np.mean(rest_g) if rest_g else 0:.3f} | GRIP mean g {np.mean(grip_g) if grip_g else 0:.3f} | onset p50/p90 {np.median(onset):.2f}/{np.percentile(onset,90):.2f} s (n={len(onset)})")
