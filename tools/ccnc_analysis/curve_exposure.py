#!/usr/bin/env python3
"""Phase 36 replay: hands-off eff-active frames 10-40 km/h — gain with/without the curve ceiling (kill), ramp smoothness."""
import glob, os, sys, numpy as np
REPO=os.environ.get("REPO","/home/user/openpilot"); sys.path.insert(0,REPO); sys.path.insert(0,os.path.join(REPO,"opendbc_repo"))
from opendbc.car.hyundai.values import CarControllerParams as P
if os.environ.get("KILL36"): P.ACIGAIN_CURVE_CEILING_V=[0.18,0.30,0.75,0.95]
from phase_tests.harness import Sim
D=os.path.join(os.path.dirname(os.path.abspath(__file__)),"npz2")
res={"10-20":[], "20-30":[], "30-40":[]}; dmax=[]; curve_g={"10-20":[], "20-30":[], "30-40":[]}
for f in sorted(glob.glob(os.path.join(D,"*.npz"))):
    if f.split("--")[0].split("_")[-1] not in set(sys.argv[1:]): continue
    z=np.load(f,allow_pickle=True); t=z["cs_t"]
    if len(t)<500 or len(z["cc_t"])<2 or len(z["sp_t"])<2: continue
    v=np.nan_to_num(z["cs_v"]); vk=v*3.6; ang=np.nan_to_num(z["cs_ang"]); rate=np.nan_to_num(z["cs_rate"]); tq=np.nan_to_num(z["cs_tq"])
    pr=z["cs_pr"]>0.5; blink=z["cs_blink"]>0.5; bsl=z["cs_bsl"]>0.5; bsr=z["cs_bsr"]>0.5; stand=z["cs_stand"]>0.5
    lat=np.interp(t,z["cc_t"],z["cc_lat"])>0.5; paused=np.interp(t,z["sp_t"],z["sp_paused"])>0.5; cc=np.interp(t,z["cc_t"],z["cc_ang"])
    m0=lat&~paused&~pr&(vk>=10)&(vk<40)
    if not m0.any(): continue
    sim=Sim(); gain=np.zeros(len(t)); eff=np.zeros(len(t),bool)
    for i in range(len(t)):
        sim.step(v=float(v[i]),tq=float(tq[i]),wheel=float(ang[i]),cmd=float(cc[i]),lat_active=bool(lat[i]),pressed=bool(pr[i]),blinker=bool(blink[i]),bs_l=bool(bsl[i]),bs_r=bool(bsr[i]),standstill=bool(stand[i]),wheel_rate=float(rate[i]),lead_dist=(6.0 if (v[i]<5.6 and lat[i] and not paused[i]) else None))
        gain[i]=float(getattr(sim.s,"aci_gain_last",0.0)); eff[i]=sim.effective_lat_active()
    m=m0&eff
    d=np.abs(np.diff(gain)); dmax.append(d[m[1:]].max() if m[1:].any() else 0)
    for k,(lo,hi) in (("10-20",(10,20)),("20-30",(20,30)),("30-40",(30,40))):
        mm=m&(vk>=lo)&(vk<hi)
        if mm.any():
            res[k]+=list(gain[mm]); curve_g[k]+=list(gain[mm&(np.abs(cc)>=8)])
print(f"KILL36={bool(os.environ.get('KILL36'))}")
for k in res:
    print(f"  {k} kph: hands-off eff frames {len(res[k])} mean gain {np.mean(res[k]) if res[k] else 0:.3f} | curve(|plan|>=8) frames {len(curve_g[k])} mean gain {np.mean(curve_g[k]) if curve_g[k] else 0:.3f}")
print(f"  max |dgain|/frame over segments: p50 {np.median(dmax):.3f} max {max(dmax):.3f}")
