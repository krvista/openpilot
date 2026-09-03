#!/usr/bin/env python3
"""High-speed baseline scan (prep for the next request): BSM presence, wheel-rate / implied lateral
jerk at >=60 km/h, post-release correction sharpness, and the resting-hand yield band. Replays the
current CarController for gain/driver_tq (hold comp)."""
import glob, os, sys, math, numpy as np
REPO=os.environ.get("REPO","/home/user/openpilot"); sys.path.insert(0,REPO); sys.path.insert(0,os.path.join(REPO,"opendbc_repo"))
from opendbc.car.hyundai.values import CarControllerParams as P
from phase_tests.harness import Sim, make_cp
from opendbc.car.vehicle_model import VehicleModel
from opendbc.car.lateral import get_max_angle_delta_vm
CP=make_cp(); VM=VehicleModel(CP); SRWB=CP.steerRatio*CP.wheelbase
class L: ANGLE_LIMITS=P.ANGLE_LIMITS; STEER_STEP=1
D=os.path.join(os.path.dirname(os.path.abspath(__file__)),"npz2")
BINS=[(16.7,22.2,"60-80"),(22.2,27.8,"80-100"),(27.8,99,"100+")]
acc={b[2]:dict(n=0,rate=[],jerk=[],cap=0,dtq=[],gain=[],div=[]) for b in BINS}
bsm=dict(routes=0,routes_any=0,frames_hi=0,bsl_hi=0,bsr_hi=0,blink_hi=0,blink_bsm=0)
rel=[]
def jerk(v,rate_dps): return v*v*np.radians(rate_dps)/SRWB
for f in sorted(glob.glob(os.path.join(D,"*.npz"))):
    if sys.argv[1:] and f.split("--")[0].split("_")[-1] not in set(sys.argv[1:]): continue
    z=np.load(f,allow_pickle=True); t=z["cs_t"]
    if len(t)<500 or len(z["cc_t"])<2 or len(z["sp_t"])<2: continue
    v=np.nan_to_num(z["cs_v"]); ang=np.nan_to_num(z["cs_ang"]); rate=np.nan_to_num(z["cs_rate"]); tq=np.nan_to_num(z["cs_tq"])
    pr=z["cs_pr"]>0.5; blink=z["cs_blink"]>0.5; bsl=z["cs_bsl"]>0.5; bsr=z["cs_bsr"]>0.5; stand=z["cs_stand"]>0.5
    lat=np.interp(t,z["cc_t"],z["cc_lat"])>0.5; paused=np.interp(t,z["sp_t"],z["sp_paused"])>0.5; cc=np.interp(t,z["cc_t"],z["cc_ang"])
    co=np.interp(t,z["co_t"],z["co_ang"]) if len(z["co_t"])>1 else cc
    hi=v>=16.7
    bsm["routes"]+=1; bsm["routes_any"]+=int((bsl|bsr).any()); bsm["frames_hi"]+=int(hi.sum()); bsm["bsl_hi"]+=int((hi&bsl).sum()); bsm["bsr_hi"]+=int((hi&bsr).sum())
    bsm["blink_hi"]+=int((hi&blink).sum()); bsm["blink_bsm"]+=int((hi&blink&(bsl|bsr)).sum())
    if not (lat&~paused&hi).any(): continue
    sim=Sim(); gain=np.zeros(len(t)); dtq=np.zeros(len(t))
    for i in range(len(t)):
        sim.step(v=float(v[i]),tq=float(tq[i]),wheel=float(ang[i]),cmd=float(cc[i]),lat_active=bool(lat[i]),pressed=bool(pr[i]),blinker=bool(blink[i]),bs_l=bool(bsl[i]),bs_r=bool(bsr[i]),standstill=bool(stand[i]),wheel_rate=float(rate[i]))
        gain[i]=float(getattr(sim.s,"aci_gain_last",0.0)); dtq[i]=max(0.0,abs(tq[i])-float(getattr(sim.s,"hold_comp_last",0.0)))
    eff=lat&~paused; ho=eff&~pr
    dco=np.abs(np.diff(co,prepend=co[0]))
    for lo,hi_,name in BINS:
        m=ho&(v>=lo)&(v<hi_)
        a=acc[name]; a["n"]+=int(m.sum())
        if m.sum():
            a["rate"]+=list(np.abs(rate[m])); a["jerk"]+=list(np.abs(jerk(v[m],rate[m]))); a["dtq"]+=list(dtq[m]); a["gain"]+=list(gain[m]); a["div"]+=list(np.abs(cc[m]-ang[m]))
            cap=np.array([min(get_max_angle_delta_vm(float(x),VM,L),5.0) for x in v[m]]); a["cap"]+=int((dco[m]>=0.9*cap).sum())
    off=np.flatnonzero(np.diff(pr.astype(np.int8))==-1)+1
    for i in off:
        if v[i]<16.7 or not eff[i] or i+200>=len(t) or blink[i]: continue
        w=slice(i,i+200); j=np.abs(jerk(v[w],rate[w]))
        rel.append(dict(v=v[i]*3.6,div=abs(cc[i]-ang[i]),maxrate=np.abs(rate[w]).max(),maxjerk=j.max(),t_peak=int(j.argmax())/100,gmax=gain[w].max(),g0=gain[i]))
def pct(x,q): return np.percentile(x,q) if len(x) else float('nan')
print("== BSM presence"); print(f"  routes {bsm['routes']} with any BSM flag {bsm['routes_any']} | >=60 km/h frames {bsm['frames_hi']}: left {100*bsm['bsl_hi']/max(bsm['frames_hi'],1):.2f}% right {100*bsm['bsr_hi']/max(bsm['frames_hi'],1):.2f}% | blinker frames {bsm['blink_hi']} of which BSM active {bsm['blink_bsm']} ({100*bsm['blink_bsm']/max(bsm['blink_hi'],1):.1f}%)")
print("== hands-off eff-active by speed")
for _,_,name in BINS:
    a=acc[name]
    if not a["n"]: print(f"  {name}: none"); continue
    d=np.array(a["dtq"]); g=np.array(a["gain"])
    print(f"  {name} km/h n={a['n']}: |wheel rate| p50/p90/p99 {pct(a['rate'],50):.1f}/{pct(a['rate'],90):.1f}/{pct(a['rate'],99):.1f} deg/s | implied lat jerk p90/p99/max {pct(a['jerk'],90):.2f}/{pct(a['jerk'],99):.2f}/{max(a['jerk']):.2f} m/s3 | VM-cap-bound cmd frames {100*a['cap']/a['n']:.2f}%")
    print(f"      driver_tq p25/50/75/90 {pct(d,25):.0f}/{pct(d,50):.0f}/{pct(d,75):.0f}/{pct(d,90):.0f} Nm | band <30: {100*(d<30).mean():.0f}% g {g[d<30].mean() if (d<30).any() else 0:.2f} | 30-80: {100*((d>=30)&(d<80)).mean():.0f}% g {g[(d>=30)&(d<80)].mean() if ((d>=30)&(d<80)).any() else 0:.2f} | >=80: {100*(d>=80).mean():.0f}% g {g[d>=80].mean() if (d>=80).any() else 0:.2f} | |cc-wheel| p50/p90 {pct(a['div'],50):.2f}/{pct(a['div'],90):.2f} deg")
print(f"== post-release (pressed->off, >=60 km/h, no blinker) n={len(rel)}")
if rel:
    mj=np.array([r['maxjerk'] for r in rel]); mr=np.array([r['maxrate'] for r in rel]); dv=np.array([r['div'] for r in rel])
    print(f"  2 s window: max|wheel rate| p50/p90 {np.median(mr):.1f}/{np.percentile(mr,90):.1f} deg/s | max implied jerk p50/p90/max {np.median(mj):.2f}/{np.percentile(mj,90):.2f}/{mj.max():.2f} m/s3 | >=1.0: {(mj>=1.0).sum()} >=2.0: {(mj>=2.0).sum()} | divergence at release p50/p90 {np.median(dv):.2f}/{np.percentile(dv,90):.2f} deg")
    for r in sorted(rel,key=lambda r:-r['maxjerk'])[:8]:
        print(f"    v {r['v']:.0f} km/h div {r['div']:.1f} deg -> max rate {r['maxrate']:.0f} deg/s jerk {r['maxjerk']:.2f} at +{r['t_peak']:.2f} s, gain {r['g0']:.2f}->{r['gmax']:.2f}")
