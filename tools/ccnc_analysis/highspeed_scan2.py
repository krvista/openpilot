#!/usr/bin/env python3
"""Refinement of highspeed_scan: 100 ms-smoothed wheel rate (steeringRateDeg is 4 deg/s-quantized),
plan (cc, pre-VM) per-frame step vs the VM jerk cap, and post-release peaks split by cause."""
import glob, os, sys, numpy as np
REPO=os.environ.get("REPO","/home/user/openpilot"); sys.path.insert(0,REPO); sys.path.insert(0,os.path.join(REPO,"opendbc_repo"))
from opendbc.car.hyundai.values import CarControllerParams as P
from phase_tests.harness import Sim, make_cp
from opendbc.car.vehicle_model import VehicleModel
from opendbc.car.lateral import get_max_angle_delta_vm
CP=make_cp(); VM=VehicleModel(CP); SRWB=CP.steerRatio*CP.wheelbase
class L: ANGLE_LIMITS=P.ANGLE_LIMITS; STEER_STEP=1
D=os.path.join(os.path.dirname(os.path.abspath(__file__)),"npz2")
BINS=[(16.7,22.2,"60-80"),(22.2,27.8,"80-100"),(27.8,99,"100+")]
acc={b[2]:dict(n=0,wr=[],jerk=[],ccstep=[],cc_over=0,cc_bound_dir=0) for b in BINS}
rel=[]
def sm_rate(ang):  # deg/s from a centered 100 ms window
    k=np.ones(11)/11.0; a=np.convolve(ang,k,mode='same'); return np.gradient(a)*100.0
for f in sorted(glob.glob(os.path.join(D,"*.npz"))):
    if sys.argv[1:] and f.split("--")[0].split("_")[-1] not in set(sys.argv[1:]): continue
    z=np.load(f,allow_pickle=True); t=z["cs_t"]
    if len(t)<500 or len(z["cc_t"])<2 or len(z["sp_t"])<2: continue
    v=np.nan_to_num(z["cs_v"]); ang=np.nan_to_num(z["cs_ang"]); rate=np.nan_to_num(z["cs_rate"]); tq=np.nan_to_num(z["cs_tq"])
    pr=z["cs_pr"]>0.5; blink=z["cs_blink"]>0.5; bsl=z["cs_bsl"]>0.5; bsr=z["cs_bsr"]>0.5; stand=z["cs_stand"]>0.5
    lat=np.interp(t,z["cc_t"],z["cc_lat"])>0.5; paused=np.interp(t,z["sp_t"],z["sp_paused"])>0.5; cc=np.interp(t,z["cc_t"],z["cc_ang"])
    if not (lat&~paused&(v>=16.7)).any(): continue
    sim=Sim(); gain=np.zeros(len(t))
    for i in range(len(t)):
        sim.step(v=float(v[i]),tq=float(tq[i]),wheel=float(ang[i]),cmd=float(cc[i]),lat_active=bool(lat[i]),pressed=bool(pr[i]),blinker=bool(blink[i]),bs_l=bool(bsl[i]),bs_r=bool(bsr[i]),standstill=bool(stand[i]),wheel_rate=float(rate[i]))
        gain[i]=float(getattr(sim.s,"aci_gain_last",0.0))
    eff=lat&~paused; ho=eff&~pr; wr=sm_rate(ang); jk=v*v*np.radians(wr)/SRWB
    dcc=np.abs(np.diff(cc,prepend=cc[0])); cap=np.array([min(get_max_angle_delta_vm(float(max(x,1.0)),VM,L),5.0) for x in v])
    for lo,hi_,name in BINS:
        m=ho&(v>=lo)&(v<hi_); a=acc[name]; a["n"]+=int(m.sum())
        if m.sum():
            a["wr"]+=list(np.abs(wr[m])); a["jerk"]+=list(np.abs(jk[m])); a["ccstep"]+=list(dcc[m]/cap[m]); a["cc_over"]+=int((dcc[m]>cap[m]).sum())
    off=np.flatnonzero(np.diff(pr.astype(np.int8))==-1)+1
    for i in off:
        if v[i]<16.7 or not eff[i] or i+200>=len(t) or blink[i:i+200].any(): continue
        w=slice(i,i+200); j=np.abs(jk[w]); k=int(j.argmax())
        rel.append(dict(v=v[i]*3.6,div=abs(cc[i]-ang[i]),maxwr=np.abs(wr[w]).max(),maxjerk=j.max(),t_peak=k/100,g0=gain[i],gpk=gain[i+k],ccstep_pk=float(dcc[i+k]/cap[i+k]),regrip=bool(pr[i:i+200].any())))
def pct(x,q): return np.percentile(x,q) if len(x) else float('nan')
print("== hands-off eff-active by speed (100 ms-smoothed wheel rate)")
for _,_,name in BINS:
    a=acc[name]
    if not a["n"]: continue
    print(f"  {name} km/h n={a['n']}: |wheel rate| p50/p90/p99 {pct(a['wr'],50):.1f}/{pct(a['wr'],90):.1f}/{pct(a['wr'],99):.1f} deg/s | lat jerk p90/p99 {pct(a['jerk'],90):.2f}/{pct(a['jerk'],99):.2f} m/s3 | share jerk>1.0 {100*np.mean(np.array(a['jerk'])>1.0):.1f}% >2.0 {100*np.mean(np.array(a['jerk'])>2.0):.1f}% | plan step/VMcap p90/p99 {pct(a['ccstep'],90):.2f}/{pct(a['ccstep'],99):.2f}, plan>cap {100*a['cc_over']/a['n']:.2f}%")
print(f"== post-release (>=60 km/h, no blinker in window) n={len(rel)}")
if rel:
    mj=np.array([r['maxjerk'] for r in rel]); mr=np.array([r['maxwr'] for r in rel]); dv=np.array([r['div'] for r in rel]); tp=np.array([r['t_peak'] for r in rel])
    print(f"  2 s: max|wheel rate| p50/p90 {np.median(mr):.1f}/{np.percentile(mr,90):.1f} deg/s | max jerk p50/p90/max {np.median(mj):.2f}/{np.percentile(mj,90):.2f}/{mj.max():.2f} m/s3 | >=1.0: {(mj>=1.0).sum()} >=2.0: {(mj>=2.0).sum()} >=3.0: {(mj>=3.0).sum()} | div@release p50/p90 {np.median(dv):.2f}/{np.percentile(dv,90):.2f} | t_peak p50 {np.median(tp):.2f} s")
    big=[r for r in rel if r['maxjerk']>=2.0]
    print(f"  jerk>=2.0 events: regrip within 2 s {sum(r['regrip'] for r in big)}/{len(big)} | gain at peak p50 {np.median([r['gpk'] for r in big]):.2f} | plan step/cap at peak p50 {np.median([r['ccstep_pk'] for r in big]):.2f} (>=0.9: {sum(r['ccstep_pk']>=0.9 for r in big)})")
    for r in sorted(rel,key=lambda r:-r['maxjerk'])[:10]:
        print(f"    v {r['v']:.0f} div {r['div']:.1f} -> wr {r['maxwr']:.0f} deg/s jerk {r['maxjerk']:.2f} at +{r['t_peak']:.2f} s gain {r['g0']:.2f}->{r['gpk']:.2f} plan/cap {r['ccstep_pk']:.2f} regrip {r['regrip']}")
