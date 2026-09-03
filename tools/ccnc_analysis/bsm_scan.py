#!/usr/bin/env python3
"""BSD prep scan (needs distill v4 npz: NPZ env, default npz4). For every blindspot-active episode at
>= 40 km/h: what happened toward that side — same-side blinker, model lane change state, lane-line
approach (|y| of the line on that side), op LDW (driverAssistance), and whether the wheel was driver-
or op-driven. Answers: how often is a lane change toward an occupied side attempted, by whom, and
which existing gate (ALC preLaneChange / blinker anchor / LDW curvature block / Phase 33 softening) saw it."""
import glob, os, sys, numpy as np
D=os.path.join(os.path.dirname(os.path.abspath(__file__)),os.environ.get("NPZ","npz4"))
HOLD=100  # frames of BSM hold (matches BLIND_HOLD_FRAMES)
ep=[]; tot_hi=0; tot_bsm=0
for f in sorted(glob.glob(os.path.join(D,"*.npz"))):
    if sys.argv[1:] and f.split("--")[0].split("_")[-1] not in set(sys.argv[1:]): continue
    z=np.load(f,allow_pickle=True); t=z["cs_t"]
    if len(t)<500 or "cs_bl" not in z.files or len(z["cc_t"])<2: continue
    v=np.nan_to_num(z["cs_v"]); ang=np.nan_to_num(z["cs_ang"]); pr=z["cs_pr"]>0.5
    bl=z["cs_bl"]>0.5; br=z["cs_br"]>0.5; bsl=z["cs_bsl"]>0.5; bsr=z["cs_bsr"]>0.5
    lat=np.interp(t,z["cc_t"],z["cc_lat"])>0.5; cc=np.interp(t,z["cc_t"],z["cc_ang"])
    paused=np.interp(t,z["sp_t"],z["sp_paused"])>0.5 if len(z["sp_t"])>1 else np.zeros(len(t),bool)
    ly1=np.interp(t,z["mv_t"],np.nan_to_num(z["mv_ly1"],nan=-9)) if len(z["mv_t"])>1 else np.full(len(t),-9.0)
    ly2=np.interp(t,z["mv_t"],np.nan_to_num(z["mv_ly2"],nan=9)) if len(z["mv_t"])>1 else np.full(len(t),9.0)
    lcs=z["mv_lcs"]; mvt=z["mv_t"]
    dal=np.interp(t,z["da_t"],z["da_l"])>0.5 if len(z["da_t"])>1 else np.zeros(len(t),bool)
    dar=np.interp(t,z["da_t"],z["da_r"])>0.5 if len(z["da_t"])>1 else np.zeros(len(t),bool)
    hi=v>=11.1; tot_hi+=int(hi.sum()); tot_bsm+=int((hi&(bsl|bsr)).sum())
    for side,bs,blk,ly,da,sgn in (("L",bsl,bl,ly1,dal,+1),("R",bsr,br,ly2,dar,-1)):
        # episodes with hold
        act=np.convolve(bs.astype(float),np.ones(HOLD),mode='full')[:len(t)]>0
        act&=hi
        d=np.diff(act.astype(np.int8),prepend=0); starts=np.flatnonzero(d==1); ends=np.flatnonzero(d==-1)
        for s0 in starts:
            e0=ends[ends>s0]; e0=int(e0[0]) if len(e0) else len(t)
            w=slice(s0,e0); n=e0-s0
            if n<10: continue
            same_blk=bool(blk[w].any())
            # lane line on that side approached within 0.9 m (ego edge ~0.9 m at lane width 3.5)
            approach=bool((np.abs(ly[w])<0.9).any()); minly=float(np.abs(ly[w]).min())
            k=(mvt>=t[s0])&(mvt<=t[min(e0,len(t)-1)]); lc=set(str(x) for x in lcs[k]) if k.any() else set()
            lc_started=any(x in ("laneChangeStarting","laneChangeFinishing") for x in lc)
            # wheel moved toward the side: angle change sign (left positive)
            dwheel=float((ang[min(e0,len(t)-1)]-ang[s0])*sgn)
            toward=bool(((ang[w]-ang[s0])*sgn>3.0).any())
            op_toward=bool((((cc[w]-ang[w])*sgn>2.0)&lat[w]&~paused[w]&~pr[w]).any())
            ep.append(dict(side=side,sec=n/100,v=float(v[w].mean()*3.6),blk=same_blk,approach=approach,minly=minly,lc=lc_started,
                           toward=toward,op_toward=op_toward,pressed=bool(pr[w].any()),ldw=bool(da[w].any()),eff=bool((lat[w]&~paused[w]).any())))
print(f"frames >=40 km/h {tot_hi}, BSM active {100*tot_bsm/max(tot_hi,1):.2f}% | episodes (with 1 s hold) {len(ep)}")
if ep:
    def c(k): return sum(1 for e in ep if e[k])
    print(f"  same-side blinker {c('blk')} | lane-line approach <0.9 m {c('approach')} | model lane change started {c('lc')} | wheel moved toward side >3 deg {c('toward')} | op command toward side >2 deg (hands-off eff) {c('op_toward')} | op LDW {c('ldw')} | pressed {c('pressed')}")
    for e in sorted(ep,key=lambda e:(not e['approach'],-e['sec']))[:15]:
        print(f"    {e['side']} {e['sec']:.1f}s v {e['v']:.0f} km/h blk {int(e['blk'])} approach {int(e['approach'])} (min |y| {e['minly']:.2f}) lc {int(e['lc'])} toward {int(e['toward'])} op_toward {int(e['op_toward'])} ldw {int(e['ldw'])} pressed {int(e['pressed'])} eff {int(e['eff'])}")
