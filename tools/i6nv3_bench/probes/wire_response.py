"""Wheel response vs (wire request rate, wire error). Hands-off (<50 Nm, >3 s since press), sent gain>=0.9, latActive, 30-60 km/h.
wire = ADAS_StrAnglReqVal decoded from LKAS_ALT (0x110). rate = d(wire)/dt over 0.2 s; err = wire - wheel; wheel rate = d(wheel)/dt over 0.2 s,
both signed toward the request (positive = moving toward / request moving away from the wheel)."""
import glob, sys, numpy as np
from openpilot.tools.lib.logreader import LogReader
r=sys.argv[1]; files=sorted(glob.glob(f"/home/user/drivelog/drivelog/*_{r}--*--rlog.zst"), key=lambda f:int(f.split("--")[-2]))[1:]
v=0; press=0; tq=0; blk=0; lat=0; gain=None; last_press=-1e9; lc="off"; wire=None; wh=[]; rh=[]; rows=[]
for f in files:
    for m in LogReader(f):
        w=m.which(); t=m.logMonoTime/1e9
        if w=="sendcan":
            for c in m.sendcan:
                if c.address==272 and len(c.dat)>=13:
                    d=bytes(c.dat); gain=d[12]*0.004; raw=((d[10]>>2)|(d[11]<<6))&0x3FFF
                    if raw&0x2000: raw-=0x4000
                    wire=raw*0.1; rh.append((t,wire)); rh=[h for h in rh if t-h[0]<=0.2]
        elif w=="carControl": lat=m.carControl.latActive
        elif w=="modelV2": lc=str(m.modelV2.meta.laneChangeState)
        elif w=="carState":
            c=m.carState; v=c.vEgo*3.6; press=c.steeringPressed; tq=abs(c.steeringTorque); blk=c.leftBlinker or c.rightBlinker; wheel=c.steeringAngleDeg
            if press: last_press=t
            wh.append((t,wheel)); wh=[h for h in wh if t-h[0]<=0.2]
            if wire is None or len(wh)<4 or len(rh)<4: continue
            if gain is not None and lat and not press and not blk and lc=="off" and 30<=v<60 and tq<50 and t-last_press>3 and gain>=0.9:
                err=wire-wheel; s=np.sign(err) if err else 1.0
                wrate=(wheel-wh[0][1])/max(t-wh[0][0],1e-3)*s; rrate=(wire-rh[0][1])/max(t-rh[0][0],1e-3)*s
                rows.append((abs(err), rrate, wrate))
a=np.array(rows)
print(f"== {r}: frames {len(a)}")
print("   rows: |err| bin ; cols: wire rate toward-away bins -> median wheel rate toward request (deg/s) [n]")
rb=[(-99,-3),(-3,-1),(-1,1),(1,3),(3,6),(6,99)]
print("   |err|   " + "".join(f"| rate {lo:>3}..{hi:<3} " for lo,hi in rb))
for lo,hi in ((0,1),(1,2),(2,3),(3,4),(4,5),(5,7),(7,10),(10,90)):
    b=a[(a[:,0]>=lo)&(a[:,0]<hi)]
    if len(b)<50: continue
    line=f"   {lo:2.0f}-{hi:2.0f}   "
    for rlo,rhi in rb:
        c=b[(b[:,1]>=rlo)&(b[:,1]<rhi)]
        line += (f"| {np.median(c[:,2]):+5.1f} [{len(c):5d}] " if len(c)>=30 else "|      -       ")
    print(line)
