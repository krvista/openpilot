"""Sent ACI gain (LKAS_ALT 0x110 byte 12 * 0.004) vs cmd-wheel gap, hands-off latActive, by speed / time since last press."""
import glob, sys, numpy as np
from openpilot.tools.lib.logreader import LogReader
r=sys.argv[1]; files=sorted(glob.glob(f"/home/user/drivelog/drivelog/*_{r}--*--rlog.zst"), key=lambda f:int(f.split("--")[-2]))[1:]
v=0; press=0; tq=0; eps=0; wheel=0; blk=0; lat=0; gap=0; gain=None; last_press_t=-1e9; addrs={}
rows=[]
for f in files:
    for m in LogReader(f):
        w=m.which(); t=m.logMonoTime/1e9
        if w=="sendcan":
            for c in m.sendcan:
                addrs[c.address]=addrs.get(c.address,0)+1
                if c.address==272 and len(c.dat)>=13:
                    d=bytes(c.dat); gain=d[12]*0.004
                    raw=((d[10]>>2)|(d[11]<<6))&0x3FFF
                    if raw&0x2000: raw-=0x4000
                    cmd_wire=raw*0.1
        elif w=="carState":
            c=m.carState; v=c.vEgo; press=c.steeringPressed; tq=c.steeringTorque; eps=c.steeringTorqueEps; wheel=c.steeringAngleDeg; blk=c.leftBlinker or c.rightBlinker
            if press: last_press_t=t
            if gain is not None and lat and not press and not blk and v*3.6>=22:
                rows.append((v*3.6, gap, gain, t-last_press_t, abs(eps), abs(tq)))
        elif w=="carControl": lat=m.carControl.latActive
        elif w=="controlsState": gap=m.controlsState.steerCmdGapDeg
a=np.array(rows)
print(f"== {r}: hands-off latActive frames {len(a)}; sendcan addrs {sorted(addrs.items(), key=lambda x:-x[1])[:4]}")
for lo,hi in ((22,30),(30,36),(36,45),(45,54),(54,72),(72,130)):
    b=a[(a[:,0]>=lo)&(a[:,0]<hi)]
    if len(b)<200: continue
    big=b[np.abs(b[:,1])>3]; small=b[np.abs(b[:,1])<=1]
    print(f"   {lo:3d}-{hi:3d} km/h n={len(b):6d} | gain p10/p50/p90 {np.percentile(b[:,2],10):.2f}/{np.percentile(b[:,2],50):.2f}/{np.percentile(b[:,2],90):.2f} | |gap|>3: {len(big)/len(b)*100:4.1f}%  gain p50 {np.median(big[:,2]) if len(big) else 0:.2f}, <0.7 in {np.mean(big[:,2]<0.7)*100 if len(big) else 0:.0f}%, EPS tq p50 {np.median(big[:,4]) if len(big) else 0:.0f} | |gap|<=1: gain p50 {np.median(small[:,2]) if len(small) else 0:.2f}, EPS tq p50 {np.median(small[:,4]) if len(small) else 0:.0f}")
    # by gain bin: share of big gap
    parts=[]
    for glo,ghi in ((0,0.5),(0.5,0.75),(0.75,0.95),(0.95,1.01)):
        c=b[(b[:,2]>=glo)&(b[:,2]<ghi)]
        if len(c)>100: parts.append(f"gain {glo:.2f}-{ghi:.2f}: {len(c)/len(b)*100:.0f}% of frames, |gap|>3 in {np.mean(np.abs(c[:,1])>3)*100:.0f}%")
    print("        " + " | ".join(parts))
    parts=[]
    for plo,phi in ((0,1.5),(1.5,3),(3,10),(10,1e9)):
        c=b[(b[:,3]>=plo)&(b[:,3]<phi)]
        if len(c)>100: parts.append(f"{plo:.0f}-{phi if phi<1e8 else 999:.0f}s since press: {len(c)/len(b)*100:.0f}%, gain p50 {np.median(c[:,2]):.2f}, |gap|>3 {np.mean(np.abs(c[:,1])>3)*100:.0f}%")
    print("        " + " | ".join(parts))
