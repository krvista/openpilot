"""MDPS (0xEA) own status bits vs the request: LKA_ACTIVE (byte6 bit0), LKA_ANGLE_ACTIVE (byte18 bits1..0), LKA_ANGLE_FAULT (byte18 bit5),
STEERING_OUT_TORQUE (bits 64..75 LE, 0.1, -204.8). Binned by |cmd-wheel| on hands-off, gain>=0.9, 30-60 km/h frames; plus a dump window."""
import glob, sys, numpy as np
from openpilot.tools.lib.logreader import LogReader
r=sys.argv[1]; dump_seg=int(sys.argv[2]); d0=float(sys.argv[3]); d1=float(sys.argv[4])
files=sorted(glob.glob(f"/home/user/drivelog/drivelog/*_{r}--*--rlog.zst"), key=lambda f:int(f.split("--")[-2]))[1:]
v=0; press=0; tq=0; blk=0; lat=0; gap=0; gain=None; last_press=-1e9; lc="off"; cmd=0; wheel=0; rate=0
mdps=None; rows=[]; last_dump=-9; sent_active=None
for f in files:
    seg=int(f.split("--")[-2])
    for m in LogReader(f):
        w=m.which(); t=m.logMonoTime/1e9
        if w=="can":
            for c in m.can:
                if c.address==234 and c.src<128 and len(c.dat)>=24:
                    d=bytes(c.dat); tq_out=((d[8]|(d[9]<<8))&0xFFF)*0.1-204.8
                    mdps=(d[6]&1, d[18]&0x3, (d[18]>>5)&1, tq_out)
        elif w=="sendcan":
            for c in m.sendcan:
                if c.address==272 and len(c.dat)>=13:
                    d=bytes(c.dat); gain=d[12]*0.004; sent_active=(d[9]>>4)&0x3   # LKAS_ANGLE_ACTIVE 77|2@0+ -> byte 9 bits 5..4
        elif w=="carControl": lat=m.carControl.latActive; cmd=m.carControl.actuators.steeringAngleDeg
        elif w=="controlsState": gap=m.controlsState.steerCmdGapDeg
        elif w=="modelV2": lc=str(m.modelV2.meta.laneChangeState)
        elif w=="carState":
            c=m.carState; v=c.vEgo*3.6; press=c.steeringPressed; tq=abs(c.steeringTorque); blk=c.leftBlinker or c.rightBlinker; wheel=c.steeringAngleDeg; rate=c.steeringRateDeg
            if press: last_press=t
            if mdps and gain is not None and lat and not press and not blk and lc=="off" and 30<=v<60 and tq<50 and t-last_press>3 and gain>=0.9:
                rows.append((abs(gap), mdps[0], mdps[1], mdps[2], abs(mdps[3]), rate*np.sign(gap if gap else 1), sent_active if sent_active is not None else -1))
            if seg==dump_seg and d0<=t<=d1 and t-last_dump>=0.2 and mdps:
                last_dump=t; print(f"  t={t:.1f} v={v:3.0f} cmd={cmd:+5.1f} wheel={wheel:+5.1f} gap={gap:+4.1f} gain={gain} sentActive={sent_active} | MDPS lka_active={mdps[0]} angle_active={mdps[1]} angle_fault={mdps[2]} outTq={mdps[3]:+6.1f} rate={rate:+5.1f}")
a=np.array(rows)
print(f"== {r}: frames {len(a)}; sent LKAS_ANGLE_ACTIVE values {np.unique(a[:,6])}, MDPS angle_active values {np.unique(a[:,2])}, lka_active {np.unique(a[:,1])}")
print("   |gap| bin | n | MDPS angle_active==2 % | angle_fault % | |outTq| p50 | rate toward cmd p50")
for lo,hi in ((0,1),(1,2),(2,3),(3,4),(4,5),(5,7),(7,90)):
    b=a[(a[:,0]>=lo)&(a[:,0]<hi)]
    if len(b)<100: continue
    print(f"   {lo:3.0f}-{hi:3.0f} | {len(b):6d} | {np.mean(b[:,2]==2)*100:5.1f}% | {np.mean(b[:,3]==1)*100:4.1f}% | {np.median(b[:,4]):5.1f} | {np.median(b[:,5]):+5.1f}")
