"""Population-controlled check of 39a/39b. Clean releases = steeringPressed 1->0 while latActive, 30-45 km/h, and for the next 3 s
|column tq| < 60 Nm with no press (driver really let go). Per release: sent ACI gain at 0.5/1/2 s, |cmd-wheel| at 1/2 s, share of
frames with |gap|>3 in 0-2 s. Also 39b cost: over latActive 30-45 km/h frames, pressed share, hand-on (100-350 Nm, not pressed)
share, and among hand-on frames the share with gain>=0.9 and |gap|>3 (op pushing against a hand)."""
import glob, sys, numpy as np
from openpilot.tools.lib.logreader import LogReader
r=sys.argv[1]; files=sorted(glob.glob(f"/home/user/drivelog/drivelog/*_{r}--*--rlog.zst"), key=lambda f:int(f.split("--")[-2]))[1:]
v=0; press=0; tq=0; blk=0; lat=0; gap=0; gain=None; prev=0; lc="off"; rel=[]; fr=[]
for f in files:
    for m in LogReader(f):
        w=m.which(); t=m.logMonoTime/1e9
        if w=="sendcan":
            for c in m.sendcan:
                if c.address==272 and len(c.dat)>=13: gain=bytes(c.dat)[12]*0.004
        elif w=="carControl": lat=m.carControl.latActive
        elif w=="controlsState": gap=m.controlsState.steerCmdGapDeg
        elif w=="modelV2": lc=str(m.modelV2.meta.laneChangeState)
        elif w=="carState":
            c=m.carState; v=c.vEgo*3.6; press=c.steeringPressed; tq=abs(c.steeringTorque); blk=c.leftBlinker or c.rightBlinker
            if gain is None: continue
            if lat and not blk and lc=="off" and 30<=v<45:
                fr.append((int(press), tq, gain, abs(gap)))
            if prev and not press and lat and not blk and lc=="off" and 30<=v<45:
                rel.append(dict(t=t,g={},gap={},big=[],clean=True))
            prev=press
            for e in rel[-3:]:
                dt=t-e["t"]
                if dt<=3.0 and (press or tq>=60): e["clean"]=False
                for tt in (0.5,1.0,2.0):
                    if tt not in e["g"] and dt>=tt: e["g"][tt]=gain; e["gap"][tt]=abs(gap)
                if dt<=2.0: e["big"].append(abs(gap)>3)
done=[e for e in rel if 2.0 in e["g"]]; clean=[e for e in done if e["clean"]]
a=np.array(fr)
def line(lst,tag):
    if len(lst)<3: print(f"   {tag}: n={len(lst)} (too few)"); return
    print(f"   {tag:28s} n={len(lst):3d} | gain p50 @0.5/1/2 s {np.median([e['g'][0.5] for e in lst]):.2f}/{np.median([e['g'][1.0] for e in lst]):.2f}/{np.median([e['g'][2.0] for e in lst]):.2f} | |gap| p50 @1/2 s {np.median([e['gap'][1.0] for e in lst]):.1f}/{np.median([e['gap'][2.0] for e in lst]):.1f} deg | 0-2 s |gap|>3 share p50 {np.median([np.mean(e['big']) for e in lst])*100:.0f}% mean {np.mean([np.mean(e['big']) for e in lst])*100:.0f}%")
print(f"== {r}: 30-45 km/h latActive frames {len(a)} ({len(a)/100/60:.1f} min) | releases {len(done)}, clean (<60 Nm & no press for 3 s) {len(clean)} = {len(clean)/max(len(done),1)*100:.0f}%")
line(done,"all releases"); line(clean,"clean releases")
hand=a[(a[:,0]==0)&(a[:,1]>=100)&(a[:,1]<350)]
print(f"   frames: pressed {np.mean(a[:,0])*100:.0f}% | hand-on 100-350 Nm not pressed {len(hand)/len(a)*100:.0f}% (|tq| p50 {np.median(hand[:,1]) if len(hand) else 0:.0f}) | of hand-on: gain>=0.9 {np.mean(hand[:,2]>=0.9)*100 if len(hand) else 0:.0f}%, gain>=0.9 & |gap|>3 (op pushing against the hand) {np.mean((hand[:,2]>=0.9)&(hand[:,3]>3))*100 if len(hand) else 0:.0f}% | gain p50 at 50-100 Nm {np.median(a[(a[:,0]==0)&(a[:,1]>=50)&(a[:,1]<100)][:,2]) if ((a[:,0]==0)&(a[:,1]>=50)&(a[:,1]<100)).any() else 0:.2f}")
print(f"   grabs (press onsets) per latActive 30-45 km/h minute: {len(done)/max(len(a)/6000,1e-3):.1f}")
