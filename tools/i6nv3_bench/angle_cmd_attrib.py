import glob, sys, math, numpy as np
from openpilot.tools.lib.logreader import LogReader
from opendbc.car.vehicle_model import VehicleModel
r=sys.argv[1]; wins=[tuple(map(float,w.split("-"))) for w in sys.argv[2:]]
files=sorted(glob.glob(f"/home/user/drivelog/drivelog/*_{r}--*--rlog.zst"), key=lambda f:int(f.split("--")[-2]))
CP=None
for m in LogReader(files[0]):
    if m.which()=="carParams": CP=m.carParams; break
VM=VehicleModel(CP)
lo=min(w[0] for w in wins)-30; hi=max(w[1] for w in wins)+1
segs=[f for f in files if lo-60 <= int(f.split("--")[-2])*60 <= hi]
st=dict(v=0,k=0,I=0,roll=0,aoff=0,ang=0,press=0); roll_lp=None; rows=[]; rolls=[]
for f in segs:
    for m in LogReader(f):
        t=m.logMonoTime/1e9
        if t<lo or t>hi: continue
        w=m.which()
        if w=="carState": st['v']=m.carState.vEgo; st['ang']=m.carState.steeringAngleDeg; st['press']=int(m.carState.steeringPressed)
        elif w=="controlsState": st["k"]=m.controlsState.desiredCurvature; st["km"]=m.controlsState.curvature; st['I']=m.controlsState.angleFbInteg
        elif w=="vehicleParameters": st["roll"]=m.vehicleParameters.roll; st["aoff"]=m.vehicleParameters.angleOffsetDeg
        elif w=="carControl":
            # replicate ROLL_LP_TAU=0.6 at 100 Hz
            roll_lp = st['roll'] if roll_lp is None else roll_lp + (0.01/(0.6+0.01))*(st['roll']-roll_lp)
            if not any(a<=t<=b for a,b in wins): continue
            v=st['v']; k=st['k']; I=st['I']
            gain=float(np.interp(v,[0,5,10,15],[0,0.2,0.5,1.0]))
            ff=math.degrees(VM.get_steer_from_curvature(-k, v, 0.0))
            fbI=math.degrees(VM.get_steer_from_curvature(-(k+I), v, 0.0))-ff
            rt=math.degrees(VM.get_steer_from_curvature(-(k+I), v, roll_lp*gain))-math.degrees(VM.get_steer_from_curvature(-(k+I), v, 0.0))
            rows.append((t, v*3.6, k*1e3, I*1e3, math.degrees(roll_lp), ff, fbI, rt, st['aoff'], ff+fbI+rt+st['aoff'], m.carControl.actuators.steeringAngleDeg, st['ang'], st['press']))
print("   t     v  kD   I    roll°  | ff   +I   +roll +off = recon | cmd  | wheel P")
for i,x in enumerate(rows):
    if i%10==0 or abs(x[7])>2 or (hi-lo)<40: print(f"{x[0]:7.1f} {x[1]:3.0f} {x[2]:+5.2f} {x[3]:+5.2f} {x[4]:+5.1f} | {x[5]:+5.1f} {x[6]:+5.1f} {x[7]:+5.1f} {x[8]:+4.1f} = {x[9]:+6.1f} | {x[10]:+6.1f} | {x[11]:+6.1f} {x[12]}")
a=np.array(rows); print(f"recon-cmd residual: median {np.median(a[:,9]-a[:,10]):+.2f} p90 {np.percentile(np.abs(a[:,9]-a[:,10]),90):.2f} deg")
