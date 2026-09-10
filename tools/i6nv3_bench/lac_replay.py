"""Replay LatControlAngle over a route with logged inputs; compare variants.
usage: lac_replay.py ROUTE [maxseg]   (env VARIANT ignored; all variants run in one pass)"""
import glob, sys, math, types, numpy as np
from openpilot.tools.lib.logreader import LogReader
from opendbc.car.vehicle_model import VehicleModel
import openpilot.selfdrive.controls.lib.latcontrol_angle as lca
r=sys.argv[1]; maxseg=int(sys.argv[2]) if len(sys.argv)>2 else 999
files=sorted(glob.glob(f"/home/user/drivelog/drivelog/*_{r}--*--rlog.zst"), key=lambda f:int(f.split("--")[-2]))[:maxseg]
for m in LogReader(files[0]):
    if m.which()=="carParams": CP=m.carParams; break
VM=VehicleModel(CP)
class CSx: pass
def make(variant):
    lac=lca.LatControlAngle.__new__(lca.LatControlAngle)
    lac.sat_check_min_speed=5.0; lac._trim_7a6=True; lac.use_steer_limited_by_safety=True; lac.dt=0.01
    lac._roll_lp=0.0; lac._roll_lp_init=False; lac._fb_integ=0.0; lac._des_slow=0.0; lac._fb_err_lp=0.0
    lac.sat_time=0.0; lac.sat_limit=0.4
    lac._variant=variant
    return lac
VARIANTS=["base","min8.3"]
lacs={v:make(v) for v in VARIANTS}
# monkeypatch hooks: variant behaviour via module constants is global, so run variants sequentially per frame by swapping constants
def run_frame(lac, active, CS, params, slbs, k, curv_lim):
    v=lac._variant
    old_min=lca.LAT_FB_MIN_SPEED
    lca.LAT_FB_MIN_SPEED = 8.3 if "min8.3" in v else 6.0
    if "rollsym" in v:
        # emulate: curv_actual uses gain-damped roll (same as FF). Patch VM.calc_curvature temporarily.
        gain=float(np.interp(CS.vEgo,[0,5,10,15],[0,0.2,0.5,1.0]))
        orig=VM.calc_curvature
        VM.calc_curvature=lambda sa,u,roll,_o=orig,_g=gain: _o(sa,u,roll*_g)
        try: out=lac.update(active, CS, VM, params, slbs, k, None, curv_lim, 0.3)
        finally: VM.calc_curvature=orig
    else:
        out=lac.update(active, CS, VM, params, slbs, k, None, curv_lim, 0.3)
    lca.LAT_FB_MIN_SPEED=old_min
    return out[1]
st=dict(v=0.0,ang=0.0,press=False,k=0.0,roll=0.0,aoff=0.0,co=0.0,lat=False,I=0.0)
rows=[]; prev_cc=None
for f in files:
    for m in LogReader(f):
        w=m.which(); t=m.logMonoTime/1e9
        if w=="carState": c=m.carState; st['v']=c.vEgo; st['ang']=c.steeringAngleDeg; st['press']=c.steeringPressed
        elif w=="controlsState": st['k']=m.controlsState.desiredCurvature; st['I']=m.controlsState.angleFbInteg
        elif w=="vehicleParameters": st['roll']=m.vehicleParameters.roll; st['aoff']=m.vehicleParameters.angleOffsetDeg
        elif w=="carOutput": st['co']=m.carOutput.actuatorsOutput.steeringAngleDeg
        elif w=="carControl":
            cc=m.carControl; st['lat']=cc.latActive
            slbs = prev_cc is not None and abs(prev_cc-st['co'])>2.5
            CS=CSx(); CS.vEgo=st['v']; CS.steeringAngleDeg=st['ang']; CS.steeringPressed=st['press']
            params=types.SimpleNamespace(roll=st['roll'], angleOffsetDeg=st['aoff'])
            outs=[run_frame(lacs[v], st['lat'], CS, params, slbs, st['k'], False) for v in VARIANTS]
            rows.append((t, st['v']*3.6, int(st['lat']), int(st['press']), cc.actuators.steeringAngleDeg, st['ang'], st['I']*1e3, st['k']*1e3, *outs, *[lacs[v]._fb_integ*1e3 for v in VARIANTS]))
            prev_cc=cc.actuators.steeringAngleDeg
a=np.array(rows); n=len(VARIANTS)
cmd_log=a[:,4]; base=a[:,8]; ho=(a[:,2]==1)&(a[:,3]==0); kD=a[:,7]
print(f"== {r}: {len(a)} frames; parity base vs log: |diff| median {np.median(np.abs(base-cmd_log)):.3f} p99 {np.percentile(np.abs(base-cmd_log),99):.3f} deg | integ parity p99 {np.percentile(np.abs(a[:,8+n]-a[:,6]),99):.3f} e-3 | parity by active: active p99 {np.percentile(np.abs(base-cmd_log)[a[:,2]==1],99):.3f}, inactive p99 {np.percentile(np.abs(base-cmd_log)[a[:,2]==0],99):.3f}")
quiet=np.zeros(len(a),bool); quiet[100:]=(np.abs(kD[100:])<0.5)&(np.abs(kD[100:]-kD[:-100])<0.3)   # plan quiet over the last 1 s
def swing(col, mask, q=95):
    x=a[:,col]; d=np.abs(x[100:]-x[:-100]); mm=mask[100:]&mask[:-100]&(a[100:,2]==1)&quiet[100:]
    return np.percentile(d[mm],q) if mm.sum() else float('nan')
print("   [1-s command swing with the plan quiet (|kD|<0.5e-3, |dkD|<0.3e-3): p95]\n   variant            | v<30 km/h hands-off: swing p95 (deg) | integ at cap % | cmd-wheel |gap| median | 30-54 km/h: swing p99 / cap % / gap | >54 km/h: swing p99 / cap %")
for i,v in enumerate(VARIANTS):
    col=8+i; icol=8+n+i
    out=[]
    for lo,hi in ((0,30),(30,54),(54,200)):
        mk=ho&(a[:,1]>=lo)&(a[:,1]<hi)
        cap=np.minimum(10e-4, 0.5/np.maximum(a[:,1]/3.6,5.0)**2)*1e3
        atcap=100*(np.abs(a[mk,icol])>=0.95*cap[mk]).mean() if mk.sum() else float('nan')
        gap=np.median(np.abs(a[mk,col]-a[mk,5])) if mk.sum() else float('nan')
        stale=100*((np.abs(a[mk,icol])>=0.95*cap[mk])&(np.abs(a[mk,col]-a[mk,5])>3)).mean() if mk.sum() else float('nan')
        out.append((swing(col,mk),atcap,gap,stale))
    print(f"   {v:18s} | {out[0][0]:5.2f} | {out[0][1]:4.1f} (stale {out[0][3]:4.1f}) | {out[0][2]:.2f} | {out[1][0]:5.2f} / {out[1][1]:4.1f} / {out[1][2]:.2f} | {out[2][0]:5.2f} / {out[2][1]:4.1f}")
    if len(sys.argv)>3:
        ta,tb=map(float,sys.argv[3].split("-")); w=(a[:,0]>=ta)&(a[:,0]<=tb)
        print(f"      window {ta}-{tb}: cmd {a[w,col].min():+.1f}..{a[w,col].max():+.1f} deg (swing {a[w,col].max()-a[w,col].min():.1f}), integ {a[w,icol].min():+.2f}..{a[w,icol].max():+.2f} e-3")
np.save(f"/tmp/claude-0/-home-user-openpilot/b24e552a-84af-537a-8cbb-81fda69ed9c1/scratchpad/rb/lacreplay_{r}.npy", a)
