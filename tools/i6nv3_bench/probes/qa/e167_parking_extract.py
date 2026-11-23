"""E167 (report §105): parking-mode study, step 1 - 10 Hz extract of speed, wheel angle, GPS fix / accuracy, lane-line
confidence, gear, lead, cruise, press from the raw rlogs present for each route (first / last segments are enough).
argv: route [route ...] -> S/pk/<route>.npy"""
import glob,re,sys,numpy as np
from multiprocessing import Pool
from openpilot.tools.lib.logreader import LogReader
S='/tmp/claude-0/-home-user-openpilot/b24e552a-84af-537a-8cbb-81fda69ed9c1/scratchpad'
def one(f):
    rows=[]; fix=0; hacc=1e4; lane=0.; gear=0; lead=0; nxt=None
    try:
        for m in LogReader(f):
            w=m.which()
            if w=='gpsLocationExternal': g=m.gpsLocationExternal; fix=int(g.hasFix); hacc=g.horizontalAccuracy
            elif w=='modelV2':
                p=m.modelV2.laneLineProbs
                if len(p)>=3: lane=min(p[1],p[2])
            elif w=='radarState': lead=int(m.radarState.leadOne.present)
            elif w=='carState':
                c=m.carState; t=m.logMonoTime/1e9
                if nxt is None or t>=nxt:
                    nxt=t+0.1; rows.append((t,c.vEgo*3.6,c.steeringAngleDeg,fix,hacc,lane,int(c.gearShifter.raw),lead,int(c.cruiseState.enabled),int(c.steeringPressed)))
    except Exception as e: print('ERR',f,e)
    return np.array(rows)
if __name__=='__main__':
    for r in sys.argv[1:]:
        fs=sorted(glob.glob(f'/home/user/dl_tree/repo/drivelog/*_{r}--*rlog.zst'),key=lambda p:int(re.search(r'--(\d+)--rlog',p).group(1)))
        idx=[int(re.search(r'--(\d+)--rlog',p).group(1)) for p in fs]
        with Pool(3) as p: out=p.map(one,fs)
        a=np.vstack([o for o in out if len(o)]); np.save(f'{S}/pk/{r}.npy',a); print(r,idx[0],idx[-1],len(a),flush=True)
