"""Upstream Blendful (sunnypilot 97135965, file as of 8c8579c1) replayed open loop on logged modelV2 frames, against the
fork's own controlsd additions (k_ctrl - k_model). Lane-line stds are logged only at x = 0, so they are held constant
along x times STD_SCALE (argv[2]); everything else is the upstream code unchanged."""
import sys, types, importlib.util
import numpy as np
S='/tmp/claude-0/-home-user-openpilot/b24e552a-84af-537a-8cbb-81fda69ed9c1/scratchpad'
def mod(name, **kw):
  m=types.ModuleType(name); m.__dict__.update(kw); sys.modules[name]=m; return m
for n in ('openpilot','openpilot.common','openpilot.sunnypilot','openpilot.sunnypilot.modeld_v2'): mod(n)
sp=importlib.util.spec_from_file_location('openpilot.common.filter_simple',f'{S}/bf/filter_up.py'); fm=importlib.util.module_from_spec(sp); sp.loader.exec_module(fm); sys.modules['openpilot.common.filter_simple']=fm
mod('openpilot.common.realtime', DT_MDL=0.05)
X=[192.0*(i/32)**2 for i in range(33)]
mod('openpilot.sunnypilot.modeld_v2.constants', ModelConstants=type('MC',(),{'X_IDXS':X}))
sp=importlib.util.spec_from_file_location('blendful_up',f'{S}/bf/blendful_up.py'); bm=importlib.util.module_from_spec(sp); sp.loader.exec_module(bm)

def run(r, scale):
  d=dict(np.load(f'{S}/r50/{r}.npz',allow_pickle=True)); p=dict(np.load(f'{S}/la/plan_{r}.npz',allow_pickle=True)); s=dict(np.load(f'{S}/bf/std_{r}.npz'))
  assert len(s['T'])==len(p['T']) and np.allclose(s['T'],p['T'])
  T=p['T']; i=np.clip(np.searchsorted(d['T'],T),0,len(d['T'])-1)
  v=d['v'][i]/3.6; lat=d['lat'][i]; lcs=s['LCS']
  B=bm.Blendful(); n=len(T); out=np.zeros((n,6))
  lines=np.zeros((1,4,33,2),np.float32); stds=np.ones((1,4,33,2),np.float32); probs=np.zeros((1,8),np.float32); plan=np.zeros((1,33,2),np.float32)
  for j in range(n):
    lines[0,1,:,0]=p['L1'][j]; lines[0,2,:,0]=p['L2'][j]
    stds[0,1,:,0]=s['SD'][j,1]*scale; stds[0,2,:,0]=s['SD'][j,2]*scale
    probs[0,1::2]=p['PR'][j]; plan[0,:,0]=p['PX'][j]; plan[0,:,1]=p['PY'][j]
    active=bool(lat[j]) and lcs[j]==0          # desire none ~ no lane change (turn desires are not in the extract)
    try: c=B.update({'lane_lines':lines,'lane_lines_stds':stds,'lane_lines_prob':probs,'plan':plan}, float(v[j]), active, float(p['K'][j]))
    except Exception as e: c=np.nan
    out[j]=(c,B.trust,B.dk,B.l_prob,B.r_prob,B.going_around)
  np.savez_compressed(f'{S}/bf/out_{r}_{scale}.npz',T=T,bf=out[:,0],trust=out[:,1],dk=out[:,2],lp=out[:,3],rp=out[:,4],ga=out[:,5])
  print(r,scale,'done nan',np.isnan(out[:,0]).sum())
if __name__=='__main__':
  run(sys.argv[1], float(sys.argv[2]))
