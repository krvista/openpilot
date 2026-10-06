"""§35 model comparison, step 1: join corner_lanes_extract .npy with route_extract .npz per route (edit G for the route->group map).
Usage: BENCH_DIR=<dir with corner47/, ext43/, ext48/> python model_compare_join.py"""
import os
import numpy as np
S=os.environ.get("BENCH_DIR", ".")
G={"2e":"OPMold","2f":"OPMold","30":"OPMold","31":"OPMold","32":"OPMold","33":"OPMold","34":"OPMold","35":"OPMold","37":"OPMold","38":"OPMold","39":"OPMnew","3a":"OPMnew","3b":"CD210","3c":"CD210"}
cols={}
for r,g in G.items():
  A=np.load(f"{S}/corner47/{r}.npy") if g=="OPMold" else np.load(f"{S}/ext48/{r}.npy")
  d=np.load((f"{S}/ext43/000000{r}.npz" if g=="OPMold" else f"{S}/ext48/000000{r}.npz"),allow_pickle=True); T=d['T']
  i=np.clip(np.searchsorted(T,A[:,0]),0,len(T)-1)
  busy=np.convolve((d['lb']|d['rb']|(d['lcs']!=0)).astype(float),np.ones(601),'same')>0
  rec=dict(t=A[:,0], yl0=A[:,1], yl10=A[:,2], yl30=A[:,3], yr0=A[:,4], yr10=A[:,5], yr30=A[:,6], py0=A[:,7], py10=A[:,8], py30=A[:,9],
           pl=A[:,10], pr=A[:,11], dk=A[:,14], lstd=A[:,16], busy=busy[i])
  for k in ("v","lat","tx_active","gain","pressed","tq_s","k_meas","k_ctrl","k_model","apply","ang","lane_min","gps_lat","gps_lon","rate","a_ego","gas","brake","yaw"):
    rec[k]=d[k][i]
  rec["sf_cc"]=d["sf_cc"][i] if "sf_cc" in d else np.zeros(len(i),np.int64)
  rec["sf_ctl"]=d["sf_ctl"][i] if "sf_ctl" in d else np.zeros(len(i),np.int64)
  rec["route"]=np.full(len(i),r); rec["grp"]=np.full(len(i),g)
  for k,v in rec.items(): cols.setdefault(k,[]).append(v)
D={k:np.concatenate(v) for k,v in cols.items()}
np.savez(f"{S}/corner47/all14.npz", **D)
print({g:round(np.sum((D['grp']==g)&D['lat'])/1200,1) for g in set(G.values())}, "lat-active min (20 Hz)")
