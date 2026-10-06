"""§35 model comparison, step 2: lane keeping / plan smoothness / curves / grabs per group, all roads and shared 50 m GPS cells.
Run from BENCH_DIR after model_compare_join.py (reads corner47/all14.npz)."""
import numpy as np
D=dict(np.load("corner47/all14.npz"))
grp=D['grp']; v=D['v']; lat=D['lat']; w=D['yr0']-D['yl0']
k=D['k_meas']; a=np.abs(k)*(v/3.6)**2; s=np.sign(k)
op=lat&(D['tx_active']==2)&~D['pressed']&(D['gain']>=0.5)
good=(D['pl']>0.7)&(D['pr']>0.7)&(v>30)&~D['busy']&(w>2.7)&(w<4.3)
# GPS 50 m cells
ok_gps=(np.abs(D['gps_lat'])>1)&(np.abs(D['gps_lon'])>1)
cell=np.where(ok_gps, np.round(D['gps_lat']/0.00045).astype(np.int64)*1000003+np.round(D['gps_lon']/0.00057).astype(np.int64), -1)
cd_cells=set(np.unique(cell[(grp=="CD210")&lat&(v>30)&ok_gps])); op_cells=set(np.unique(cell[(grp!="CD210")&lat&(v>30)&ok_gps]))
both=cd_cells&op_cells
inboth=np.isin(cell,list(both))
print(f"GPS cells (lat active, >30 km/h): CD210 {len(cd_cells)}, OPM {len(op_cells)}, shared {len(both)}")
def quad_k(y0,y10,y30):
  # fit y = a + b x + c x^2 through x=0,10,30 -> curvature ~ 2c (y right +)
  X=np.array([[1,0,0],[1,10,100],[1,30,900]]); Xi=np.linalg.inv(X)
  c=Xi[2,0]*y0+Xi[2,1]*y10+Xi[2,2]*y30; return 2*c
klane=quad_k((D['yl0']+D['yr0'])/2,(D['yl10']+D['yr10'])/2,(D['yl30']+D['yr30'])/2)
kplan=quad_k(D['py0'],D['py10'],D['py30'])
car_right=-(D['yl0']+D['yr0'])/2
def stats(name, m0):
  out={}
  st=m0&good&op&(a<0.3)
  b0=np.median(car_right[st]) if st.sum() else np.nan
  out['straight min']=st.sum()/1200
  out['centre offset cm (bias)']=b0*100
  out['offset spread p10-p90 cm']=(np.percentile(car_right[st],90)-np.percentile(car_right[st],10))*100 if st.sum()>100 else np.nan
  # wander: 2 s moving-window std of offset, median
  x=car_right.copy(); x[~st]=np.nan
  out['lane conf both>0.7 share (op, >30)']=np.mean(((D['pl']>0.7)&(D['pr']>0.7))[m0&op&(v>30)&~D['busy']])*100
  cu=m0&good&op&(a>=0.5)
  gL=np.median(-D['yl0'][st]); gR=np.median(D['yr0'][st])
  gin=np.where(s>0,D['yr0'],-D['yl0'])-np.where(s>0,gR,gL)
  out['curve min (>=0.5 m/s2)']=cu.sum()/1200
  out['inside-line gap vs straight cm']=np.median(gin[cu])*100 if cu.sum()>100 else np.nan
  cl=m0&good&op&(np.abs(klane)>0.0015)&(np.sign(klane)==np.sign(kplan))
  out['plan/lane curvature (curves)']=np.median(np.abs(kplan[cl])/np.abs(klane[cl])) if cl.sum()>100 else np.nan
  out['vehicle/lane curvature (curves)']=np.median(np.abs(k[cl])/np.abs(klane[cl])) if cl.sum()>100 else np.nan
  # plan smoothness on straights: diff of model action curvature as lateral jerk (m/s^3) at 20 Hz
  dd=np.zeros(len(v)); dd[1:]=np.diff(D['dk'])*20*(v[1:]/3.6)**2
  same=np.zeros(len(v),bool); same[1:]=D['route'][1:]==D['route'][:-1]
  sm=st&same
  out['plan lat-jerk RMS straight m/s3']=np.sqrt(np.mean(dd[sm]**2)) if sm.sum() else np.nan
  out['wheel rate RMS straight deg/s']=np.sqrt(np.mean(D['rate'][st]**2)) if st.sum() else np.nan
  tr=m0&op&(v>30)&~D['busy']
  e=np.abs(D['k_meas']-D['k_ctrl'])[tr]*1e3
  out['|k_meas-k_ctrl| p50/p90 1e-3']=f"{np.percentile(e,50):.2f}/{np.percentile(e,90):.2f}"
  act=m0&lat&(D['tx_active']==2)&(v>30)
  pr=D['pressed'].astype(int); onset=np.zeros(len(v),bool); onset[1:]=(np.diff(pr)==1)&same[1:]
  out['grabs per active min (>30 km/h, no blinker)']=np.sum(onset&act&~D['busy'])/(np.sum(act&~D['busy'])/1200)
  out['op-led share of active >30']=np.sum(op&m0&(v>30))/max(np.sum(lat&m0&(v>30)),1)*100
  out['lane_min p50 (op >30)']=np.median(D['lane_min'][m0&op&(v>30)])
  print(f"\n== {name}")
  for kk,vv in out.items(): print(f"   {kk:45s} {vv if isinstance(vv,str) else round(float(vv),3)}")
for scope,mask in (("ALL ROADS",np.ones(len(v),bool)),("SHARED GPS CELLS",inboth)):
  print("\n########", scope)
  for g in ("OPMold","OPMnew","CD210"):
    stats(g, (grp==g)&mask)
