import sys, numpy as np
S='/tmp/claude-0/-home-user-openpilot/b24e552a-84af-537a-8cbb-81fda69ed9c1/scratchpad'
def since(m,dt=0.01):
  o=np.zeros(len(m)); c=1e9
  for i,x in enumerate(m):
    c=0 if x else c+dt; o[i]=c
  return o
def load(r,sc):
  d=dict(np.load(f'{S}/r50/{r}.npz',allow_pickle=True)); p=dict(np.load(f'{S}/la/plan_{r}.npz',allow_pickle=True)); b=dict(np.load(f'{S}/bf/out_{r}_{sc}.npz'))
  T=p['T']; i=np.clip(np.searchsorted(d['T'],T),0,len(d['T'])-1)
  ho=(since(d['pressed'])>1.0)&(since(d['lb']|d['rb'])>3.0)&((d['gw_flags']&8)>0)&d['lat']&(d['lcs']==0)
  return dict(T=T,v=d['v'][i],ho=ho[i],ours=(d['k_ctrl']-d['k_model'])[i]*1e3,lc_on=((d['sf_ctl']>>12)&1)[i]>0,K=p['K']*1e3,bf=b['bf']*1e3,trust=b['trust'],
              pr=np.minimum(p['PR'][:,1],p['PR'][:,2]),prmax=np.maximum(p['PR'][:,1],p['PR'][:,2]),off=-(p['L1'][:,0]+p['L2'][:,0])/2*100,ga=b['ga'])
def boot(x,n=20):
  if len(x)<n*10: return np.nan
  return np.std([np.mean(c) for c in np.array_split(x,n)])/np.sqrt(n)*np.sqrt(n/(n))  # se of the mean from 20 blocks
for sc in sys.argv[1:]:
  R=[load(r,sc) for r in ('000000a8','000000a9')]
  A={k:np.concatenate([x[k] for x in R]) for k in R[0]}
  Kp=np.concatenate([np.r_[np.zeros(20),x['K'][:-20]] for x in R])
  st=A['ho']&(np.abs(A['K'])<0.3)&(np.abs(Kp)<0.3)
  cu=A['ho']&(np.abs(A['K'])>=1.0)&(Kp*A['K']>0)&(np.abs(Kp)>=1.0)
  sg=np.sign(A['K'])
  print(f'===== std scale {sc}   (units 1e-3 1/m; outward = reduces the curve; 0.1e-3 ~ 0.25 deg of wheel)')
  for name,lo,hi in (('30-45',30,45),('45-60',45,60),('60-80',60,80),('80+',80,200)):
    m=st&(A['v']>=lo)&(A['v']<hi)
    if m.sum()>100:
      print(f' straight {name:6s} {m.sum()/20:5.0f}s | blendful mean {A["bf"][m].mean():+.3f} sd {A["bf"][m].std():.3f} trust {A["trust"][m].mean():.2f} | ours mean {A["ours"][m].mean():+.3f} sd {A["ours"][m].std():.3f} (term on {A["lc_on"][m].mean()*100:.0f}%) | corr {np.corrcoef(A["bf"][m],A["ours"][m])[0,1]:+.2f} | lane off {A["off"][m].mean():+.1f}cm')
    m=cu&(A['v']>=lo)&(A['v']<hi)
    if m.sum()>60:
      ob=-A['bf'][m]*sg[m]; oo=-A['ours'][m]*sg[m]; ins=A['off'][m]*sg[m]
      print(f' curve    {name:6s} {m.sum()/20:5.0f}s | blendful outward {ob.mean():+.3f} sd {ob.std():.3f} trust {A["trust"][m].mean():.2f} | ours outward {oo.mean():+.3f} sd {oo.std():.3f} (term on {A["lc_on"][m].mean()*100:.0f}%) | corr {np.corrcoef(ob,oo)[0,1]:+.2f} | car inside {ins.mean():+.1f}cm | |K| {np.abs(A["K"][m]).mean():.2f}')
  h=A['ho']&(A['v']>=30)
  one=h&(A['pr']<0.5)&(A['prmax']>=0.5); none=h&(A['prmax']<0.5); both=h&(A['pr']>=0.5)
  for nm,m in (('both lines',both),('one line',one),('no line',none)):
    print(f' {nm:10s} {m.sum()/20:5.0f}s ({m.sum()/max(h.sum(),1)*100:4.1f}% of hands-off >=30) | ours on {A["lc_on"][m].mean()*100:3.0f}% | blendful trust>0.3 {np.mean(A["trust"][m]>0.3)*100:3.0f}% mean|bf| {np.abs(A["bf"][m]).mean():.3f} p95 {np.percentile(np.abs(A["bf"][m]),95):.3f}')
  d1=lambda x: x[10:]-x[:-10]
  hh=h[10:]&h[:-10]
  print(f' roughness (sd of 0.5 s change, hands-off >=30): blendful {d1(A["bf"])[hh].std():.4f}  ours {d1(A["ours"])[hh].std():.4f}  model action {d1(A["K"])[hh].std():.4f}')
  print(f' |blendful| hands-off >=30: p50 {np.percentile(np.abs(A["bf"][h]),50):.3f} p90 {np.percentile(np.abs(A["bf"][h]),90):.3f} p99 {np.percentile(np.abs(A["bf"][h]),99):.3f} max {np.abs(A["bf"][h]).max():.3f} | ours p50 {np.percentile(np.abs(A["ours"][h]),50):.3f} p90 {np.percentile(np.abs(A["ours"][h]),90):.3f} p99 {np.percentile(np.abs(A["ours"][h]),99):.3f} | going-around {A["ga"][h].mean()*100:.1f}%')
