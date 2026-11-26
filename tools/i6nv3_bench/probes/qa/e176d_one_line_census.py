"""One lane line only: how often (all driving, not only hands-off), does the driver keep hands on, and how good are the
model's plan and a 'visible line + learned width' centre there - the driver's own path as the yardstick."""
import sys, numpy as np
S='/tmp/claude-0/-home-user-openpilot/b24e552a-84af-537a-8cbb-81fda69ed9c1/scratchpad'
X=np.array([192.0*(i/32)**2 for i in range(33)])
def since(m,dt):
  o=np.zeros(len(m)); c=1e9
  for i,x in enumerate(m):
    c=0 if x else c+dt; o[i]=c
  return o
acc={}
def add(k,v): acc.setdefault(k,[]).append(np.asarray(v))
for r in sys.argv[1:]:
  d=dict(np.load(f'{S}/r50/{r}.npz',allow_pickle=True)); p=dict(np.load(f'{S}/la/plan_{r}.npz'))
  T=p['T']; i=np.clip(np.searchsorted(d['T'],T),0,len(d['T'])-1)
  v=d['v'][i]; pr_on=since(d['pressed'],0.01)[i]<1.0; bl=since(d['lb']|d['rb'],0.01)[i]<3.0
  ok=d['lat'][i]&(v>=30)&~bl&(d['lcs'][i]==0)
  pl,prr=p['PR'][:,1],p['PR'][:,2]; mn=np.minimum(pl,prr); mx=np.maximum(pl,prr)
  cls=np.where(mn>=0.5,0,np.where((mx>=0.5)&(mn<0.3),1,np.where(mx<0.3,2,3)))   # both / one / none / in between
  L=np.clip(v/3.6*1.5,10,None)
  l0,r0=p['L1'][:,0],p['L2'][:,0]
  lL=np.array([np.interp(L[j],X,p['L1'][j]) for j in range(len(T))]); rL=np.array([np.interp(L[j],X,p['L2'][j]) for j in range(len(T))])
  planL=np.array([np.interp(L[j],p['PX'][j],p['PY'][j]) for j in range(len(T))])
  # learned width: 10 s filter while both lines are good, held otherwise
  w=np.zeros(len(T)); cur=3.0; a=0.05/10.05
  for j in range(len(T)):
    if cls[j]==0 and 2.5<r0[j]-l0[j]<4.6: cur+=a*((r0[j]-l0[j])-cur)
    w[j]=cur
  left_vis=pl>=prr
  c1_0=np.where(left_vis,l0+w/2,r0-w/2); c1_L=np.where(left_vis,lL+w/2,rL-w/2)     # one-line centre at the car / at L
  c2_0=(l0+r0)/2; c2_L=(lL+rL)/2
  cen0=np.where(cls==1,c1_0,c2_0); cenL=np.where(cls==1,c1_L,c2_L)
  kd=(d['k_meas'][i]-p['K'])*1e3                                  # what the car did minus what the model asked
  # hands-off -> press transitions
  press_edge=np.r_[False,pr_on[1:]&~pr_on[:-1]]
  for c,nm in ((0,'both'),(1,'one'),(2,'none')):
    m=ok&(cls==c)
    add(nm+'_n',m.sum()); add(nm+'_ok',ok.sum()); add(nm+'_handson',pr_on[m]); add(nm+'_v',v[m])
    add(nm+'_edges',(press_edge&m).sum()); add(nm+'_ho_n',(m&~pr_on).sum())
    add(nm+'_carpos_on',-cen0[m&pr_on]*100); add(nm+'_carpos_off',-cen0[m&~pr_on]*100)     # car vs the centre estimate (+ right)
    add(nm+'_planaim_on',(planL-cenL)[m&pr_on]*100); add(nm+'_planaim_off',(planL-cenL)[m&~pr_on]*100)  # where the plan aims vs the centre estimate
    add(nm+'_kd_on',kd[m&pr_on]); add(nm+'_curve',(np.abs(p['K'][m])>=1e-3))
  # check of the one-line estimate where the truth is known: both lines good, pretend the weaker one is gone
  m=ok&(cls==0); add('chk',(np.where(left_vis,lL+w/2,rL-w/2)-c2_L)[m]*100)
  # episode durations of one-line
  one=ok&(cls==1); e=np.flatnonzero(np.diff(np.r_[0,one.astype(int),0])); add('dur',(e[1::2]-e[::2])/20)
A={k:np.concatenate([np.atleast_1d(x) for x in v]) for k,v in acc.items()}
tot=A['both_ok'].sum()
print('routes',sys.argv[1:],' lateral on, >=30 km/h, no blinker / lane change: %.0f s'%(tot/20))
for nm in ('both','one','none'):
  n=A[nm+'_n'].sum()
  print(f'\n{nm:5s}: {n/20:6.0f} s = {100*n/tot:4.1f} % | hands-on {100*A[nm+"_handson"].mean():3.0f} % | speed p50 {np.median(A[nm+"_v"]):.0f} | in curves {100*A[nm+"_curve"].mean():.0f} % | driver takes over {A[nm+"_edges"].sum()/max(A[nm+"_ho_n"].sum()/20/60,1e-9):.1f} per hands-off min ({A[nm+"_edges"].sum()} in {A[nm+"_ho_n"].sum()/20:.0f} s)')
  for tag,t2 in (('on','hands-on '),('off','hands-off')):
    x=A[f'{nm}_carpos_{tag}']; y=A[f'{nm}_planaim_{tag}']
    if len(x)>40: print(f'   {t2} {len(x)/20:5.0f} s: car vs centre estimate mean {x.mean():+5.1f} sd {x.std():5.1f} cm | plan aim (1.5 s ahead) vs centre estimate mean {y.mean():+5.1f} sd {y.std():5.1f} cm, |aim|>30 cm {100*np.mean(np.abs(y)>30):.0f} %, >50 cm {100*np.mean(np.abs(y)>50):.0f} %')
  x=A[nm+'_kd_on']
  if len(x)>40: print(f'   hands-on: car curvature minus model action: sd {x.std():.2f}e-3, |.|>1e-3 {100*np.mean(np.abs(x)>1):.0f} %')
print('\none-line centre estimate checked where both lines are good (one hidden): error sd %.1f cm, p95 %.1f cm'%(A['chk'].std(),np.percentile(np.abs(A['chk']),95)))
dd=A['dur']; print('one-line episodes: %d, >=1 s %d, >=3 s %d, longest %.1f s, time in >=3 s episodes %.0f s'%(len(dd),(dd>=1).sum(),(dd>=3).sum(),dd.max(),dd[dd>=3].sum()))
