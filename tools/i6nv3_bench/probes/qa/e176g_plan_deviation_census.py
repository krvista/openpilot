"""Phase 83 design: how far the plan (2 s ahead) aims from its usual spot relative to the lane centre - straights vs
curves, to set the curve allowance and the thresholds. usual = 10 s filter of (plan - centre) at L."""
import numpy as np, glob, os, sys
S='/tmp/claude-0/-home-user-openpilot/b24e552a-84af-537a-8cbb-81fda69ed9c1/scratchpad'
X=np.array([192.0*(i/32)**2 for i in range(33)])
def since(m,dt=0.01):
  o=np.zeros(len(m)); c=1e9
  for i,x in enumerate(m):
    c=0 if x else c+dt; o[i]=c
  return o
def load(r):
  d=dict(np.load(f'{S}/r50/{r}.npz',allow_pickle=True)); p=dict(np.load(f'{S}/la/plan_{r}.npz'))
  T=p['T']; i=np.clip(np.searchsorted(d['T'],T),0,len(d['T'])-1); v=d['v'][i]/3.6
  ok=((since(d['lb']|d['rb'])>3.0)&d['lat'].astype(bool)&(d['lcs']==0))[i]&(v>=30/3.6)&(np.minimum(p['PR'][:,1],p['PR'][:,2])>=0.5)
  L=np.clip(v*2.0,12,80); n=len(T)
  cL=np.array([0.5*(np.interp(L[j],X,p['L1'][j])+np.interp(L[j],X,p['L2'][j])) for j in range(n)])
  pL=np.array([np.interp(L[j],p['PX'][j],p['PY'][j]) for j in range(n)])
  off=pL-cL; usual=np.zeros(n); u=0.0; init=False; a=0.05/10.05
  for j in range(n):
    if ok[j]:
      if not init: u=off[j]; init=True
      else: u+=a*(off[j]-u)
    usual[j]=u
  pr=(since(d['pressed'])<1.0)[i]
  return dict(T=T,v=v,ok=ok,dev=off-usual,K=p['K'],L=L,pr=pr)
if __name__=='__main__':
  rs=sorted(os.path.basename(f)[5:-4] for f in glob.glob(f'{S}/la/plan_*.npz') if os.path.exists(f'{S}/r50/'+os.path.basename(f)[5:]))
  A={}
  for r in rs:
    try: D=load(r)
    except Exception: continue
    for k,v in D.items(): A.setdefault(k,[]).append(v)
  A={k:np.concatenate(v) for k,v in A.items()}
  m=A['ok']&~A['pr']; h=np.abs(A['K'])*A['L']**2/2; sg=np.sign(A['K']); ins=A['dev']*sg   # + = toward the inside of the curve
  print('hands-off usable %.1f h'%(m.sum()/72000))
  for nm,lo,hi in (('h<0.2',0,0.2),('0.2-0.5',0.2,0.5),('0.5-1.0',0.5,1.0),('1.0-2.0',1.0,2.0),('>2.0',2.0,99)):
    q=m&(h>=lo)&(h<hi)
    print(f' plan offset of the curve K L^2/2 {nm:8s} {q.sum()/20:6.0f} s: deviation toward inside mean {ins[q].mean()*100:+5.1f} cm, p5 {np.percentile(ins[q],5)*100:+5.1f} p50 {np.percentile(ins[q],50)*100:+5.1f} p95 {np.percentile(ins[q],95)*100:+5.1f} p99 {np.percentile(ins[q],99)*100:+5.1f} | |dev| p99 {np.percentile(np.abs(A["dev"][q]),99)*100:.0f} p99.9 {np.percentile(np.abs(A["dev"][q]),99.9)*100:.0f}')
  q=m&(h>0.3); c=np.polyfit(h[q],ins[q],1); print('fit inside deviation = %.3f * K L^2/2 %+.3f m'%(c[0],c[1]))
  for coef in (0.25,0.15,0.10,0.0):
    for both in (True,False):
      x=np.abs(A['dev'])-coef*h if both else np.where(ins>0,np.maximum(ins-coef*h,0),-ins)
      w=np.interp(x,[0.2,0.45],[1,0])
      st=m&(h<0.2); cu=m&(h>=0.5)
      print(f' allowance {coef:.2f} x K L^2/2 ({"both sides" if both else "inside only"}): weight lost on straights {100*(1-w[st]).mean():.2f} %, in curves {100*(1-w[cu]).mean():.2f} %; full release time straights {100*(w[st]==0).mean():.2f} %, curves {100*(w[cu]==0).mean():.2f} %')
