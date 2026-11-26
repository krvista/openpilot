"""The 'section fit under the car' on real modelV2 frames: lane centre (two inner lines) fitted with a quadratic over
[0, Xf] -> c0, psi, k_lane; term = g w (a0 (c0 + BIAS) + a1 psi + k_lane - K). Noise on straights, push in curves,
timing of k_lane against the car's own curvature."""
import sys, numpy as np
S='/tmp/claude-0/-home-user-openpilot/b24e552a-84af-537a-8cbb-81fda69ed9c1/scratchpad'
X=np.array([192.0*(i/32)**2 for i in range(33)]); BIAS=-0.034
def since(m,dt=0.01):
  o=np.zeros(len(m)); c=1e9
  for i,x in enumerate(m):
    c=0 if x else c+dt; o[i]=c
  return o
def lp(x,tau,dt=0.05):
  y=np.copy(x); a=dt/(tau+dt)
  for i in range(1,len(x)): y[i]=y[i-1]+a*(x[i]-y[i-1])
  return y
def fits(p,v,mode):
  n=len(p['T']); out=np.zeros((n,3))
  cen=(p['L1']+p['L2'])/2
  for j in range(n):
    Xf={'30':30.0,'40':40.0,'1.0s':np.clip(v[j]*1.0,15,45),'1.5s':np.clip(v[j]*1.5,20,60)}[mode]
    m=X<=Xf+1e-6
    if m.sum()<4: m=np.arange(33)<4
    c=np.polyfit(X[m],cen[j][m],2); out[j]=(c[2],c[1],2*c[0])
  return out
R={}
for r in sys.argv[1:]:
  d=dict(np.load(f'{S}/r50/{r}.npz',allow_pickle=True)); p=dict(np.load(f'{S}/la/plan_{r}.npz'))
  T=p['T']; i=np.clip(np.searchsorted(d['T'],T),0,len(d['T'])-1); v=d['v'][i]/3.6
  rel=((d['gw_flags']&8)>0) if d['gw_flags'].any() else d['lat'].astype(bool)
  ho=((since(d['pressed'])>1.0)&(since(d['lb']|d['rb'])>3.0)&rel&d['lat']&(d['lcs']==0))[i]
  pr=np.minimum(p['PR'][:,1],p['PR'][:,2]); wd=p['L2'][:,0]-p['L1'][:,0]
  w=np.interp(pr,[0.5,0.8],[0,1])*np.interp(v,[45/3.6,60/3.6],[0,1])*((wd>2.5)&(wd<4.6))
  R[r]=dict(p=p,v=v,ho=ho,w=w,K=p['K'],kmeas=d['k_meas'][i],ours=(d['k_ctrl']-d['k_model'])[i],T=T)
for mode in ('30','40','1.0s','1.5s'):
  acc={k:[] for k in ('st_raw','st_lp','st_r','st_ours_r','st_K_r','cu_out','cu_ours','kl_st','kl_err','lag','st_term_parts')}
  for r,D in R.items():
    f=fits(D['p'],D['v'],mode); c0,psi,kl=f[:,0],f[:,1],f[:,2]
    L=np.maximum(D['v']*1.5,10); K=D['K']
    raw=D['w']*(1.432/L**2*(c0+BIAS)+1.653/L*psi+kl-K); cap=0.4/np.maximum(D['v'],1)**2
    Kp=np.r_[np.zeros(20),K[:-20]]
    st=D['ho']&(np.abs(K)<0.3e-3)&(np.abs(Kp)<0.3e-3)&(D['v']>=60/3.6)&(D['w']>0.5)
    cu=D['ho']&(np.abs(K)>=1.0e-3)&(Kp*K>0)&(np.abs(Kp)>=1e-3)&(D['v']>=45/3.6)&(D['w']>0.2)
    for g in (0.35,):
      t=np.clip(g*raw,-cap,cap); tl=lp(t,0.3)
      d5=lambda x:(x[10:]-x[:-10])
      ss=st[10:]&st[:-10]
      acc['st_raw'].append(t[st]); acc['st_lp'].append(tl[st]); acc['st_r'].append(d5(tl)[ss]); acc['st_ours_r'].append(d5(D['ours'])[ss]); acc['st_K_r'].append(d5(K)[ss])
      acc['cu_out'].append(-tl[cu]*np.sign(K[cu])); acc['cu_ours'].append(-D['ours'][cu]*np.sign(K[cu]))
    acc['kl_st'].append(kl[st]); acc['kl_err'].append((kl-D['kmeas'])[cu]/np.abs(D['kmeas'][cu]).clip(1e-4))
    # timing: lag of k_lane vs the car's own curvature over hands-off frames with curvature activity
    m=D['ho']&(D['v']>=45/3.6); a=lp(kl,0.2); b=D['kmeas']
    best=None
    for sh in range(-30,31):
      aa=a[max(0,sh):len(a)+min(0,sh)]; bb=b[max(0,-sh):len(b)+min(0,-sh)]; mm=m[max(0,sh):len(a)+min(0,sh)]&m[max(0,-sh):len(b)+min(0,-sh)]
      if mm.sum()>500:
        cc=np.corrcoef(aa[mm],bb[mm])[0,1]
        if best is None or cc>best[0]: best=(cc,sh*0.05)
    acc['lag'].append(best)
  cat=lambda k:np.concatenate(acc[k])
  print(f'fit range {mode:5s}: STRAIGHT >=60 km/h {len(cat("st_raw"))/20:.0f}s: k_lane sd {cat("kl_st").std()*1e3:.3f}e-3 | term(g 0.35) raw sd {cat("st_raw").std()*1e3:.3f} after 0.3 s LP sd {cat("st_lp").std()*1e3:.3f} mean {cat("st_lp").mean()*1e3:+.3f} | 0.5 s change sd: term {cat("st_r").std()*1e3:.3f}, current additions {cat("st_ours_r").std()*1e3:.3f}, model action {cat("st_K_r").std()*1e3:.3f}')
  print(f'                 CURVE >=45 km/h {len(cat("cu_out"))/20:.0f}s: term outward {cat("cu_out").mean()*1e3:+.3f} sd {cat("cu_out").std()*1e3:.3f} (current additions {cat("cu_ours").mean()*1e3:+.3f}) | k_lane / car curvature - 1: median {np.median(cat("kl_err"))*100:+.0f} % | k_lane leads the car curvature by (s, corr): {[(round(-b[1],2),round(b[0],2)) for b in acc["lag"] if b]}')
