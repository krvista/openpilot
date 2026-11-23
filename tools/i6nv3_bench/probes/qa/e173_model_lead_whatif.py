"""E173 (report §108): what an extra lead told to the model would do - the logged action re-timed with modeld's own
formula (K + (2 / v) (psi(T2) / T2 - psi(T) / T)) for told delay + lead: how much earlier the action rises at hands-off
curve entries, the extra curvature, the open-loop inside shift, and the extra activity on straights. argv: route [...]"""
import numpy as np,sys
S='/tmp/claude-0/-home-user-openpilot/b24e552a-84af-537a-8cbb-81fda69ed9c1/scratchpad'
TI=np.array([10.0*(k/32)**2 for k in range(33)])
for r in sys.argv[1:]:
    d=dict(np.load(S+f'/r50/{r}.npz',allow_pickle=True)); pl=dict(np.load(S+f'/la/plan_{r}.npz')); T=d['T'];n=len(T);v=d['v']
    Tp=pl['T']; idx=np.clip(np.searchsorted(T,Tp),0,n-1); v20=np.maximum(v[idx]/3.6,1.0); K=pl['K']; PSI=pl['PSI']
    told=0.316
    def act(tau):
        t=told+0.075; t2=told+tau+0.075
        a=np.array([np.interp(t2,TI,PSI[i])/t2-np.interp(t,TI,PSI[i])/t for i in range(len(Tp))])
        return K+(2.0/v20)*a
    req=d['tx_active']==2; pr=d['pressed'].astype(bool); bl=(d['lb']|d['rb']).astype(bool)
    ho=(req&~(np.convolve(pr|bl,np.ones(100),'full')[:n]>0))[idx]
    Ks=np.convolve(K,np.ones(8)/8,'same'); s=np.sign(Ks); ak=np.abs(Ks)
    cross=np.flatnonzero((ak[1:]>=0.6e-3)&(ak[:-1]<0.6e-3))+1; ent=[]; last=-999
    for i in cross:
        if i-last<100 or i<60 or i+80>=len(Tp): continue
        if not ho[i-40:i+80].all() or ak[i-50:i-10].max()>0.4e-3 or ak[i:i+80].max()<1.2e-3 or v20[i]<11: continue
        last=i; ent.append(i)
    print(f"== {r}: {len(ent)} hands-off entries (model action passes 0.6e-3, peak >= 1.2e-3)")
    for tau in (0.10,0.15,0.20):
        A=act(tau); dt_=[];lat=[];dk=[];exitlat=[]
        for i in ent:
            sg=s[i]; w=slice(i-40,i+80); t=(np.arange(120)-40)/20.0
            k0=sg*np.convolve(K,np.ones(4)/4,'same')[w]; k1=sg*np.convolve(A,np.ones(4)/4,'same')[w]
            def tf(x,f):
                a=x[:10].mean(); b=x[-10:].mean(); y=(x-a)/(b-a) if abs(b-a)>1e-9 else x*0
                k=np.flatnonzero(y>=f); return t[k[0]] if len(k) else np.nan
            dt_.append([tf(k1,f)-tf(k0,f) for f in (0.2,0.5)])
            dd=(k1-k0); dk.append(dd[30:70].max())
            # extra lateral displacement toward the inside if the car followed the earlier action: double integral of v^2 dK
            vv=v20[w]**2; lat.append(np.cumsum(np.cumsum(dd*vv)*0.05)[-1]*0.05*100)
        dt_=np.array(dt_)
        st=ho&(np.abs(Ks)<0.4e-3)
        print(f"   lead {tau:.2f} s: action earlier by {-np.nanmedian(dt_[:,0])*1000:.0f} ms at 20 %, {-np.nanmedian(dt_[:,1])*1000:.0f} ms at 50 % | peak extra curvature p50 {np.median(dk)*1e3:.2f}e-3 | open-loop inside shift after 4 s p50 {np.median(lat):+.0f} cm (p90 {np.percentile(lat,90):+.0f}) | straights: extra action std {np.std((A-K)[st])*1e3:.3f}e-3 (action itself {np.std(K[st])*1e3:.3f}e-3)")
