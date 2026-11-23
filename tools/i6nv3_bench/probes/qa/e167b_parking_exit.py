"""E167b (report §105): parking-mode exit criteria on the car-park departures - seconds released too early (before the
last |wheel| >= 200 deg) and delay / metres on the road until release."""
import numpy as np,glob
S='/tmp/claude-0/-home-user-openpilot/b24e552a-84af-537a-8cbb-81fda69ed9c1/scratchpad'
def first_sustain(m,n,start=0):
    k=0
    for i in range(start,len(m)):
        k=k+1 if m[i] else 0
        if k>=n: return i
    return None
def dist_calm(v,aw,lim,need):   # metres travelled since |wheel| last exceeded lim
    d=0; out=np.zeros(len(v))
    for i in range(len(v)):
        d = 0 if aw[i]>=lim else d+v[i]/3.6*0.1
        out[i]=d
    return out>=need
CR={
 'A v>=30 3s (user)':lambda p:(p['v']>=30,30),
 'B v>=33 2s (now)':lambda p:(p['v']>=33,20),
 'C v>=25 3s':lambda p:(p['v']>=25,30),
 'D lane>=0.8 wheel<30 v>=15 3s (now)':lambda p:((p['lane']>=0.8)&(p['aw']<30)&(p['v']>=15),30),
 'E lane>=0.5 wheel<30 v>=15 3s':lambda p:((p['lane']>=0.5)&(p['aw']<30)&(p['v']>=15),30),
 'F lane>=0.5 wheel<30 v>=20 5s':lambda p:((p['lane']>=0.5)&(p['aw']<30)&(p['v']>=20),50),
 'G gps hAcc<10 wheel<90 v>=15 5s':lambda p:((p['fix'])&(p['hacc']<10)&(p['aw']<90)&(p['v']>=15),50),
 'H 60 m with wheel<90':lambda p:(dist_calm(p['v'],p['aw'],90,60),1),
 'I 100 m with wheel<90':lambda p:(dist_calm(p['v'],p['aw'],90,100),1),
 'J 100 m wheel<90 and v>=20':lambda p:(dist_calm(p['v'],p['aw'],90,100)&(p['v']>=20),1),
 'K 150 m with wheel<90':lambda p:(dist_calm(p['v'],p['aw'],90,150),1),
 'L A or E':None,'M A or I':None,'N A or F or K':None,'O A or (G and I)':None,
}
res={k:[] for k in CR}
for f in sorted(glob.glob(S+'/pk/0*.npy')):
    a=np.load(f); t=a[:,0]; cut=np.flatnonzero(np.diff(t)>30)
    p0=a[:cut[0]+1] if len(cut) else a[a[:,0]<a[0,0]+420]
    p=dict(t=p0[:,0]-p0[0,0],v=p0[:,1],aw=np.abs(p0[:,2]),fix=p0[:,3]>0,hacc=p0[:,4],lane=p0[:,5])
    big=np.flatnonzero(p['aw']>=200)
    if not len(big) or p['t'][big[-1]]<20: continue          # no car park at this start
    iroad=big[-1]; fire={}
    for k,fn in CR.items():
        if fn is None: continue
        m,n=fn(p); fire[k]=m if n==1 else None
        # premature: fires before the last big turn (while a signature precedes); count from first big turn
        i0=big[0]
        ip=first_sustain(m,n,i0); ir=first_sustain(m,n,iroad)
        fire[k]=(ip,ir)
    def comb(*ks):
        ips=[fire[k][0] for k in ks if fire[k][0] is not None]; irs=[fire[k][1] for k in ks if fire[k][1] is not None]
        return (min(ips) if ips else None, min(irs) if irs else None)
    fire['L A or E']=comb('A v>=30 3s (user)','E lane>=0.5 wheel<30 v>=15 3s'); fire['M A or I']=comb('A v>=30 3s (user)','I 100 m with wheel<90')
    fire['N A or F or K']=comb('A v>=30 3s (user)','F lane>=0.5 wheel<30 v>=20 5s','K 150 m with wheel<90')
    g=fire['G gps hAcc<10 wheel<90 v>=15 5s']; i_=fire['I 100 m with wheel<90']
    fire['O A or (G and I)']=comb('A v>=30 3s (user)') if (g[1] is None or i_[1] is None) else (min([x for x in [fire['A v>=30 3s (user)'][0], (max(g[0],i_[0]) if g[0] is not None and i_[0] is not None else None)] if x is not None],default=None), min([x for x in [fire['A v>=30 3s (user)'][1],max(g[1],i_[1])] if x is not None]))
    for k,(ip,ir) in fire.items():
        early=(iroad-ip)/10 if (ip is not None and ip<iroad) else 0.0
        delay=(ir-iroad)/10 if ir is not None else (len(p['t'])-iroad)/10
        # metres on the road while still latched
        m_road=np.sum(p['v'][iroad:(ir if ir is not None else len(p['t']))])/36
        res[k].append((f[-12:-4],early,delay,m_road))
print(f"{len(res['A v>=30 3s (user)'])} starts in a car park")
print(f"{'exit criterion':40s} early>0 (n) early s (sum/max)   road delay s p50/mean/max   road m latched p50/max")
for k,x in res.items():
    e=np.array([r[1] for r in x]); d=np.array([r[2] for r in x]); m=np.array([r[3] for r in x])
    print(f"{k:40s} {np.sum(e>0):3d}     {e.sum():6.0f}/{e.max():4.0f}        {np.median(d):5.1f}/{d.mean():5.1f}/{d.max():5.0f}          {np.median(m):5.0f}/{m.max():5.0f}")
print('per start (A, E, I): route early/delay')
for i,r in enumerate(res['A v>=30 3s (user)']):
    print(r[0], ' A %5.0f/%5.1f'%(r[1],r[2]), ' D %5.0f/%5.1f'%res['D lane>=0.8 wheel<30 v>=15 3s (now)'][i][1:3], ' E %5.0f/%5.1f'%res['E lane>=0.5 wheel<30 v>=15 3s'][i][1:3],' I %5.0f/%5.1f'%res['I 100 m with wheel<90'][i][1:3],' G %5.0f/%5.1f'%res['G gps hAcc<10 wheel<90 v>=15 5s'][i][1:3])
