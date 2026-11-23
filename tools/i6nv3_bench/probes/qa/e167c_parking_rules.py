"""E167c (report §105): complete entry + exit rule sets (wheel angle, no GPS fix since start, exit by speed / lane lines /
straight distance) - latched time on the road, unlatched time in the car park at departure and arrival, and mid-drive
entries from the 100 Hz route files."""
import numpy as np,glob
S='/tmp/claude-0/-home-user-openpilot/b24e552a-84af-537a-8cbb-81fda69ed9c1/scratchpad'
def sus(m,n):
    c=np.zeros(len(m),int);k=0
    for i,x in enumerate(m): k=k+1 if x else 0; c[i]=k
    return c>=n
def dcalm(v,aw,lim):
    d=0;o=np.zeros(len(v))
    for i in range(len(v)): d=0 if aw[i]>=lim else d+v[i]/36; o[i]=d
    return o
def sim(p,wheel=300,gps=False,exitE=True,exitI=True,vout=30,tout=30):
    v=p[:,1];aw=np.abs(p[:,2]);fix=p[:,3]>0;lane=p[:,5];n=len(v)
    A=sus(v>=vout,tout); E=sus((lane>=0.5)&(aw<30)&(v>=15),30) if exitE else np.zeros(n,bool); I=(dcalm(v,aw,90)>=100) if exitI else np.zeros(n,bool)
    on=np.zeros(n,bool);st=False;low=0;sig=False;ever=False
    for i in range(n):
        ever|=fix[i]; low=low+1 if v[i]<30 else 0
        if low==0: sig=False
        if aw[i]>=wheel: sig=True
        s=sig or (gps and not ever)
        if not st and low>=30 and s: st=True
        elif st and (A[i] or E[i] or I[i]): st=False; sig=False
        on[i]=st
    return on
cfg={'user: wheel300|noGPS-since-boot, exit 30/3s':dict(gps=True,exitE=False,exitI=False),
     'wheel300 only, exit 30/3s':dict(exitE=False,exitI=False),
     'wheel270 only, exit 30/3s':dict(wheel=270,exitE=False,exitI=False),
     'wheel300, exit 30/3s | lane0.5 3s':dict(exitI=False),
     'wheel300, exit 30/3s | 100 m straight':dict(exitE=False),
     'wheel300, exit 30/3s | lane | 100 m':dict(),
     'wheel300|noGPS-boot, exit 30/3s | lane | 100 m':dict(gps=True),
     'wheel270|noGPS-boot, exit 30/3s | lane | 100 m':dict(gps=True,wheel=270)}
tot={k:np.zeros(6) for k in cfg}
for f in sorted(glob.glob(S+'/pk/0*.npy')):
    a=np.load(f); t=a[:,0]; cut=np.flatnonzero(np.diff(t)>30)
    parts=[a[:cut[0]+1],a[cut[0]+1:]] if len(cut) else [a[a[:,0]<a[0,0]+420],a[a[:,0]>a[-1,0]-240]]
    for pi,p in enumerate(parts):
        v=p[:,1];aw=np.abs(p[:,2]);big=np.flatnonzero(aw>=200);mv=v>5;n=len(v);idx=np.arange(n)
        if pi==0: inpark=idx<=(big[-1] if len(big) else -1)
        else: inpark=idx>=(big[0] if len(big) else n)
        for k,c in cfg.items():
            on=sim(p,**c); x=tot[k]
            x[0]+=np.sum(on&~inpark&mv)/10; x[1]+=np.sum(~inpark&mv)/10
            if pi==0: x[2]+=np.sum(~on&inpark&mv)/10; x[3]+=np.sum(inpark&mv)/10
            else: x[4]+=np.sum(~on&inpark&mv)/10; x[5]+=np.sum(inpark&mv)/10
print(f"{'rule':50s} road latched   | departure: car park NOT latched | arrival: car park NOT latched")
for k,x in tot.items(): print(f"{k:50s} {x[0]:5.0f}/{x[1]:.0f}s     | {x[2]:5.0f}/{x[3]:.0f}s                  | {x[4]:5.0f}/{x[5]:.0f}s")
# mid-drive false entries: wheel>=270/300 below 30 km/h outside the first/last 4 min, all 100 Hz routes
for lim in (270,300):
    nev=0;hrs=0
    for f in sorted(glob.glob(S+'/r50/0*.npz')):
        d=np.load(f,allow_pickle=True);T=d['T'];v=d['v'];aw=np.abs(d['ang']);mid=(T>T[0]+240)&(T<T[-1]-240)
        m=(aw>=lim)&(v<30)&(v>3)&mid; e=np.flatnonzero(np.diff(np.r_[0,m.astype(int)])==1)
        e=[x for j,x in enumerate(e) if j==0 or T[x]-T[e[j-1]]>30]; nev+=len(e); hrs+=mid.sum()/360000
    print(f"mid-drive |wheel|>={lim} below 30 km/h: {nev} events in {hrs:.1f} h")
