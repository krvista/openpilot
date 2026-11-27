import sys, numpy as np
sys.path.insert(0, 'S/n2')
import pool; K = pool.K
S = 'S'
G = {'P79 (a8,a9)': [f'{S}/nd/build/000000a8.npz', f'{S}/nd/build/000000a9.npz'],
     'P81 (aa,ac,ad)': [f'{S}/n2/000000{r}.npz' for r in ('aa', 'ac', 'ad')],
     'P84 (b0-b6)': [f'{S}/n2/000000b{i}.npz' for i in range(7)]}
bands = ((45, 60), (60, 80), (80, 100), (100, 200))
for g, ps in G.items():
    acc = {b: [] for b in bands}; st = {b: [] for b in bands}; E = []; ho = 0; ep = {b: [] for b in bands}
    for p in ps:
        o = pool.get(p); v = o['v']; pos = o['pos']; ks = o['ks']; ho += (o['ho'] & (v >= 45)).sum() / 100
        E += [list(e) for e in o['E']]
        for b in bands:
            m = (v >= b[0]) & (v < b[1]); s = o['st'] & m; c = o['cv'] & m
            ref = pos[s].mean() if s.sum() > 300 else o['smean']
            acc[b].append((pos[c] - ref) * np.sign(ks[c]))
            for a, e in K.runs(c):
                if e - a >= 100: ep[b].append(((pos[a:e] - ref) * np.sign(ks[a:e])).mean())
            # straight segments: detrended sd per >=15 s run
            for a, e in K.runs(s):
                if e - a >= 1500:
                    x = pos[a:e]; st[b].append((np.std(x - K.box(x, 1000)), np.std(K.band(x, 6, 12)), e - a))
    E = np.array(E)
    print(f'== {g}: hands-off >=45 km/h {ho:.0f} s, entries {len(E)}')
    for b in bands:
        x = np.concatenate(acc[b]); e = np.array(ep[b]); s = np.array(st[b])
        q = E[(E[:, 1] >= b[0]) & (E[:, 1] < b[1])] if len(E) else E
        print(f'  {b[0]:3d}-{b[1]:3d}: steady curve {len(x)/100:5.0f} s inside {x.mean() if len(x)>100 else np.nan:+5.1f} cm (episodes n {len(e)} median {np.median(e) if len(e) else np.nan:+5.1f}, outside<-10: {(e<-10).sum()}, inside>15: {(e>15).sum()})'
              f' | straight segs {len(s)} / {s[:,2].sum()/100 if len(s) else 0:.0f} s sd {np.average(s[:,0],weights=s[:,2]) if len(s) else np.nan:.1f} band {np.average(s[:,1],weights=s[:,2]) if len(s) else np.nan:.1f}'
              f' | entries n {len(q)} w-m {np.nanmedian(q[:,2]) if len(q) else np.nan:+.0f} c-m {np.nanmedian(q[:,3]) if len(q) else np.nan:+.0f} max inside {np.nanmedian(q[:,4]) if len(q) else np.nan:+.1f} late {np.nanmedian(q[:,5]) if len(q) else np.nan:+.1f}')
