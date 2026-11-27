import glob, numpy as np, warnings, sys
warnings.filterwarnings('ignore')
S = 'S'
fs = sorted(glob.glob(f'{S}/r50/*.npz')) + [f'{S}/n2/000000{r}.npz' for r in ('aa','ac','ad','b0','b1','b2','b3','b4','b5','b6')]
tot = {}
for p in fs:
    d = dict(np.load(p, allow_pickle=True))
    if 'sf_cc' not in d: continue
    T = d['T']; sf = d['sf_cc'].astype(np.int64); o = (sf >> 25) & 1; v = d['v']; ang = d['ang']; pr = d['pressed'].astype(bool)
    bl = (d['lb'] | d['rb']).astype(bool)
    ed = np.flatnonzero((o[1:] == 1) & (o[:-1] == 0)) + 1
    r = p.split('/')[-1][:8]; c = str(d['commit'])[:8] if 'commit' in d else '?'
    n_all = len(ed); ev = []
    for i in ed:
        if i < 200 or i + 100 >= len(T): continue
        if v[i] < 40 or pr[i - 100:i + 1].any() or bl[i - 300:i + 1].any(): continue
        a0 = ang[i - 5]; amin = ang[i:i + 60][np.argmin(np.abs(ang[i:i + 60]))]
        if abs(a0) >= 3 and abs(amin) < 0.5 * abs(a0):
            ev.append((T[i], v[i], a0, amin, np.abs(d['tq_s'][i - 10:i + 1]).max(), (o[i:i + 150]).sum() / 100))
    hrs = (v > 40).sum() / 360000
    k = (c, r); tot[k] = (n_all, len(ev), hrs)
    if ev or r >= '000000a7':
        print(r, c, f'{hrs:.2f} h >=40: outrun episodes {n_all}, hands-off release in a turn: {len(ev)}')
        for e in ev: print('     t=%.1f v=%.0f ang %+.1f -> %+.1f  |tq| max %.0f  passive %.2f s' % e)
print('older routes total:', sum(v[0] for k, v in tot.items() if k[1] < '000000a7'), 'episodes, releases', sum(v[1] for k, v in tot.items() if k[1] < '000000a7'), 'hours', round(sum(v[2] for k, v in tot.items() if k[1] < '000000a7'), 1))
