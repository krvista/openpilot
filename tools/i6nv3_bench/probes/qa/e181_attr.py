import sys, numpy as np
sys.path.insert(0, 'S/n2')
import pool; K = pool.K
S = 'S'
for r in sys.argv[1:]:
    o = pool.get(f'{S}/n2/{r}.npz'); T = o['T']
    R = {v: dict(np.load(f'{S}/n2/rep/rep_{r}_{v}.npz')) for v in ('head', 'nofit', 'noavoid', 'nolead')}
    Tr = R['head']['T']; idx = np.searchsorted(T, Tr)
    kh = R['head']['O'][:, 0]
    # logged vs replay agreement
    m = o['ho'][idx] & (o['v'][idx] >= 45)
    print(r, 'log vs head replay |dk| p50/p95 (1e-3):', np.round(np.percentile(np.abs(o['kc'][idx][m] - kh[m]) * 1e3, [50, 95]), 3),
          'bit14 agree', np.mean(K.bit(o['sft'][idx][m], 14) == K.bit(R['head']['O'][m, 1].astype(np.int64), 14)).round(3),
          'bit15 agree', np.mean(K.bit(o['sft'][idx][m], 15) == K.bit(R['head']['O'][m, 1].astype(np.int64), 15)).round(3))
    ak = np.abs(o['kms']); rows = []
    for e in K.entries(o):
        j = e + int(np.argmax(ak[e:e + 400])); s = np.sign(o['kms'][j]); pk = ak[j]
        def t20(x, base=e):
            i = np.flatnonzero(x[base - 150:base + 400] * s >= 0.2 * pk)
            return (i[0] - 150) * 10 if len(i) else np.nan
        tm = t20(o['km'])
        er = np.searchsorted(Tr, T[e])
        out = [T[e], o['v'][e], t20(o['kc']) - tm]
        for v in ('head', 'nofit', 'noavoid', 'nolead'):
            x = R[v]['O'][:, 0]
            i = np.flatnonzero(x[er - 150:er + 400] * s >= 0.2 * pk)
            out.append(((i[0] - 150) * 10 if len(i) else np.nan) - tm)
        # mean additions in the build-up window (model 20% .. +1 s), signed along the curve
        a = er + int(tm / 10); O = R['head']['O'][a:a + 100]
        out += [np.mean(O[:, 0] - o['km'][idx][a:a + 100]) * s * 1e3]
        rows.append(out)
    E = np.array(rows)
    np.save(f'{S}/n2/rep/attr_{r}.npy', E)
    for e in E: print('  t=%7.1f v=%3.0f log %+5.0f | head %+5.0f nofit %+5.0f noavoid %+5.0f nolead %+5.0f | add along curve %+.3f' % tuple(e))
    print('  median: log %+.0f head %+.0f nofit %+.0f noavoid %+.0f nolead %+.0f ; add %+.3f' % tuple(np.nanmedian(E[:, 2:], 0)))
