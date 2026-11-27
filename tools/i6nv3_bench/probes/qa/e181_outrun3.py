import glob, numpy as np, warnings
warnings.filterwarnings('ignore')
S = 'S'
fs = sorted(glob.glob(f'{S}/r50/*.npz')) + [f'{S}/n2/000000{r}.npz' for r in ('aa','ac','ad','b0','b1','b2','b3','b4','b5','b6')]
R = []
for p in fs:
    d = dict(np.load(p, allow_pickle=True))
    if 'sf_cc' not in d or 'gain' not in d: continue
    T = d['T']; sf = d['sf_cc'].astype(np.int64); o = (sf >> 25) & 1; grip = (((sf >> 8) & 1) | ((sf >> 9) & 1) | d['pressed'].astype(int)) > 0
    tq = d['tq_s']; sg = np.where(np.abs(tq) >= 150, np.sign(tq), 0); run = np.zeros(len(tq), int)
    for i in range(1, len(tq)):
        run[i] = run[i - 1] + sg[i] if sg[i] * run[i - 1] > 0 else sg[i]
    ed = np.flatnonzero((o[1:] == 1) & (o[:-1] == 0)) + 1
    for i in ed:
        if i < 50 or i + 150 >= len(T): continue
        w = slice(i - 14, i - 1)
        a0 = d['ang'][i - 5]; amin = d['ang'][i:i + 60][np.argmin(np.abs(d['ang'][i:i + 60]))]
        rel = abs(a0) >= 3 and abs(amin) < 0.5 * abs(a0)
        R.append((d['v'][i], d['gain'][i - 12:i - 4].min(), grip[w].any(), np.abs(run[w]).max(), rel, bool(d['lb'][i] or d['rb'][i]), int(d['tx_reject'][i - 10:i + 30].sum()) if 'tx_reject' in d else 0))
R = np.array(R, float)
for gth in (0.0, 0.3, 0.5):
    for nfr in (5, 8):
        ev = (R[:, 2] > 0) | (R[:, 3] >= nfr) | (R[:, 1] < gth)
        q = R[~ev]
        print(f'gain<{gth} | grip | torque run>={nfr}: kept {ev.sum()} (releases {int(R[ev,4].sum())}), suppressed {len(q)} (releases {int(q[:,4].sum())}; with blinker {int(q[:,5].sum())}; median v {np.median(q[:,0]):.0f}; tx rejects near {int((q[:,6]>0).sum())})')
