"""pooled entries / steady curves / straights over several routes using e179's own definitions"""
import sys, io, contextlib
import numpy as np
sys.path.insert(0, '/home/user/wt3/tools/i6nv3_bench/probes/qa')
import e179_next_drive_check as K
def get(p):
    o = K.load(p)
    with contextlib.redirect_stdout(io.StringIO()):
        K.curve_entries(o)
    return o
if __name__ == '__main__':
    for grp in sys.argv[1:]:
        name, paths = grp.split('=')[0], grp.split('=')[1].split(',')
        E = []; ho = 0
        for p in paths:
            o = get(p); r = p.split('/')[-1][6:8]
            for e in o['E']:
                E.append([int(r, 16)] + list(e))
            ho += (o['ho'] & (o['v'] >= 45)).sum() / 100
        E = np.array(E)
        print(f"== {name}: hands-off>=45 {ho:.0f} s, entries {len(E)}")
        if len(E):
            for lo, hi in ((45, 60), (60, 80), (80, 200), (45, 200)):
                q = E[(E[:, 2] >= lo) & (E[:, 2] < hi)]
                if len(q):
                    print(f"  {lo}-{hi}: n {len(q)} wheel-model {np.nanmedian(q[:,3]):+.0f} ms (IQR {np.nanpercentile(q[:,3],25):+.0f}..{np.nanpercentile(q[:,3],75):+.0f}), cmd-model {np.nanmedian(q[:,4]):+.0f} (IQR {np.nanpercentile(q[:,4],25):+.0f}..{np.nanpercentile(q[:,4],75):+.0f}), max inside {np.nanmedian(q[:,5]):+.1f} cm p90 {np.nanpercentile(q[:,5],90):+.1f}, 1.5-2.5s {np.nanmedian(q[:,6]):+.1f}, bit15>0 in {(q[:,7]>0).sum()}")
            if '-v' in name:
                for e in E: print("   %02x t=%7.1f v=%3.0f w-m %+5.0f c-m %+5.0f ins %+5.1f late %+5.1f b15 %.2f b13 %.2f" % (int(e[0]), *e[1:]))
