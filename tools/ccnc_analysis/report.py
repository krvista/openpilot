#!/usr/bin/env python3
"""Unified drive-log report (rebuilt after container reset; Phase 34 build semantics).
usage: report.py <route-prefix> [<route-prefix> ...]   e.g. report.py 00000058 00000059
Metrics: faults/events, motion-normalized shake (0.8-2.5 Hz), yank scan, urgent-regrab audit
(counterfactual driver_tq with the CURRENT hold model incl. crawl gate), BSM dwell/caution,
crawl-gate checkpoints, tau0.30 turn-onset lag, S-reversal retention, lookahead fallback rate."""
import glob, os, sys
import numpy as np

D = os.path.join(os.path.dirname(os.path.abspath(__file__)), "npz2")
SR, WB = 14.96, 2.965
A_PER_C = SR * WB * 180.0 / np.pi

def bandpass(x, fs=100.0, lo=0.8, hi=2.5):
    n = len(x); F = np.fft.rfft(x - x.mean()); fr = np.fft.rfftfreq(n, 1.0 / fs)
    F[(fr < lo) | (fr > hi)] = 0.0; return np.fft.irfft(F, n)

def hold_comp(v, la, ang, rate):
    B = np.interp(v, [5.5, 10.5, 16.5, 25, 36], [140.0, 102.0, 64.0, 62.0, 62.0])
    G = np.interp(v, [5.5, 10.5, 16.5, 25, 36], [53.0, 86.0, 133.0, 123.0, 87.0])
    c = np.minimum(B + G * np.interp(la, [0, .1, .3], [0, .5, 1.0]), 220.0)
    still = (v < 3.0) & (np.abs(ang) < 5.0) & (np.abs(rate) < 10.0)   # Phase 34b gate (no hysteresis in this offline proxy)
    return np.where(still, 70.0, c)

def pressed_machine(dtq, comp_on):
    out = np.zeros(len(dtq), bool); cnt = 0; p = False
    for i in range(len(dtq)):
        base = 230.0 if comp_on[i] else 250.0
        cnt = min(max(cnt + (1 if dtq[i] > base * (0.8 if p else 1.0) else -1), 0), 11)
        p = cnt > 5; out[i] = p
    return out

def load(prefixes):
    files = []
    for f in sorted(glob.glob(os.path.join(D, "*.npz"))):
        rt = os.path.basename(f).split("--")[0].split("_")[-1]
        if rt in prefixes: files.append(f)
    return files

def main(prefixes):
    files = load(prefixes)
    print(f"routes {prefixes}: {len(files)} segments")
    T = dict(dur=0.0, act=0, paused=0, pressed=0, n=0)
    ev = {}; errs = {}; commits = set()
    sh_low = []; sh_low_w = []; sh_tx = []; sh_mid = []; sh_mid_w = []
    yanks = []; regrabs = []; bsm = dict(act=0, blind=0, caution=0)
    crawl = dict(n=0, pressed=0, paused_entries=0, still=0)
    onsets = []; revs = []; fb = dict(n=0, fallback=0)
    for f in files:
        z = np.load(f, allow_pickle=True)
        t = np.asarray(z["cs_t"], float)
        if len(t) < 500 or len(z["cc_t"]) < 2: continue
        commits.add(str(z["git_commit"])[:8])
        v = np.nan_to_num(np.asarray(z["cs_v"], float)); vk = v * 3.6
        ang = np.nan_to_num(np.asarray(z["cs_ang"], float)); rate = np.nan_to_num(np.asarray(z["cs_rate"], float))
        tq = np.abs(np.nan_to_num(np.asarray(z["cs_tq"], float))); pr = np.asarray(z["cs_pr"], float) > 0.5
        stand = np.asarray(z["cs_stand"], float) > 0.5
        bsl = np.asarray(z["cs_bsl"], float) > 0.5; bsr = np.asarray(z["cs_bsr"], float) > 0.5
        lat = np.interp(t, z["cc_t"], z["cc_lat"]) > 0.5
        cc = np.interp(t, z["cc_t"], z["cc_ang"]); co = np.interp(t, z["co_t"], z["co_ang"]) if len(z["co_t"]) > 1 else cc
        paused = np.interp(t, z["sp_t"], z["sp_paused"]) > 0.5 if len(z["sp_t"]) > 1 else np.zeros(len(t), bool)
        eff = lat & ~paused
        la = v ** 2 * np.abs(np.tan(np.radians(ang / SR))) / WB
        T["dur"] += t[-1] - t[0]; T["n"] += len(t); T["act"] += int(lat.sum()); T["paused"] += int((lat & paused).sum()); T["pressed"] += int(pr.sum())
        for n in z["ev_n"]: ev[str(n)] = ev.get(str(n), 0) + 1
        for e in z["errlogs"]: k = str(e)[:60]; errs[k] = errs.get(k, 0) + 1
        # --- shake: TX (co) band RMS normalized by wheel std, 1 s windows, active & hands-off, wheelStd>=0.4
        fco = bandpass(co); ftx_raw = fco
        for s in range(0, len(t) - 100, 100):
            w = slice(s, s + 100)
            if not (eff[w].all() and not pr[w].any()): continue
            ws = ang[w].std()
            if ws < 0.4: continue
            r = np.sqrt(np.mean(fco[w] ** 2))
            if 3 <= vk[w].mean() <= 30: sh_low.append(r / ws); sh_low_w.append(1); sh_tx.append(r)
            elif 30 < vk[w].mean() <= 60: sh_mid.append(r / ws); sh_mid_w.append(1)
        # --- yank scan: wheel rate spike while op active hands-off, op command leading the wheel
        wr = np.abs(np.gradient(np.convolve(ang, np.ones(5) / 5, "same"))) * 100
        i = 300
        while i < len(t) - 50:
            if eff[i] and v[i] >= 5.0 and wr[i] >= 90.0 and not pr[i - 50:i].any() and tq[i - 50:i].max() < 220:
                # OP-led only if the wheel moved TOWARD the TX command (same sign as co-ang) with a
                # real gap; wheel drifting AWAY from the command (caster unwind under weak authority)
                # is an authority/regrab class, not a yank op commanded
                gap = float(co[i - 30] - ang[i - 30]); motion = float(ang[i + 20] - ang[i - 20])
                toward = gap * motion > 0
                lbl = "OP" if (abs(gap) >= 3.0 and toward) else ("unwind/authority" if abs(gap) >= 3.0 else "driver/road")
                yanks.append((os.path.basename(f)[-12:-4], round(t[i] - t[0], 1), int(vk[i]), int(wr[i]), round(gap, 1), lbl))
                i += 200
            i += 1
        # --- urgent regrab audit (counterfactual current-model driver_tq)
        comp = hold_comp(v, la, ang, rate); comp = np.where(eff, comp, 0.0)
        dtq = np.maximum(0.0, tq - comp)
        for i in range(100, len(t) - 10):
            if tq[i] >= 350 and tq[i - 30:i - 20].max() < 150:
                pre = slice(i - 100, i)
                if not lat[pre].all() or pr[pre][:70].any(): continue
                div = np.abs(cc[pre] - ang[pre])
                if div.mean() < 4.0: continue
                aw = float(np.abs(co[pre] - ang[pre]).mean()); ap = float(np.abs(co[pre] - cc[pre]).mean())
                cause = "paused/passive" if paused[pre].mean() > 0.5 else ("lowspeed" if vk[i] < 35 and aw < 1.5 else ("authority-weak" if ap < 1.5 else "other"))
                regrabs.append((os.path.basename(f)[-12:-4], round(t[i] - t[0], 1), int(vk[i]), round(float(div.mean()), 1), cause)); break
        # --- BSM
        bsm["act"] += int(eff.sum()); bsm["blind"] += int((eff & (bsl | bsr)).sum())
        err = cc - ang
        bsm["caution"] += int((eff & (((err > 3.0) & bsl) | ((err < -3.0) & bsr))).sum())
        # --- crawl gate checkpoints (Phase 34b): eff-active creep, pressed-machine ON share, still-straight share
        m = eff & ~stand & (v < 3.0)
        if m.any():
            pm = pressed_machine(dtq, eff)
            crawl["n"] += int(m.sum()); crawl["pressed"] += int((pm & m).sum())
            crawl["still"] += int((m & (np.abs(ang) < 5) & (np.abs(rate) < 10)).sum())
            # low-speed passive entries while creeping (paused rising edge with v<3 and eff just before)
            pe = np.flatnonzero(np.diff(paused.astype(np.int8)) == 1)
            crawl["paused_entries"] += int(sum(1 for k in pe if v[k] < 3.0 and lat[k] and not stand[k]))
        # --- tau0.30 checkpoint: turn-onset lag plan->wheel at 25-45 kph, |Δplan| onset >= 5 deg within 1 s
        ccs = np.convolve(cc, np.ones(21) / 21, "same")
        for i in range(200, len(t) - 300, 5):
            if not (eff[i] and 25 <= vk[i] <= 45 and not pr[i:i + 200].any()): continue
            if abs(ccs[i + 100] - ccs[i]) >= 5.0 and abs(ccs[i] - ccs[i - 100]) < 1.5:
                d = ccs[i + 100] - ccs[i]; thr = ccs[i] + 0.5 * d
                cu = np.flatnonzero(np.sign(d) * (ccs[i:i + 300] - thr) >= 0); cw = np.flatnonzero(np.sign(d) * (ang[i:i + 300] - thr) >= 0)
                if len(cu) and len(cw): onsets.append((cw[0] - cu[0]) * 0.01)
        # --- S-reversal retention: plan sign change with |peaks|>=8 deg both sides within 3 s
        zc = np.flatnonzero(np.diff(np.sign(ccs + 1e-9)) != 0)
        for i in zc:
            if i < 300 or i > len(t) - 300 or not eff[i] or pr[i - 300:i + 300].any() or vk[i] < 15: continue
            pre_pk = np.abs(ccs[i - 300:i]).max(); post_pk = np.abs(ccs[i:i + 300]).max()
            if pre_pk < 8 or post_pk < 8: continue
            revs.append(np.abs(ang[i:i + 300]).max() / post_pk)
        # --- lookahead fallback rate (x[11] < dist_ahead)
        if len(z["mv_t"]) > 10:
            mvt = np.asarray(z["mv_t"], float); px = np.asarray(z["mv_px11"], float); dc = np.asarray(z["mv_dc"], float)
            vv = np.interp(mvt, t, v); active = np.interp(mvt, t, eff.astype(float)) > 0.5
            tau = np.interp(vv, [8, 13, 18], [0.30, 0.12, 0.08])
            base_s = np.interp(vv, [5.6, 13.9, 27.8, 38.9], [0.08, 0.10, 0.13, 0.18]); boost = np.interp(np.abs(dc), [0.0008, 0.005], [0, 0.20])
            dist = np.minimum(vv * (np.minimum(base_s + boost, 0.27) + tau), 10.0)
            ok = active & np.isfinite(px) & (np.abs(dc) >= 0.0008) & (dist >= 0.3)
            fb["n"] += int(ok.sum()); fb["fallback"] += int((ok & (px < dist)).sum())
    # ---- print
    print(f"build commits: {sorted(commits)}")
    print(f"duration {T['dur']/60:.1f} min | latActive {100*T['act']/T['n']:.1f}% | paused-of-active {100*T['paused']/max(T['act'],1):.1f}% | EPS pressed {100*T['pressed']/T['n']:.1f}%")
    keys = [k for k in ev if any(s in k.lower() for s in ("fault", "mismatch", "commissue", "canerror", "steer", "lanedep", "ldw", "controls"))]
    print("events:", {k: ev[k] for k in sorted(keys)} or "none of interest")
    if errs: print("errorLogs (top):", sorted(errs.items(), key=lambda x: -x[1])[:4])
    print(f"shake 3-30kph: motion-normalized {np.mean(sh_low) if sh_low else float('nan'):.3f} (n={len(sh_low)} win) | TX band RMS {np.mean(sh_tx) if sh_tx else float('nan'):.3f} deg | 30-60kph norm {np.mean(sh_mid) if sh_mid else float('nan'):.3f} (n={len(sh_mid)})")
    print(f"yank candidates: {len(yanks)} (OP-led: {sum(1 for y in yanks if y[5]=='OP')})")
    for y in yanks[:8]: print("   ", y)
    print(f"urgent regrabs: {len(regrabs)}  causes: { {c: sum(1 for r in regrabs if r[4]==c) for c in set(r[4] for r in regrabs)} }")
    for r in regrabs[:8]: print("   ", r)
    print(f"BSM: blind dwell {100*bsm['blind']/max(bsm['act'],1):.1f}% of eff-active | caution {bsm['caution']/100:.0f} s")
    print(f"crawl (eff-active v<3): {crawl['n']/100:.0f} s | still-straight share {100*crawl['still']/max(crawl['n'],1):.0f}% | pressed-machine ON {100*crawl['pressed']/max(crawl['n'],1):.2f}% | passive entries while creeping {crawl['paused_entries']}")
    print(f"tau0.30 turn-onset lag 25-45kph: n={len(onsets)} median {np.median(onsets) if onsets else float('nan'):+.2f}s p90 {np.percentile(onsets,90) if onsets else float('nan'):+.2f}s")
    print(f"S-reversal peak retention: n={len(revs)} median {np.median(revs) if revs else float('nan'):.2f}")
    print(f"lookahead fallback rate: {100*fb['fallback']/max(fb['n'],1):.2f}% of {fb['n']} eligible model frames")

if __name__ == "__main__":
    main(sys.argv[1:])
