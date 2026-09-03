#!/usr/bin/env python3
"""Distill v4 — steering/safety/MADS fields from ccnc-drivelog rlogs.
v3: cs_rate (steeringRateDeg), sp_paused (carStateSP.lateralControlPaused), mv_px11 (model position.x[11]).
v4 (BSD / high-speed prep): cs_bl/cs_br (per-side blinker), da_* (driverAssistance = op LDW from plannerd),
mv_ly1/mv_ly2 (lane line lateral position at x[0], left/right of ego), mv_lcs/mv_lcd (model lane change state/dir)."""
import os, subprocess, sys
import numpy as np
import zstandard as zstd
import capnp

REPO = os.environ.get("REPO", "/home/user/openpilot")
OUT = os.environ.get("NPZ_OUT", os.path.join(os.path.dirname(os.path.abspath(__file__)), "npz2"))
os.makedirs(OUT, exist_ok=True)
capnp.remove_import_hook()
_cereal_dir = os.path.join(REPO, "cereal") if os.path.exists(os.path.join(REPO, "cereal", "log.capnp")) else os.path.join(REPO, "openpilot", "cereal")
log_capnp = capnp.load(os.path.join(_cereal_dir, "log.capnp"),
                       imports=[os.path.join(REPO, "opendbc_repo", "opendbc", "car"), os.path.join(REPO, "opendbc_repo", "opendbc", "car", "include")])

def distill(route, seg, oid):
    base = f"{route}--{seg}"
    out_path = os.path.join(OUT, base + ".npz")
    if os.path.exists(out_path):
        return "cached"
    if os.path.sep in oid:   # path mode (checked-out drivelog clone)
        raw = open(oid, "rb").read()
    else:
        raw = subprocess.run(["git", "-C", REPO, "cat-file", "blob", oid], capture_output=True, check=True).stdout
    data = zstd.ZstdDecompressor().decompressobj().decompress(raw); del raw
    cs = {k: [] for k in ("t", "v", "ang", "rate", "tq", "pr", "blink", "bl", "br", "bsl", "bsr", "brake", "stand", "gear")}
    cc_t, cc_lat, cc_ang = [], [], []
    co_t, co_ang = [], []
    sp_t, sp_p = [], []
    da_t, da_l, da_r = [], [], []
    mv_ly1, mv_ly2, mv_lcs, mv_lcd = [], [], [], []
    mv_t, mv_conf, mv_llp, mv_dc, mv_px11 = [], [], [], [], []
    events, errlogs = [], []
    n_mv = 0; meta = {}
    try:
        for m in log_capnp.Event.read_multiple_bytes(data):
            w = m.which(); t = m.logMonoTime * 1e-9
            if w == 'carState':
                c = m.carState
                cs["t"].append(t); cs["v"].append(c.vEgo); cs["ang"].append(c.steeringAngleDeg); cs["rate"].append(c.steeringRateDeg)
                cs["tq"].append(c.steeringTorque); cs["pr"].append(1.0 if c.steeringPressed else 0.0)
                cs["blink"].append(1.0 if (c.leftBlinker or c.rightBlinker) else 0.0)
                cs["bl"].append(1.0 if c.leftBlinker else 0.0); cs["br"].append(1.0 if c.rightBlinker else 0.0)
                cs["bsl"].append(1.0 if c.leftBlindspot else 0.0); cs["bsr"].append(1.0 if c.rightBlindspot else 0.0)
                cs["brake"].append(1.0 if c.brakePressed else 0.0); cs["stand"].append(1.0 if c.standstill else 0.0)
                cs["gear"].append(str(c.gearShifter))
            elif w == 'driverAssistance':
                da_t.append(t); da_l.append(1.0 if m.driverAssistance.leftLaneDeparture else 0.0); da_r.append(1.0 if m.driverAssistance.rightLaneDeparture else 0.0)
            elif w == 'carStateSP':
                sp_t.append(t); sp_p.append(1.0 if m.carStateSP.lateralControlPaused else 0.0)
            elif w == 'carControl':
                c = m.carControl; cc_t.append(t); cc_lat.append(1.0 if c.latActive else 0.0); cc_ang.append(c.actuators.steeringAngleDeg)
            elif w == 'carOutput':
                co_t.append(t); co_ang.append(m.carOutput.actuatorsOutput.steeringAngleDeg)
            elif w == 'modelV2':
                n_mv += 1
                if n_mv % 2 == 0:
                    mm = m.modelV2
                    mv_t.append(t); mv_conf.append(str(mm.confidence))
                    llp = list(mm.laneLineProbs); mv_llp.append(max(llp[1:3]) if len(llp) >= 3 else 0.0)
                    mv_dc.append(float(mm.action.desiredCurvature))
                    px = mm.position.x; mv_px11.append(float(px[11]) if len(px) >= 12 else float('nan'))
                    ll = mm.laneLines
                    mv_ly1.append(float(ll[1].y[0]) if len(ll) >= 3 and len(ll[1].y) else float('nan'))
                    mv_ly2.append(float(ll[2].y[0]) if len(ll) >= 3 and len(ll[2].y) else float('nan'))
                    mv_lcs.append(str(mm.meta.laneChangeState)); mv_lcd.append(str(mm.meta.laneChangeDirection))
            elif w == 'onroadEvents':
                for e in m.onroadEvents: events.append((t, str(e.name)))
            elif w == 'errorLogMessage':
                s = m.errorLogMessage
                if s: errlogs.append(s[:200])
            elif w == 'initData':
                meta['gitCommit'] = m.initData.gitCommit; meta['gitBranch'] = m.initData.gitBranch
    except Exception as ex:  # truncated tail (power cut) — keep the parsed prefix
        errlogs.append(f"DISTILL_TRUNCATED {type(ex).__name__}")
    del data
    np.savez_compressed(out_path,
        cs_t=cs["t"], cs_v=cs["v"], cs_ang=cs["ang"], cs_rate=cs["rate"], cs_tq=cs["tq"], cs_pr=cs["pr"], cs_blink=cs["blink"],
        cs_bsl=cs["bsl"], cs_bsr=cs["bsr"], cs_brake=cs["brake"], cs_stand=cs["stand"], cs_gear=cs["gear"],
        cs_bl=cs["bl"], cs_br=cs["br"], da_t=da_t, da_l=da_l, da_r=da_r,
        mv_ly1=mv_ly1, mv_ly2=mv_ly2, mv_lcs=mv_lcs, mv_lcd=mv_lcd,
        sp_t=sp_t, sp_paused=sp_p,
        cc_t=cc_t, cc_lat=cc_lat, cc_ang=cc_ang, co_t=co_t, co_ang=co_ang,
        ev_t=[e[0] for e in events], ev_n=[e[1] for e in events],
        mv_t=mv_t, mv_conf=mv_conf, mv_llp=mv_llp, mv_dc=mv_dc, mv_px11=mv_px11,
        errlogs=errlogs, git_commit=meta.get('gitCommit', ''), git_branch=meta.get('gitBranch', ''))
    return "ok"

if __name__ == "__main__":
    rows = [l.rstrip("\n").split("\t") for l in open(sys.argv[1]) if l.strip()]
    for i, row in enumerate(rows):
        try: r = distill(row[0], row[1], row[2])
        except Exception as ex: r = f"ERR {type(ex).__name__}: {str(ex)[:80]}"
        print(f"[{i+1}/{len(rows)}] {row[0]}--{row[1]}: {r}", flush=True)
