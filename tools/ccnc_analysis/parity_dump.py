"""Run the CURRENT tree's CarController on a distilled route (npz, schema-free) and dump per-frame wire angle/gain.
usage: cd <tree>; PYTHONPATH=$PWD:$PWD/opendbc_repo python3 parity_dump.py <route> <out.npz>"""
import os, sys, glob, numpy as np
sys.path.insert(0, os.getcwd()); sys.path.insert(0, os.path.join(os.getcwd(), "opendbc_repo"))
from phase_tests.harness import Sim
D="/tmp/claude-0/-home-user-openpilot/b24e552a-84af-537a-8cbb-81fda69ed9c1/scratchpad/npz2"
route=sys.argv[1]; out=sys.argv[2]; G=[]; A=[]; T=[]
for f in sorted(glob.glob(os.path.join(D,f"*{route}*.npz")), key=lambda f:int(f.split("--")[-1].split(".")[0])):
    z=np.load(f,allow_pickle=True); t=z["cs_t"]
    if len(t)<500 or len(z["cc_t"])<2 or len(z["sp_t"])<2: continue
    v=np.nan_to_num(z["cs_v"]); ang=np.nan_to_num(z["cs_ang"]); rate=np.nan_to_num(z["cs_rate"]); tq=np.nan_to_num(z["cs_tq"])
    pr=z["cs_pr"]>0.5; bl=z["cs_bl"]>0.5; br=z["cs_br"]>0.5; bsl=z["cs_bsl"]>0.5; bsr=z["cs_bsr"]>0.5; stand=z["cs_stand"]>0.5
    lat=np.interp(t,z["cc_t"],z["cc_lat"])>0.5; cc=np.interp(t,z["cc_t"],z["cc_ang"])
    sim=Sim()
    for i in range(len(t)):
        sim.step(v=float(v[i]),tq=float(tq[i]),wheel=float(ang[i]),cmd=float(cc[i]),lat_active=bool(lat[i]),pressed=bool(pr[i]),blinker=bool(bl[i]),blinker_right=bool(br[i]),bs_l=bool(bsl[i]),bs_r=bool(bsr[i]),standstill=bool(stand[i]),wheel_rate=float(rate[i]))
        G.append(sim.s.aci_gain_last); A.append(sim.s.tx_angle_last); T.append(t[i])
np.savez_compressed(out, gain=np.array(G), tx=np.array(A), t=np.array(T)); print("dumped", len(G), "frames to", out)
