"""Open-loop replay of the real car controller over every logged lane change: shipped vs Phase 80c (concession ends after release)."""
import sys, numpy as np
W='/home/user/wt3'; sys.path.insert(0,W); sys.path.insert(0,W+'/opendbc_repo')
from phase_tests.harness import Sim
from opendbc.car.hyundai.values import CarControllerParams as P
S='/tmp/claude-0/-home-user-openpilot/b24e552a-84af-537a-8cbb-81fda69ed9c1/scratchpad'
def run(d,a,b,frames):
  P.LC_RELEASE_FRAMES=frames; sim=Sim(); out=[]
  for j in range(a,b):
    sim.step(v=d['v'][j]/3.6,tq=d['tq_s'][j],wheel=d['ang'][j],cmd=d['apply'][j],lat_active=bool(d['lat'][j]),pressed=bool(d['pressed'][j]),
             blinker=bool(d['lb'][j]),blinker_right=bool(d['rb'][j]),cc_blinker_left=bool(d['lcs'][j]>0 and d['lb'][j]),cc_blinker_right=bool(d['lcs'][j]>0 and d['rb'][j]),
             cc_lc_active=bool(d['lcs'][j]==2),wheel_rate=d['rate'][j],a_ego=d['a_ego'][j],gas=bool(d['gas'][j]),lane_prob_min=float(d['lane_min'][j]),
             lfa_state=0,gw_req_angle=float(d['cb_req'][j]),gw_req_lvl2=int(d['cb_lvl2'][j]),lat_granted=bool(d['lat_granted'][j]),bs_l=bool(d['bsl'][j]),bs_r=bool(d['bsr'][j]),mdps_angle_2=float(d['mdps_ang'][j]))
    m=sim.lkas_alt(); out.append((m['ADAS_ACIAnglTqRedcGainVal'],m['ADAS_StrAnglReqVal']) if m else (np.nan,np.nan))
  return np.array(out)
for r in sys.argv[1:]:
  d=dict(np.load(f'{S}/r50/{r}.npz',allow_pickle=True)); T=d['T']; lc=(d['lcs']==2)&(d['v']>30)
  e=np.flatnonzero(np.diff(lc.astype(int))); st=[i+1 for i in e if lc[i+1]]; en=[i+1 for i in e if not lc[i+1]]
  for a0,b0 in zip(st,en):
    a=max(0,a0-600); b=min(len(T),b0+200)
    o=run(d,a,b,0); n=run(d,a,b,30); k=slice(a0-a,b0-a)
    ho=(~d['pressed'][a0:b0].astype(bool)); bl=(d['lb'][a0:b0]|d['rb'][a0:b0]).astype(bool); m=ho&bl
    req=d['apply'][a0:b0]; wh=d['ang'][a0:b0]; den=req-wh; ok=m&(np.abs(den)>0.5)
    au=lambda x: np.nanmedian((x[k][:,1][ok]-wh[ok])/den[ok]) if ok.sum()>5 else np.nan
    dw=np.nanmax(np.abs(n[k][:,1]-o[k][:,1])) if (b0-a0)>0 else 0
    print(f'{r} {T[a0]:7.1f}-{T[b0]:7.1f} v{d["v"][a0]:3.0f} handsoff+blinker {m.sum()/100:4.2f}s | gain p50 log {np.nanmedian(d["gain"][a0:b0][m]) if m.sum() else np.nan:.2f} old {np.nanmedian(o[k][:,0][m]) if m.sum() else np.nan:.2f} new {np.nanmedian(n[k][:,0][m]) if m.sum() else np.nan:.2f} | authority old {au(o):.2f} new {au(n):.2f} (n={ok.sum()}) | max wire change {dw:.1f} deg | pressed-frames wire change {np.nanmax(np.abs(n[k][:,1]-o[k][:,1])[~ho]) if (~ho).sum() else 0:.2f}')
