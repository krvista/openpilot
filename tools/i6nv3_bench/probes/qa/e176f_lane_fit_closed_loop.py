"""Closed-loop check of the section-fit term as it would really be built: lane curvature from a fit over v*1 s (leads
the road under the car by ~0.5 s), heading corrected for the camera lever arm with an error DA (m), measured noise on
k_lane (0.12e-3) and heading (1.5 mrad), g, KE off. Ensemble x both drive bias sets."""
import sys, numpy as np
S='/tmp/claude-0/-home-user-openpilot/b24e552a-84af-537a-8cbb-81fda69ed9c1/scratchpad'
sys.path.insert(0,S+'/opt/sim'); import roadsim as rs
DT=rs.DT; _i=rs._interp
KES=(45/3.6,60/3.6,80/3.6,110/3.6); KE79=(0.0,0.3e-3,0.3e-3,0.15e-3); Z=(0,0,0,0)
def P_(**kw):
  P=dict(rs.DEFAULTS); P.update(LANE_CENTER_KE_SPEED_MS=KES,LANE_CENTER_KE=KE79,X_G=0.0,X_DA=0.0,X_NOISE=0.0,X_SEED=0); P.update(kw); return P
def ctrl(P):
  base=rs.head_controller(P); G=P['X_G']; a=DT/(P['LANE_CENTER_TAU_S']+DT); s=[0.0,0.0,0.0]
  rng=np.random.default_rng(P['X_SEED']); NZ=P['X_NOISE']
  def step(o):
    k,parts=base(o); raw=0.0
    if o.new_frame and NZ>0:
      s[1]+=0.05/0.35*(-s[1])+rng.normal()*0.12e-3*NZ*np.sqrt(2*0.05/0.35); s[2]+=0.05/0.55*(-s[2])+rng.normal()*1.5e-3*NZ*np.sqrt(2*0.05/0.55)
    if o.usable and G>0:
      v=max(o.v,1.0); L=max(v*1.5,10.0)
      w=_i(o.lane_prob,P['LANE_CENTER_PROB'],(0.0,1.0))*_i(v,(45/3.6,60/3.6),(0.0,1.0))
      kl=o.k_lane_ahead[0]+s[1]; psi=o.psi_lane+s[2]+P['X_DA']*kl
      fit=1.432/L**2*(o.c0+P['LANE_CENTER_BIAS_M'])+1.653/L*psi+kl-o.k_action
      cap=P['LANE_CENTER_ACC']/(v*v); used=parts[0]+parts[2]
      raw=float(np.clip(G*w*fit,-cap-used,cap-used))
    s[0]+=a*(raw-s[0]); return k+s[0],parts
  step.state=base.state; return step
V=[('HEAD (P79)',P_()),('fit 0.25, no KE',P_(X_G=0.25,LANE_CENTER_KE=Z)),('fit 0.35, no KE',P_(X_G=0.35,LANE_CENTER_KE=Z)),
   ('fit 0.35, no KE, arm +1 m',P_(X_G=0.35,LANE_CENTER_KE=Z,X_DA=1.0)),('fit 0.35, no KE, arm -1 m',P_(X_G=0.35,LANE_CENTER_KE=Z,X_DA=-1.0)),('fit 0.5, no KE',P_(X_G=0.5,LANE_CENTER_KE=Z))]
ENS=list(rs.ENSEMBLE); DR=('a1','a7')
print('CURVE R400: steady inside [min..max] | peak inside (worst) | exit peak outside (worst) | S-curve swing | step overshoot nominal/worst | delay margin nominal/worst')
for vk in (50,65,85,100):
  for nm,P in V:
    ins=[];ex=[];pk=[];sw=[]
    for en in ENS:
      for dr in DR:
        for kk in (2.5e-3,-2.5e-3):
          m=rs.event_metrics(rs.simulate(rs.synthetic('curve',v_kph=vk,k=kk),P=dict(P,LINEAR=True),controller=ctrl,ens=en,drive=dr,noise=False)); ins.append(m['curve_inside_cm']); ex.append(m['exit_peak_out_cm']); pk.append(m['peak_inside_cm'])
        sw.append(rs.event_metrics(rs.simulate(rs.synthetic('s_curve',v_kph=vk),P=dict(P,LINEAR=True),controller=ctrl,ens=en,drive=dr,noise=False))['peak_swing_cm'])
    dm=[rs.margins(vk,P=P,controller=ctrl,ens=en,drive='a7')['delay_margin_s'] for en in ENS]
    ov=[rs.event_metrics(rs.simulate(rs.synthetic('step',v_kph=vk),P=dict(P,LINEAR=True),controller=ctrl,ens=en,drive='a7',noise=False))['overshoot_pct'] for en in ENS]
    print(f' {vk:3d} {nm:27s} inside {np.mean(ins):5.1f} [{min(ins):5.1f}..{max(ins):5.1f}] | peak {np.mean(pk):5.1f} ({max(pk):5.1f}) | exit {np.mean(ex):4.1f} ({max(ex):4.1f}) | S {np.mean(sw):5.1f} ({max(sw):5.1f}) | ov {ov[0]:3.0f}/{max(ov):3.0f} | dm {dm[0]:.2f}/{min(dm):.2f}')
print('\nSTRAIGHT 180 s, noise on (model + lane-fit noise), 3 seeds x 2 drives x {nominal, quiet, soft}: scatter cm | wheel 3 s hp std deg | wheel rate rms deg/s')
from numpy import std
def hp(x,n=300):
  c=np.cumsum(np.r_[0,x]); m=(c[n:]-c[:-n])/n; return x[n//2:n//2+len(m)]-m
for vk in (55,70,90):
  for nm,P in V[:3]+V[5:]:
    sc=[];wh=[];wr=[]
    for en in ('nominal','quiet','soft'):
      for dr in DR:
        for sd in (1,2,3):
          r=rs.simulate(rs.synthetic('straight',v_kph=vk,d=0.0,T=180.0),P=dict(P,X_NOISE=1.0,X_SEED=sd),controller=ctrl,ens=en,drive=dr,seed=sd)
          sc.append(np.std(r['e_meas'][3000:])*100); wh.append(np.std(hp(r['wheel'][3000:]))); wr.append(np.sqrt(np.mean(np.gradient(r['wheel'][3000:],DT)**2)))
    print(f' {vk:3d} {nm:27s} scatter {np.mean(sc):4.1f} | wheel hp {np.mean(wh):.3f} | wheel rate {np.mean(wr):.2f}')
