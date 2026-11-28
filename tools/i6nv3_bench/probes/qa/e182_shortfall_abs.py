import numpy as np
for r in ['b4','b5','b6']:
    d=dict(np.load(f'n3/000000{r}.npz',allow_pickle=True))
    v=d['v'];ho=d['lat']&~d['pressed']&(np.abs(d['tq_s'])<60)&(d['blink_state']==0)
    kd=d['k_ctrl']-d['k_model']
    yk=np.deg2rad(d['yaw'])/np.maximum(v/3.6,1)
    # straights >=80
    st=ho&(v>=80)&(np.abs(d['k_model'])<1.5e-4)&(np.abs(d['k_meas'])<1.5e-4)
    # deg->curv slope from data
    m=ho&(v>=80)
    sl=np.polyfit(d['ang'][m],d['k_meas'][m],1)[0]
    print(r,'STRAIGHT n_s',st.sum()/100,'kctrl-kmodel e-4',round(kd[st].mean()*1e4,3),'req-ang deg',round((d['cb_req']-d['ang'])[st].mean(),3),'req-mdps',round((d['cb_req']-d['mdps_ang'])[st].mean(),3),'-> (req-ang)*slope e-4',round((d['cb_req']-d['ang'])[st].mean()*sl*1e4,3),'yaw-kmeas e-4',round((yk-d['k_meas'])[st].mean()*1e4,3))
    for nm,lo,hi in [('tight25-60',25,60)]:
        for sgn,sn in [(1,'R'),(-1,'L')]:
            t=ho&(v>=lo)&(v<hi)&(sgn*d['k_meas']>=5e-3)
            if t.sum()<200: print(r,sn,'few');continue
            req=d['cb_req'][t];a=d['ang'][t];ma=d['mdps_ang'][t]
            ay=(yk*(v/3.6)**2)[t]
            print(r,nm,sn,'s',t.sum()/100,'ang',round(a.mean(),1),'ang/req',round(a.mean()/req.mean(),4),'mdps/req',round(ma.mean()/req.mean(),4),'req-ang',round(abs(req.mean())-abs(a.mean()),3),'mdps-ang(abs)',round(abs(ma.mean())-abs(a.mean()),3),'mtq',round(d['mdps_tq'][t].mean(),2),'ay',round(ay.mean(),2),'yaw/kctrl',round(yk[t].mean()/d['k_ctrl'][t].mean(),4),'kmodel/yaw',round(d['k_model'][t].mean()/yk[t].mean(),4),'gain',round(d['gain'][t].mean(),3))
    for sgn,sn in [(1,'R'),(-1,'L')]:
        t=ho&(v>=80)&(sgn*d['k_meas']>=4e-4)
        req=d['cb_req'][t];a=d['ang'][t];ma=d['mdps_ang'][t]
        print(r,'hwy',sn,'s',t.sum()/100,'ang/req',round(a.mean()/req.mean(),4),'req-ang',round(abs(req.mean())-abs(a.mean()),3),'mdps-ang',round(abs(ma.mean())-abs(a.mean()),3),'mtq',round(d['mdps_tq'][t].mean(),2),'yaw/kctrl',round(yk[t].mean()/d['k_ctrl'][t].mean(),4),'kctrl-kmodel e-4',round(kd[t].mean()*1e4,3))
