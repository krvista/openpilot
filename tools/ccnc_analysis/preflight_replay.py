#!/usr/bin/env python3
"""PRE-FLIGHT replay: regenerate op's LKAS_ALT frames from a logged drive with the CURRENT
CarController (real packer), feed the logged CAN RX and the regenerated TX through the real
panda safety code (libsafety), and count what the panda would reject. Run BEFORE flashing.
usage (i6nv3 tree): PYTHONPATH=$PWD:$PWD/opendbc_repo python3 preflight_replay.py <route> <seg,seg,...> [KILL_GOVERNOR=1]"""
import os, sys, glob, collections, zstandard as zstd, capnp, numpy as np
capnp.remove_import_hook()
W=os.getcwd()
IS3 = (not os.path.islink(W+"/openpilot")) and os.path.exists(W+"/openpilot/cereal/log.capnp")
if IS3:
    log = capnp.load(W+"/openpilot/cereal/log.capnp", imports=[W+"/openpilot/cereal", W+"/opendbc_repo/opendbc/car", W])
else:
    from cereal import log   # i6nv2 layout: the package loader wires the capnp imports
from opendbc.safety.tests.libsafety import libsafety_py
from opendbc.safety.tests.libsafety.libsafety_py import make_CANPacket
from opendbc.can.dbc import DBC
from opendbc.can.parser import get_raw_value
from opendbc.can import CANPacker
from opendbc.car.hyundai.values import CarControllerParams as P
import phase_tests.harness as H
if os.environ.get("KILL_GOVERNOR"): P.TX_GOVERNOR=False
dbc=DBC("hyundai_canfd_generated")
def sigs(addr): return {s.name:s for s in dbc.addr_to_msg[addr].sigs.values()}
LK=sigs(0x110); MD=sigs(0xea)
def dec(sig, dat):
    r=get_raw_value(dat,sig)
    if sig.is_signed and r>=(1<<(sig.size-1)): r-=(1<<sig.size)
    return r*sig.factor+sig.offset
route=sys.argv[1]; seg_list=sys.argv[2].split(",")
def load(seg):
    f=sorted(glob.glob(f"/home/user/drivelog/drivelog/*_{route}--*--{seg}--rlog.zst"))[0]
    with open(f,"rb") as fh: return zstd.ZstdDecompressor().stream_reader(fh).read(400_000_000)
def events(buf):
    it=log.Event.read_multiple_bytes(buf)
    while True:
        try: m=next(it)
        except StopIteration: return
        except Exception: return
        try: w=m.which()
        except Exception: continue
        yield w,m
# safety config from the log itself (model, param, alt experience, SP param when the tree has it)
cfg=None; spv=None; alt=None
for w,m in events(load(seg_list[0])):
    if w=="carParams" and cfg is None: cfg=(int(m.carParams.safetyConfigs[0].safetyModel.raw), m.carParams.safetyConfigs[0].safetyParam); alt=m.carParams.alternativeExperience
    if w=="carParamsSP" and spv is None:
        try: spv=m.carParamsSP.safetyParam
        except Exception: spv=0
    if cfg and spv is not None: break
S=libsafety_py.libsafety
def arm_safety():
    if hasattr(S,"set_current_safety_param_sp"): S.set_current_safety_param_sp(spv or 0)
    S.set_alternative_experience(alt or 0); S.set_safety_hooks(cfg[0],cfg[1]); S.init_tests(); S.set_alternative_experience(alt or 0)
    if hasattr(S,"mads_apply_alternative_experience"): S.mads_apply_alternative_experience(alt or 0)
arm_safety()
print(f"safety config from log: {cfg} alt {alt} sp {spv} | mads {S.get_enable_mads() if hasattr(S,'get_enable_mads') else '?'}")
# Boot segments: the log's panda sits in elm327/noOutput with the harness relay CLOSED, so the camera's own 0x110 is seen on
# bus 0 (1600+ frames on a boot segment). Feeding those to an armed libsafety latches relay_malfunction and every later TX is
# rejected (100%) — a tool artifact, not the car. Mirror the log: hooks run only once pandaStates reports the target model.
armed=False; model_name=None
sim=H.Sim(); sim.cc.packer=CANPacker("hyundai_canfd_generated")
t0=None; cc_cmd=0.0; lat=False; en=False; cam=None; mdps2=None; total=collections.Counter(); tx_rej_flag=False
for seg in seg_list:
    cnt=collections.Counter(); lag=[]; diag=[]
    for w,m in events(load(seg)):
        t=m.logMonoTime; t0=t0 or t; S.set_timer(int((t-t0)/1000)&0xFFFFFFFF)
        if w=="pandaStates" and len(m.pandaStates):
            now_on = int(m.pandaStates[0].safetyModel.raw)==cfg[0]
            if now_on and not armed: arm_safety(); armed=True; print(f"   safety armed at t={(t-t0)/1e9:.2f}s (log pandaStates -> model {cfg[0]})")
            elif not now_on and armed: armed=False; print(f"   safety disarmed at t={(t-t0)/1e9:.2f}s (log pandaStates -> {m.pandaStates[0].safetyModel})")
        if not armed and w in ("can","carState"): continue
        if w=="can":
            for c in m.can:
                if c.src>=128: continue
                d=bytes(c.dat); S.safety_rx_hook(make_CANPacket(c.address,c.src,d))
                if c.address==0x110 and c.src==2:
                    cam={k:dec(s,d) for k,s in LK.items()}
                elif c.address==0xea and c.src==1: mdps2=dec(MD["STEERING_ANGLE_2"],d)
        elif w=="carControl": cc_cmd=m.carControl.actuators.steeringAngleDeg; lat=m.carControl.latActive; en=m.carControl.enabled
        elif w=="carState":
            cs=m.carState
            if cam is not None:
                for k,v in cam.items():
                    if k in H.CAM_MSG_TEMPLATE and k!="COUNTER": H.CAM_MSG_TEMPLATE[k]=v
            msgs=sim.step(v=cs.vEgo, tq=cs.steeringTorque, wheel=cs.steeringAngleDeg, cmd=cc_cmd, lat_active=lat, enabled=en,
                          pressed=cs.steeringPressed, blinker=cs.leftBlinker, blinker_right=cs.rightBlinker, bs_l=cs.leftBlindspot, bs_r=cs.rightBlindspot,
                          standstill=cs.standstill, wheel_rate=cs.steeringRateDeg, mdps_angle_2=mdps2, v_raw=cs.vEgoRaw, tx_rejected=tx_rej_flag)
            tx_rej_flag=False
            for addr,dat,bus in msgs:
                if addr!=0x110: continue
                amin=S.get_angle_meas_min(); amax=S.get_angle_meas_max(); dl=S.get_desired_angle_last()
                ok=bool(S.safety_tx_hook(make_CANPacket(addr,bus,bytes(dat))))
                active=((dat[9]>>4)&3)!=1; cnt[("active" if active else "passive","ok" if ok else "REJ")]+=1
                if not ok: tx_rej_flag=True   # closed loop: the src-192 echo the car controller will see next frame
                if not ok and os.environ.get("DIAG") and len(diag)<12:
                    des=(dat[11]<<6)|(dat[10]>>2); des=des-16384 if des>=8192 else des
                    diag.append((round((t-t0)/1e9,2), "A" if active else "P", des, dl, amin, amax, dat[12], round(cs.steeringAngleDeg*10), None if mdps2 is None else round(mdps2*10), lat, sim.s.tx_angle_last))
                if active: lag.append(abs(sim.s.apply_angle_last-sim.s.tx_angle_last))
    tot=sum(cnt.values()); rej=cnt[("active","REJ")]+cnt[("passive","REJ")]
    rm = S.get_relay_malfunction() if hasattr(S,"get_relay_malfunction") else None
    print(f"seg {seg}: frames {tot} | {dict(cnt)} | rejected {100*rej/max(tot,1):.2f}% | wire lag behind internal apply p50/p99 {np.median(lag) if lag else 0:.2f}/{np.percentile(lag,99) if lag else 0:.2f} deg" + (f" | RELAY_MALFUNCTION latched" if rm else ""))
    total.update(cnt)
    for d in diag: print('   REJ', d)
tot=sum(total.values()); rej=total[("active","REJ")]+total[("passive","REJ")]
print(f"TOTAL {route}: frames {tot} rejected {rej} ({100*rej/max(tot,1):.2f}%) governor={'OFF' if os.environ.get('KILL_GOVERNOR') else 'ON'}")
