#!/usr/bin/env python3
"""Panda-in-the-loop replay: feed a route's logged CAN RX into the real libsafety rx hook and the
logged sendcan TX into the tx hook, in log order with the log clock; compare predicted rejections
with the echoes the car saw (src 128 accepted / 192 rejected). State carries across segments.
usage (i6nv3 tree): PYTHONPATH=$PWD:$PWD/opendbc_repo python3 panda_replay.py <route> <seg,seg,...> [sp_param]"""
import sys, glob, collections, zstandard as zstd, capnp
capnp.remove_import_hook()
W="/tmp/claude-0/-home-user-openpilot/b24e552a-84af-537a-8cbb-81fda69ed9c1/scratchpad/wt3"
log = capnp.load(W+"/openpilot/cereal/log.capnp", imports=[W+"/openpilot/cereal", W+"/opendbc_repo/opendbc/car", W])
from opendbc.safety.tests.libsafety import libsafety_py
from opendbc.safety.tests.libsafety.libsafety_py import make_CANPacket
route = sys.argv[1]; seg_list = sys.argv[2].split(",")
sp_param = int(sys.argv[3]) if len(sys.argv) > 3 else None
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
        yield w, m
cfg=None; spv=None; alt=None
for w,m in events(load(seg_list[0])):
    if w=="carParams" and cfg is None: cfg=(int(m.carParams.safetyConfigs[0].safetyModel.raw), m.carParams.safetyConfigs[0].safetyParam); alt=m.carParams.alternativeExperience
    if w=="carParamsSP" and spv is None: spv=m.carParamsSP.safetyParam
    if cfg and spv is not None: break
sp = sp_param if sp_param is not None else (spv or 0)
print(f"log safety config: model {cfg} alt_exp {alt} carParamsSP.safetyParam {spv} -> using sp {sp}")
S=libsafety_py.libsafety
S.set_current_safety_param_sp(sp); S.set_alternative_experience(alt or 0)
S.set_safety_hooks(cfg[0], cfg[1]); S.init_tests(); S.set_alternative_experience(alt or 0); S.mads_apply_alternative_experience(alt or 0)
print("enable_mads:", S.get_enable_mads())
t0=None
for seg in seg_list:
    pred=collections.Counter(); actual=collections.Counter(); mism=collections.Counter(); n_tx=0; pending={}; detail=[]
    for w,m in events(load(seg)):
        t=m.logMonoTime; t0=t0 or t
        S.set_timer(int((t-t0)/1000) & 0xFFFFFFFF)
        if w=="can":
            for c in m.can:
                if c.src < 128:
                    S.safety_rx_hook(make_CANPacket(c.address, c.src, bytes(c.dat)))
                elif c.address==0x110:
                    key=bytes(c.dat)
                    if key in pending:
                        p=pending.pop(key); a=(c.src==128); actual[a]+=1
                        if p!=a: mism[(p,a)]+=1; detail.append((round((t-t0)/1e9,2), p, a))
        elif w=="sendcan":
            for c in m.sendcan:
                ok=bool(S.safety_tx_hook(make_CANPacket(c.address, c.src, bytes(c.dat))))
                if c.address==0x110: n_tx+=1; pred[ok]+=1; pending[bytes(c.dat)]=ok
    print(f"seg {seg}: TX 0x110 {n_tx} | predicted ok {pred[True]} rej {pred[False]} | actual ok {actual[True]} rej {actual[False]} | mismatches {dict(mism)} | ctrl {S.get_controls_allowed()} lat {S.get_controls_allowed_lateral()}")
    for d in detail[:3]: print("   mismatch", d)
