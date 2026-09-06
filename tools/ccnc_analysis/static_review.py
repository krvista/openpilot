#!/usr/bin/env python3
"""Static review of key / attribute / dimension / exception assumptions for one tree
(i6nv2 or i6nv3 layout). Motivated by the first i6nv3 road test: selfdrived died on
sm.updated['audioFeedback'] (a service DEPRECATED in the new base) — a class of bug
no unit test exercises. Checks: sm/pm keys vs services.py; enum/flag/class members;
capnp field chains on known message variables; np.interp table pairs and *_SPEEDS/*_V
naming pairs; Params keys vs params_keys.h; hard-coded model indices and their guards;
dict lookups; every except clause (listed for manual review). Exit 1 on any issue.
usage: cd <tree>; PYTHONPATH=<tree>:<tree>/opendbc_repo python3 tools/ccnc_analysis/static_review.py"""
import os, re, sys, glob, dataclasses, importlib
ROOT=os.getcwd(); IS3=(not os.path.islink("openpilot")) and os.path.exists("openpilot/cereal/services.py")
SD="openpilot/selfdrive" if IS3 else "selfdrive"; SP="openpilot/sunnypilot" if IS3 else "sunnypilot"; CER="openpilot/cereal" if IS3 else "cereal"
FILES=[f"{SD}/selfdrived/selfdrived.py",f"{SD}/controls/controlsd.py",f"{SD}/controls/plannerd.py",f"{SD}/controls/lib/desire_helper.py",f"{SD}/controls/lib/ldw.py",
       f"{SD}/controls/lib/bsm_guard.py",f"{SD}/modeld/modeld.py",f"{SD}/car/card.py",f"{SD}/car/helpers.py",f"{SP}/selfdrive/controls/controlsd_ext.py",
       f"{SP}/selfdrive/controls/lib/auto_lane_change.py",f"{SP}/selfdrive/controls/lib/lane_turn_desire.py",
       "opendbc_repo/opendbc/car/hyundai/carcontroller.py","opendbc_repo/opendbc/car/hyundai/carstate.py","opendbc_repo/opendbc/car/hyundai/values.py",
       "opendbc_repo/opendbc/car/hyundai/interface.py","opendbc_repo/opendbc/car/hyundai/hyundaicanfd.py"]
FILES=[f for f in FILES if os.path.exists(f)]
def _strip(t):
    out=[]
    for l in t.splitlines():
        q=l.find("#")
        out.append(l if q<0 or l.count('"',0,q)%2 or l.count("'",0,q)%2 else l[:q])
    return "\n".join(out)+"\n"
src={f:_strip(open(f).read()) for f in FILES}
def where(f,pat):
    return [i+1 for i,l in enumerate(src[f].splitlines()) if re.search(pat,l)]
issues=[]; notes=[]
# ---- A. services ----
services=set(re.findall(r'^\s*"(\w+)":', open(f"{CER}/services.py").read(), re.M))
for f in FILES:
    for k in set(re.findall(r"\bsm(?:\.updated|\.valid|\.alive|\.recv_frame|\.recv_time|\.freq_ok|\.seen|\.logMonoTime)?\['(\w+)'\]", src[f])):
        if k not in services: issues.append(("A-service",f,where(f,rf"\['{k}'\]")[:2],f"sm['{k}'] not in services.py"))
    for k in set(re.findall(r"\bpm\.send\('(\w+)'", src[f])):
        if k not in services: issues.append(("A-service",f,where(f,rf"pm\.send\('{k}'")[:2],f"pm.send('{k}') not in services.py"))
# ---- B. enum / class attribute existence ----
sys.path.insert(0,ROOT); sys.path.insert(0,os.path.join(ROOT,"opendbc_repo"))
from opendbc.car.hyundai.values import HyundaiFlags, Buttons, CarControllerParams, CAR
from opendbc.car import structs, Bus
try:
    from opendbc.safety import HyundaiSafetyFlags
except Exception:
    from opendbc.car.hyundai.values import HyundaiSafetyFlags
cereal = importlib.import_module("openpilot.cereal" if IS3 else "cereal")
log = cereal.log; custom = cereal.custom
EventName = log.OnroadEvent.EventName
enums = {"EventNameSP":set(custom.OnroadEventSP.EventName.schema.enumerants), "HyundaiFlags":set(HyundaiFlags.__members__), "Buttons":set(getattr(Buttons,"__members__",None) or [n for n in dir(Buttons) if not n.startswith("_")]), "CarControllerParams":set(dir(CarControllerParams)),
         "EventName":set(EventName.schema.enumerants), "log.Desire":set(log.Desire.schema.enumerants), "LaneChangeState":set(log.LaneChangeState.schema.enumerants),
         "LaneChangeDirection":set(log.LaneChangeDirection.schema.enumerants), "CAR":set(getattr(CAR,"__members__",None) or [n for n in dir(CAR) if not n.startswith("_")]), "Bus":set(getattr(Bus,"__members__",None) or [n for n in dir(Bus) if not n.startswith("_")]),
         "HyundaiSafetyFlags":set(getattr(HyundaiSafetyFlags,"__members__",None) or [n for n in dir(HyundaiSafetyFlags) if not n.startswith("_")]), "custom":set(dir(custom)), "structs":set(dir(structs))}
try: enums["TurnDirection"]=set(custom.ModelDataV2SP.TurnDirection.schema.enumerants)
except Exception as e: enums["TurnDirection"]=None
pat={"HyundaiFlags":r"HyundaiFlags\.(\w+)","Buttons":r"Buttons\.(\w+)","CarControllerParams":r"CarControllerParams\.(\w+)","EventName":r"(?<!OnroadEventSP\.)EventName\.(\w+)","EventNameSP":r"OnroadEventSP\.EventName\.(\w+)",
     "log.Desire":r"log\.Desire\.(\w+)","LaneChangeState":r"LaneChangeState\.(\w+)","LaneChangeDirection":r"LaneChangeDirection\.(\w+)","CAR":r"\bCAR\.(\w+)",
     "Bus":r"\bBus\.(\w+)","HyundaiSafetyFlags":r"HyundaiSafetyFlags\.(\w+)","TurnDirection":r"TurnDirection\.(\w+)","custom":r"\bcustom\.(\w+)"}
for f in FILES:
    for name,rx in pat.items():
        if enums.get(name) is None: continue
        for m in set(re.findall(rx, src[f])):
            if m in ("value","raw","schema","new_message","__members__"): continue
            if name=="CAR" and hasattr(CAR,m): continue
            if m not in enums[name]: issues.append(("B-member",f,where(f,rf"{name.split('.')[-1]}\.{m}\b")[:2],f"{name}.{m} does not exist"))
# ---- C. capnp field access on known variables ----
def struct_fields(schema):
    try: return set(schema.fields.keys())
    except Exception: return None
ev_fields=log.Event.schema.fields
def field_schema(sch, name):
    fl=sch.fields[name].proto
    if fl.which()=="slot":
        t=fl.slot.type
        if t.which()=="struct": return sch.fields[name].schema
        if t.which()=="list": return ("list", None)
    return None
VARMAP={"CS":"carState","CC":"carControl","co":"carOutput","CO":"carOutput","model_v2":"modelV2","modelV2":"modelV2","lp":"liveParameters","ss":"selfdriveState",
        "sm['carState']":"carState","sm['modelV2']":"modelV2","sm['carOutput']":"carOutput","sm['carControl']":"carControl","sm['controlsState']":"controlsState",
        "sm['driverAssistance']":"driverAssistance","sm['selfdriveStateSP']":"selfdriveStateSP","sm['radarState']":"radarState","sm['liveParameters']":"liveParameters",
        "sm['longitudinalPlan']":"longitudinalPlan","sm['deviceState']":"deviceState","sm['managerState']":"managerState","sm['lateralPlan']":"lateralPlan"}
def check_chain(f, root_svc, chain, lineno):
    try: sch=ev_fields[root_svc].schema
    except Exception: return
    for a in chain:
        if a in ("which","to_dict","as_reader","as_builder","schema","raw","secoc_key"): return
        try: fields=set(sch.fields.keys())
        except Exception: return
        if a not in fields:
            issues.append(("C-capnp",f,[lineno],f"{root_svc}.{'.'.join(chain)} — field '{a}' not in {sch.node.displayName.split(':')[-1]}")); return
        fl=sch.fields[a].proto
        if fl.which()=="slot" and fl.slot.type.which()=="struct": sch=sch.fields[a].schema
        else: return
for f in FILES:
    if not (f.startswith(SD) or f.startswith(SP)): continue
    for i,l in enumerate(src[f].splitlines(),1):
        for m in re.finditer(r"(?:self\.)?(sm\['\w+'\]|\bCS\b|\bCC\b|\bco\b|\bmodel_v2\b|\bmodelV2\b|\bss\b|\blp\b)((?:\.\w+)+)", l):
            var=m.group(1); chain=m.group(2).lstrip(".").split(".")
            if var.startswith("sm["): key="sm["+var[3:]
            else: key=var
            svc=VARMAP.get(key)
            if not svc or key=="lp" and "liveParameters" not in src[f]: continue
            if var=="CS" and f.startswith("opendbc"): continue
            check_chain(f, svc, chain, i)
# opendbc CS.out.<field> vs structs.CarState dataclass
cs_fields={n for n in dir(structs.CarState()) if not n.startswith("_")}
cc_fields={n for n in dir(structs.CarControl()) if not n.startswith("_")}
for f in FILES:
    if not f.startswith("opendbc"): continue
    for i,l in enumerate(src[f].splitlines(),1):
        for m in re.finditer(r"CS\.out\.(\w+)", l):
            if m.group(1) not in cs_fields: issues.append(("C-struct",f,[i],f"CS.out.{m.group(1)} not a structs.CarState field"))
        for m in re.finditer(r"\bCC\.(\w+)", l):
            if m.group(1) not in cc_fields and m.group(1) not in ("actuators","hudControl","cruiseControl"): issues.append(("C-struct",f,[i],f"CC.{m.group(1)} not a structs.CarControl field"))
# ---- D. np.interp table pairs ----
for f in FILES:
    for i,l in enumerate(src[f].splitlines(),1):
        for m in re.finditer(r"np\.interp\([^,]+,\s*(?:CarControllerParams|P|self\.params)\.(\w+),\s*(?:CarControllerParams|P|self\.params)\.(\w+)", l):
            a,b=m.groups()
            try:
                la,lb=len(getattr(CarControllerParams,a)),len(getattr(CarControllerParams,b))
                if la!=lb: issues.append(("D-interp",f,[i],f"{a}({la}) vs {b}({lb}) length mismatch"))
            except Exception as e: issues.append(("D-interp",f,[i],f"{a}/{b}: {type(e).__name__}"))
        for m in re.finditer(r"np\.interp\([^,\[]+,\s*\[([^\]]*)\],\s*\[([^\]]*)\]", l):
            la=len([x for x in m.group(1).split(",") if x.strip()]); lb=len([x for x in m.group(2).split(",") if x.strip()])
            if la!=lb: issues.append(("D-interp",f,[i],f"inline interp lengths {la} vs {lb}"))
# CarControllerParams *_BP/_SPEEDS vs *_V pairs by naming convention
names=[n for n in dir(CarControllerParams) if not n.startswith("_")]
for n in names:
    for suf_bp,suf_v in (("_SPEEDS_KPH","_V"),("_SPEEDS_MS","_V"),("_BP","_V"),("_SPEEDS_KPH","_NM")):
        if n.endswith(suf_bp):
            base=n[:-len(suf_bp)]
            cands=[c for c in names if c==base+suf_v]
            for c in cands:
                try:
                    if len(getattr(CarControllerParams,n))!=len(getattr(CarControllerParams,c)): issues.append(("D-table","values.py",[],f"{n}({len(getattr(CarControllerParams,n))}) vs {c}({len(getattr(CarControllerParams,c))})"))
                except TypeError: pass
# ---- E. Params keys ----
pk=open(("openpilot/common/params_keys.h" if IS3 else "common/params_keys.h")).read(); pkeys=set(re.findall(r'\{"(\w+)",', pk))
for f in FILES:
    for m in set(re.findall(r'(?:self\.params|\bparams|Params\(\))\.(?:get|get_bool|put|put_bool|get_int|get_float|remove)\(\s*"(\w+)"', src[f])):
        if m not in pkeys: issues.append(("E-params",f,where(f,rf'"{m}"')[:2],f"Params key {m} not in params_keys.h"))
# ---- F. hard-coded model array indices (list for manual guard review) ----
idx=[]
for f in FILES:
    for i,l in enumerate(src[f].splitlines(),1):
        if re.search(r"laneLines\[\d\]|laneLineProbs\[\d\]|\.y\[\d+\]|\.x\[\d+\]|desireState\[|roadEdges\[\d\]|\.t\[\d+\]", l) and not l.strip().startswith("#"):
            ctx="\n".join(src[f].splitlines()[max(0,i-6):i-1])
            guarded=bool(re.search(r"len\(|try:", ctx))
            idx.append((f,i,l.strip()[:110],guarded))
# ---- G. dict lookups ----
for f in FILES:
    for m in set(re.findall(r"\b(DESIRES|TURN_DESIRES|AUTO_LANE_CHANGE_TIMER)\[", src[f])):
        notes.append(("G-dict",f,where(f,rf"\b{m}\[")[:3],f"dict lookup {m}[...] — verify key coverage"))
# ---- H. except clauses ----
exc=[]
for f in FILES:
    for i,l in enumerate(src[f].splitlines(),1):
        m=re.match(r"\s*except(\s+[^:]+)?:", l)
        if m:
            nxt=src[f].splitlines()[i] if i < len(src[f].splitlines()) else ""
            exc.append((f,i,(m.group(1) or "<bare>").strip(),nxt.strip()[:60]))
print(f"tree={ROOT} IS3={IS3} files={len(FILES)}")
RC = 1 if issues else 0
print("\n## ISSUES"); [print(" ",*x) for x in sorted(issues)] if issues else print("  none")
print("\n## notes (manual)"); [print(" ",*x) for x in sorted(notes)]
print("\n## hard-coded model indices (guarded?)"); [print(f"  {'OK ' if g else 'UNGUARDED'} {f}:{i}: {l}") for f,i,l,g in idx]
print("\n## except clauses"); [print(f"  {f}:{i}: except {t} -> {n}") for f,i,t,n in exc]
sys.exit(RC)
