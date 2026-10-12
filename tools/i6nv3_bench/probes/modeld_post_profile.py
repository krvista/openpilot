"""CPU cost of stock modeld's post-model path per frame (x86): slice+parse, action, fill_model_msg, DesireHelper, RELC,
fill_driving_model_data, fill_pose_msg, serialization. Random raw outputs with the real CD210 output_slices."""
import pickle, sys, time, cProfile, pstats, io
import numpy as np
import openpilot.cereal.messaging as messaging
from openpilot.cereal import log
from openpilot.selfdrive.modeld.parse_model_outputs import Parser
from openpilot.selfdrive.modeld.fill_model_msg import fill_model_msg, fill_driving_model_data, fill_pose_msg, PublishState
from openpilot.selfdrive.modeld.modeld import get_action_from_model
from openpilot.selfdrive.controls.lib.desire_helper import DesireHelper
from openpilot.sunnypilot.selfdrive.controls.lib.relc import RoadEdgeLaneChangeController
meta = pickle.load(open(sys.argv[1], 'rb')); sl = meta['output_slices']; N = meta['output_shapes']['outputs'][1]
rng = np.random.default_rng(0)
parser = Parser(); ps = PublishState(); DH = DesireHelper(); RELC = RoadEdgeLaneChangeController()
from opendbc.car.structs import car as _car
cs = _car.CarState.new_message(); cs.vEgo = 20.0
prev = log.ModelDataV2.Action()
stages = {k: 0.0 for k in ("slice+parse", "action", "fill_model_msg", "desire+relc", "fill_dmd+pose", "serialize")}
def frame(raw):
  global prev
  t0 = time.perf_counter()
  out = parser.parse_outputs({k: raw[np.newaxis, v] for k, v in sl.items()})
  t1 = time.perf_counter()
  a = get_action_from_model(out, prev, 0.3, 0.5, 20.0); prev = a
  t2 = time.perf_counter()
  mv2 = messaging.new_message('modelV2'); dmd = messaging.new_message('drivingModelData'); pose = messaging.new_message('cameraOdometry')
  fill_model_msg(mv2, out, a, ps, 1, 1, 1, 0.0, 0, 0.02, True)
  t3 = time.perf_counter()
  ds = mv2.modelV2.meta.desireState; p = ds[log.Desire.laneChangeLeft] + ds[log.Desire.laneChangeRight]
  DH.update(cs, True, p); sp = messaging.new_message('modelDataV2SP'); RELC.update_and_fill(mv2.modelV2, sp.modelDataV2SP, 20.0)
  t4 = time.perf_counter()
  fill_driving_model_data(dmd, mv2); fill_pose_msg(pose, out, 1, 0, 0, True)
  t5 = time.perf_counter()
  for m in (mv2, dmd, pose, sp): m.to_bytes()
  t6 = time.perf_counter()
  for k, d in zip(stages, (t1-t0, t2-t1, t3-t2, t4-t3, t5-t4, t6-t5)): stages[k] += d
raws = [rng.standard_normal(N).astype(np.float32) for _ in range(50)]
for r in raws[:10]: frame(r)
for k in stages: stages[k] = 0.0
n = 400
for i in range(n): frame(raws[i % 50])
tot = sum(stages.values())
print(f"per frame total {tot/n*1e3:.3f} ms (x86)")
for k, v in stages.items(): print(f"  {k:16} {v/n*1e3:.3f} ms  {v/tot:5.1%}")
pr = cProfile.Profile(); pr.enable()
for i in range(200): frame(raws[i % 50])
pr.disable(); s = io.StringIO(); pstats.Stats(pr, stream=s).sort_stats('tottime').print_stats(18); print(s.getvalue()[:4000])
