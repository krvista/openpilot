"""E95a: cache the lagd input messages (carControl, carState, controlsState, extrinsicsCalibration, deviceMotion) of a
route as raw event bytes, so lagd can be replayed offline with different settings (e95_lagd_replay.py)."""
import glob, pickle, sys
from openpilot.tools.lib.logreader import LogReader
D, OUT = sys.argv[1], sys.argv[2]
W = {"carControl", "carState", "controlsState", "extrinsicsCalibration", "deviceMotion"}
for r in sys.argv[3:]:
  fs = sorted(glob.glob(f"{D}/*_{r}--*--rlog.zst"), key=lambda f: int(f.split("--")[-2]))
  ev = []
  for f in fs:
    try:
      for m in LogReader(f):
        if m.which() in W:
          ev.append((m.logMonoTime, m.which(), m.as_builder().to_bytes()))
    except Exception as e:  # noqa: BLE001
      print(r, f, e)
  ev.sort(key=lambda x: x[0])
  pickle.dump(ev, open(f"{OUT}/{r}.pkl", "wb"))
  print(r, len(fs), "segs", len(ev), "events", flush=True)
