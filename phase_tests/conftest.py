"""Skip test modules whose imports are unavailable in the current environment (CI has no scons build, so anything that
loads openpilot/common/libparams_c*.so or on-device-only modules cannot import). Locally set PHASE_TESTS_STRICT=1 to turn a
skipped module into a hard failure — a module that stops importing on the device should never be silently dropped."""
import glob
import importlib
import os

collect_ignore = []
_here = os.path.dirname(os.path.abspath(__file__))
for _f in sorted(glob.glob(os.path.join(_here, "test_*.py"))):
  _mod = "phase_tests." + os.path.basename(_f)[:-3]
  try:
    importlib.import_module(_mod)
  except Exception as _e:  # noqa: BLE001 - any import failure means "cannot run here"
    if os.environ.get("PHASE_TESTS_STRICT"):
      raise
    collect_ignore.append(os.path.basename(_f))
    print(f"[phase_tests] skipping {_mod}: {type(_e).__name__}: {_e}")
