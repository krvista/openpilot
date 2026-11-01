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


import pytest  # noqa: E402


@pytest.fixture(autouse=True)
def _phase59_off_for_legacy_tests(request):
  """Phase 59 (MDPS inverse-model compensator) closes the loop on the measured wheel. The legacy phase tests drive a
  synthetic wheel that does not respond to the request (stuck / scripted), on which the compensator would integrate to its
  clamp — they keep testing the layers as they behave with Phase 59 killed (MDPS_COMP_LAMBDA = 0, the fallback path).
  Modules that test Phase 59 set USE_MDPS_COMP = True and drive a plant that responds to the request."""
  if getattr(request.module, "USE_MDPS_COMP", False):
    yield
    return
  from opendbc.car.hyundai.values import CarControllerParams as P
  old = P.MDPS_COMP_LAMBDA
  P.MDPS_COMP_LAMBDA = 0.0
  try:
    yield
  finally:
    P.MDPS_COMP_LAMBDA = old
