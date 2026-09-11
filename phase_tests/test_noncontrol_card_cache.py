"""card seeds fingerprinting from CarParamsPersistent when CarParamsCache (CLEAR_ON_MANAGER_START) is empty."""
from phase_tests.harness import make_cp  # noqa: F401 (path setup side effect)
from openpilot.selfdrive.car.card import load_cached_params_raw


class _P:
  def __init__(self, d):
    self.d = d

  def get(self, k):
    return self.d.get(k)


def test_cache_preferred():
  assert load_cached_params_raw(_P({"CarParamsCache": b"c", "CarParamsPersistent": b"p"})) == b"c"


def test_persistent_fallback():
  assert load_cached_params_raw(_P({"CarParamsPersistent": b"p"})) == b"p"


def test_nothing():
  assert load_cached_params_raw(_P({})) is None
