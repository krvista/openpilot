"""Harness for the NON-driving-control custom code (ldw, mads, controlsd helpers).

The device native modules (params_pyx / msgq ipc_pyx) are aarch64 builds that
cannot load on a development host, and cereal.messaging drags them in — so the
few leaf modules the code under test only touches at *runtime* are stubbed at
import time.  Everything under test (LaneDepartureWarning, the MADS event/state
machines, controlsd's _lookahead_curvature / _predicted_lat_accel_excess) is
the real code.
"""
import os
import sys
import types

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for p in (os.path.join(REPO, 'opendbc_repo'), REPO):
  if p not in sys.path:
    sys.path.insert(0, p)


class FakeParams:
  """In-memory Params. Class-level store so Params() inside the code under
  test observes values a test primed via FakeParams().put(...)."""
  store: dict = {}

  def __init__(self, *a, **k):
    pass

  def get(self, k, block=False, return_default=False):
    return self.store.get(k)

  def get_bool(self, k, block=False):
    return bool(self.store.get(k, False))

  def put(self, k, v):
    self.store[k] = v

  def put_bool(self, k, v):
    self.store[k] = bool(v)

  @classmethod
  def reset(cls):
    cls.store.clear()


def _install_stubs():
  if 'openpilot.common.params_pyx' in sys.modules:
    return

  ppx = types.ModuleType('openpilot.common.params_pyx')
  ppx.Params = FakeParams
  ppx.ParamKeyFlag = types.SimpleNamespace(ALL=0)
  ppx.ParamKeyType = types.SimpleNamespace(ALL=0)

  class UnknownKeyName(Exception):
    pass

  ppx.UnknownKeyName = UnknownKeyName
  sys.modules['openpilot.common.params_pyx'] = ppx

  cm = types.ModuleType('cereal.messaging')

  class _Sock:
    def __init__(self, *a, **k):
      pass

    def __getattr__(self, n):
      return lambda *a, **k: None

  cm.SubMaster = _Sock
  cm.PubMaster = _Sock
  cm.new_message = lambda *a, **k: None
  cm.log_from_bytes = lambda *a, **k: None
  sys.modules['cereal.messaging'] = cm

  class _AnyModule(types.ModuleType):
    def __getattr__(self, name):
      if name.startswith('__'):
        raise AttributeError(name)
      v = type(name, (), {})
      setattr(self, name, v)
      return v

  for m in ('msgq', 'msgq.ipc_pyx', 'msgq.visionipc', 'msgq.visionipc.visionipc_pyx'):
    sys.modules[m] = _AnyModule(m)


_install_stubs()
