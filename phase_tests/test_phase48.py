"""Phase 48: modeld_v2 must not make tinygrad pick a device at import (report §35).

A `Device.DEFAULT` default argument in sunnypilot/modeld_v2/compile_modeld.py was evaluated at import and made tinygrad
probe METAL/AMD/NV/CUDA before QCOM — 6-11 s of every boot on the device. The defaults are now None, resolved on use."""
import ast
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
SRC = os.path.join(ROOT, "openpilot", "sunnypilot", "modeld_v2", "compile_modeld.py")


def test_no_device_default_in_signatures():
  tree = ast.parse(open(SRC).read())
  for node in ast.walk(tree):
    if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.Lambda)):
      for d in node.args.defaults + [d for d in node.args.kw_defaults if d is not None]:
        assert "Device" not in ast.unparse(d), f"{getattr(node, 'name', 'lambda')}: default {ast.unparse(d)} runs at import"


def test_import_does_not_select_a_device():
  code = (
    "import sys, types; sys.modules.setdefault('jwt', types.ModuleType('jwt'))\n"
    "from phase_tests.harness_noncontrol import FakeParams\n"
    "import openpilot.sunnypilot.modeld_v2.compile_modeld as c\n"
    "from tinygrad.device import Device\n"
    "print('selected', '_select_device' in Device.__dict__)\n"
  )
  env = dict(os.environ, PYTHONPATH=os.pathsep.join([ROOT, os.path.join(ROOT, "opendbc_repo"), os.path.join(ROOT, "openpilot"),
                                                     os.environ.get("PYTHONPATH", "")]))
  env.pop("DEV", None)
  r = subprocess.run([sys.executable, "-c", code], env=env, capture_output=True, text=True, timeout=300, cwd=ROOT)
  assert r.returncode == 0, r.stderr[-2000:]
  assert "selected False" in r.stdout


def test_none_device_resolves_to_default_on_use():
  src = open(SRC).read()
  start = src.index("def generate_queues_and_npy(")
  assert "device = device or Device.DEFAULT" in src[start:src.index("road_key", start)]
