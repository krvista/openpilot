"""comma 4 (MICI) Developer screen: Quickboot Mode toggle — the file/param logic, exercised with the UI toolkit
(pyray) and the widget stack stubbed, since neither is importable off-device."""
import importlib
import os
import sys
import tempfile
import types
from unittest.mock import MagicMock

import pytest

STUB_PREFIXES = ("pyray", "openpilot.system.ui", "openpilot.selfdrive.ui.mici.widgets", "openpilot.selfdrive.ui.widgets",
                 "openpilot.selfdrive.ui.layouts.settings.common", "openpilot.selfdrive.ui.ui_state")


class _StubFinder(importlib.abc.MetaPathFinder, importlib.abc.Loader):
  def find_spec(self, name, path, target=None):
    if name.startswith(STUB_PREFIXES):
      return importlib.machinery.ModuleSpec(name, self, is_package=True)
    return None

  def create_module(self, spec):
    m = MagicMock(); m.__path__ = []; m.__spec__ = spec; return m

  def exec_module(self, module):
    pass


@pytest.fixture
def dev(monkeypatch):
  for k in [k for k in sys.modules if k.startswith(STUB_PREFIXES) or k == "openpilot.selfdrive.ui.mici.layouts.settings.developer"]:
    monkeypatch.delitem(sys.modules, k)
  finder = _StubFinder(); monkeypatch.setattr(sys, "meta_path", [finder] + sys.meta_path)
  import openpilot.system.ui.widgets.scroller as scroller   # the layout subclasses NavScroller: give it a real base class
  scroller.NavScroller = type("NavScroller", (), {"__init__": lambda self: None, "_update_state": lambda self: None,
                                                  "show_event": lambda self: None})
  import openpilot.selfdrive.ui.mici.layouts.settings.developer as dev
  return dev


class _Params:
  def __init__(self): self.d = {}
  def put_bool(self, k, v, block=False): self.d[k] = bool(v)
  def get_bool(self, k): return self.d.get(k, False)


class _Toggle:
  def __init__(self): self.checked = None
  def set_checked(self, v): self.checked = v


def test_toggle_creates_and_removes_prebuilt_and_mirrors_param(dev, tmp_path):
  dev.PREBUILT_PATH = str(tmp_path / "prebuilt")
  dev.ui_state.params = _Params()
  me = types.SimpleNamespace(_quickboot_toggle=_Toggle())
  dev.DeveloperLayoutMici._on_quickboot_toggled(me, True)
  assert os.path.exists(dev.PREBUILT_PATH) and dev.ui_state.params.get_bool("QuickBootToggle")
  dev.DeveloperLayoutMici._on_quickboot_toggled(me, False)
  assert not os.path.exists(dev.PREBUILT_PATH) and not dev.ui_state.params.get_bool("QuickBootToggle")
  dev.DeveloperLayoutMici._on_quickboot_toggled(me, False)      # off with no file: no error
  assert not os.path.exists(dev.PREBUILT_PATH)


def test_unwritable_path_reports_and_resets_toggle(dev):
  dev.PREBUILT_PATH = "/proc/does-not-exist/prebuilt"
  dev.ui_state.params = _Params()
  me = types.SimpleNamespace(_quickboot_toggle=_Toggle())
  dev.DeveloperLayoutMici._on_quickboot_toggled(me, True)
  assert me._quickboot_toggle.checked is False                  # reset to the on-disk truth
  assert not dev.ui_state.params.get_bool("QuickBootToggle")    # param untouched on failure
  dev.gui_app.push_widget.assert_called_once()                  # dialog shown


def test_toggle_is_in_the_scroller_and_default_state_follows_the_file(dev):
  src = open(dev.__file__).read()
  assert "self._quickboot_toggle," in src and "initial_state=os.path.exists(PREBUILT_PATH)" in src
  assert dev.PREBUILT_PATH.endswith("prebuilt")
