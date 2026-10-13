"""Phase 53 (report §50): comma 4 UI at 20 fps (was 30, CPU-bound at 24), and the vblank-skip branch no longer yields to
the consumer loop — ui_state.update and every timer counted in target_fps frames run once per rendered frame instead of
60 times a second (they ran 2x fast at 30 fps, would be 3x at 20)."""
import os
import re

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
SRC = open(os.path.join(ROOT, "openpilot/system/ui/lib/application.py")).read()   # parsed: pyray cannot open a window headless


def test_mici_default_is_20_fps():
  m = re.search(r"^_DEFAULT_FPS = int\(os\.getenv\(\"FPS\", (\{[^}]*\})", SRC, re.M)
  table = eval(m.group(1))   # noqa: S307 - a literal dict from our own source
  assert table["mici"] == 20 and table["tizi"] == 20


def test_vblank_skip_renders_every_third_vblank_at_20():
  assert "self._vblank_skip = int(round(60 / fps)) - 1 if (vblank_control and not OFFSCREEN and fps < 60) else 0" in SRC
  assert int(round(60 / 20)) - 1 == 2


def test_skip_branch_does_not_yield():
  i = SRC.index("if self._vblank_skip and (self._frame % (self._vblank_skip + 1)) != 0 and self._should_render:")
  branch = SRC[i:SRC.index("continue", i) + len("continue")]
  assert "yield" not in branch and "self._frame += 1" in branch and "time.sleep" in branch
  assert "_mouse_events = []" not in branch      # events stay queued for the rendered frame
