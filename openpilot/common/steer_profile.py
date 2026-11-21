"""i6n Phase 72 (report §96-97): which steering profile this drive uses.

solo    (default) openpilot steers alone. The factory LFA is kept switched off while openpilot is in charge, so the
        vehicle's ADAS controller relays our angle request to the MDPS 1:1; the request is the model's action through
        the plain angle path (no lead / trim / boost layers - those were fitted to a wheel that did not follow, which
        was the factory LFA holding it), and the lateral delay is learned for that plant (about 0.3 s).
legacy  the pre-72 behaviour, bit for bit: the factory LFA is switched on together with MADS and steers the car while
        it is active; the Phase 6-69 stack runs. Developer toggle 'stock LFA with op (old)' (Params StockLfaWithOp).

Kill switch for the whole phase: SOLO_DEFAULT = False (always legacy, whatever the toggle says).
controlsd latches the profile (ProfileLatch) and hands it to the car controller in CarControlSP.stockLfaWithOp; lagd
reads the toggle itself."""

PARAM = "StockLfaWithOp"
SOLO_DEFAULT = True
PARAM_PERIOD_FRAMES = 300   # the toggle is re-read every 3 s at 100 Hz


def legacy_requested(params) -> bool:
  """True when this drive should keep the pre-72 behaviour."""
  if not SOLO_DEFAULT:
    return True
  try:
    return bool(params.get_bool(PARAM))
  except Exception:  # noqa: BLE001 - a param read must never take controls down; solo is the default
    return False


class ProfileLatch:
  """Re-reads the toggle every `period` updates and lets the profile change only while lateral control is NOT active,
  so the request never steps between the two stacks mid-corner."""

  def __init__(self, params, period: int = PARAM_PERIOD_FRAMES):
    self._params = params
    self._period = max(int(period), 1)
    self._n = 0
    self.requested = legacy_requested(params)
    self.legacy = self.requested

  def update(self, lat_active: bool) -> bool:
    if self._n % self._period == 0:
      self.requested = legacy_requested(self._params)
    self._n += 1
    if not lat_active:
      self.legacy = self.requested
    return self.legacy
