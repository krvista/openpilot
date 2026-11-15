"""i6n Phase 70 (report §92): stock-LFA capture mode.

The drive logs hold no frame of the factory LFA actually steering this MDPS (§91.5: every camera 0x110 frame in them is
a standby echo of the wheel angle). To record one, a single drive can be run with openpilot passive: card marks the car
passive, so the panda stays in noOutput, the harness relay leaves the factory camera connected to the car and the factory
LFA / HDA2 work as built, while logging goes on as on any drive (the "Dashcam Mode" alert is shown).

The mode is for ONE drive: the param is CLEAR_ON_MANAGER_START (a boot never starts in it) and manager removes it on the
falling edge of the ignition. It is set from the comma 4 Developer screen (the toggle after "quickboot mode"), which asks
for an onroad cycle so it takes effect at once when the car is on.

Kill: STOCK_LFA_CAPTURE = False (the param is then ignored everywhere)."""

STOCK_LFA_CAPTURE = True
PARAM = "StockLfaCapture"


def capture_requested(params) -> bool:
  """True when this drive is to run passive for a stock-LFA recording."""
  return bool(STOCK_LFA_CAPTURE and params.get_bool(PARAM))


def controller_allowed(has_controller: bool, openpilot_enabled_toggle: bool, capture: bool, dashcam_only: bool) -> bool:
  """card's 'may openpilot control this car' decision, with the capture mode as one more reason not to."""
  return bool(has_controller and openpilot_enabled_toggle and not capture and not dashcam_only)


def on_ignition(params, ignition: bool, ignition_prev: bool) -> bool:
  """Called by manager every loop: the ignition going off ends the capture mode. Returns True when it cleared it."""
  if ignition_prev and not ignition and params.get_bool(PARAM):
    params.remove(PARAM)
    return True
  return False
