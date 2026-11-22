"""i6n Phase 70 / 70b (report §92-94): stock-LFA capture mode.

The drive logs hold no frame of the factory LFA actually steering this MDPS (§91.5: every camera 0x110 frame in them is
a standby echo of the wheel angle). To record one, a drive is run with openpilot passive: card marks the car passive and
never sets ControlsReady, so pandad never arms the car's safety mode - the panda stays in elm327, the harness relay
leaves the factory camera connected to the car and the factory LFA / HDA2 work as built, while logging goes on as on any
drive (the "Dashcam Mode" alert is shown).

One way in, from the comma 4 Developer screen (the toggle after "quickboot mode"):

  "stock LFA capture" - now. Sets StockLfaCapture and asks for an onroad cycle: openpilot restarts into the passive session
    on this drive. In the first version every such restart left the car's camera features in "check system": pandad armed
    the ENDED session's safety mode again (relay in intercept) off params that outlived it, and the passive session sent
    nothing. 70b: manager drops those params when a session ends (common/session_gates.py).

The mode is for ONE drive: StockLfaCapture is CLEAR_ON_MANAGER_START and manager removes it when the ignition goes off.
(Phase 70b also had "stock LFA next start", an arming for the next ignition on, as a fallback while the switch-over still
brought warnings. With 70b verified on the car - routes 97-9f, no warnings - it was removed in Phase 74.)

Kill: STOCK_LFA_CAPTURE = False (the param is then ignored)."""

STOCK_LFA_CAPTURE = True
PARAM = "StockLfaCapture"            # this ignition cycle runs passive


def capture_requested(params) -> bool:
  """True when this drive is to run passive for a stock-LFA recording."""
  return bool(STOCK_LFA_CAPTURE and params.get_bool(PARAM))


def controller_allowed(has_controller: bool, openpilot_enabled_toggle: bool, capture: bool, dashcam_only: bool) -> bool:
  """card's 'may openpilot control this car' decision, with the capture mode as one more reason not to."""
  return bool(has_controller and openpilot_enabled_toggle and not capture and not dashcam_only)


def on_ignition(params, ignition: bool, ignition_prev: bool) -> str | None:
  """Called by manager every loop, before it starts the onroad processes.
  Ignition going off: the drive's capture mode ends. Returns 'ended' when it changed the mode, else None."""
  if ignition_prev and not ignition:
    if params.get_bool(PARAM):
      params.remove(PARAM)
      return "ended"
  return None
