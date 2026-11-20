"""i6n Phase 70 / 70b (report §92-94): stock-LFA capture mode.

The drive logs hold no frame of the factory LFA actually steering this MDPS (§91.5: every camera 0x110 frame in them is
a standby echo of the wheel angle). To record one, a drive is run with openpilot passive: card marks the car passive and
never sets ControlsReady, so pandad never arms the car's safety mode - the panda stays in elm327, the harness relay
leaves the factory camera connected to the car and the factory LFA / HDA2 work as built, while logging goes on as on any
drive (the "Dashcam Mode" alert is shown).

Two ways in, both from the comma 4 Developer screen (the two toggles after "quickboot mode"):

  "stock LFA capture" - now. Sets StockLfaCapture and asks for an onroad cycle: openpilot restarts into the passive session
    on this drive. In the first version every such restart left the car's camera features in "check system": pandad armed
    the ENDED session's safety mode again (relay in intercept) off params that outlived it, and the passive session sent
    nothing. 70b: manager drops those params when a session ends (common/session_gates.py).

  "stock LFA next start" - armed. Sets StockLfaCaptureNext (persistent); manager turns it into StockLfaCapture on the rising
    edge of the ignition, before card starts. That drive is passive from its first second: no restart, the relay is never
    switched and openpilot never sends a frame. The fallback if switching now still brings warnings.

Either way the mode is for ONE drive: StockLfaCapture is CLEAR_ON_MANAGER_START and manager removes it when the ignition
goes off; the arming is used up when its drive starts.

Kill: STOCK_LFA_CAPTURE = False (both params are then ignored; an arming is still used up at the next ignition on)."""

STOCK_LFA_CAPTURE = True
PARAM = "StockLfaCapture"            # this ignition cycle runs passive
PARAM_NEXT = "StockLfaCaptureNext"   # armed: the next ignition cycle will


def capture_requested(params) -> bool:
  """True when this drive is to run passive for a stock-LFA recording."""
  return bool(STOCK_LFA_CAPTURE and params.get_bool(PARAM))


def controller_allowed(has_controller: bool, openpilot_enabled_toggle: bool, capture: bool, dashcam_only: bool) -> bool:
  """card's 'may openpilot control this car' decision, with the capture mode as one more reason not to."""
  return bool(has_controller and openpilot_enabled_toggle and not capture and not dashcam_only)


def on_ignition(params, ignition: bool, ignition_prev: bool) -> str | None:
  """Called by manager every loop, before it starts the onroad processes.
  Ignition coming on: an armed capture becomes this drive's mode, and the arming is used up.
  Ignition going off: the drive's mode ends. Returns 'started' / 'ended' when it changed the mode."""
  if ignition and not ignition_prev:
    if params.get_bool(PARAM_NEXT):
      params.put_bool(PARAM_NEXT, False)
      if STOCK_LFA_CAPTURE:
        params.put_bool(PARAM, True)
        return "started"
  elif ignition_prev and not ignition:
    if params.get_bool(PARAM):
      params.remove(PARAM)
      return "ended"
  return None
