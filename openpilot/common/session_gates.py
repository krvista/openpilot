"""i6n Phase 70b (report §93-94): the params pandad arms the car's safety mode on must not outlive the session that set them.

pandad puts the panda into the car's safety mode - harness relay in INTERCEPT: the factory camera's LKAS_ALT (0x110) and
CAM_0x362 are cut off from the car and only openpilot's own frames reach it - as soon as it is onroad and both
FirmwareQueryDone and ControlsReady are set (panda_safety.cc), with whatever the CarParams param holds at that moment. It
reads deviceState by itself. Both gates and CarParams are CLEAR_ON_ONROAD_TRANSITION: the values of the session that just
ended stay until manager clears them at the NEXT onroad edge, and manager and pandad race for that edge. When openpilot
restarts with the ignition on (an onroad cycle asked by a settings change), manager is usually still collecting the exiting
processes when the edge comes, pandad gets there first and arms the ENDED session's CarParams.

Measured on the car (routes 8b-91, three of three restarts into the stock-LFA capture mode): the panda was back in
hyundaiCanfd within 0.2 s of the restart, the passive session sent nothing, so the car had neither the camera's nor
openpilot's 0x110 / 0x362; 0.5 s later the camera features were in "check system" (high beam assist, forward / lane safety,
speed limit, driver attention warning) and stayed there for the whole session. A passive card never sets ControlsReady, so
pandad was never corrected. (A restart into a normal session would have the same hole for the ~10 s until card sends.)
The two restarts back out of the capture mode did not arm early: the passive session had left ControlsReady unset.

So manager drops the gates when a session ends (the offroad transition). pandad then waits for the NEW card to set them:
a passive card never does, the panda stays in elm327 with the relay released and the factory camera drives the car's ADAS.

Kill: DROP_SESSION_GATES = False (the gates live until the next onroad edge, as before)."""

DROP_SESSION_GATES = True
SESSION_GATES = ("ControlsReady", "FirmwareQueryDone")   # panda_safety.cc fetchCarParams(): both must be set to arm


def drop(params) -> tuple[str, ...]:
  """Called by manager on the offroad transition. Returns the gates it removed."""
  if not DROP_SESSION_GATES:
    return ()
  for gate in SESSION_GATES:
    params.remove(gate)
  return SESSION_GATES
