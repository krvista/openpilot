"""i6n Phase 70b (report §93): leaving onroad while the ignition is on - the harness relay is released BEFORE openpilot
stops sending.

While openpilot drives, the panda holds the harness relay in intercept: the factory camera's LKAS_ALT (0x110) and CAM_0x362
are cut off from the car and openpilot's own streams take their place. When openpilot leaves onroad with the ignition still
on (an onroad cycle asked by a settings change - the stock-LFA capture toggle is one -, forced offroad, a blocking device
condition) two things used to happen off the same deviceState message: manager stopped the onroad processes at once, so
card's frames ended within ~10 ms, and pandad, which reads deviceState in its 10 Hz state loop, released the relay up to
0.1 s later. In between the car had neither openpilot's nor the camera's 0x110 / 0x362.

Pulling the comma cable in the middle of a drive does not do that - the relay drops in the same instant openpilot's frames
end - and the car carries on with its factory ADAS without a warning. The first stock-LFA capture switch (the order with the
gap) left the camera features in "check system" (high beam assist, forward / lane safety, speed limit, driver attention
warning). The order is the one difference found between the two; no log of that switch exists, so it is the suspected
cause, not a proven one.

So manager acts on such a transition one step later: it keeps the onroad processes running until the panda reports a mode
in which it does not drive the relay (pandad has switched it to noOutput), RELAY_FIRST_HOLD_MAX_S at most. card keeps
sending during the hold; from the moment the relay is released the panda refuses those frames, so on the wire the handover
is the cable-pull one: openpilot's last frame, the relay, the camera's first frame. loggerd runs through the hold as well,
so the handover itself is in the log.

Ignition off is untouched (pandad sees the ignition go by itself and releases the relay before hardwared has even reported
offroad), and so is a transition with no panda or with the relay already released.

Two more things belong to the same handover:

  SESSION_GATES - pandad arms the car's safety mode (relay in intercept) once FirmwareQueryDone and ControlsReady are set,
  and it reads deviceState by itself. Both params used to stay set from the session that ended until manager's clear at
  the NEXT onroad edge, so a pandad tick that fell just before that clear armed the old CarParams: the relay in intercept
  seconds before card sends anything, and for good if the new session is passive (a passive card never sets ControlsReady,
  so pandad is never corrected). manager now drops both as the session ends.

  a missed offroad window - if deviceState is onroad again while manager still holds (a stalled loop), pandad has already
  re-armed the running session; stopping it then would cut openpilot's frames with the relay in intercept. The session is
  left running and a new onroad cycle is asked for instead (RELAY_FIRST_MAX_RETRIES in a row at most).

Kill: RELAY_FIRST_STOP = False (manager acts on deviceState.started at once, as before); SESSION_GATES = () keeps the gates."""

RELAY_FIRST_STOP = True
RELAY_FIRST_HOLD_MAX_S = 0.8    # pandad needs ~0.1 s; manager's loop runs on deviceState (2 Hz, plus the edges)
RELAY_FIRST_MAX_RETRIES = 2     # new onroad cycles asked in a row after a missed offroad window
RELAY_RELEASED_MODES = ("silent", "noOutput", "elm327")   # panda main.c: set_intercept_relay(false, ...) in these modes
SESSION_GATES = ("ControlsReady", "FirmwareQueryDone")    # pandad's conditions for arming the car's safety mode


def relay_released(panda_states) -> bool:
  """True when no connected panda holds the harness relay in intercept (no panda at all counts as released)."""
  for ps in panda_states:
    if str(ps.pandaType) == "unknown":
      continue
    if str(ps.safetyModel) not in RELAY_RELEASED_MODES:
      return False
  return True


class RelayFirstStop:
  """deviceState.started as manager should act on it: the offroad edge is held back until the relay is released."""

  def __init__(self):
    self.acted = False      # what manager acted on last loop
    self.hold_t0 = None     # monotonic time the hold began; None = not holding
    self.event = None       # a line for the log when a hold begins or ends, else None
    self.retry = False      # True for one loop: manager is to ask for a new onroad cycle
    self.retries = 0        # missed windows in a row

  def update(self, started: bool, ignition: bool, released: bool, now: float) -> bool:
    self.event = None
    self.retry = False
    if self.hold_t0 is not None:
      held = now - self.hold_t0
      if started:
        # the offroad window passed during the hold: pandad has re-armed the session that is still running
        self.hold_t0 = None
        self.acted = True
        self.retry = self.retries < RELAY_FIRST_MAX_RETRIES
        self.retries += 1
        self.event = f"offroad window missed after {held:.2f} s, the session keeps running" + (": asking for a new onroad cycle" if self.retry else "")
        return True
      if released:
        why = "relay released"
      elif not ignition:
        why = "ignition off"
      elif held >= RELAY_FIRST_HOLD_MAX_S:
        why = "timeout"
      elif not RELAY_FIRST_STOP:
        why = "disabled"
      else:
        return True          # openpilot keeps running (and sending) until the relay is out
      self.hold_t0 = None
      self.acted = False
      self.retries = 0
      self.event = f"acting on offroad after {held:.2f} s ({why})"
      return False
    if RELAY_FIRST_STOP and self.acted and not started and ignition and not released:
      self.hold_t0 = now
      self.event = "holding the onroad processes until the panda releases the relay"
      return True
    self.acted = bool(started)
    return self.acted
