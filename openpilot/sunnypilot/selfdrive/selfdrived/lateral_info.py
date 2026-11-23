"""i6n Phase 75o (report §103.6-7): tell the driver when nobody is steering although lateral is on.

The car controller's passive states (parking latch, low-speed gate, angle-passive, VM reject, reverse, camera fault)
were shown by a colour change only. From carStateSP (card.py mirrors the car controller's flags):

  lateralPaused      one prompt (chime + text) per passive episode: the request has been passive above
                     PAUSED_PROMPT_MIN_KPH with no driver press for PAUSED_PROMPT_DWELL_S (with cruise engaged - the
                     driver then expects steering - passive for PAUSED_PROMPT_DWELL_CRUISE_S and no press in the
                     last PAUSED_PROMPT_HANDS_OFF_S). Level-based: an episode that starts at a
                     stop or with hands on prompts once those conditions are met. Under the parking latch without
                     cruise the speed floor is PAUSED_PROMPT_PARKING_MIN_KPH (a1: 4 of 7 prompts were inside the
                     car park at 16-19 km/h; the departure road is caught by the cruise engagement). Never within
                     PAUSED_PROMPT_HANDS_OFF_S of a driver press, at most one per PAUSED_PROMPT_MIN_GAP_S, and an
                     episode only ends after PAUSED_EPISODE_END_S without the flag, so it cannot chatter.
  factoryLfaActive   text while the factory LFA has been active for FACTORY_LFA_DWELL_S with lateral on in the solo
                     profile (a normal hold-off removes it sooner).
  lfaHoldOffPaused   text once when the hold-off pauses (the factory LFA is then left on for a while).

Display only: nothing here feeds the car's hands-on detection or its driver attention warning.
Kill: PAUSED_PROMPT = False / FACTORY_LFA_INFO = False."""
from openpilot.cereal import custom
from openpilot.common.constants import CV
from openpilot.common.realtime import DT_CTRL

EventNameSP = custom.OnroadEventSP.EventName

PAUSED_PROMPT = True
PAUSED_PROMPT_MIN_KPH = 15.0
PAUSED_PROMPT_PARKING_MIN_KPH = 25.0   # parking latch, cruise not engaged (= PAUSED_PROMPT_MIN_KPH: no distinction)
PAUSED_PROMPT_DWELL_S = 1.0
PAUSED_PROMPT_DWELL_CRUISE_S = 0.1
PAUSED_PROMPT_HANDS_OFF_S = 1.0
PAUSED_PROMPT_MIN_GAP_S = 20.0
PAUSED_EPISODE_END_S = 2.0
FACTORY_LFA_INFO = True
FACTORY_LFA_DWELL_S = 1.0

GW_LFA_ACTIVE = 1 << 2   # carStateSP.gatewayFlags bits (opendbc hyundai carcontroller.py GW_*)
GW_SOLO = 1 << 4         # (in the legacy profile the factory LFA is meant to be active)
GW_GAVE_UP = 1 << 7
PR_PARKING = 4           # carStateSP.lateralPausedReason (opendbc hyundai carcontroller.py PR_PARKING)


class LateralInfo:
  def __init__(self, dt: float = DT_CTRL):
    self.dt = dt
    self.since_press = 1e6          # s since the last steeringPressed frame
    self.cond_time = 0.0            # s the prompt condition has held
    self.clear_time = 1e6           # s the paused flag has been clear
    self.since_prompt = 1e6         # s since the last prompt
    self.episode_prompted = False
    self.lfa_active_time = 0.0
    self.gave_up_prev = False

  def update(self, CS, cs_sp, lat_active: bool) -> list[int]:
    """cs_sp: carStateSP, or None when it is not fresh. Returns the events of this frame."""
    events: list[int] = []
    dt = self.dt
    self.since_press = 0.0 if CS.steeringPressed else min(self.since_press + dt, 1e6)
    self.since_prompt = min(self.since_prompt + dt, 1e6)

    paused = bool(cs_sp is not None and cs_sp.lateralControlPaused)
    gw = int(cs_sp.gatewayFlags) if cs_sp is not None else 0

    # --- request passive
    if paused:
      self.clear_time = 0.0
    else:
      self.clear_time = min(self.clear_time + dt, 1e6)
      if self.clear_time >= PAUSED_EPISODE_END_S:
        self.episode_prompted = False
    min_kph = PAUSED_PROMPT_MIN_KPH
    if paused and int(cs_sp.lateralPausedReason) == PR_PARKING and not CS.cruiseState.enabled:
      min_kph = max(min_kph, PAUSED_PROMPT_PARKING_MIN_KPH)
    cond = paused and CS.vEgo * CV.MS_TO_KPH > min_kph and not CS.steeringPressed
    self.cond_time = self.cond_time + dt if cond else 0.0
    dwell = PAUSED_PROMPT_DWELL_CRUISE_S if CS.cruiseState.enabled else PAUSED_PROMPT_DWELL_S
    if (PAUSED_PROMPT and cond and self.cond_time >= dwell - 1e-6
        and self.since_press >= PAUSED_PROMPT_HANDS_OFF_S - 1e-6 and not self.episode_prompted
        and self.since_prompt >= PAUSED_PROMPT_MIN_GAP_S):
      self.episode_prompted = True
      self.since_prompt = 0.0
      events.append(EventNameSP.lateralPaused)

    # --- factory LFA steering / hold-off paused
    lfa_active = bool(gw & GW_LFA_ACTIVE) and bool(gw & GW_SOLO) and bool(lat_active)
    self.lfa_active_time = self.lfa_active_time + dt if lfa_active else 0.0
    gave_up = bool(gw & GW_GAVE_UP)
    if FACTORY_LFA_INFO:
      if self.lfa_active_time >= FACTORY_LFA_DWELL_S - 1e-6:
        events.append(EventNameSP.factoryLfaActive)
      if gave_up and not self.gave_up_prev:
        events.append(EventNameSP.lfaHoldOffPaused)
    self.gave_up_prev = gave_up
    return events
