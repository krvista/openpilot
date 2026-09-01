"""MADS boot-enable retry (_boot_enable_pending) + event/state interactions,
driving the REAL ModularAssistiveDrivingSystem + StateMachine with real Events
containers and a stub selfdrived."""
import types

import pytest

from phase_tests.harness_noncontrol import FakeParams

from openpilot.cereal import log, custom
from opendbc.car import structs
from opendbc.car.hyundai.values import HyundaiFlags
from openpilot.selfdrive.selfdrived.events import Events
from openpilot.sunnypilot.selfdrive.selfdrived.events import EventsSP
from openpilot.selfdrive.selfdrived.state import StateMachine as SSStateMachine
from openpilot.sunnypilot.mads.mads import ModularAssistiveDrivingSystem
from openpilot.sunnypilot.mads.helpers import MadsSteeringModeOnBrake

State = custom.ModularAssistiveDrivingSystem.ModularAssistiveDrivingSystemState
EventName = log.OnroadEvent.EventName
EventNameSP = custom.OnroadEventSP.EventName
ButtonType = structs.CarState.ButtonEvent.Type


def mk_mads(main_cruise_allowed=False, steering_mode=None):
  FakeParams.reset()
  p = FakeParams()
  p.put_bool('Mads', True)
  p.put_bool('MadsMainCruiseAllowed', main_cruise_allowed)
  if steering_mode is not None:
    p.put('MadsSteeringMode', steering_mode)

  CP = structs.CarParams()
  CP.brand = 'hyundai'
  CP.flags = HyundaiFlags.CANFD.value  # allow_always platform (Ioniq 6 N is CANFD)
  CP.passive = False
  CP_SP = structs.CarParamsSP()

  sd = types.SimpleNamespace(
    CP=CP, CP_SP=CP_SP, params=p,
    enabled=False, initialized=True,
    events=Events(), events_sp=EventsSP(),
    state_machine=SSStateMachine(),
    CS_prev=structs.CarState(),
    # i6nv3: upstream mads now cross-checks panda controlsAllowedLateral via
    # selfdrived.sm; empty list = no mismatch in these unit scenarios.
    sm={'pandaStates': []},
  )
  return ModularAssistiveDrivingSystem(sd), sd


def mk_cs(cruise_available=True, standstill=False, v_ego=10.0, gas=False):
  CS = structs.CarState()
  CS.cruiseState.available = cruise_available
  CS.standstill = standstill
  CS.vEgo = v_ego
  CS.gasPressed = gas
  return CS


def lkas_btn():
  be = structs.CarState.ButtonEvent()
  be.type = ButtonType.lkas
  be.pressed = True
  return be


def frame(mads, sd, CS, events=()):
  """One selfdrived-style cycle: fresh event containers, then mads.update."""
  sd.events.clear()
  sd.events_sp.clear()
  for e in events:
    sd.events.add(e)
  mads.update(CS)
  sd.CS_prev = CS


class TestBootEnableRetry:
  def test_boot_auto_enable_without_main_cruise_toggle(self):
    mads, sd = mk_mads(main_cruise_allowed=False)
    frame(mads, sd, mk_cs())
    assert mads.enabled and mads.state_machine.state == State.enabled

  def test_boot_retry_through_no_entry_window(self):
    # the race the fork block exists for: rising edge lands inside a NO_ENTRY
    # window; without the retry MADS would stay off until user interaction
    mads, sd = mk_mads()
    for _ in range(10):
      frame(mads, sd, mk_cs(), events=[EventName.selfdriveInitializing])
      assert not mads.enabled
    frame(mads, sd, mk_cs())
    assert mads.enabled

  def test_user_button_disable_stays_disabled(self):
    mads, sd = mk_mads()
    frame(mads, sd, mk_cs())
    assert mads.enabled
    CS = mk_cs()
    CS.buttonEvents = [lkas_btn()]
    frame(mads, sd, CS, events=[])
    assert not mads.enabled and mads.state_machine.state == State.disabled
    assert not mads._boot_enable_pending
    # retry must NOT resurrect it
    for _ in range(100):
      frame(mads, sd, mk_cs())
      assert not mads.enabled

  def test_button_reenable_after_button_disable(self):
    mads, sd = mk_mads()
    frame(mads, sd, mk_cs())
    CS = mk_cs()
    CS.buttonEvents = [lkas_btn()]
    frame(mads, sd, CS)
    assert not mads.enabled
    CS2 = mk_cs()
    CS2.buttonEvents = [lkas_btn()]
    frame(mads, sd, CS2)
    assert mads.enabled  # explicit user press still enables

  def test_cruise_loss_then_return_no_reenable_without_toggle(self):
    mads, sd = mk_mads(main_cruise_allowed=False)
    frame(mads, sd, mk_cs())
    assert mads.enabled
    frame(mads, sd, mk_cs(cruise_available=False))  # falling edge -> lkasDisable
    assert not mads.enabled
    # pending was consumed by the first successful enable; availability alone
    # must not re-enable when MadsMainCruiseAllowed is off
    for _ in range(20):
      frame(mads, sd, mk_cs())
      assert not mads.enabled

  def test_cruise_loss_then_return_reenables_with_toggle(self):
    mads, sd = mk_mads(main_cruise_allowed=True)
    frame(mads, sd, mk_cs())
    frame(mads, sd, mk_cs(cruise_available=False))
    assert not mads.enabled
    frame(mads, sd, mk_cs())  # rising edge + toggle -> lkasEnable (stock path)
    assert mads.enabled

  def test_boot_disengage_mode_brake_held_no_enable_flap(self):
    # DISENGAGE-on-brake: the silent retry must not slip past the pedal gate
    # (would enable for one frame against the configured behavior, then
    # immediately user-disable again -> flap + alert churn)
    mads, sd = mk_mads(steering_mode=MadsSteeringModeOnBrake.DISENGAGE)
    for _ in range(10):
      frame(mads, sd, mk_cs(), events=[EventName.pedalPressed])
      assert not mads.enabled
    frame(mads, sd, mk_cs())  # pedal released -> retry resumes
    assert mads.enabled

  def test_boot_pause_mode_brake_held_no_enable_flap(self):
    mads, sd = mk_mads(steering_mode=MadsSteeringModeOnBrake.PAUSE)
    for _ in range(10):
      frame(mads, sd, mk_cs(), events=[EventName.pedalPressed])
      assert not mads.enabled
    frame(mads, sd, mk_cs())
    assert mads.enabled

  def test_boot_wrong_gear_enters_paused_then_enables(self):
    mads, sd = mk_mads()
    frame(mads, sd, mk_cs(v_ego=0.0, standstill=True), events=[EventName.wrongGear])
    # NO_ENTRY + paused-allowed gear -> paused, which counts as enabled
    assert mads.state_machine.state == State.paused and mads.enabled
    assert not mads._boot_enable_pending
    frame(mads, sd, mk_cs())  # gear ok -> paused resumes silently
    assert mads.state_machine.state == State.enabled


class TestEventInteractions:
  def test_standstill_door_open_pauses_silently(self):
    mads, sd = mk_mads()
    frame(mads, sd, mk_cs())
    assert mads.enabled
    frame(mads, sd, mk_cs(standstill=True), events=[EventName.doorOpen])
    # doorOpen replaced by silent variant, MADS paused (not hard-disabled)
    assert mads.state_machine.state == State.paused
    assert not sd.events.has(EventName.doorOpen)
    assert sd.events_sp.has(EventNameSP.silentDoorOpen)
    # door closed again -> silent resume from paused
    frame(mads, sd, mk_cs())
    assert mads.state_machine.state == State.enabled

  def test_moving_door_open_soft_disables(self):
    mads, sd = mk_mads()
    frame(mads, sd, mk_cs())
    frame(mads, sd, mk_cs(standstill=False), events=[EventName.doorOpen])
    # not standstill: no silent replacement; doorOpen soft-disables
    assert mads.state_machine.state == State.softDisabling

  def test_immediate_disable_wins(self):
    mads, sd = mk_mads()
    frame(mads, sd, mk_cs())
    frame(mads, sd, mk_cs(), events=[EventName.controlsMismatch])
    assert mads.state_machine.state == State.disabled

  def test_lkas_button_while_op_enabled_disables_with_alert(self):
    mads, sd = mk_mads()
    frame(mads, sd, mk_cs())
    sd.enabled = True  # openpilot cruise engaged
    CS = mk_cs()
    CS.buttonEvents = [lkas_btn()]
    frame(mads, sd, CS)
    # manualSteeringRequired is itself an ET.USER_DISABLE (alert flavor of
    # lkasDisable) -> MADS turns off and the boot retry must not resurrect it
    assert not mads.enabled
    assert sd.events_sp.has(EventNameSP.manualSteeringRequired)
    for _ in range(50):
      frame(mads, sd, mk_cs())
      assert not mads.enabled
