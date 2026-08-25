"""Phase 26/26b/27: hold-compensated driver-torque domain for latches, and
the intentional-passive UI flag.

Geometry (Ioniq 6 N: steerRatio ~14.96, wheelbase 2.965):
  straight (op active)  -> hold_comp settles at 0.8*122 ~ 97.6 Nm
  wheel 30 @ 10 m/s     -> lat_acc 1.18 -> hold_comp ~ 222 Nm
  wheel 45 @  8 m/s     -> lat_acc 1.14 -> hold_comp ~ 218 Nm

Phase 26b: hold compensation applies ONLY while op was actuating on the
previous frame (effective_lat_active). In passive states the bar reading is
entirely the driver's, so driver_tq == raw there.
"""
from phase_tests.harness import Sim, run_signal

from opendbc.car.hyundai.values import CarControllerParams as P


def settle(sim, n=120, **kw):
  """Run n frames with no driver torque so hold_comp reaches steady state."""
  kw.setdefault('tq', 0.0)
  run_signal(sim, n, **kw)


class TestDriverPressed:
  def test_straight_line_equivalence_fires(self):
    # raw 360 in a straight = driver_tq ~262 > 250: pressed, like the old raw-350 flag
    sim = Sim()
    settle(sim, v=15.0, wheel=0.0)
    run_signal(sim, 20, v=15.0, wheel=0.0, tq=360.0)
    assert sim.s.driver_pressed

  def test_straight_line_equivalence_no_fire(self):
    # Phase 31 comp at 54 km/h straight is ~73.5: raw 290 = driver_tq ~216
    # < 230 -> not pressed (the pressed raw-equivalence point is now ~303)
    sim = Sim()
    settle(sim, v=15.0, wheel=0.0)
    run_signal(sim, 100, v=15.0, wheel=0.0, tq=290.0)
    assert not sim.s.driver_pressed

  def test_curve_hold_immunity(self):
    # wheel 30 deg @ 10 m/s, op active: hold ~222. Raw 380 would have tripped
    # the old raw-350 EPS-style test; driver_tq is only ~158 -> un-pressed.
    sim = Sim()
    settle(sim, v=10.0, wheel=30.0)
    run_signal(sim, 200, v=10.0, wheel=30.0, tq=380.0)
    assert not sim.s.driver_pressed

  def test_curve_real_grip_still_fires(self):
    # measured real-grip episode peaks are p50 519-562 Nm: a genuine hard
    # grip in the same curve still trips driver_pressed.
    sim = Sim()
    settle(sim, v=10.0, wheel=30.0)
    run_signal(sim, 200, v=10.0, wheel=30.0, tq=540.0)
    assert sim.s.driver_pressed

  def test_release_hysteresis(self):
    # enter at >250 driver-domain, hold at >0.8*250=200, release below it
    sim = Sim()
    settle(sim, v=15.0, wheel=0.0)
    run_signal(sim, 20, v=15.0, wheel=0.0, tq=360.0)
    assert sim.s.driver_pressed
    run_signal(sim, 20, v=15.0, wheel=0.0, tq=270.0)   # driver ~196 > 184: held
    assert sim.s.driver_pressed
    run_signal(sim, 20, v=15.0, wheel=0.0, tq=245.0)   # driver ~171 < 184: releases
    assert not sim.s.driver_pressed

  def test_engagement_instant_reads_full_driver_torque(self):
    # Phase 26b: before op ever actuates, the bar is all driver — a 330 raw
    # push at the first active frames must count as driver_tq ~330 (pressed),
    # unlike the steady-state case above where ~98 is op's.
    sim = Sim()
    run_signal(sim, 20, v=15.0, wheel=0.0, tq=330.0)
    assert sim.s.driver_pressed


class TestHoldSlewGuard:
  def test_single_frame_spike_bounded(self):
    sim = Sim()
    settle(sim, v=15.0, wheel=0.0)
    before = sim.s.hold_comp_last
    sim.step(v=15.0, wheel=400.0, tq=0.0)   # 1-frame angle spike
    assert abs(sim.s.hold_comp_last - before) <= P.ACIGAIN_HOLD_SLEW_NM + 1e-6
    # and it recovers toward baseline once the spike clears
    settle(sim, v=15.0, wheel=0.0)
    assert abs(sim.s.hold_comp_last - before) < 2.0

  def test_real_curve_entry_tracks(self):
    # a genuine sustained curve is ~1-3 Nm/frame — the slew must not lag it
    sim = Sim()
    settle(sim, v=10.0, wheel=0.0)
    run_signal(sim, 100, v=10.0, wheel=30.0)
    assert sim.s.hold_comp_last > 175.0     # reached the Phase 31 ~188 target

  def test_passive_state_gates_compensation_to_zero(self):
    # Phase 26b: once op is passive (parking mode), hold_comp bleeds to 0
    sim = Sim()
    settle(sim, v=15.0, wheel=0.0)
    assert sim.s.hold_comp_last > 65.0      # Phase 31: ~73.5 at 54 km/h straight
    run_signal(sim, 320, v=5.0, cmd=0.0, wheel=280.0)   # parking mode -> passive
    assert sim.s.parking_mode_active
    run_signal(sim, 100, v=5.0, cmd=0.0, wheel=280.0)
    assert sim.s.hold_comp_last < 1.0


class TestBlinkerAnchorDomain:
  def test_straight_equivalence(self):
    # raw 225 straight = driver ~127 >= 120: fires (old raw-220 test: same)
    sim = Sim()
    settle(sim, v=15.0, wheel=0.0)
    run_signal(sim, 10, v=15.0, wheel=0.0, tq=225.0, blinker=True)
    assert sim.s.blinker_anchor_on

  def test_straight_below_threshold(self):
    sim = Sim()
    settle(sim, v=15.0, wheel=0.0)
    run_signal(sim, 50, v=15.0, wheel=0.0, tq=185.0, blinker=True)  # driver ~112 < 120 (comp ~73.5)
    assert not sim.s.blinker_anchor_on

  def test_curve_self_fire_removed(self):
    # signaled turn on a curved road, hands off: raw reads ~hold (use 240 —
    # above the old raw 220 threshold, which would have self-fired).
    sim = Sim()
    settle(sim, v=10.0, wheel=30.0)
    run_signal(sim, 100, v=10.0, wheel=30.0, tq=240.0, blinker=True)
    assert not sim.s.blinker_anchor_on


class TestAnglePassiveDomain:
  def _enter(self, sim):
    # geo entry: |wheel| >= 40 with a real grip (driver_tq > 250 needs
    # raw > ~468 at this curve's steady-state hold level)
    settle(sim, v=8.0, wheel=45.0)
    run_signal(sim, 60, v=8.0, wheel=45.0, tq=520.0)
    assert sim.s.angle_passive_active

  def test_release_on_genuine_letgo_in_curve(self):
    # in the passive state compensation is gated off (26b), so a light
    # residual touch of raw 100 reads as driver_tq 100 < 260 and releases
    sim = Sim()
    self._enter(sim)
    run_signal(sim, 100, v=8.0, wheel=45.0, tq=100.0)
    assert not sim.s.angle_passive_active

  def test_holding_driver_keeps_passive(self):
    # Phase 26b (review BLOCKER fix): a driver holding 300 raw in the passive
    # state is all driver torque — op must NOT take the wheel back. Under the
    # pre-fix code hold_comp (218) made this read driver_tq 82 and op
    # re-engaged against the driver's grip; with comp gated to 0 the exit
    # test sees the full 300 >= 260 and stays passive.
    sim = Sim()
    self._enter(sim)
    run_signal(sim, 300, v=8.0, wheel=45.0, tq=300.0)
    assert sim.s.angle_passive_active

  def test_no_limit_cycle_at_steady_grip(self):
    # Phase 26b: a steady driver grip must hold ONE stable passive state, not
    # the ~1.2 Hz enter/release limit cycle of the pre-fix code. Closed loop:
    # op's own bar contribution (hold * ACIGain) appears only while op
    # actuates — the exact mechanism that drove the pre-fix cycle.
    sim = Sim()
    self._enter(sim)
    def tq_fn(_i):
      op_share = 218.0 * sim.s.aci_gain_last if sim.effective_lat_active() else 0.0
      return 360.0 + op_share
    tr = run_signal(sim, 600, tq_fn=tq_fn, v=8.0, wheel=45.0)
    flips = sum(1 for a, b in zip(tr['angle_passive_active'], tr['angle_passive_active'][1:]) if a != b)
    assert flips == 0
    assert sim.s.angle_passive_active

  def test_geo_entry_needs_real_grip(self):
    # raw 350 at wheel 45 / 8 m/s op-active is inside the hold band
    # (driver ~132 < 250): the old raw-350 pressed flag would have dropped op
    # passive hands-off here. cmd tracks the wheel so the intent-disagree arm
    # (delta >= 5 deg) stays out of the picture.
    sim = Sim()
    run_signal(sim, 100, v=8.0, wheel=45.0, cmd=45.0, tq=0.0)   # apply settles on the wheel
    run_signal(sim, 200, v=8.0, wheel=45.0, cmd=45.0, tq=350.0)
    assert not sim.s.angle_passive_active


class TestIntentDisagreeDomain:
  def test_fires_on_real_opposing_push(self):
    # op holds cmd=20 while the driver pushes the wheel the other way at 10
    # deg: raw -300 at hold ~124 -> driver_tq 176 with opposing sign.
    # Phase 28: the heavy-grip anchor (driver_tq >= 160 OR-arm) preempts the
    # intent-disagree latch by pinning apply to the held wheel — either
    # resolution means op stops pulling toward its own plan, which is the
    # property under test.
    sim = Sim()
    settle(sim, v=8.0, wheel=10.0, cmd=20.0)
    run_signal(sim, 100, v=8.0, wheel=10.0, cmd=20.0, tq=-350.0)
    assert sim.s.angle_passive_active or abs(sim.s.apply_angle_last - 10.0) < 1.5

  def test_below_threshold_no_fire(self):
    sim = Sim()
    settle(sim, v=8.0, wheel=10.0, cmd=20.0)
    run_signal(sim, 100, v=8.0, wheel=10.0, cmd=20.0, tq=-270.0)  # driver ~146 < 160
    assert not sim.s.angle_passive_active


class TestYankFix:
  """Phase 28 v2: 0x41 field yank — EPS anchor restored, gated release re-anchor."""

  def test_anchor_holds_in_350_490_band(self):
    # v=24 m/s, wheel 8°: lat_acc ~1.8 -> comp at the 240 cap -> raw 460 is
    # driver_tq ~220: driver_pressed can NOT fire, but the restored EPS
    # pressed OR-arm must anchor apply to the held wheel instead of letting
    # it track the diverging plan (the seg17 failure).
    sim = Sim()
    settle(sim, v=24.0, wheel=8.0, cmd=8.0)
    run_signal(sim, 100, v=24.0, wheel=8.0, cmd=14.0, tq=400.0)
    assert not sim.s.driver_pressed          # confirms we are in the gap band
    assert abs(sim.s.apply_angle_last - 8.0) < 1.5

  def test_release_reanchor_dumps_divergence(self):
    # anchor dropout gap: a real grip (460, anchored) decays to 300 — EPS
    # pressed and override both drop, the anchor disengages, and apply
    # diverges toward the plan while the driver still holds. On the final
    # let-go the stored divergence must be dumped to the wheel, not
    # delivered, and the wound trim must be zeroed.
    sim = Sim()
    settle(sim, v=15.0, wheel=0.0, cmd=0.0)
    run_signal(sim, 100, v=15.0, wheel=0.0, cmd=8.0, tq=460.0)   # anchored grip
    assert abs(sim.s.apply_angle_last) < 1.0
    run_signal(sim, 150, v=15.0, wheel=0.0, cmd=8.0, tq=300.0)   # dropout gap
    assert sim.s.reanchor_arm >= 30                              # driver_tq ~202
    assert abs(sim.s.apply_angle_last) > 2.0                     # divergence stored
    sim.step(v=15.0, wheel=0.0, cmd=8.0, tq=0.0)                 # driver lets go
    assert abs(sim.s.apply_angle_last) < 1.0                     # dumped to the wheel
    assert not sim.s.reanchor_ready                              # edge consumed (G2)
    assert sim.s.curve_trim == 0.0

  def test_memory_survives_fire_for_refire(self):
    # F1 review fix: the fire must NOT disarm the memory — a second
    # divergence inside the same episode window must dump again after the
    # refractory.
    # timings stay inside the 2 s anchor_recent window (its expiry is the
    # hands-off protection, covered by test_handsoff_release_cannot_fire)
    sim = Sim()
    settle(sim, v=15.0, wheel=0.0, cmd=0.0)
    run_signal(sim, 100, v=15.0, wheel=0.0, cmd=8.0, tq=460.0)
    run_signal(sim, 100, v=15.0, wheel=0.0, cmd=8.0, tq=300.0)
    sim.step(v=15.0, wheel=0.0, cmd=8.0, tq=0.0)                 # first fire
    assert sim.s.reanchor_arm >= 30                              # memory kept
    run_signal(sim, 50, v=15.0, wheel=0.0, cmd=8.0, tq=300.0)    # re-grip, diverge again
    sim.step(v=15.0, wheel=0.0, cmd=8.0, tq=0.0)
    assert abs(sim.s.apply_angle_last) < 1.0                     # second dump

  def test_reanchor_noop_when_tracking(self):
    # normal hands-off tracking: no anchor episode, apply follows the plan
    sim = Sim()
    settle(sim, v=15.0, wheel=0.0, cmd=0.0)
    run_signal(sim, 300, v=15.0, wheel=5.0, cmd=5.0, tq=0.0)
    assert sim.s.anchor_recent_frames == 0
    assert abs(sim.s.apply_angle_last - 5.0) < 1.0

  def test_handsoff_release_cannot_fire(self):
    # hands-off-indistinguishable torque (driver_tq ~111 arms the memory —
    # corpus p75 = 111, unavoidable) with a static deficit divergence and NO
    # anchor episode: the let-go must NOT dump apply (the v1 failure mode:
    # 647 hands-off fires, p90 6.7°).
    sim = Sim()
    settle(sim, v=10.0, wheel=10.0, cmd=14.0)                    # static 4° divergence
    run_signal(sim, 300, v=10.0, wheel=10.0, cmd=14.0, tq=300.0) # driver_tq ~111 (comp ~188), no anchor
    assert sim.s.reanchor_arm >= 30                              # memory arms (by design)
    assert sim.s.anchor_recent_frames == 0                       # but no anchor episode
    before = sim.s.apply_angle_last
    sim.step(v=10.0, wheel=10.0, cmd=14.0, tq=0.0)
    assert abs(sim.s.apply_angle_last - before) < 0.5            # no dump

  def test_blinker_anchor_is_not_grip_evidence(self):
    # G3: a blinker-arm anchor (no pressed arm) must NOT arm the re-anchor
    # memory — otherwise hands-off blinker episodes enable context-free dumps
    sim = Sim()
    # Phase 31: at 54 km/h straight the pressed raw-equivalent (~303) sits
    # below the override-0.9 point (325), so the G3 property is exercised in
    # a mild curve (comp ~195 -> pressed equiv ~425, blinker fire ~315).
    # Settle AT the curve so comp is at steady state before torque applies
    # (an instant 0->20 wheel step with comp still ramping transiently
    # crosses driver_pressed — a scenario artifact, not the property).
    settle(sim, v=15.0, wheel=20.0, cmd=20.0)
    run_signal(sim, 50, v=15.0, wheel=20.0, cmd=20.0, tq=340.0, blinker=True)
    assert sim.s.blinker_anchor_on
    assert sim.s.anchor_recent_frames == 0

  def test_no_repeat_fire_without_regrip(self):
    # G2: after a dump, a persisting divergence must NOT re-fire until the
    # driver genuinely re-grips past the arm level
    sim = Sim()
    settle(sim, v=15.0, wheel=0.0, cmd=0.0)
    run_signal(sim, 100, v=15.0, wheel=0.0, cmd=8.0, tq=460.0)
    run_signal(sim, 100, v=15.0, wheel=0.0, cmd=8.0, tq=300.0)
    sim.step(v=15.0, wheel=0.0, cmd=8.0, tq=0.0)                 # fire
    assert not sim.s.reanchor_ready
    tr = run_signal(sim, 60, v=15.0, wheel=0.0, cmd=8.0, tq=0.0) # stay released
    # apply may re-approach the plan but must not sawtooth back to the wheel
    assert not sim.s.reanchor_ready
    assert min(abs(a) for a in tr['apply'][20:]) > 1.0


class TestRecoveryTaper:
  """Phase 29: gain recovery/boost must back off at delivery-scale errors."""

  def test_rate_up_tapers_at_large_error_post_grip(self):
    from opendbc.car.hyundai.carcontroller import compute_torque_reduction_gain as f
    g_mid = f(0.0, 90.0, True, 0.10, 1.5, post_grip=True)   # drift-scale: fast
    g_big = f(0.0, 90.0, True, 0.10, 5.0, post_grip=True)   # delivery-scale: reference
    assert g_mid - 0.10 > 3 * (g_big - 0.10) > 0
    # the floor must survive gain quantization (0.004 steps): a sub-quantum
    # rate would freeze recovery outright and deadlock re-engagement
    assert g_big - 0.10 >= 0.004 - 1e-9

  def test_rate_up_legacy_without_grip_evidence(self):
    # hands-off drift recovery (deficit curves sit at |err| p50 2.22°) must
    # keep the full legacy fast path when there is no recent grip
    from opendbc.car.hyundai.carcontroller import compute_torque_reduction_gain as f
    g_big_ho = f(0.0, 90.0, True, 0.10, 5.0, post_grip=False)
    g_big_pg = f(0.0, 90.0, True, 0.10, 5.0, post_grip=True)
    assert g_big_ho - 0.10 > 5 * (g_big_pg - 0.10)

  def test_boost_tapers_at_large_error_post_grip_only(self):
    from opendbc.car.hyundai.carcontroller import compute_torque_reduction_gain as f
    # near-steady gain so the returned value ~= target ceiling
    g_drift = f(0.0, 90.0, True, 0.85, 1.0, post_grip=True)
    g_big = f(0.0, 90.0, True, 0.85, 5.0, post_grip=True)
    assert g_drift >= g_big                  # boost must not raise the big-error target
    g_big_ho = f(0.0, 90.0, True, 0.85, 5.0, post_grip=False)
    # STRICT: equal values would mean the taper applies regardless of the
    # gate — the exact regression this test exists to catch
    assert g_big_ho > g_big                  # hands-off keeps the boost


class TestHandover:
  """Phase 30: after the driver releases, op must resume plan tracking
  promptly — the Phase 29 leash that pinned apply to the released wheel was
  removed (36 corpus urgent-regrab windows, field lane-departure 0x47)."""

  def test_release_resumes_plan_tracking_promptly(self):
    # the 0x47 seg15 class: anchored grip holds the wheel off-plan, driver
    # fully releases -> apply must move toward the plan well inside the old
    # 2 s leash window instead of staying pinned at the wheel
    sim = Sim()
    settle(sim, v=25.0, wheel=0.0, cmd=0.0)
    run_signal(sim, 60, v=25.0, wheel=0.0, cmd=8.0, tq=460.0)    # anchored: apply pinned ~0
    run_signal(sim, 100, v=25.0, wheel=0.0, cmd=8.0, tq=0.0)     # released, 1.0 s
    assert sim.s.apply_angle_last > 4.0                          # already en route to the plan

  def test_release_with_lingering_touch_still_resumes(self):
    # an invisible lingering touch (raw 150 -> driver_tq ~52) must not pin
    # apply either — the one-shot + taper own the residual slam risk
    sim = Sim()
    settle(sim, v=25.0, wheel=0.0, cmd=0.0)
    run_signal(sim, 60, v=25.0, wheel=0.0, cmd=8.0, tq=460.0)
    run_signal(sim, 150, v=25.0, wheel=0.0, cmd=8.0, tq=150.0)   # tail: old leash pinned here
    assert sim.s.apply_angle_last > 4.0                          # no pinning

  def test_release_approach_is_rate_bounded(self):
    # the resumed approach must stay inside the VM comfort envelope: apply
    # moves toward the plan monotonically with no single-frame jump beyond
    # the VM step (no slam re-introduced by the removal)
    sim = Sim()
    settle(sim, v=25.0, wheel=0.0, cmd=0.0)
    run_signal(sim, 60, v=25.0, wheel=0.0, cmd=8.0, tq=460.0)
    tr = run_signal(sim, 150, v=25.0, wheel=0.0, cmd=8.0, tq=0.0)
    steps = [b - a for a, b in zip(tr['apply'], tr['apply'][1:])]
    assert max(abs(s) for s in steps) < 1.2                      # per-frame VM bound
    assert all(s > -0.2 for s in steps)                          # monotone toward the plan


class TestLatPassiveIndicated:
  def test_parking_mode_sets_flag(self):
    sim = Sim()
    run_signal(sim, 320, v=5.0, cmd=0.0, wheel=280.0)
    assert sim.s.parking_mode_active
    assert sim.s.lat_passive_indicated

  def test_lat_inactive_clears_flag(self):
    sim = Sim()
    run_signal(sim, 320, v=5.0, cmd=0.0, wheel=280.0)
    assert sim.s.lat_passive_indicated
    sim.step(v=5.0, wheel=280.0, lat_active=False)
    assert not sim.s.lat_passive_indicated

  def test_blinker_anchor_does_not_set_flag(self):
    # driver-initiated override states must not flash the paused icon
    sim = Sim()
    settle(sim, v=15.0, wheel=0.0)
    run_signal(sim, 10, v=15.0, wheel=0.0, tq=225.0, blinker=True)
    assert sim.s.blinker_anchor_on
    assert not sim.s.lat_passive_indicated

  def test_low_speed_passthrough_sets_flag(self):
    sim = Sim()
    # low-speed latch: pressed entry below the enter speed
    run_signal(sim, 20, v=4.0, wheel=0.0, tq=380.0)
    assert sim.s.low_speed_cam_latched
    run_signal(sim, 5, v=4.0, wheel=0.0, tq=380.0)
    assert sim.s.lat_passive_indicated

  def test_reverse_sets_flag(self):
    # Phase 26b: the post-reverse/reverse hold has no alert of its own
    sim = Sim()
    run_signal(sim, 10, v=1.0, wheel=0.0, gear='reverse')
    assert sim.s.lat_passive_indicated
