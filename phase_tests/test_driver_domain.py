"""Phase 26/26b/27/31: hold-compensated driver-torque domain for latches,
and the intentional-passive UI flag.

Phase 31 model comp(v, la) = B(v) + G(v)*S(la) — reference points used by
the tests below (Ioniq 6 N: steerRatio ~14.96, wheelbase 2.965):
  straight @ 15 m/s (54 km/h)  -> comp ~ 73.5 Nm
  straight @ 25 m/s+           -> comp ~ 62
  straight @ crawl (<5.5 m/s)  -> comp ~ 140
    (Phase 34b: EXCEPT still-straight crawl — v<3 m/s AND |ang|<5 AND
    |rate|<10 deg/s -> comp 70, raw pressed equiv 300; eff-active fit
    point is 28 (0x51-0x54), 70 chosen above it for the light-hand tail)
  wheel 30 @ 10 m/s (la 1.18)  -> comp ~ 188
  wheel 45 @  8 m/s (la 1.14)  -> comp ~ 189
  wheel  8 @ 24 m/s (la 1.81)  -> comp ~ 180
driver_pressed: 230 comp-on, raw 250 when comp is gated off (31b).

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
    # raw 360 in a straight = driver_tq ~286 > 230: pressed
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
    # enter at >230 driver-domain (comp-on), hold at >0.8*230=184, release below
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


class TestHoldModel:
  # Phase 31b: the model as a pure function (compute_hold_torque).

  def test_reference_points(self):
    from opendbc.car.hyundai.carcontroller import compute_hold_torque as h
    assert abs(h(15.0, 0.0) - 73.5) < 2.0
    assert abs(h(3.0, 0.0) - 140.0) < 0.1      # crawl clamp (no data < ~3 m/s)
    assert abs(h(10.0, 1.18) - 188.5) < 2.0    # saturated curve

  def test_monotone_in_lat_acc(self):
    from opendbc.car.hyundai.carcontroller import compute_hold_torque as h
    for v in (5.0, 12.0, 20.0, 33.0):
      vals = [h(v, la) for la in (0.0, 0.05, 0.1, 0.2, 0.3, 1.0, 3.0)]
      assert all(b >= a - 1e-9 for a, b in zip(vals, vals[1:]))

  def test_cap_has_headroom(self):
    # the 220 cap is a table-edit guard: it must NOT bind for the current
    # tables (structural max 197). If this fails, a table edit started
    # clipping silently -- make that a conscious decision.
    import numpy as np
    from opendbc.car.hyundai.carcontroller import compute_hold_torque as h
    peak = max(h(v, 5.0) for v in np.arange(0.0, 45.0, 0.25))
    assert peak < P.ACIGAIN_HOLD_MAX_NM - 10.0

  def test_kill_switch_reaches_raw_domain(self):
    import numpy as np
    from opendbc.car.hyundai import carcontroller as cc
    oldB, oldG = P.ACIGAIN_HOLD_BASE_V, P.ACIGAIN_HOLD_LAGAIN_V
    try:
      P.ACIGAIN_HOLD_BASE_V = np.zeros(5)
      P.ACIGAIN_HOLD_LAGAIN_V = np.zeros(5)
      assert cc.compute_hold_torque(15.0, 1.0) == 0.0
    finally:
      P.ACIGAIN_HOLD_BASE_V, P.ACIGAIN_HOLD_LAGAIN_V = oldB, oldG


class TestPhase34CrawlGate:
  # Still-straight crawl override (values.py CRAWL_STILL_*). The comp is only
  # ON where op effectively actuates, and at v<3 that is traffic-following
  # stop-and-go creep (Phase 18 passthroughs lead-less creep — the garage
  # attempt 0x55-0x57 measured 100% PAUSED, a released wheel, and was
  # discarded). Eff-active commute frames (0x51-0x54) read p45=28 Nm on a
  # centred non-moving wheel vs the extrapolated B=140, which belongs to the
  # MOVING/turned dry-friction states. All scenarios below therefore carry a
  # near lead (lead_dist=6 < TRAFFIC_FOLLOW_NEAR 8 m) so op stays active at
  # v=2 like the real regime. Gate lives at the compute_hold_torque CALL
  # SITE only; the pure function is untouched.

  def test_still_straight_crawl_comp_drops(self):
    sim = Sim()
    settle(sim, v=15.0, wheel=0.0)                       # comp on, ~73.5
    run_signal(sim, 120, v=2.0, wheel=0.0, tq=0.0, lead_dist=6.0)
    assert abs(sim.s.hold_comp_last - P.ACIGAIN_CRAWL_STILL_COMP_NM) < 1.0

  def test_straight_crawl_grip_sensitivity_restored(self):
    # raw 310 straight at crawl: driver_tq ~240 > 230 -> pressed (the
    # user-set raw-300 recognition point). Under the old flat 140 this read
    # ~170 and pressed needed raw ~370 (behind the EPS hardware flag at 350).
    sim = Sim()
    settle(sim, v=15.0, wheel=0.0)
    run_signal(sim, 120, v=2.0, wheel=0.0, tq=0.0, lead_dist=6.0)
    run_signal(sim, 20, v=2.0, wheel=0.0, tq=310.0, lead_dist=6.0)
    assert sim.s.driver_pressed

  def test_straight_crawl_light_touch_stays_unpressed(self):
    # a light resting hand (raw 260 < the 300 recognition point) must NOT
    # read as a grip — the 70 override absorbs the light-touch tail
    sim = Sim()
    settle(sim, v=15.0, wheel=0.0)
    run_signal(sim, 120, v=2.0, wheel=0.0, tq=0.0, lead_dist=6.0)
    run_signal(sim, 100, v=2.0, wheel=0.0, tq=260.0, lead_dist=6.0)
    assert not sim.s.driver_pressed

  def test_gate_exits_on_angle(self):
    sim = Sim()
    settle(sim, v=15.0, wheel=0.0)
    run_signal(sim, 120, v=2.0, wheel=0.0, tq=0.0, lead_dist=6.0)
    run_signal(sim, 120, v=2.0, wheel=10.0, tq=0.0, lead_dist=6.0)  # |ang| >= 5
    assert sim.s.hold_comp_last > 130.0

  def test_gate_exits_on_rate(self):
    sim = Sim()
    settle(sim, v=15.0, wheel=0.0)
    run_signal(sim, 120, v=2.0, wheel=0.0, tq=0.0, lead_dist=6.0)
    run_signal(sim, 120, v=2.0, wheel=0.0, tq=0.0, lead_dist=6.0, wheel_rate=20.0)
    assert sim.s.hold_comp_last > 130.0                  # moving-friction regime

  def test_rate_hysteresis_no_flap(self):
    # the derived wheel rate is quantized in 10 deg/s steps with p90 exactly
    # ON 10.0 — a single threshold measured 4.2 toggles/s (verify round).
    # In the 10-14 band the gate must HOLD its current state, both ways.
    sim = Sim()
    settle(sim, v=15.0, wheel=0.0)
    run_signal(sim, 120, v=2.0, wheel=0.0, tq=0.0, lead_dist=6.0)   # gate on
    run_signal(sim, 60, v=2.0, wheel=0.0, tq=0.0, lead_dist=6.0, wheel_rate=12.0)
    assert abs(sim.s.hold_comp_last - P.ACIGAIN_CRAWL_STILL_COMP_NM) < 1.0  # held on
    run_signal(sim, 120, v=2.0, wheel=0.0, tq=0.0, lead_dist=6.0, wheel_rate=20.0)
    assert sim.s.hold_comp_last > 130.0                              # exited (>14)
    run_signal(sim, 60, v=2.0, wheel=0.0, tq=0.0, lead_dist=6.0, wheel_rate=12.0)
    assert sim.s.hold_comp_last > 130.0                              # held off (needs <10)
    run_signal(sim, 120, v=2.0, wheel=0.0, tq=0.0, lead_dist=6.0, wheel_rate=0.0)
    assert abs(sim.s.hold_comp_last - P.ACIGAIN_CRAWL_STILL_COMP_NM) < 1.0  # re-entered

  def test_gate_off_above_speed(self):
    sim = Sim()
    settle(sim, v=15.0, wheel=0.0)
    run_signal(sim, 150, v=4.0, wheel=0.0, tq=0.0, lead_dist=6.0)  # 4 m/s: no gate
    assert sim.s.hold_comp_last > 130.0

  def test_leadless_creep_stays_passthrough(self):
    # WITHOUT a lead, Phase 18 passthroughs creep: comp gates to 0 (26b) and
    # the raw 250 pressed base applies — the Phase 34 gate must not resurrect
    # compensation there.
    sim = Sim()
    settle(sim, v=15.0, wheel=0.0)
    run_signal(sim, 200, v=2.0, wheel=0.0, tq=0.0)
    assert sim.s.hold_comp_last < 1.0

  def test_kill_switch(self):
    old = P.CRAWL_STILL_SPEED_MS
    try:
      P.CRAWL_STILL_SPEED_MS = 0.0
      sim = Sim()
      settle(sim, v=15.0, wheel=0.0)
      run_signal(sim, 150, v=2.0, wheel=0.0, tq=0.0, lead_dist=6.0)
      assert sim.s.hold_comp_last > 130.0                # Phase 31 behaviour
    finally:
      P.CRAWL_STILL_SPEED_MS = old

  def test_nan_rate_fails_sensitive(self):
    # a NaN rate frame must not crash and lands on the SENSITIVE side (gate
    # on -> comp low -> a gripping driver is seen sooner, never later)
    sim = Sim()
    settle(sim, v=15.0, wheel=0.0)
    run_signal(sim, 120, v=2.0, wheel=0.0, tq=0.0, lead_dist=6.0,
               wheel_rate=float('nan'))
    assert abs(sim.s.hold_comp_last - P.ACIGAIN_CRAWL_STILL_COMP_NM) < 1.0

  def test_pure_function_untouched(self):
    from opendbc.car.hyundai.carcontroller import compute_hold_torque as h
    assert abs(h(2.0, 0.0) - 140.0) < 0.1


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
    # pinned to the intent arm specifically (31b review: the or-form could
    # silently pass via the anchor within 1 Nm of harness inequality)
    assert sim.s.angle_passive_active

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

  def test_anchor_holds_at_field_event_torque(self):
    # 0x41 seg17 measured 430-500 Nm: at 460 driver_pressed fires (driver_tq
    # ~274 > 230) and, since 31b, anchors on its own -- apply stays on the
    # held wheel at the documented field torque
    sim = Sim()
    settle(sim, v=24.0, wheel=8.0, cmd=8.0)
    run_signal(sim, 100, v=24.0, wheel=8.0, cmd=14.0, tq=460.0)
    assert sim.s.driver_pressed
    assert abs(sim.s.apply_angle_last - 8.0) < 1.5

  def test_anchor_from_driver_pressed_alone_midband(self):
    # 31b review fix: a steady 300 Nm one-hand hold at ~100 km/h (raw-equiv
    # band 292-325 where override_factor < 0.9) must still anchor
    sim = Sim()
    settle(sim, v=28.0, wheel=0.0, cmd=0.0)
    run_signal(sim, 100, v=28.0, wheel=0.0, cmd=6.0, tq=300.0)
    assert sim.s.driver_pressed
    assert abs(sim.s.apply_angle_last) < 1.5   # anchored to the held wheel

  def test_release_reanchor_dumps_divergence(self):
    # anchor dropout gap: a real grip (460, anchored) decays to 250 — 31b
    # widened the anchor into the pressed-hysteresis band (raw >= ~258
    # keeps it latched), so the true gap now starts below that: pressed
    # exits, the arm holds, and apply diverges toward the plan while the
    # driver still lightly holds. On the final let-go the stored divergence
    # must be dumped to the wheel, not delivered, and the trim zeroed.
    sim = Sim()
    settle(sim, v=15.0, wheel=0.0, cmd=0.0)
    run_signal(sim, 100, v=15.0, wheel=0.0, cmd=8.0, tq=460.0)   # anchored grip
    assert abs(sim.s.apply_angle_last) < 1.0
    run_signal(sim, 150, v=15.0, wheel=0.0, cmd=8.0, tq=250.0)   # dropout gap (driver_tq ~176)
    assert sim.s.reanchor_arm >= 30
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
    run_signal(sim, 100, v=15.0, wheel=0.0, cmd=8.0, tq=250.0)   # below the 31b anchor band
    sim.step(v=15.0, wheel=0.0, cmd=8.0, tq=0.0)                 # first fire
    assert sim.s.reanchor_arm >= 30                              # memory kept
    run_signal(sim, 50, v=15.0, wheel=0.0, cmd=8.0, tq=250.0)    # re-grip, diverge again
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
    run_signal(sim, 100, v=15.0, wheel=0.0, cmd=8.0, tq=250.0)   # below the 31b anchor band
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


class TestPhase32LowSpeedShake:
  # 0x4a-0x4b regression fix: churn absorption + ceiling ladder step

  def test_low_speed_churn_absorbed(self):
    # 0.4 deg 1.5 Hz plan churn at 14 km/h must not reach TX (band 0.5 deg)
    import math
    import numpy as np
    sim = Sim()
    settle(sim, v=4.0, wheel=0.0, cmd=0.0)
    ap = []
    for i in range(600):
      sim.step(v=4.0, tq=0.0, wheel=0.0,
               cmd=0.4 * math.sin(2 * math.pi * 1.5 * i * 0.01))
      if i > 200: ap.append(sim.s.apply_angle_last)
    x = np.array(ap); x = x - x.mean()
    f = np.fft.rfftfreq(len(x), 0.01); X = np.abs(np.fft.rfft(x)) ** 2
    band = float(np.sqrt(X[(f >= 0.8) & (f <= 2.5)].sum() / len(x) ** 2 * 2))
    assert band < 0.06

  def test_highway_hysteresis_unchanged(self):
    # at 54 km/h the band is the legacy 0.15: a 0.4 deg move passes through
    sim = Sim()
    settle(sim, v=15.0, wheel=0.0, cmd=0.0)
    run_signal(sim, 100, v=15.0, wheel=0.0, cmd=0.4, tq=0.0)
    assert sim.s.apply_angle_last > 0.2

  def test_large_low_speed_turn_not_lagged(self):
    # intersection-scale command (30 deg) at low speed: the 0.5 band is
    # negligible, apply must keep advancing normally
    sim = Sim()
    settle(sim, v=4.0, wheel=0.0, cmd=0.0)
    tr = run_signal(sim, 300, v=4.0, wheel=lambda i: min(0.15 * i, 30.0), cmd=30.0, tq=0.0)
    assert sim.s.apply_angle_last > 20.0


class TestBlindspotCorrectionGate:
  # Phase 33: a large pending correction toward an occupied lane recovers
  # at the tapered rate; a clear side keeps the legacy fast recovery.

  def _recovery_gain(self, bs_l=False, bs_r=False, cmd=8.0):
    # grip to drop the gain, then release with the plan far LEFT (err > 0)
    sim = Sim()
    settle(sim, v=25.0, wheel=0.0, cmd=0.0)
    run_signal(sim, 100, v=25.0, wheel=0.0, cmd=cmd, tq=460.0)   # gain at floor
    run_signal(sim, 250, v=25.0, wheel=0.0, cmd=cmd, tq=0.0,     # release, recover
               bs_l=bs_l, bs_r=bs_r)
    return sim.s.aci_gain_last

  def test_occupied_side_slows_recovery(self):
    g_clear = self._recovery_gain()
    g_occ = self._recovery_gain(bs_l=True)      # err>0 pulls LEFT, left occupied
    assert g_occ < g_clear - 0.1

  def test_opposite_side_occupied_is_ignored(self):
    g_clear = self._recovery_gain()
    g_opp = self._recovery_gain(bs_r=True)      # pending swing is LEFT; right car
    assert abs(g_opp - g_clear) < 0.05

  def test_small_error_not_gated(self):
    # err < 3 deg: normal recovery even with the side occupied
    sim = Sim()
    settle(sim, v=25.0, wheel=0.0, cmd=0.0)
    run_signal(sim, 100, v=25.0, wheel=0.0, cmd=1.5, tq=460.0)
    run_signal(sim, 250, v=25.0, wheel=0.0, cmd=1.5, tq=0.0, bs_l=True)
    sim2 = Sim()
    settle(sim2, v=25.0, wheel=0.0, cmd=0.0)
    run_signal(sim2, 100, v=25.0, wheel=0.0, cmd=1.5, tq=460.0)
    run_signal(sim2, 250, v=25.0, wheel=0.0, cmd=1.5, tq=0.0)
    assert abs(sim.s.aci_gain_last - sim2.s.aci_gain_last) < 0.05

  def test_hold_decays(self):
    sim = Sim()
    settle(sim, v=25.0, wheel=0.0, cmd=0.0)
    sim.step(v=25.0, wheel=0.0, cmd=0.0, tq=0.0, bs_l=True)
    assert sim.s.blind_left_hold == P.BLIND_HOLD_FRAMES
    run_signal(sim, 120, v=25.0, wheel=0.0, cmd=0.0, tq=0.0)
    assert sim.s.blind_left_hold == 0


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


class TestPhase35GripAtSpeed:
  # 35a: at >= 60 km/h a real push drops authority fast and deep; city unchanged.
  def _grip(self, v, frames=40, tq=460.0):
    sim = Sim()
    settle(sim, v=v, wheel=0.0)
    run_signal(sim, 200, v=v, wheel=0.0, cmd=0.0, tq=0.0)      # gain at ceiling
    g0 = sim.s.aci_gain_last
    tr = run_signal(sim, frames, v=v, wheel=0.0, cmd=0.0, tq=tq)  # EPS-pressed grip
    return g0, tr['gain']

  def test_highway_grip_drops_fast_and_deep(self):
    g0, g = self._grip(v=22.0)                                     # 79 km/h
    assert g0 > 0.7
    assert g[29] <= 0.10, f"gain after 0.3 s of grip = {g[29]:.3f}"   # ~0.03/frame drop
    assert min(g) <= 0.03 + 0.004 + 1e-6                           # floor 0.03 (0.004 quantizer)

  def test_city_grip_unchanged(self):
    g0, g = self._grip(v=11.0, frames=60)                          # 40 km/h (schedule start; below = passthrough in harness)
    assert g[29] > 0.15, "city drop rate must keep the slow legacy curve"
    assert min(g) >= 0.08 - 1e-6                                   # floor 0.08 kept

  def test_resting_hand_keeps_slow_curve_at_speed(self):
    # driver_tq below the descent gate: rate_dn floor must NOT apply
    g0, g = self._grip(v=22.0, tq=140.0)                           # driver_tq ~78 at 22 m/s
    assert g[29] > 0.5

  def test_resting_hand_crossing_gate_band_no_ratchet(self):
    # verification round: hands-off driver_tq crosses 100 ~3x/s at speed. A
    # descent gate at 100 ran a 7.5x asymmetric ratchet (mean hands-off gain
    # -8%). Drive driver_tq across 100-150 at 3 Hz for 3 s: mean gain must
    # stay near the untouched level (gate is pressed OR >= 160).
    import math
    sim = Sim()
    settle(sim, v=22.0, wheel=0.0)
    run_signal(sim, 200, v=22.0, wheel=0.0, cmd=0.0, tq=0.0)
    g_ref = sim.s.aci_gain_last
    comp = sim.s.hold_comp_last
    tr = run_signal(sim, 300, v=22.0, wheel=0.0, cmd=0.0,
                    tq_fn=lambda i: comp + 125.0 + 25.0 * math.sin(2 * math.pi * 3.0 * i * 0.01))
    g = tr['gain']
    assert not sim.s.driver_pressed
    # the yield curve itself lowers gain in this band by design (Phase 22);
    # the assertion is that 35a adds NO ratchet on top: compare against the
    # same scenario with the descent floor killed.
    old = P.ACIGAIN_GRIP_RATE_DN_FLOOR_V
    try:
      P.ACIGAIN_GRIP_RATE_DN_FLOOR_V = [0.0, 0.0]
      sim2 = Sim()
      settle(sim2, v=22.0, wheel=0.0)
      run_signal(sim2, 200, v=22.0, wheel=0.0, cmd=0.0, tq=0.0)
      tr2 = run_signal(sim2, 300, v=22.0, wheel=0.0, cmd=0.0,
                       tq_fn=lambda i: comp + 125.0 + 25.0 * math.sin(2 * math.pi * 3.0 * i * 0.01))
    finally:
      P.ACIGAIN_GRIP_RATE_DN_FLOOR_V = old
    m35, m0 = sum(g) / len(g), sum(tr2['gain']) / len(tr2['gain'])
    assert m35 >= 0.97 * m0, f"ratchet: mean gain {m35:.3f} vs no-floor {m0:.3f}"
    assert g_ref > 0.7

  def test_kill_switch_restores_phase25_schedule(self):
    # value-only kill must reproduce the pre-35a Phase 23/25 schedules: at
    # 120 km/h floor 0.06 (= [80,140]->[0.08,0.05]) and full 80; slow descent.
    old = (P.ACIGAIN_GRIP_RATE_DN_FLOOR_V, P.ACIGAIN_GRIP_FLOOR35_V, P.ACIGAIN_GRIP_FULL35_V)
    try:
      P.ACIGAIN_GRIP_RATE_DN_FLOOR_V = [0.0, 0.0]
      P.ACIGAIN_GRIP_FLOOR35_V = [0.08, 0.08, 0.08, 0.05]
      P.ACIGAIN_GRIP_FULL35_V = [110.0, 102.5, 80.0]
      g0, g = self._grip(v=33.3, frames=120)                       # 120 km/h
      assert g[29] > 0.15                                          # no fast descent
      assert abs(min(g) - 0.06) <= 0.004 + 1e-6, f"floor {min(g):.3f} != Phase 25 0.06"
    finally:
      P.ACIGAIN_GRIP_RATE_DN_FLOOR_V, P.ACIGAIN_GRIP_FLOOR35_V, P.ACIGAIN_GRIP_FULL35_V = old


class TestPhase35AnchoredRecovery:
  # 35b: after a released grip at >= 40 km/h the recovery from the wheel is 3x
  # faster in the >2 deg region; the delivery stays VM-bounded.
  def _release(self, v):
    sim = Sim()
    settle(sim, v=v, wheel=0.0)
    run_signal(sim, 100, v=v, wheel=10.0, cmd=0.0, tq=460.0)      # driver holds 10 deg off plan (anchored)
    tr = run_signal(sim, 150, v=v, wheel=10.0, cmd=0.0, tq=0.0)   # release, plan still 10 deg away
    return tr

  def test_recovery_faster_at_speed(self):
    tr = self._release(v=17.0)                                     # 61 km/h
    g = tr['gain']
    k = next((i for i, x in enumerate(g) if x >= 0.6), None)
    assert k is not None and k <= 70, f"gain reached 0.6 at frame {k}"
    d = [abs(b - a) for a, b in zip(tr['apply'], tr['apply'][1:])]
    assert max(d) <= 0.6, f"apply step {max(d):.2f} deg/frame — delivery must stay VM-bounded"

  def test_recovery_legacy_below_40kph(self):
    tr = self._release(v=8.0)                                      # 29 km/h
    g = tr['gain']
    k = next((i for i, x in enumerate(g) if x >= 0.6), None)
    assert k is None or k > 70

  def test_decayed_arm_keeps_taper(self):
    # verification round: the fast anchored recovery is gated on the arm —
    # if the arm has decayed (an invisible touch let it run out while the
    # wheel walked off), no one-shot can dump the stored divergence, so the
    # command is STALE and recovery must stay on the 0.004 taper. The
    # harness cannot hold gain low through a sub-arm touch (the error boost
    # lifts it), so the decayed arm is set directly at the release edge.
    def k60(tr):
      return next((i for i, x in enumerate(tr['gain']) if x >= 0.6), 10**6)
    def release(decay_arm):
      sim = Sim()
      settle(sim, v=17.0, wheel=0.0)
      run_signal(sim, 100, v=17.0, wheel=8.0, cmd=0.0, tq=460.0)   # anchored grip, wheel 8 off plan
      if decay_arm:
        sim.s.reanchor_arm = 0                                     # arm ran out during an invisible touch
      return k60(run_signal(sim, 150, v=17.0, wheel=8.0, cmd=0.0, tq=0.0))
    fresh, stale = release(False), release(True)
    assert fresh <= 70, fresh
    assert stale >= fresh + 30, f"decayed-arm release recovered like a fresh one ({stale} vs {fresh})"

  def test_window_expires(self):
    old = P.ANCHORED_RECOVERY_FRAMES
    try:
      P.ANCHORED_RECOVERY_FRAMES = 0                               # kill switch
      tr = self._release(v=17.0)
      g = tr['gain']
      k = next((i for i, x in enumerate(g) if x >= 0.6), None)
      assert k is None or k > 70
    finally:
      P.ANCHORED_RECOVERY_FRAMES = old
