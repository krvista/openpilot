#!/usr/bin/env python3
import os
import time

from openpilot.cereal import messaging
from openpilot.common.hardware import COMMA_HARDWARE
import gc

from openpilot.common.realtime import set_core_affinity
from openpilot.system.ui.lib.application import gui_app
from openpilot.selfdrive.ui.layouts.main import MainLayout
from openpilot.selfdrive.ui.mici.layouts.main import MiciMainLayout
from openpilot.selfdrive.ui.ui_state import ui_state

BIG_UI = gui_app.big_ui()
UI_CORES = [0, 1, 2]   # Phase 52: pool cores only, never a control core (3-7)
UI_NICE = 5


def main():
  # Phase 52 (report §49): the UI never shares a core with a control process and never runs realtime. It sits on the
  # pool cores 0-2 (locationd / paramsd / lagd / torqued / calibrationd / dmonitoringd are SCHED_FIFO 5 there and preempt
  # it) as SCHED_OTHER nice +5; cores 3 (pandad), 4 (card / controlsd), 5 (selfdrived / plannerd / radard), 6 (camerad)
  # and 7 (modeld) are off limits. Before this it shared core 5 with selfdrived at the same FIFO level and a long
  # lane-change redraw stalled selfdrived 120 ms ("System Lagging"). gc stays disabled as before (no GC pause inside a
  # frame; the UI's memory profile is unchanged).
  cores = set(UI_CORES)
  gc.disable()
  set_core_affinity(UI_CORES)
  if COMMA_HARDWARE:
    try:
      os.nice(UI_NICE)
    except OSError:
      pass

  gui_app.init_window("UI")
  if BIG_UI:
    MainLayout()
  else:
    MiciMainLayout()

  pm = messaging.PubMaster(['uiDebug'])
  for should_render, frame_time, cpu_time in gui_app.render():
    extra_start = time.monotonic()
    ui_state.update()

    if should_render:
      # reaffine after power save offlines our core
      if COMMA_HARDWARE and os.sched_getaffinity(0) != cores:
        try:
          set_core_affinity(UI_CORES)
        except OSError:
          pass

      extra_cpu = time.monotonic() - extra_start
      msg = messaging.new_message('uiDebug')
      msg.uiDebug.cpuTimeMillis = (cpu_time + extra_cpu) * 1000
      msg.uiDebug.frameTimeMillis = frame_time * 1000
      pm.send('uiDebug', msg)


if __name__ == "__main__":
  main()
