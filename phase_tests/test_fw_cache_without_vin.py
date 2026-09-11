"""CarParamsCache accepted on its FW list when the VIN is unknown (i6nv3: the VIN query never
succeeds on this car). No OBD-multiplexing window, no FW query; the kill switch restores upstream."""
from phase_tests.harness import make_cp  # noqa: F401 (path setup side effect)
import opendbc.car.car_helpers as ch
from opendbc.car.structs import CarParams
from opendbc.car.vin import VIN_UNKNOWN


def _cached(vin):
  cp = CarParams()
  cp.brand = "hyundai"
  cp.carVin = vin
  fw = cp.init("carFw", 1)
  fw[0].ecu = CarParams.Ecu.fwdCamera
  fw[0].fwVersion = b"\xf1\x00XX"
  return cp


def _run(monkeypatch, cached, kill=False):
  calls = []
  monkeypatch.setattr(ch, "FW_CACHE_WITHOUT_VIN", not kill)
  monkeypatch.setattr(ch, "match_fw_to_car", lambda car_fw, vin: (True, {"HYUNDAI_IONIQ_6"}))
  monkeypatch.setattr(ch, "can_fingerprint", lambda can_recv: (None, {}))
  monkeypatch.setattr(ch, "get_vin", lambda *a, **k: calls.append("vin") or (-1, -1, VIN_UNKNOWN))
  monkeypatch.setattr(ch, "get_present_ecus", lambda *a, **k: calls.append("ecus") or set())
  monkeypatch.setattr(ch, "get_fw_versions_ordered", lambda *a, **k: calls.append("fw") or [])
  obd = []
  out = ch.fingerprint(lambda *a, **k: [], lambda *a, **k: None, obd.append, cached, None)
  return out, calls, obd


def test_cache_used_without_vin(monkeypatch):
  (fp, _, vin, car_fw, source, exact), calls, obd = _run(monkeypatch, _cached(VIN_UNKNOWN))
  assert calls == []                      # no VIN / ECU / FW query at all
  assert obd == [False]                   # only the final "OBD off" -> no OBD window
  assert len(car_fw) == 1 and fp == "HYUNDAI_IONIQ_6" and source == CarParams.FingerprintSource.fw


def test_kill_switch_restores_upstream_rule(monkeypatch):
  (_, _, _, car_fw, _, _), calls, obd = _run(monkeypatch, _cached(VIN_UNKNOWN), kill=True)
  assert calls == ["vin", "ecus", "fw"]   # full query
  assert obd[0] is True and obd[-1] is False


def test_known_vin_unchanged(monkeypatch):
  (_, _, vin, car_fw, _, _), calls, obd = _run(monkeypatch, _cached("KMHL14JA0PA000000"), kill=True)
  assert calls == [] and obd == [False] and vin == "KMHL14JA0PA000000"


def test_empty_cache_still_queries(monkeypatch):
  cp = _cached(VIN_UNKNOWN)
  cp.init("carFw", 0)
  _, calls, obd = _run(monkeypatch, cp)
  assert calls == ["vin", "ecus", "fw"]
