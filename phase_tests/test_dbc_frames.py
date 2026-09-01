"""i6nv3 acceptance criterion #3 (verification round 2): packed-frame pinning.

Three defect classes reached green suites unseen during the rebase port:
phase_tests could not see the C safety layer, the safety suite could not see
DBC packing, and neither could see the other. This file is the third leg:

  1. the generated hyundai CAN FD DBC must contain NO duplicate SG_ names
     inside a single message (CANPacker silently resolves duplicates to the
     LAST definition — round 2 found our own ADAS_StrAnglReqVal shadowed at
     a wrong bit position by an alias-injection bug);
  2. the CCNC command frames our CarController transmits must pack to
     byte-identical golden frames (captured 2026-09-01 after proving the
     i6nv3 DBC packs identically to the road-validated i6nv2 DBC, checksums
     included).

If a DBC edit changes a golden frame, that is a CONSCIOUS decision: re-prove
equivalence against the last road-validated DBC before updating the bytes.
"""
import os
import re
import subprocess
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ODBC = os.path.join(REPO, 'opendbc_repo')
if ODBC not in sys.path:
  sys.path.insert(0, ODBC)

GEN_DBC = os.path.join(ODBC, 'opendbc', 'dbc', 'hyundai_canfd_generated.dbc')

# (message, values, golden hex) — checksum/counter included, so these pin the
# checksum algorithm binding (dbc-name startswith 'hyundai_canfd_generated')
# as well as the signal layout.
GOLDEN_FRAMES = [
  ("LKAS_ALT", {'ADAS_StrAnglReqVal': -12.3, 'LKAS_ANGLE_ACTIVE': 2, 'ADAS_ACIAnglTqRedcGainVal': 50,
                'LKA_MODE': 2, 'LKA_ICON': 1, 'LKAS_BYTE9_HIDDEN': 5, 'COUNTER': 7},
   "a2b0070240000000002514fed400000000000000000000000000000000000000"),
  ("LKAS_ALT", {'ADAS_StrAnglReqVal': 176.6, 'LKAS_ANGLE_ACTIVE': 1, 'ADAS_ACIAnglTqRedcGainVal': 0},
   "de440700000000000010981b0000000000000000000000000000000000000000"),
  ("LKAS", {'TORQUE_REQUEST': 240, 'STEER_REQ': 1, 'LKA_MODE': 2, 'COUNTER': 3},
   "a342030200e019000000000000000000"),
  ("LFA", {'TORQUE_REQUEST': -100, 'STEER_REQ': 1, 'LKA_MODE': 2},
   "55bf0002003817000000000000000000"),
  ("CAM_0x2a4", {'COUNTER': 1},
   "664701000000000000000000000000000000000000000000"),
  ("CCNC_0x161", {'COUNTER': 9},
   "5504090000000000000000000000000000000000000000000000000000000000"),
  ("CCNC_0x162", {'COUNTER': 2},
   "be50020000000000000000000000000000000000000000000000000000000000"),
]


def _ensure_generated():
  if not os.path.exists(GEN_DBC):
    subprocess.run([sys.executable, os.path.join(ODBC, 'opendbc', 'dbc', 'generator', 'generator.py')],
                   cwd=ODBC, env={**os.environ, 'PYTHONPATH': ODBC}, check=True,
                   capture_output=True)


class TestGeneratedDbc:
  def test_no_duplicate_signal_names_per_message(self):
    _ensure_generated()
    txt = open(GEN_DBC).read()
    offenders = []
    for m in re.finditer(r"^BO_ (\d+) (\w+): \d+.*\n((?: SG_ .*\n)+)", txt, re.M):
      names = re.findall(r" SG_ (\w+) ", m.group(3))
      dups = sorted({n for n in names if names.count(n) > 1})
      if dups:
        offenders.append((m.group(2), dups))
    assert not offenders, f"duplicate SG_ names (packer silently uses LAST def): {offenders}"

  def test_ccnc_command_frames_pack_to_golden_bytes(self):
    _ensure_generated()
    from opendbc.can import CANPacker
    packer = CANPacker("hyundai_canfd_generated")
    for msg, values, golden in GOLDEN_FRAMES:
      _, data, _ = packer.make_can_msg(msg, 0, dict(values))
      assert bytes(data).hex() == golden, (
        f"{msg} packed frame changed vs road-validated golden — a DBC edit moved "
        f"bits our TX path depends on:\n  got    {bytes(data).hex()}\n  golden {golden}")
