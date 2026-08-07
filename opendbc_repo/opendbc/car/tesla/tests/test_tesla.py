import re
import unittest

from opendbc.can import CANPacker, CANParser
from opendbc.car import Bus, gen_empty_fingerprint, structs
from opendbc.car.structs import CarParams
from opendbc.car.tesla.carstate import CarState, DAS_SETTINGS_COUNTER_POLICY
from opendbc.car.tesla.interface import CarInterface
from opendbc.car.tesla.fingerprints import FW_VERSIONS
from opendbc.car.tesla.radar_interface import RADAR_START_ADDR
from opendbc.car.tesla.values import CANBUS, CAR, DBC, FSD_14_FW, TeslaFlags, TeslaSafetyFlags

Ecu = CarParams.Ecu

# Fields prefixed unknown_* we observe structurally but don't know the meaning of.
# Only `platform` has evidence-backed semantic meaning (matches car_model in FW_VERSIONS).
#
# unknown_prefix is everything before the comma; we don't split it because we don't know what its
# parts mean, but observed shape is: <family>_<package>_<triplet> (<build>), e.g.
#   TeMYG4 _ Main     _ 0.0.0 (78)     or     TeM3 _ SP_XP002p2 _ 0.0.0 (23)
#   family   package    triplet build           family  package    triplet build
#
# After the comma, the version string decomposes into:
#   platform             : E/Y/X = car model (Model 3 / Y / X). The only field with known meaning.
#   variant_code         : differentiator WITHIN a platform — hardware/trim/calibration bits packed
#                          into <digit?><letters?><3-digit series>, e.g. '4HP015', '4003', 'L014',
#                          'PR003'. We don't fully know what the parts mean individually, but the
#                          whole string identifies a specific variant within the car model.
#   software_major/minor : numeric components after the first '.' — conventional release numbers.
#                          minor is optional (e.g. 'E4S014.27' has no minor).
#
# Suspected (not confirmed): for M3/MY, `TeM3_*` outer + no-leading-digit variant_code == HW3, and
# `TeMYG4_*` outer + leading-'4' variant_code == HW4 (the 'G4' in TeMYG4 likely denotes Gen 4).
#
# Example full parse of 'TeMYG4_Main_0.0.0 (78),E4HP015.05.0':
#   unknown_prefix='TeMYG4_Main_0.0.0 (78)'
#   platform=E  variant_code=4HP015  software_major=05  software_minor=0
FW_RE = re.compile(
  rb'^(?P<unknown_prefix>.+),' +
  rb'(?P<platform>[EYX])' +
  rb'(?P<variant_code>\d?[A-Z]*\d{3})' +
  rb'\.(?P<software_major>\d+)' +
  rb'(?:\.(?P<software_minor>\d+))?$'
)

PLATFORM_TO_CAR = {
  b'E': CAR.TESLA_MODEL_3,
  b'Y': CAR.TESLA_MODEL_Y,
  b'X': CAR.TESLA_MODEL_X,
}

# Hypothesized FSD 14 profile, in terms of variant_code bookends (given software_major >= 4):
#   M3: variant_code starts with '4H',  ends with '015'
#   MY: variant_code starts with '4',   ends with '003'
# Older series (M3 '014', MY '002') are never FSD 14.
FSD_14_FW_RULE = {
  CAR.TESLA_MODEL_3: (b'4H', b'015'),
  CAR.TESLA_MODEL_Y: (b'4',  b'003'),
}


class TestTeslaFingerprint(unittest.TestCase):
  @staticmethod
  def car_fw(ecu, version, address=0x730, sub_address=0, bus=0):
    return CarParams.CarFw(ecu=ecu, fwVersion=version, address=address, subAddress=sub_address, bus=bus)

  @staticmethod
  def fingerprint(das_settings_bus=CANBUS.autopilot_party, das_settings_size=8):
    fingerprint = gen_empty_fingerprint()
    if das_settings_bus is not None:
      fingerprint[das_settings_bus][0x293] = das_settings_size
    return fingerprint

  def test_fw_platform_code(self):
    # Every EPS FW must parse and its platform letter must match the car it's filed under.
    for car_model, ecus in FW_VERSIONS.items():
      for fw in ecus.get((Ecu.eps, 0x730, None), []):
        m = FW_RE.match(fw)

        assert m is not None, f"Unparsable FW: {fw}"
        assert PLATFORM_TO_CAR[m['platform']] == car_model, f"Platform letter {m['platform']!r} != {car_model.value}: {fw}"

  def test_fsd_14_fw(self):
    for car_model, ecus in FW_VERSIONS.items():
      if car_model not in FSD_14_FW_RULE:
        continue

      variant_prefix, variant_suffix = FSD_14_FW_RULE[car_model]
      for fw in ecus.get((Ecu.eps, 0x730, None), []):
        m = FW_RE.match(fw)
        assert m is not None, f"Unparsable FW: {fw}"

        is_fsd_14 = fw in FSD_14_FW.get(car_model, [])
        expected = (
          m['variant_code'].startswith(variant_prefix)
          and m['variant_code'].endswith(variant_suffix)
          and int(m['software_major']) >= 4
        )
        assert is_fsd_14 == expected, f"{fw}"

  def test_fsd_14_interface_flags(self):
    for candidate, versions in FSD_14_FW.items():
      for version in versions:
        with self.subTest(candidate=candidate, version=version):
          CP = CarInterface.get_params(candidate, self.fingerprint(),
                                       [self.car_fw(Ecu.eps, version)], False, False, False)
          assert CP.flags & TeslaFlags.FSD_14
          assert CP.safetyConfigs[0].safetyParam & TeslaSafetyFlags.FSD_14

  def test_radar_detection(self):
    # Test radar availability detection for cars with radar DBC defined
    for radar in (True, False):
      fingerprint = gen_empty_fingerprint()
      if radar:
        fingerprint[1][RADAR_START_ADDR] = 8
      CP = CarInterface.get_params(CAR.TESLA_MODEL_3, fingerprint, [], False, False, False)
      assert CP.radarUnavailable != radar

  def test_no_radar_car(self):
    # Model X doesn't have radar DBC defined, should always be unavailable
    for radar in (True, False):
      fingerprint = gen_empty_fingerprint()
      if radar:
        fingerprint[1][RADAR_START_ADDR] = 8
      CP = CarInterface.get_params(CAR.TESLA_MODEL_X, fingerprint, [], False, False, False)
      assert CP.radarUnavailable  # Always unavailable since no radar DBC

  def test_das_settings_counter_policy_all_tesla(self):
    cases = (
      ("model 3", CAR.TESLA_MODEL_3, self.fingerprint(), []),
      ("model Y", CAR.TESLA_MODEL_Y, self.fingerprint(), []),
      ("model X without DAS_settings", CAR.TESLA_MODEL_X, self.fingerprint(None), []),
      ("unknown firmware", CAR.TESLA_MODEL_Y, self.fingerprint(), [self.car_fw(Ecu.eps, b'unknown')]),
      ("unexpected DLC", CAR.TESLA_MODEL_3, self.fingerprint(das_settings_size=7), []),
    )

    for name, candidate, fingerprint, car_fw in cases:
      with self.subTest(name=name):
        CP = CarInterface.get_params(candidate, fingerprint, car_fw, False, False, False)
        parsers = CarState.get_can_parsers(CP, structs.CarParamsSP())
        assert parsers[Bus.party].counter_policies == {}
        assert parsers[Bus.ap_party].counter_policies == {0x293: DAS_SETTINGS_COUNTER_POLICY}

  def test_das_settings_counter_policy_parser_scope(self):
    fingerprint = self.fingerprint()
    CP = CarInterface.get_params(CAR.TESLA_MODEL_Y, fingerprint, [], False, False, False)
    parsers = CarState.get_can_parsers(CP, structs.CarParamsSP())

    assert parsers[Bus.party].counter_policies == {}
    assert parsers[Bus.ap_party].counter_policies == {0x293: DAS_SETTINGS_COUNTER_POLICY}
    assert 0x293 not in parsers[Bus.ap_party].message_states

    # DAS_settings is registered dynamically when CarState first consumes it.
    parsers[Bus.ap_party].vl["DAS_settings"]
    assert parsers[Bus.ap_party].message_states[0x293].counter_policy == DAS_SETTINGS_COUNTER_POLICY

  def test_das_settings_counter_policy_ignores_phase_and_mode(self):
    CP = CarInterface.get_params(CAR.TESLA_MODEL_Y, self.fingerprint(), [], False, False, False)
    packer = CANPacker(DBC[CAR.TESLA_MODEL_Y][Bus.party])

    for start_counter in (0, 1, 15):
      for acceleration_mode in (0, 1):
        for autosteer_enabled in (0, 1):
          with self.subTest(start_counter=start_counter, acceleration_mode=acceleration_mode,
                            autosteer_enabled=autosteer_enabled):
            parser = CarState.get_can_parsers(CP, structs.CarParamsSP())[Bus.ap_party]
            parser.vl["DAS_settings"]
            first = packer.make_can_msg("DAS_settings", CANBUS.autopilot_party, {
              "DAS_settingCounter": start_counter,
              "DAS_driverAccelerationMode": acceleration_mode,
              "DAS_autosteerEnabled": autosteer_enabled,
            })
            second = packer.make_can_msg("DAS_settings", CANBUS.autopilot_party, {
              "DAS_settingCounter": (start_counter + 2) & 0xF,
              "DAS_driverAccelerationMode": acceleration_mode,
              "DAS_autosteerEnabled": autosteer_enabled,
            })

            assert parser.update([1_000_000_000, [first]]) == {0x293}
            assert parser.update([2_250_000_000, [second]]) == {0x293}
            state = parser.message_states[0x293]
            assert state.counter_fail == 0

  def test_das_settings_incident_sequence(self):
    # Exact contiguous five-+2 run and normalized timestamps from incident rlog 3.
    payloads = (
      "000c559005046ffe",  # counter 6
      "000c559005048f1e",  # counter 8
      "000c55900504af3e",  # counter 10
      "000c55900504cf5e",  # counter 12
      "000c55900504ef7e",  # counter 14
      "000c559005040f9e",  # counter 0
    )
    timestamps = (
      0,
      999_859_743,
      2_000_466_404,
      2_999_962_142,
      3_999_774_960,
      5_000_238_603,
    )

    CP = CarInterface.get_params(CAR.TESLA_MODEL_Y, self.fingerprint(), [], False, False, False)
    parser = CarState.get_can_parsers(CP, structs.CarParamsSP())[Bus.ap_party]
    parser.vl["DAS_settings"]

    for nanos, payload in zip(timestamps, payloads, strict=True):
      msg = (0x293, bytes.fromhex(payload), CANBUS.autopilot_party)
      assert parser.update([1_000_000_000 + nanos, [msg]]) == {0x293}
      assert parser.can_valid

    state = parser.message_states[0x293]
    assert state.counter == 0
    assert state.counter_fail == 0

    corrupt = bytearray(bytes.fromhex("000c559005042fbe"))
    corrupt[-1] ^= 0x1
    assert parser.update([7_000_000_000, [(0x293, bytes(corrupt), CANBUS.autopilot_party)]]) == set()
    assert state.counter == 0
    assert state.counter_fail == 0

    strict_parser = CANParser(DBC[CAR.TESLA_MODEL_Y][Bus.party], [], CANBUS.autopilot_party)
    strict_parser.vl["DAS_settings"]
    for nanos, payload in zip(timestamps, payloads, strict=True):
      msg = (0x293, bytes.fromhex(payload), CANBUS.autopilot_party)
      strict_parser.update([1_000_000_000 + nanos, [msg]])

    assert strict_parser.message_states[0x293].counter_fail == 5
    assert not strict_parser.can_valid

  def test_das_settings_counter_rejection_preserves_autosteer_interlock(self):
    CP = CarInterface.get_params(CAR.TESLA_MODEL_Y, self.fingerprint(), [], False, False, False)
    CP_SP = structs.CarParamsSP()
    car_state = CarState(CP, CP_SP)
    parsers = car_state.get_can_parsers(CP, CP_SP)
    parser = parsers[Bus.ap_party]
    parser.vl["DAS_settings"]
    packer = CANPacker(DBC[CAR.TESLA_MODEL_Y][Bus.party])

    disabled = packer.make_can_msg("DAS_settings", CANBUS.autopilot_party, {
      "DAS_settingCounter": 0,
      "DAS_autosteerEnabled": 0,
    })
    rejected_enabled = packer.make_can_msg("DAS_settings", CANBUS.autopilot_party, {
      "DAS_settingCounter": 2,
      "DAS_autosteerEnabled": 1,
    })

    assert parser.update([1_000_000_000, [disabled]]) == {0x293}
    state, _ = car_state.update(parsers)
    assert not state.invalidLkasSetting

    # +2 beyond 1.25 seconds is not an accepted policy transition, but its checksum-valid
    # signals retain legacy grace behavior so stock Autosteer fails safe.
    assert parser.update([2_250_000_001, [rejected_enabled]]) == {0x293}
    assert parser.vl["DAS_settings"]["DAS_autosteerEnabled"] == 1
    assert parser.message_states[0x293].counter == 2
    assert parser.message_states[0x293].counter_fail == 1
    state, _ = car_state.update(parsers)
    assert state.invalidLkasSetting
