from types import SimpleNamespace
import random

import pytest

from opendbc.can import CANPacker, CANParser
from opendbc.car.tesla.values import TeslaFlags
from opendbc.sunnypilot.car.interfaces import _initialize_tesla
from opendbc.sunnypilot.car.tesla.speed_profile import (
  AP_FIRST_STABLE_FRAMES,
  AUTOPILOT_CONTROL,
  AUTOPILOT_CONTROL_BYTES,
  DRIVER_ASSIST_CONTROL,
  DRIVER_ASSIST_CONTROL_BYTES,
  GTW_CAR_CONFIG,
  GTW_CAR_CONFIG_BYTES,
  SpeedProfileProtocol,
  TeslaSpeedProfileCarController,
  TeslaSpeedProfileState,
  apply_speed_profile,
  detect_das_hardware,
  profile_for_follow_distance,
  raw_can_values,
  select_protocol,
)
from opendbc.sunnypilot.car.tesla.values import TeslaFlagsSP, TeslaSafetyFlagsSP


@pytest.mark.parametrize("protocol,mapping", [
  (SpeedProfileProtocol.hw3, {1: 2, 2: 1, 3: 0}),
  (SpeedProfileProtocol.hw4_fsd14, {1: 3, 2: 2, 3: 1, 4: 0, 5: 4}),
])
def test_follow_distance_mapping(protocol, mapping):
  for follow_distance in range(8):
    assert profile_for_follow_distance(protocol, follow_distance) == mapping.get(follow_distance)


@pytest.mark.parametrize("dat,expected", [
  (b"\x80\x00\x00\x00\x00\x00\x00\x00", 2),
  (b"\xC0\x00\x00\x00\x00\x00\x00\x00", 3),
  (b"\x00" * 8, 0),
  (b"\x05\x00\x00\x55\x00\x00\x00\x00", 0),
  (b"\x80" * 7, 0),
])
def test_detect_das_hardware(dat, expected):
  assert detect_das_hardware(dat) == expected


@pytest.mark.parametrize("das_hw,fsd_14,expected", [
  (2, False, SpeedProfileProtocol.hw3),
  (3, True, SpeedProfileProtocol.hw4_fsd14),
  (2, True, SpeedProfileProtocol.unknown),
  (3, False, SpeedProfileProtocol.unknown),
  (0, False, SpeedProfileProtocol.unknown),
])
def test_protocol_requires_confirmed_hardware_and_firmware(das_hw, fsd_14, expected):
  assert select_protocol(das_hw, fsd_14) == expected


def test_hw3_changes_only_profile_bits():
  rng = random.Random(0x3FD)
  for profile in range(3):
    for _ in range(100):
      original = bytearray(rng.randbytes(8))
      original[0] &= 0xF8  # mux 0
      original[4] |= 0x40  # FSD selected
      original[6] = (original[6] & ~0x06) | (((profile + 1) % 3) << 1)
      modified = apply_speed_profile(bytes(original), SpeedProfileProtocol.hw3, profile)
      assert modified is not None
      assert modified[6] & 0x06 == profile << 1
      assert bytes(a ^ b for a, b in zip(original, modified, strict=True)) == bytes([0, 0, 0, 0, 0, 0, (original[6] ^ modified[6]), 0])


def test_hw4_changes_only_profile_bits():
  rng = random.Random(14)
  for profile in range(5):
    for _ in range(100):
      original = bytearray(rng.randbytes(8))
      original[0] = (original[0] & 0xF8) | 2  # mux 2
      original[7] = (original[7] & ~0xE0) | (((profile + 1) % 5) << 5)
      modified = apply_speed_profile(bytes(original), SpeedProfileProtocol.hw4_fsd14, profile)
      assert modified is not None
      assert modified[7] & 0xE0 == profile << 5
      assert bytes(a ^ b for a, b in zip(original, modified, strict=True)) == bytes([0, 0, 0, 0, 0, 0, 0, (original[7] ^ modified[7])])


@pytest.mark.parametrize("dat,protocol,profile", [
  (b"\x01" + b"\x00" * 7, SpeedProfileProtocol.hw3, 1),
  (b"\x00\x00\x00\x00\x00\x00\x00\x00", SpeedProfileProtocol.hw3, 1),
  (b"\x01" + b"\x00" * 7, SpeedProfileProtocol.hw4_fsd14, 1),
  (b"\x02" + b"\x00" * 7, SpeedProfileProtocol.unknown, 1),
  (b"\x02" + b"\x00" * 7, SpeedProfileProtocol.hw4_fsd14, None),
])
def test_non_target_or_unconfirmed_frame_is_not_sent(dat, protocol, profile):
  assert apply_speed_profile(dat, protocol, profile) is None


@pytest.mark.parametrize("dat,protocol,profile", [
  (bytes.fromhex("00 11 22 33 40 55 04 77"), SpeedProfileProtocol.hw3, 2),
  (bytes.fromhex("02 11 22 33 44 55 66 60"), SpeedProfileProtocol.hw4_fsd14, 3),
])
def test_profile_already_present_is_not_shadowed(dat, protocol, profile):
  assert apply_speed_profile(dat, protocol, profile) is None


@pytest.mark.parametrize("message,signals,address", [
  (GTW_CAR_CONFIG, GTW_CAR_CONFIG_BYTES, 0x398),
  (DRIVER_ASSIST_CONTROL, DRIVER_ASSIST_CONTROL_BYTES, 0x3F8),
  (AUTOPILOT_CONTROL, AUTOPILOT_CONTROL_BYTES, 0x3FD),
])
def test_raw_dbc_round_trip(message, signals, address):
  dat = bytes.fromhex("12 34 56 78 9a bc de f0")
  packer = CANPacker("tesla_model3_party")
  assert packer.make_can_msg(message, 0, raw_can_values(dat, signals)) == (address, dat, 0)


def _make_state(*, fsd_14: bool) -> TeslaSpeedProfileState:
  cp = SimpleNamespace(flags=TeslaFlags.FSD_14 if fsd_14 else 0)
  cp_sp = SimpleNamespace(flags=TeslaFlagsSP.SPEED_PROFILE)
  return TeslaSpeedProfileState(cp, cp_sp)


def test_state_consumes_only_fresh_frames_and_keeps_latest_per_mux():
  state = _make_state(fsd_14=True)
  parser = CANParser("tesla_model3_party", TeslaSpeedProfileState.parser_messages(True), 0)

  mux2_old = bytes.fromhex("02 10 20 30 40 50 60 00")
  mux1 = bytes.fromhex("01 11 21 31 41 51 61 71")
  mux2_new = bytes.fromhex("02 12 22 32 42 52 62 20")
  parser.update([(1, [
    (0x398, bytes.fromhex("c0 00 00 00 00 00 00 00"), 0),
    (0x3F8, bytes.fromhex("00 00 00 00 00 a0 00 00"), 0),
    (0x3FD, mux2_old, 0),
    (0x3FD, mux1, 0),
    (0x3FD, mux2_new, 0),
  ])])
  state.update(parser)

  assert state.protocol == SpeedProfileProtocol.hw4_fsd14
  assert state.follow_distance is None
  assert state.profile is None
  assert state.autopilot_control_frames == []

  parser.update([(2, [
    (0x3F8, bytes.fromhex("00 00 00 00 00 a0 00 00"), 0),
    (0x3FD, mux2_new, 0),
  ])])
  state.update(parser)
  assert state.follow_distance == 5
  assert state.profile == 4
  assert state.autopilot_control_frames == []

  parser.update([(3, [(0x3FD, mux2_new, 0)])])
  state.update(parser)
  assert state.autopilot_control_frames == [mux2_new]

  parser.update([(4, [
    (0x3F8, bytes.fromhex("00 00 00 00 00 20 00 00"), 0),
    (0x3FD, mux2_new, 0),
  ])])
  state.update(parser)
  assert state.profile == 3
  assert state.autopilot_control_frames == []

  parser.update([(5, [(0x3FD, mux2_new, 0)])])
  state.update(parser)
  assert state.autopilot_control_frames == [mux2_new]

  parser.update([(6, [])])
  state.update(parser)
  assert state.autopilot_control_frames == []


def test_all_zero_hardware_stub_disables_profile_tx():
  state = _make_state(fsd_14=True)
  parser = CANParser("tesla_model3_party", TeslaSpeedProfileState.parser_messages(True), 0)
  parser.update([(1, [
    (0x398, b"\x00" * 8, 0),
    (0x3F8, bytes.fromhex("00 00 00 00 00 a0 00 00"), 0),
    (0x3FD, bytes.fromhex("02 00 00 00 00 00 00 00"), 0),
  ])])
  state.update(parser)
  assert state.protocol == SpeedProfileProtocol.unknown
  assert state.profile is None


def test_hardware_recovery_requires_a_new_follow_distance_frame():
  state = _make_state(fsd_14=True)
  parser = CANParser("tesla_model3_party", TeslaSpeedProfileState.parser_messages(True), 0)

  parser.update([(1, [(0x398, bytes.fromhex("c0 00 00 00 00 00 00 00"), 0)])])
  state.update(parser)
  parser.update([(2, [(0x3F8, bytes.fromhex("00 00 00 00 00 20 00 00"), 0)])])
  state.update(parser)
  assert state.profile == 3

  parser.update([(3, [(0x398, b"\x00" * 8, 0)])])
  state.update(parser)
  assert state.protocol == SpeedProfileProtocol.unknown
  assert state.follow_distance is None
  assert state.profile is None

  # Even if recovery and a source frame share one drain, cross-ID ordering is
  # unavailable in vl_all. Wait for a later source frame just like Panda does.
  parser.update([(4, [
    (0x398, bytes.fromhex("c0 00 00 00 00 00 00 00"), 0),
    (0x3F8, bytes.fromhex("00 00 00 00 00 20 00 00"), 0),
  ])])
  state.update(parser)
  assert state.protocol == SpeedProfileProtocol.hw4_fsd14
  assert state.profile is None

  parser.update([(5, [(0x3F8, bytes.fromhex("00 00 00 00 00 20 00 00"), 0)])])
  state.update(parser)
  assert state.profile == 3


def test_transient_hardware_loss_within_one_drain_clears_source_state():
  state = _make_state(fsd_14=True)
  parser = CANParser("tesla_model3_party", TeslaSpeedProfileState.parser_messages(True), 0)
  valid_hardware = bytes.fromhex("c0 00 00 00 00 00 00 00")

  parser.update([(1, [(0x398, valid_hardware, 0)])])
  state.update(parser)
  parser.update([(2, [(0x3F8, bytes.fromhex("00 00 00 00 00 20 00 00"), 0)])])
  state.update(parser)
  assert state.profile == 3

  parser.update([(3, [
    (0x398, b"\x00" * 8, 0),
    (0x398, valid_hardware, 0),
    (0x3F8, bytes.fromhex("00 00 00 00 00 20 00 00"), 0),
  ])])
  state.update(parser)
  assert state.protocol == SpeedProfileProtocol.hw4_fsd14
  assert state.follow_distance is None
  assert state.profile is None


def test_controller_waits_for_stable_engagement_and_never_replays_stale_frame():
  cp_sp = SimpleNamespace(flags=TeslaFlagsSP.SPEED_PROFILE)
  controller = TeslaSpeedProfileCarController(cp_sp)
  state = SimpleNamespace(
    protocol=SpeedProfileProtocol.hw4_fsd14,
    profile=3,
    autopilot_control_frames=[bytes.fromhex("02 11 22 33 44 55 66 00")],
  )
  cs = SimpleNamespace(speed_profile=state)
  enabled = SimpleNamespace(enabled=True)

  for _ in range(AP_FIRST_STABLE_FRAMES):
    assert controller.update_speed_profile(enabled, cs) == []

  sends = controller.update_speed_profile(enabled, cs)
  assert sends == [(0x3FD, bytes.fromhex("02 11 22 33 44 55 66 60"), 2)]

  state.autopilot_control_frames = []
  assert controller.update_speed_profile(enabled, cs) == []

  assert controller.update_speed_profile(SimpleNamespace(enabled=False), cs) == []
  assert controller.speed_profile_engaged_frames == 0


@pytest.mark.parametrize("openpilot_longitudinal,expected", [(False, True), (True, False)])
def test_param_opens_host_and_safety_paths_only_for_stock_longitudinal(openpilot_longitudinal, expected):
  cp = SimpleNamespace(brand="tesla", openpilotLongitudinalControl=openpilot_longitudinal)
  cp_sp = SimpleNamespace(flags=0, safetyParam=0)
  _initialize_tesla(cp, cp_sp, {"TeslaSpeedProfile": "1"})

  assert bool(cp_sp.flags & TeslaFlagsSP.SPEED_PROFILE) is expected
  assert bool(cp_sp.safetyParam & TeslaSafetyFlagsSP.SPEED_PROFILE) is expected
