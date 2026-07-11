from types import SimpleNamespace
import math

import pytest

from opendbc.can import CANPacker, CANParser
from opendbc.car.tesla.values import TeslaFlags
from opendbc.sunnypilot.car.tesla.speed_profile import (
  DRIVER_ASSIST_CONTROL,
  DRIVER_ASSIST_CONTROL_BYTES,
  GTW_CAR_CONFIG,
  GTW_CAR_CONFIG_BYTES,
  PERSONALITY_CONFIRMATION_SAMPLES,
  SpeedProfileProtocol,
  SunnypilotPersonality,
  TeslaSpeedProfile,
  TeslaSpeedProfileInputState,
  detect_das_hardware,
  personality_for_follow_distance,
  personality_for_profile,
  profile_for_follow_distance,
  raw_can_values,
  select_protocol,
)


HW3_PROFILES = {
  1: TeslaSpeedProfile.hurry,
  2: TeslaSpeedProfile.normal,
  3: TeslaSpeedProfile.chill,
}
HW4_PROFILES = {
  1: TeslaSpeedProfile.mad_max,
  2: TeslaSpeedProfile.hurry,
  3: TeslaSpeedProfile.normal,
  4: TeslaSpeedProfile.chill,
  5: TeslaSpeedProfile.sloth,
}
HW3_PERSONALITIES = {
  1: SunnypilotPersonality.aggressive,
  2: SunnypilotPersonality.standard,
  3: SunnypilotPersonality.relaxed,
}
HW4_PERSONALITIES = {
  1: SunnypilotPersonality.aggressive,
  2: SunnypilotPersonality.aggressive,
  3: SunnypilotPersonality.standard,
  4: SunnypilotPersonality.relaxed,
  5: SunnypilotPersonality.relaxed,
}


@pytest.mark.parametrize("protocol,profiles,personalities", [
  (SpeedProfileProtocol.hw3, HW3_PROFILES, HW3_PERSONALITIES),
  (SpeedProfileProtocol.hw4_fsd14, HW4_PROFILES, HW4_PERSONALITIES),
  (SpeedProfileProtocol.unknown, {}, {}),
])
def test_follow_distance_mapping_is_exhaustive(protocol, profiles, personalities):
  for follow_distance in range(8):
    assert profile_for_follow_distance(protocol, follow_distance) == profiles.get(follow_distance)
    assert personality_for_follow_distance(protocol, follow_distance) == personalities.get(follow_distance)


@pytest.mark.parametrize("profile,personality", [
  (TeslaSpeedProfile.sloth, SunnypilotPersonality.relaxed),
  (TeslaSpeedProfile.chill, SunnypilotPersonality.relaxed),
  (TeslaSpeedProfile.normal, SunnypilotPersonality.standard),
  (TeslaSpeedProfile.hurry, SunnypilotPersonality.aggressive),
  (TeslaSpeedProfile.mad_max, SunnypilotPersonality.aggressive),
  (None, None),
])
def test_named_tesla_profiles_collapse_to_three_personalities(profile, personality):
  assert personality_for_profile(profile) == personality


@pytest.mark.parametrize("dat,expected", [
  (b"\x80\x00\x00\x00\x00\x00\x00\x00", 2),
  (b"\xC0\x00\x00\x00\x00\x00\x00\x00", 3),
  (b"\x00" * 8, None),
  (b"\x05\x00\x00\x55\x00\x00\x00\x00", 0),
  (b"\x40\x00\x00\x55\x00\x00\x00\x00", 1),
  (b"\x80" * 7, None),
])
def test_detect_das_hardware(dat, expected):
  assert detect_das_hardware(dat) == expected


@pytest.mark.parametrize("das_hw,fsd_14,expected", [
  (2, False, SpeedProfileProtocol.hw3),
  (3, True, SpeedProfileProtocol.hw4_fsd14),
  (None, True, SpeedProfileProtocol.hw4_fsd14),
  (2, True, SpeedProfileProtocol.unknown),
  (3, False, SpeedProfileProtocol.unknown),
  (None, False, SpeedProfileProtocol.unknown),
  (0, True, SpeedProfileProtocol.unknown),
  (1, True, SpeedProfileProtocol.unknown),
])
def test_protocol_requires_confirmed_hardware_and_firmware(das_hw, fsd_14, expected):
  assert select_protocol(das_hw, fsd_14) == expected


@pytest.mark.parametrize("message,signals,address", [
  (GTW_CAR_CONFIG, GTW_CAR_CONFIG_BYTES, 0x398),
  (DRIVER_ASSIST_CONTROL, DRIVER_ASSIST_CONTROL_BYTES, 0x3F8),
])
def test_raw_input_dbc_round_trip(message, signals, address):
  dat = bytes.fromhex("12 34 56 78 9a bc de f0")
  packer = CANPacker("tesla_model3_party")
  assert packer.make_can_msg(message, 0, raw_can_values(dat, signals)) == (address, dat, 0)


def test_no_autopilot_control_input_or_dbc_message():
  messages = TeslaSpeedProfileInputState.parser_messages(True)
  assert [name for name, _ in messages] == [GTW_CAR_CONFIG, DRIVER_ASSIST_CONTROL]
  assert all(math.isnan(frequency) for _, frequency in messages)

  dbc = CANPacker("tesla_model3_party").dbc
  assert 0x3FD not in dbc.msgs
  assert "UI_autopilotControl" not in dbc.name_to_msg


@pytest.mark.parametrize("size", [0, 1, 7, 9])
def test_raw_can_values_rejects_wrong_payload_size(size):
  with pytest.raises(ValueError):
    raw_can_values(bytes(size), GTW_CAR_CONFIG_BYTES)


def _make_state(*, fsd_14: bool, openpilot_longitudinal: bool = True) -> TeslaSpeedProfileInputState:
  cp = SimpleNamespace(
    flags=TeslaFlags.FSD_14 if fsd_14 else 0,
    openpilotLongitudinalControl=openpilot_longitudinal,
  )
  return TeslaSpeedProfileInputState(cp)


def _make_parser(state: TeslaSpeedProfileInputState) -> CANParser:
  return CANParser("tesla_model3_party", TeslaSpeedProfileInputState.parser_messages(state.enabled), 0)


def _hardware_frame(das_hw: int) -> bytes:
  return bytes([das_hw << 6]) + b"\x00" * 7


def _follow_frame(follow_distance: int) -> bytes:
  return b"\x00" * 5 + bytes([follow_distance << 5]) + b"\x00" * 2


def _update(state: TeslaSpeedProfileInputState, parser, frame: int,
            can_frames: list[tuple[int, bytes, int]]) -> SimpleNamespace:
  parser.update([(frame, can_frames)])
  ret_sp = SimpleNamespace()
  state.update(parser, ret_sp)
  return ret_sp


@pytest.mark.parametrize("fsd_14,das_hw,profiles,personalities", [
  (False, 2, HW3_PROFILES, HW3_PERSONALITIES),
  (True, 3, HW4_PROFILES, HW4_PERSONALITIES),
])
def test_every_valid_selector_requires_two_identical_samples(fsd_14, das_hw, profiles, personalities):
  assert PERSONALITY_CONFIRMATION_SAMPLES == 2

  for follow_distance, expected_personality in personalities.items():
    state = _make_state(fsd_14=fsd_14)
    parser = _make_parser(state)

    ret_sp = _update(state, parser, 1, [(0x398, _hardware_frame(das_hw), 0)])
    assert state.protocol != SpeedProfileProtocol.unknown
    assert ret_sp.longitudinalPersonalityRequest == SunnypilotPersonality.standard
    assert not ret_sp.longitudinalPersonalityRequestValid

    ret_sp = _update(state, parser, 2, [(0x3F8, _follow_frame(follow_distance), 0)])
    assert state.follow_distance is None
    assert not ret_sp.longitudinalPersonalityRequestValid

    ret_sp = _update(state, parser, 3, [(0x3F8, _follow_frame(follow_distance), 0)])
    assert state.follow_distance == follow_distance
    assert state.profile == profiles[follow_distance]
    assert ret_sp.longitudinalPersonalityRequest == expected_personality
    assert ret_sp.longitudinalPersonalityRequestValid


@pytest.mark.parametrize("fsd_14,das_hw,invalid_values", [
  (False, 2, (0, 4, 5, 6, 7)),
  (True, 3, (0, 6, 7)),
])
def test_invalid_startup_values_never_publish_or_default_aggressive(fsd_14, das_hw, invalid_values):
  for follow_distance in invalid_values:
    state = _make_state(fsd_14=fsd_14)
    parser = _make_parser(state)
    _update(state, parser, 1, [(0x398, _hardware_frame(das_hw), 0)])

    for frame in (2, 3):
      ret_sp = _update(state, parser, frame, [(0x3F8, _follow_frame(follow_distance), 0)])
      assert ret_sp.longitudinalPersonalityRequest == SunnypilotPersonality.standard
      assert not ret_sp.longitudinalPersonalityRequestValid
      assert state.follow_distance is None
      assert state.profile is None


def test_protocol_and_selector_in_same_drain_are_not_ordered():
  state = _make_state(fsd_14=True)
  parser = _make_parser(state)
  relaxed = _follow_frame(5)

  ret_sp = _update(state, parser, 1, [
    (0x398, _hardware_frame(3), 0),
    (0x3F8, relaxed, 0),
    (0x3F8, relaxed, 0),
  ])
  assert state.protocol == SpeedProfileProtocol.hw4_fsd14
  assert not ret_sp.longitudinalPersonalityRequestValid

  ret_sp = _update(state, parser, 2, [(0x3F8, relaxed, 0)])
  assert not ret_sp.longitudinalPersonalityRequestValid
  ret_sp = _update(state, parser, 3, [(0x3F8, relaxed, 0)])
  assert ret_sp.longitudinalPersonalityRequest == SunnypilotPersonality.relaxed
  assert ret_sp.longitudinalPersonalityRequestValid


def test_two_identical_fresh_frames_in_a_later_drain_confirm():
  state = _make_state(fsd_14=True)
  parser = _make_parser(state)
  _update(state, parser, 1, [(0x398, _hardware_frame(3), 0)])

  hurry = _follow_frame(2)
  ret_sp = _update(state, parser, 2, [(0x3F8, hurry, 0), (0x3F8, hurry, 0)])
  assert ret_sp.longitudinalPersonalityRequest == SunnypilotPersonality.aggressive
  assert ret_sp.longitudinalPersonalityRequestValid


def test_fsd14_firmware_fallback_accepts_no_hardware_frame_or_zero_stub():
  for hardware_frames in ([], [(0x398, b"\x00" * 8, 0)]):
    state = _make_state(fsd_14=True)
    parser = _make_parser(state)
    assert state.protocol == SpeedProfileProtocol.hw4_fsd14

    for frame in (1, 2):
      ret_sp = _update(state, parser, frame, [*hardware_frames, (0x3F8, _follow_frame(5), 0)])

    assert state.das_hw is None
    assert ret_sp.longitudinalPersonalityRequest == SunnypilotPersonality.relaxed
    assert ret_sp.longitudinalPersonalityRequestValid


def test_populated_hardware_conflicting_with_fsd14_fails_closed():
  state = _make_state(fsd_14=True)
  parser = _make_parser(state)

  for frame in (1, 2):
    ret_sp = _update(state, parser, frame, [(0x3F8, _follow_frame(1), 0)])
  assert ret_sp.longitudinalPersonalityRequestValid

  ret_sp = _update(state, parser, 3, [(0x398, _hardware_frame(2), 0)])
  assert state.protocol == SpeedProfileProtocol.unknown
  assert ret_sp.longitudinalPersonalityRequest == SunnypilotPersonality.standard
  assert not ret_sp.longitudinalPersonalityRequestValid

  for frame in (4, 5):
    ret_sp = _update(state, parser, frame, [(0x3F8, _follow_frame(1), 0)])
  assert not ret_sp.longitudinalPersonalityRequestValid


def test_invalid_sample_breaks_pending_confirmation_without_changing_stable_request():
  state = _make_state(fsd_14=True)
  parser = _make_parser(state)
  _update(state, parser, 1, [(0x398, _hardware_frame(3), 0)])

  aggressive = _follow_frame(1)
  invalid = _follow_frame(0)
  assert not _update(state, parser, 2, [(0x3F8, aggressive, 0)]).longitudinalPersonalityRequestValid
  assert not _update(state, parser, 3, [(0x3F8, invalid, 0)]).longitudinalPersonalityRequestValid
  assert not _update(state, parser, 4, [(0x3F8, aggressive, 0)]).longitudinalPersonalityRequestValid
  ret_sp = _update(state, parser, 5, [(0x3F8, aggressive, 0)])
  assert ret_sp.longitudinalPersonalityRequest == SunnypilotPersonality.aggressive
  assert ret_sp.longitudinalPersonalityRequestValid

  ret_sp = _update(state, parser, 6, [(0x3F8, invalid, 0)])
  assert ret_sp.longitudinalPersonalityRequest == SunnypilotPersonality.aggressive
  assert ret_sp.longitudinalPersonalityRequestValid


def test_new_selection_is_direct_and_debounced_independent_of_previous_personality():
  state = _make_state(fsd_14=True)
  parser = _make_parser(state)
  _update(state, parser, 1, [(0x398, _hardware_frame(3), 0)])

  for frame in (2, 3):
    ret_sp = _update(state, parser, frame, [(0x3F8, _follow_frame(1), 0)])
  assert ret_sp.longitudinalPersonalityRequest == SunnypilotPersonality.aggressive

  ret_sp = _update(state, parser, 4, [(0x3F8, _follow_frame(5), 0)])
  assert ret_sp.longitudinalPersonalityRequest == SunnypilotPersonality.aggressive
  assert ret_sp.longitudinalPersonalityRequestValid

  ret_sp = _update(state, parser, 5, [(0x3F8, _follow_frame(5), 0)])
  assert ret_sp.longitudinalPersonalityRequest == SunnypilotPersonality.relaxed
  assert ret_sp.longitudinalPersonalityRequestValid


def test_conflicting_protocol_clears_request_and_requires_later_confirmed_selector():
  state = _make_state(fsd_14=True)
  parser = _make_parser(state)
  _update(state, parser, 1, [(0x398, _hardware_frame(3), 0)])
  for frame in (2, 3):
    ret_sp = _update(state, parser, frame, [(0x3F8, _follow_frame(1), 0)])
  assert ret_sp.longitudinalPersonalityRequestValid

  ret_sp = _update(state, parser, 4, [(0x398, _hardware_frame(2), 0)])
  assert state.protocol == SpeedProfileProtocol.unknown
  assert ret_sp.longitudinalPersonalityRequest == SunnypilotPersonality.standard
  assert not ret_sp.longitudinalPersonalityRequestValid

  relaxed = _follow_frame(5)
  ret_sp = _update(state, parser, 5, [
    (0x398, _hardware_frame(3), 0),
    (0x3F8, relaxed, 0),
    (0x3F8, relaxed, 0),
  ])
  assert state.protocol == SpeedProfileProtocol.hw4_fsd14
  assert not ret_sp.longitudinalPersonalityRequestValid

  assert not _update(state, parser, 6, [(0x3F8, relaxed, 0)]).longitudinalPersonalityRequestValid
  ret_sp = _update(state, parser, 7, [(0x3F8, relaxed, 0)])
  assert ret_sp.longitudinalPersonalityRequest == SunnypilotPersonality.relaxed
  assert ret_sp.longitudinalPersonalityRequestValid


@pytest.mark.parametrize("openpilot_longitudinal", [True, False])
def test_input_parser_is_only_enabled_for_openpilot_longitudinal(openpilot_longitudinal):
  state = _make_state(fsd_14=True, openpilot_longitudinal=openpilot_longitudinal)
  expected = openpilot_longitudinal
  assert state.enabled is expected
  assert bool(TeslaSpeedProfileInputState.parser_messages(state.enabled)) is expected

  if not expected:
    ret_sp = SimpleNamespace()
    state.update(SimpleNamespace(vl_all={}), ret_sp)
    assert ret_sp.longitudinalPersonalityRequest == SunnypilotPersonality.standard
    assert not ret_sp.longitudinalPersonalityRequestValid
