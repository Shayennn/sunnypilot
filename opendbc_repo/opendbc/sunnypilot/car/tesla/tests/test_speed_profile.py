from itertools import permutations
from types import SimpleNamespace
import math

import pytest

from opendbc.can import CANPacker, CANParser
from opendbc.car import Bus, structs
from opendbc.car.tesla.values import TeslaFlags
from opendbc.sunnypilot.car.tesla.carstate_ext import CarStateExt
from opendbc.sunnypilot.car.tesla.speed_profile import (
  DRIVER_ASSIST_CONTROL,
  DRIVER_ASSIST_CONTROL_SELECTOR,
  GTW_CAR_CONFIG,
  GTW_CAR_CONFIG_BYTES,
  SELECTOR_CONFIRMATION_SAMPLES,
  VALID_SELECTORS,
  SpeedProfileProtocol,
  TeslaSpeedProfileInputState,
  detect_das_hardware,
  select_protocol,
)

ButtonType = structs.CarState.ButtonEvent.Type

HW3_SELECTORS = frozenset((1, 2, 3))
HW4_SELECTORS = frozenset((1, 2, 3, 4, 5))


def _assert_gap_release(events: list[structs.CarState.ButtonEvent]) -> None:
  assert len(events) == 1
  assert events[0].type == ButtonType.gapAdjustCruise
  assert not events[0].pressed


@pytest.mark.parametrize("protocol,selectors", [
  (SpeedProfileProtocol.hw3, HW3_SELECTORS),
  (SpeedProfileProtocol.hw4_fsd14, HW4_SELECTORS),
])
def test_valid_raw_selectors_are_exhaustive(protocol, selectors):
  assert VALID_SELECTORS[protocol] == selectors


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


def test_gateway_config_dbc_round_trip():
  dat = bytes.fromhex("12 34 56 78 9a bc de f0")
  packer = CANPacker("tesla_model3_party")
  values = dict(zip(GTW_CAR_CONFIG_BYTES, dat, strict=True))
  assert packer.make_can_msg(GTW_CAR_CONFIG, 0, values) == (0x398, dat, 0)


@pytest.mark.parametrize("selector", range(8))
def test_follow_distance_dbc_round_trip(selector):
  dat = b"\x00" * 5 + bytes([selector << 5]) + b"\x00" * 2
  packer = CANPacker("tesla_model3_party")
  values = {DRIVER_ASSIST_CONTROL_SELECTOR: selector}
  assert packer.make_can_msg(DRIVER_ASSIST_CONTROL, 0, values) == (0x3F8, dat, 0)


def test_no_autopilot_control_input_or_dbc_message():
  messages = TeslaSpeedProfileInputState.parser_messages(True)
  assert [name for name, _ in messages] == [GTW_CAR_CONFIG, DRIVER_ASSIST_CONTROL]
  assert all(math.isnan(frequency) for _, frequency in messages)

  dbc = CANPacker("tesla_model3_party").dbc
  assert 0x3FD not in dbc.msgs
  assert "UI_autopilotControl" not in dbc.name_to_msg


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


def _selector_frame(selector: int) -> bytes:
  return b"\x00" * 5 + bytes([selector << 5]) + b"\x00" * 2


def _update(state: TeslaSpeedProfileInputState, parser: CANParser, frame: int,
            can_frames: list[tuple[int, bytes, int]]) -> list[structs.CarState.ButtonEvent]:
  parser.update([(frame, can_frames)])
  return state.update(parser)


def _establish_protocol(state: TeslaSpeedProfileInputState, parser: CANParser, das_hw: int) -> None:
  assert not _update(state, parser, 1, [(0x398, _hardware_frame(das_hw), 0)])
  assert state.protocol != SpeedProfileProtocol.unknown


def _establish_baseline(state: TeslaSpeedProfileInputState, parser: CANParser, selector: int,
                        *, first_frame: int = 2) -> None:
  for frame in range(first_frame, first_frame + SELECTOR_CONFIRMATION_SAMPLES):
    assert not _update(state, parser, frame, [(0x3F8, _selector_frame(selector), 0)])
  assert state.selector == selector


@pytest.mark.parametrize("fsd_14,das_hw,selectors", [
  (False, 2, HW3_SELECTORS),
  (True, 3, HW4_SELECTORS),
])
def test_first_confirmed_selector_is_only_a_baseline(fsd_14, das_hw, selectors):
  assert SELECTOR_CONFIRMATION_SAMPLES == 2

  for selector in selectors:
    state = _make_state(fsd_14=fsd_14)
    parser = _make_parser(state)
    _establish_protocol(state, parser, das_hw)

    assert not _update(state, parser, 2, [(0x3F8, _selector_frame(selector), 0)])
    assert state.selector is None
    assert not _update(state, parser, 3, [(0x3F8, _selector_frame(selector), 0)])
    assert state.selector == selector
    assert not _update(state, parser, 4, [(0x3F8, _selector_frame(selector), 0)])


@pytest.mark.parametrize("fsd_14,das_hw,selectors", [
  (False, 2, HW3_SELECTORS),
  (True, 3, HW4_SELECTORS),
])
def test_every_confirmed_raw_selector_change_emits_one_release(fsd_14, das_hw, selectors):
  # Ordered pairs cover every direction, notably HW4's 1<->2 and 4<->5.
  for baseline, changed in permutations(selectors, 2):
    state = _make_state(fsd_14=fsd_14)
    parser = _make_parser(state)
    _establish_protocol(state, parser, das_hw)
    _establish_baseline(state, parser, baseline)

    assert not _update(state, parser, 4, [(0x3F8, _selector_frame(changed), 0)])
    assert state.selector == baseline
    events = _update(state, parser, 5, [(0x3F8, _selector_frame(changed), 0)])
    assert state.selector == changed
    _assert_gap_release(events)
    assert not _update(state, parser, 6, [(0x3F8, _selector_frame(changed), 0)])


def test_batched_changes_are_drained_as_distinct_button_edges():
  state = _make_state(fsd_14=True)
  parser = _make_parser(state)
  _establish_protocol(state, parser, 3)
  _establish_baseline(state, parser, 1)

  # Two confirmed changes can appear after a delayed CAN drain. Emit one edge
  # now and queue the second so selfdrived cannot collapse them through any(...).
  events = _update(state, parser, 4, [
    (0x3F8, _selector_frame(2), 0),
    (0x3F8, _selector_frame(2), 0),
    (0x3F8, _selector_frame(4), 0),
    (0x3F8, _selector_frame(4), 0),
  ])
  assert state.selector == 4
  _assert_gap_release(events)
  _assert_gap_release(_update(state, parser, 5, []))
  assert not _update(state, parser, 6, [])


@pytest.mark.parametrize("fsd_14,das_hw,invalid_values", [
  (False, 2, (0, 4, 5, 6, 7)),
  (True, 3, (0, 6, 7)),
])
def test_invalid_startup_values_never_establish_a_baseline_or_emit(fsd_14, das_hw, invalid_values):
  for selector in invalid_values:
    state = _make_state(fsd_14=fsd_14)
    parser = _make_parser(state)
    _establish_protocol(state, parser, das_hw)

    for frame in (2, 3):
      assert not _update(state, parser, frame, [(0x3F8, _selector_frame(selector), 0)])
      assert state.selector is None


def test_invalid_sample_breaks_confirmation_without_changing_baseline():
  state = _make_state(fsd_14=True)
  parser = _make_parser(state)
  _establish_protocol(state, parser, 3)
  _establish_baseline(state, parser, 1)

  assert not _update(state, parser, 4, [(0x3F8, _selector_frame(5), 0)])
  assert not _update(state, parser, 5, [(0x3F8, _selector_frame(0), 0)])
  assert not _update(state, parser, 6, [(0x3F8, _selector_frame(5), 0)])
  assert state.selector == 1
  _assert_gap_release(_update(state, parser, 7, [(0x3F8, _selector_frame(5), 0)]))
  assert state.selector == 5

  assert not _update(state, parser, 8, [(0x3F8, _selector_frame(0), 0)])
  assert state.selector == 5


def test_protocol_and_selector_in_same_drain_are_not_ordered():
  state = _make_state(fsd_14=True)
  parser = _make_parser(state)
  selector = _selector_frame(5)

  assert not _update(state, parser, 1, [
    (0x398, _hardware_frame(3), 0),
    (0x3F8, selector, 0),
    (0x3F8, selector, 0),
  ])
  assert state.protocol == SpeedProfileProtocol.hw4_fsd14
  assert state.selector is None

  assert not _update(state, parser, 2, [(0x3F8, selector, 0)])
  assert not _update(state, parser, 3, [(0x3F8, selector, 0)])
  assert state.selector == 5


def test_two_identical_fresh_frames_in_one_later_drain_confirm():
  state = _make_state(fsd_14=True)
  parser = _make_parser(state)
  _establish_protocol(state, parser, 3)

  baseline = _selector_frame(2)
  assert not _update(state, parser, 2, [(0x3F8, baseline, 0), (0x3F8, baseline, 0)])
  assert state.selector == 2

  changed = _selector_frame(3)
  _assert_gap_release(_update(state, parser, 3, [(0x3F8, changed, 0), (0x3F8, changed, 0)]))
  assert state.selector == 3


def test_fsd14_firmware_fallback_accepts_no_hardware_frame_or_zero_stub():
  for hardware_frames in ([], [(0x398, b"\x00" * 8, 0)]):
    state = _make_state(fsd_14=True)
    parser = _make_parser(state)
    assert state.protocol == SpeedProfileProtocol.hw4_fsd14

    for frame in (1, 2):
      assert not _update(state, parser, frame, [*hardware_frames, (0x3F8, _selector_frame(5), 0)])
    assert state.das_hw is None
    assert state.selector == 5

    assert not _update(state, parser, 3, [(0x3F8, _selector_frame(4), 0)])
    _assert_gap_release(_update(state, parser, 4, [(0x3F8, _selector_frame(4), 0)]))


def test_populated_hardware_conflicting_with_fsd14_fails_closed_and_drops_queued_edges():
  state = _make_state(fsd_14=True)
  parser = _make_parser(state)
  _establish_protocol(state, parser, 3)
  _establish_baseline(state, parser, 1)

  # Queue two edges, then consume only the first one.
  _assert_gap_release(_update(state, parser, 4, [
    (0x3F8, _selector_frame(2), 0),
    (0x3F8, _selector_frame(2), 0),
    (0x3F8, _selector_frame(3), 0),
    (0x3F8, _selector_frame(3), 0),
  ]))

  assert not _update(state, parser, 5, [(0x398, _hardware_frame(2), 0)])
  assert state.protocol == SpeedProfileProtocol.unknown
  assert state.selector is None
  for frame in (6, 7):
    assert not _update(state, parser, frame, [(0x3F8, _selector_frame(1), 0)])


def test_protocol_recovery_requires_a_new_baseline_without_an_edge():
  state = _make_state(fsd_14=True)
  parser = _make_parser(state)
  _establish_protocol(state, parser, 3)
  _establish_baseline(state, parser, 1)

  assert not _update(state, parser, 4, [(0x398, _hardware_frame(2), 0)])
  assert state.protocol == SpeedProfileProtocol.unknown
  assert state.selector is None

  # The selector in the recovery drain is skipped because message ordering is
  # unknown, then two later samples establish a fresh baseline without cycling.
  assert not _update(state, parser, 5, [
    (0x398, _hardware_frame(3), 0),
    (0x3F8, _selector_frame(5), 0),
    (0x3F8, _selector_frame(5), 0),
  ])
  assert not _update(state, parser, 6, [(0x3F8, _selector_frame(5), 0)])
  assert not _update(state, parser, 7, [(0x3F8, _selector_frame(5), 0)])
  assert state.selector == 5


@pytest.mark.parametrize("openpilot_longitudinal", [True, False])
def test_input_parser_is_only_enabled_for_openpilot_longitudinal(openpilot_longitudinal):
  state = _make_state(fsd_14=True, openpilot_longitudinal=openpilot_longitudinal)
  assert state.enabled is openpilot_longitudinal
  assert bool(TeslaSpeedProfileInputState.parser_messages(state.enabled)) is openpilot_longitudinal

  if not openpilot_longitudinal:
    assert not state.update(SimpleNamespace(vl_all={}))
    assert state.selector is None


def test_carstate_ext_preserves_existing_events_and_appends_gap_release():
  cp = SimpleNamespace(flags=0, openpilotLongitudinalControl=True)
  cp_sp = SimpleNamespace(flags=0)
  carstate_ext = CarStateExt(cp, cp_sp)
  gap_release = structs.CarState.ButtonEvent(pressed=False, type=ButtonType.gapAdjustCruise)
  carstate_ext.speed_profile_input = SimpleNamespace(update=lambda _cp_party: [gap_release])
  carstate_ext.can_define = SimpleNamespace(dv={
    "DI_state": {"DI_speedUnits": {0: "KPH"}},
    "DAS_status": {"DAS_fusedSpeedLimit": {0: "NONE"}},
  })

  cp_party = SimpleNamespace(vl={"DI_state": {"DI_speedUnits": 0}})
  cp_ap_party = SimpleNamespace(vl={"DAS_status": {"DAS_fusedSpeedLimit": 0}})
  existing = structs.CarState.ButtonEvent(pressed=False, type=ButtonType.cancel)
  ret = structs.CarState(buttonEvents=[existing])
  ret_sp = structs.CarStateSP()

  carstate_ext.update(ret, ret_sp, {Bus.party: cp_party, Bus.ap_party: cp_ap_party})
  assert [event.type for event in ret.buttonEvents] == [ButtonType.cancel, ButtonType.gapAdjustCruise]
  assert not ret.buttonEvents[-1].pressed
