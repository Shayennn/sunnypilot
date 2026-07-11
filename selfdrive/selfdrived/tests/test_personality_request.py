from contextlib import contextmanager
from dataclasses import asdict
import importlib.util
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace

import pytest

from cereal import car, custom, log
from opendbc.car import structs


ROOT = Path(__file__).resolve().parents[3]


class _Dummy:
  CTRL_HIGH = 0


class _DummyBase:
  pass


def _stub_module(name: str, **attributes) -> ModuleType:
  module = ModuleType(name)
  for attribute, value in attributes.items():
    setattr(module, attribute, value)
  return module


@contextmanager
def _patched_modules(modules: dict[str, ModuleType]):
  previous = {name: sys.modules.get(name) for name in modules}
  sys.modules.update(modules)
  try:
    yield
  finally:
    for name, module in previous.items():
      if module is None:
        sys.modules.pop(name, None)
      else:
        sys.modules[name] = module


def _load_selfdrived() -> ModuleType:
  stubs = {
    "cereal.messaging": _stub_module("cereal.messaging"),
    "msgq.visionipc": _stub_module("msgq.visionipc", VisionIpcClient=_Dummy, VisionStreamType=_Dummy),
    "openpilot.common.params": _stub_module("openpilot.common.params", Params=_Dummy),
    "openpilot.common.realtime": _stub_module("openpilot.common.realtime", config_realtime_process=lambda *_: None,
                                                       Priority=_Dummy, Ratekeeper=_Dummy, DT_CTRL=0.01),
    "openpilot.common.swaglog": _stub_module("openpilot.common.swaglog", cloudlog=_Dummy()),
    "openpilot.common.gps": _stub_module("openpilot.common.gps", get_gps_location_service=lambda *_: None),
    "openpilot.selfdrive.car.car_specific": _stub_module("car_specific", CarSpecificEvents=_Dummy),
    "openpilot.selfdrive.locationd.helpers": _stub_module("locationd_helpers", PoseCalibrator=_Dummy, Pose=_Dummy),
    "openpilot.selfdrive.selfdrived.events": _stub_module("selfdrived_events", Events=_Dummy, ET=_Dummy),
    "openpilot.selfdrive.selfdrived.helpers": _stub_module("selfdrived_helpers", ExcessiveActuationCheck=_Dummy),
    "openpilot.selfdrive.selfdrived.state": _stub_module("selfdrived_state", StateMachine=_Dummy),
    "openpilot.selfdrive.selfdrived.alertmanager": _stub_module("alertmanager", AlertManager=_Dummy,
                                                                 set_offroad_alert=lambda *_: None),
    "openpilot.system.version": _stub_module("system_version", get_build_metadata=lambda: None),
    "openpilot.system.hardware": _stub_module("system_hardware", HARDWARE=_Dummy()),
    "openpilot.sunnypilot.mads.mads": _stub_module("mads", ModularAssistiveDrivingSystem=_Dummy),
    "openpilot.sunnypilot": _stub_module("sunnypilot", get_sanitize_int_param=lambda *_: 1),
    "openpilot.sunnypilot.selfdrive.car.car_specific": _stub_module("car_specific_sp", CarSpecificEventsSP=_Dummy),
    "openpilot.sunnypilot.selfdrive.car.cruise_helpers": _stub_module("cruise_helpers", CruiseHelper=_DummyBase),
    "openpilot.sunnypilot.selfdrive.car.intelligent_cruise_button_management.controller":
      _stub_module("icbm", IntelligentCruiseButtonManagement=_Dummy),
    "openpilot.sunnypilot.selfdrive.selfdrived.events": _stub_module("selfdrived_events_sp", EventsSP=_Dummy),
  }

  path = ROOT / "selfdrive/selfdrived/selfdrived.py"
  spec = importlib.util.spec_from_file_location("_selfdrived_personality_test", path)
  assert spec is not None and spec.loader is not None
  module = importlib.util.module_from_spec(spec)
  with _patched_modules(stubs):
    spec.loader.exec_module(module)
  return module


@pytest.fixture(scope="module")
def selfdrived_module() -> ModuleType:
  return _load_selfdrived()


class _ParamsRecorder:
  def __init__(self):
    self.writes: list[tuple[str, int]] = []

  def put(self, key: str, value: int) -> None:
    self.writes.append((key, value))


class _EventsRecorder:
  def __init__(self):
    self.added = []

  def add(self, event) -> None:
    self.added.append(event)


class _SubMasterStub(dict):
  def __init__(self, *args, **kwargs):
    super().__init__(*args, **kwargs)
    self.valid = {"carStateSP": True}


def _make_subject(selfdrived_module: ModuleType, *, personality: int = log.LongitudinalPersonality.standard,
                  openpilot_longitudinal: bool = True):
  subject = selfdrived_module.SelfdriveD.__new__(selfdrived_module.SelfdriveD)
  subject.CP = SimpleNamespace(openpilotLongitudinalControl=openpilot_longitudinal)
  subject.personality = personality
  subject._last_car_personality_request = None
  subject.params = _ParamsRecorder()
  subject.events = _EventsRecorder()
  subject.experimental_mode_switched = False
  subject.sm = _SubMasterStub({
    "carStateSP": SimpleNamespace(
      longitudinalPersonalityRequest=0,
      longitudinalPersonalityRequestValid=False,
    ),
  })
  return subject


def _car_state(*, gap_release: bool = False):
  button_events = []
  if gap_release:
    button_events.append(SimpleNamespace(pressed=False, type=car.CarState.ButtonEvent.Type.gapAdjustCruise))
  return SimpleNamespace(buttonEvents=button_events)


def _set_request(subject, personality: int, valid: bool = True) -> None:
  request = subject.sm["carStateSP"]
  request.longitudinalPersonalityRequest = personality
  request.longitudinalPersonalityRequestValid = valid


def test_car_state_sp_dynamic_round_trip():
  state = structs.CarStateSP(
    speedLimit=12.5,
    longitudinalPersonalityRequest=log.LongitudinalPersonality.relaxed,
    longitudinalPersonalityRequestValid=True,
  )
  encoded = custom.CarStateSP.new_message(**asdict(state)).to_bytes()

  with custom.CarStateSP.from_bytes(encoded) as decoded:
    assert decoded.speedLimit == 12.5
    assert decoded.longitudinalPersonalityRequest == log.LongitudinalPersonality.relaxed
    assert decoded.longitudinalPersonalityRequestValid

  legacy = custom.CarStateSP.new_message(speedLimit=4.0)
  assert legacy.longitudinalPersonalityRequest == 0
  assert not legacy.longitudinalPersonalityRequestValid


def test_initial_request_sync_and_stable_deduplication(selfdrived_module):
  subject = _make_subject(selfdrived_module)
  _set_request(subject, log.LongitudinalPersonality.aggressive)

  subject._update_longitudinal_personality(_car_state())
  subject._update_longitudinal_personality(_car_state())

  assert subject.personality == log.LongitudinalPersonality.aggressive
  assert subject.params.writes == [("LongitudinalPersonality", log.LongitudinalPersonality.aggressive)]
  assert subject.events.added == [log.OnroadEvent.EventName.personalityChanged]


def test_absolute_requests_support_bidirectional_jumps(selfdrived_module):
  subject = _make_subject(selfdrived_module)

  for personality in (log.LongitudinalPersonality.aggressive,
                      log.LongitudinalPersonality.relaxed,
                      log.LongitudinalPersonality.aggressive):
    _set_request(subject, personality)
    subject._update_longitudinal_personality(_car_state())
    assert subject.personality == personality

  assert [value for _, value in subject.params.writes] == [
    log.LongitudinalPersonality.aggressive,
    log.LongitudinalPersonality.relaxed,
    log.LongitudinalPersonality.aggressive,
  ]


@pytest.mark.parametrize("requested_personality,request_valid", [
  (log.LongitudinalPersonality.aggressive, False),
  (255, True),
])
def test_invalid_request_resets_last_seen_without_changing_personality(selfdrived_module, requested_personality, request_valid):
  subject = _make_subject(selfdrived_module)
  _set_request(subject, log.LongitudinalPersonality.aggressive)
  subject._update_longitudinal_personality(_car_state())

  subject.personality = log.LongitudinalPersonality.standard
  subject.params.writes.clear()
  subject.events.added.clear()
  _set_request(subject, requested_personality, valid=request_valid)
  subject._update_longitudinal_personality(_car_state())
  assert subject.personality == log.LongitudinalPersonality.standard
  assert subject._last_car_personality_request is None
  assert subject.params.writes == []

  _set_request(subject, log.LongitudinalPersonality.aggressive)
  subject._update_longitudinal_personality(_car_state())
  assert subject.personality == log.LongitudinalPersonality.aggressive
  assert subject.params.writes == [("LongitudinalPersonality", log.LongitudinalPersonality.aggressive)]


def test_stock_longitudinal_ignores_request_and_gap_button(selfdrived_module):
  subject = _make_subject(selfdrived_module, openpilot_longitudinal=False)
  _set_request(subject, log.LongitudinalPersonality.aggressive)

  subject._update_longitudinal_personality(_car_state(gap_release=True))

  assert subject.personality == log.LongitudinalPersonality.standard
  assert subject._last_car_personality_request is None
  assert subject.params.writes == []
  assert subject.events.added == []


def test_invalid_car_state_sp_message_ignores_request(selfdrived_module):
  subject = _make_subject(selfdrived_module)
  _set_request(subject, log.LongitudinalPersonality.aggressive)
  subject.sm.valid["carStateSP"] = False

  subject._update_longitudinal_personality(_car_state())

  assert subject.personality == log.LongitudinalPersonality.standard
  assert subject._last_car_personality_request is None
  assert subject.params.writes == []
  assert subject.events.added == []


def test_manual_override_is_preserved_until_car_request_changes(selfdrived_module):
  subject = _make_subject(selfdrived_module)
  _set_request(subject, log.LongitudinalPersonality.aggressive)
  subject._update_longitudinal_personality(_car_state())

  subject.personality = log.LongitudinalPersonality.relaxed
  subject.params.writes.clear()
  subject.events.added.clear()
  subject._update_longitudinal_personality(_car_state())
  assert subject.personality == log.LongitudinalPersonality.relaxed
  assert subject.params.writes == []

  _set_request(subject, log.LongitudinalPersonality.standard)
  subject._update_longitudinal_personality(_car_state())
  assert subject.personality == log.LongitudinalPersonality.standard
  assert subject.params.writes == [("LongitudinalPersonality", log.LongitudinalPersonality.standard)]


def test_stable_absolute_request_wins_over_gap_button(selfdrived_module):
  subject = _make_subject(selfdrived_module)
  _set_request(subject, log.LongitudinalPersonality.aggressive)
  subject._update_longitudinal_personality(_car_state())
  subject.params.writes.clear()
  subject.events.added.clear()

  subject._update_longitudinal_personality(_car_state(gap_release=True))

  assert subject.personality == log.LongitudinalPersonality.aggressive
  assert subject.params.writes == []
  assert subject.events.added == []


def test_gap_button_keeps_generic_cycle_behavior(selfdrived_module):
  subject = _make_subject(selfdrived_module)

  subject._update_longitudinal_personality(_car_state(gap_release=True))
  assert subject.personality == log.LongitudinalPersonality.aggressive

  subject._update_longitudinal_personality(_car_state(gap_release=True))
  assert subject.personality == log.LongitudinalPersonality.relaxed
  assert [value for _, value in subject.params.writes] == [
    log.LongitudinalPersonality.aggressive,
    log.LongitudinalPersonality.relaxed,
  ]


def _load_long_mpc() -> ModuleType:
  stubs = {
    "openpilot.common.realtime": _stub_module("openpilot.common.realtime", DT_MDL=0.05),
    "openpilot.common.swaglog": _stub_module("openpilot.common.swaglog", cloudlog=_Dummy()),
    "openpilot.selfdrive.controls.radard": _stub_module("radard", _LEAD_ACCEL_TAU=1.5),
    "openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.c_generated_code.acados_ocp_solver_pyx":
      _stub_module("acados_solver", AcadosOcpSolverCython=_Dummy),
    "casadi": _stub_module("casadi", SX=_Dummy, vertcat=lambda *values: values),
  }

  path = ROOT / "selfdrive/controls/lib/longitudinal_mpc_lib/long_mpc.py"
  spec = importlib.util.spec_from_file_location("_long_mpc_personality_test", path)
  assert spec is not None and spec.loader is not None
  module = importlib.util.module_from_spec(spec)
  with _patched_modules(stubs):
    spec.loader.exec_module(module)
  return module


def test_personality_following_gaps():
  long_mpc = _load_long_mpc()

  assert long_mpc.get_T_FOLLOW(log.LongitudinalPersonality.aggressive) == 1.25
  assert long_mpc.get_T_FOLLOW(log.LongitudinalPersonality.standard) == 1.45
  assert long_mpc.get_T_FOLLOW(log.LongitudinalPersonality.relaxed) == 1.75
