from collections import defaultdict
from contextlib import contextmanager
import importlib.util
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace

from cereal import car, custom, log


ROOT = Path(__file__).resolve().parents[3]


class _Dummy:
  CTRL_HIGH = 0


class _CruiseHelperStub:
  @staticmethod
  def update(*_args) -> None:
    pass


class _ET:
  NO_ENTRY = 0
  SOFT_DISABLE = 1
  IMMEDIATE_DISABLE = 2


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
    "openpilot.selfdrive.selfdrived.events": _stub_module("selfdrived_events", Events=_Dummy, ET=_ET),
    "openpilot.selfdrive.selfdrived.helpers": _stub_module("selfdrived_helpers", ExcessiveActuationCheck=_Dummy),
    "openpilot.selfdrive.selfdrived.state": _stub_module("selfdrived_state", StateMachine=_Dummy),
    "openpilot.selfdrive.selfdrived.alertmanager": _stub_module("alertmanager", AlertManager=_Dummy,
                                                                 set_offroad_alert=lambda *_args, **_kwargs: None),
    "openpilot.system.version": _stub_module("system_version", get_build_metadata=lambda: None),
    "openpilot.system.hardware": _stub_module("system_hardware", HARDWARE=_Dummy()),
    "openpilot.sunnypilot.mads.mads": _stub_module("mads", ModularAssistiveDrivingSystem=_Dummy),
    "openpilot.sunnypilot": _stub_module("sunnypilot", get_sanitize_int_param=lambda *_: 1),
    "openpilot.sunnypilot.selfdrive.car.car_specific": _stub_module("car_specific_sp", CarSpecificEventsSP=_Dummy),
    "openpilot.sunnypilot.selfdrive.car.cruise_helpers":
      _stub_module("cruise_helpers", CruiseHelper=_CruiseHelperStub),
    "openpilot.sunnypilot.selfdrive.car.intelligent_cruise_button_management.controller":
      _stub_module("icbm", IntelligentCruiseButtonManagement=_Dummy),
    "openpilot.sunnypilot.selfdrive.selfdrived.events": _stub_module("selfdrived_events_sp", EventsSP=_Dummy),
  }

  path = ROOT / "selfdrive/selfdrived/selfdrived.py"
  spec = importlib.util.spec_from_file_location("_selfdrived_personality_cycle_test", path)
  assert spec is not None and spec.loader is not None
  module = importlib.util.module_from_spec(spec)
  with _patched_modules(stubs):
    spec.loader.exec_module(module)
  return module


class _EventsRecorder:
  def __init__(self):
    self.added = []

  def __len__(self) -> int:
    return len(self.added)

  def clear(self) -> None:
    self.added.clear()

  def add(self, event, **_kwargs) -> None:
    self.added.append(event)

  def add_from_msg(self, _events) -> None:
    pass

  def contains(self, _event_type) -> bool:
    return False

  def remove(self, event) -> None:
    if event in self.added:
      self.added.remove(event)


class _ParamsRecorder:
  def __init__(self):
    self.writes: list[tuple[str, int]] = []

  def put(self, key: str, value: int) -> None:
    self.writes.append((key, value))


class _LateralControlState:
  def __init__(self):
    self.pidState = SimpleNamespace(active=False, saturated=False)

  @staticmethod
  def which() -> str:
    return "pidState"


class _SubMaster(dict):
  def __init__(self):
    super().__init__({
      "controlsState": SimpleNamespace(lateralControlState=_LateralControlState(), curvature=0.0),
      "deviceState": SimpleNamespace(thermalStatus=log.DeviceState.ThermalStatus.ok,
                                     freeSpacePercent=100, memoryUsagePercent=0, fanSpeedPercentDesired=0),
      "peripheralState": SimpleNamespace(pandaType=log.PandaState.PandaType.unknown, fanSpeedRpm=1000),
      "liveCalibration": SimpleNamespace(calStatus=log.LiveCalibrationData.Status.calibrated),
      "driverAssistance": SimpleNamespace(leftLaneDeparture=False, rightLaneDeparture=False),
      "modelV2": SimpleNamespace(
        meta=SimpleNamespace(laneChangeState=log.LaneChangeState.off,
                             laneChangeDirection=log.LaneChangeDirection.none,
                             hardBrakePredicted=False),
        action=SimpleNamespace(desiredCurvature=0.0), frameDropPerc=0.0,
      ),
      "modelDataV2SP": SimpleNamespace(laneTurnDirection=custom.ModelDataV2SP.TurnDirection.none),
      "pandaStates": [],
      "managerState": SimpleNamespace(processes=[]),
      "radarState": SimpleNamespace(radarErrors=SimpleNamespace(
        canError=False, radarUnavailableTemporary=False, to_dict=dict,
      )),
      "livePose": SimpleNamespace(inputsOK=True, posenetOK=True),
      "longitudinalPlan": SimpleNamespace(fcw=False),
      "carControl": SimpleNamespace(),
      "longitudinalPlanSP": SimpleNamespace(),
    })
    self.frame = 0
    self.recv_frame = defaultdict(int)
    self.updated = defaultdict(bool)
    self.valid = defaultdict(lambda: True)
    self.alive = defaultdict(lambda: True)
    self.freq_ok = defaultdict(lambda: True)

  @staticmethod
  def all_checks(*_args) -> bool:
    return True

  @staticmethod
  def all_alive(*_args) -> bool:
    return True

  @staticmethod
  def all_freq_ok(*_args) -> bool:
    return True


def _make_subject(module: ModuleType):
  subject = module.SelfdriveD.__new__(module.SelfdriveD)
  subject.CP = SimpleNamespace(
    passive=False,
    pcmCruise=True,
    notCar=True,
    openpilotLongitudinalControl=True,
    safetyConfigs=[],
    alternativeExperience=0,
  )
  subject.sm = _SubMaster()
  subject.events = _EventsRecorder()
  subject.events_sp = _EventsRecorder()
  subject.params = _ParamsRecorder()
  subject.startup_event = None
  subject.initialized = True
  subject.CS_prev = car.CarState.new_message()
  subject.disengage_on_accelerator = False
  subject.recalibrating_seen = False
  subject.is_ldw_enabled = False
  subject.calibrated_pose = None
  subject.excessive_actuation = False
  subject.mismatch_counter = 0
  subject.not_running_prev = set()
  subject.ignored_processes = set()
  subject.rk = SimpleNamespace(lagging=False)
  subject.logged_comm_issue = None
  subject.camera_packets = []
  subject.sensor_packets = []
  subject.enabled = False
  subject.cruise_mismatch_counter = 0
  subject.last_steering_pressed_frame = 0
  subject.gps_location_service = "gps"
  subject.distance_traveled = 0.0
  subject.mads = SimpleNamespace(enabled=False)
  subject.experimental_mode = False
  subject.experimental_mode_switched = False
  subject.personality = log.LongitudinalPersonality.schema.enumerants["standard"]
  subject.icbm = SimpleNamespace(run=lambda *_: None)
  subject.is_metric = True
  return subject


def test_gap_release_cycles_and_persists_longitudinal_personality():
  module = _load_selfdrived()
  subject = _make_subject(module)
  release = car.CarState.ButtonEvent(type=car.CarState.ButtonEvent.Type.gapAdjustCruise, pressed=False)
  car_state = car.CarState(buttonEvents=[release])

  expected = [
    log.LongitudinalPersonality.schema.enumerants["aggressive"],
    log.LongitudinalPersonality.schema.enumerants["relaxed"],
    log.LongitudinalPersonality.schema.enumerants["standard"],
  ]
  for personality in expected:
    subject.update_events(car_state)
    assert subject.personality == personality
    assert module.EventName.personalityChanged in subject.events.added

  assert subject.params.writes == [("LongitudinalPersonality", personality) for personality in expected]
