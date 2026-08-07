
from openpilot.common.parameterized import parameterized
from openpilot.common.test import OpenpilotTestCase
from openpilot.system.hardware.fan_controller import (
  BASE_IDLE_FAN_LIMIT, MAX_IDLE_FAN_LIMIT, SAN_DIEGO_AVERAGE_TEMP_C, FanController,
  get_climate_adjustment, get_idle_fan_limit,
)

ALL_CONTROLLERS = [FanController]

class TestFanController(OpenpilotTestCase):
  def wind_up(self, controller, ignition=True):
    for _ in range(1000):
      controller.update(100, ignition)

  def wind_down(self, controller, ignition=False):
    for _ in range(1000):
      controller.update(10, ignition)

  @parameterized.expand(ALL_CONTROLLERS)
  def test_hot_onroad(self, controller_class):
    controller = controller_class(2)
    self.wind_up(controller)
    assert controller.update(100, True) >= 70

  @parameterized.expand(ALL_CONTROLLERS)
  def test_offroad_limits(self, controller_class):
    controller = controller_class(2)
    self.wind_up(controller)
    assert controller.update(100, False) <= 30

  @parameterized.expand(ALL_CONTROLLERS)
  def test_no_fan_wear(self, controller_class):
    controller = controller_class(2)
    self.wind_down(controller)
    assert controller.update(10, False) == 0

  @parameterized.expand(ALL_CONTROLLERS)
  def test_limited(self, controller_class):
    controller = controller_class(2)
    self.wind_up(controller, True)
    assert controller.update(100, True) == 100

  @parameterized.expand(ALL_CONTROLLERS)
  def test_windup_speed(self, controller_class):
    controller = controller_class(2)
    self.wind_down(controller, True)
    for _ in range(10):
      controller.update(90, True)
    assert controller.update(90, True) >= 60

  @parameterized.expand(ALL_CONTROLLERS)
  def test_san_diego_default_is_backward_compatible(self, controller_class):
    implicit_default = controller_class(2)
    explicit_default = controller_class(2)

    for temp, ignition in [(55, False), (75, False), (75, True), (90, True), (70, False)]:
      assert (implicit_default.update(temp, ignition) ==
              explicit_default.update(temp, ignition, SAN_DIEGO_AVERAGE_TEMP_C))

  @parameterized.expand(ALL_CONTROLLERS)
  def test_hot_climate_increases_onroad_feedforward(self, controller_class):
    default = controller_class(2)
    hot_climate = controller_class(2)

    default_output = default.update(80, True, SAN_DIEGO_AVERAGE_TEMP_C)
    hot_output = hot_climate.update(80, True, SAN_DIEGO_AVERAGE_TEMP_C + 10)

    assert hot_output > default_output
    assert hot_climate.controller.f > default.controller.f
    assert hot_climate.controller.i == default.controller.i  # the absolute PID target is unchanged

  @parameterized.expand(ALL_CONTROLLERS)
  def test_hot_climate_raises_idle_limit(self, controller_class):
    default = controller_class(2)
    hot_climate = controller_class(2)

    for _ in range(1000):
      default.update(100, False, SAN_DIEGO_AVERAGE_TEMP_C)
      hot_climate.update(100, False, SAN_DIEGO_AVERAGE_TEMP_C + 10)

    assert default.update(100, False, SAN_DIEGO_AVERAGE_TEMP_C) == BASE_IDLE_FAN_LIMIT
    assert hot_climate.update(100, False, SAN_DIEGO_AVERAGE_TEMP_C + 10) == BASE_IDLE_FAN_LIMIT + 10

  def test_idle_limit_never_drops_below_stock(self):
    assert get_idle_fan_limit(-100) == BASE_IDLE_FAN_LIMIT
    assert get_idle_fan_limit(100) == MAX_IDLE_FAN_LIMIT

  def test_climate_adjustment_is_bounded(self):
    assert get_climate_adjustment(-100) == -10
    assert get_climate_adjustment(SAN_DIEGO_AVERAGE_TEMP_C) == 0
    assert get_climate_adjustment(100) == 20
    assert get_climate_adjustment(float("nan")) == 0
