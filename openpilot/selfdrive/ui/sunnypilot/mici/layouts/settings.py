"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from openpilot.selfdrive.ui.mici.layouts.settings import settings as OP
from openpilot.selfdrive.ui.mici.layouts.settings.settings import SettingsBigButton
from openpilot.selfdrive.ui.mici.layouts.settings.device import DeviceLayoutMici
from openpilot.selfdrive.ui.mici.widgets.button import BigButton, BigCircleButton
from openpilot.selfdrive.ui.mici.widgets.dialog import BigConfirmationDialog, BigDialog
from openpilot.selfdrive.ui.sunnypilot.mici.layouts.sunnylink import SunnylinkLayoutMici
from openpilot.selfdrive.ui.sunnypilot.mici.layouts.models import ModelsLayoutMici
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.hardware.fan_controller import MAX_AVERAGE_TEMP_C, MIN_AVERAGE_TEMP_C, SAN_DIEGO_AVERAGE_TEMP_C
from openpilot.system.ui.widgets import DialogResult
from openpilot.system.ui.widgets.option_dialog import MultiOptionDialog

ICON_SIZE = 70
BIG_ICON_SIZE = 110


class FanClimateButton(BigButton):
  def __init__(self):
    self._params = ui_state.params
    super().__init__(tr("fan climate"), self._temperature_label(self._get_temperature()))
    self.set_enabled(lambda: ui_state.is_offroad())
    self.set_click_callback(self._show_temperature_picker)

  def _get_temperature(self) -> int:
    temperature = int(self._params.get("FanControlAmbientTemperatureC", return_default=True))
    return max(int(MIN_AVERAGE_TEMP_C), min(int(MAX_AVERAGE_TEMP_C), temperature))

  @staticmethod
  def _temperature_label(temperature: int) -> str:
    default = f" · {tr('San Diego default')}" if temperature == SAN_DIEGO_AVERAGE_TEMP_C else ""
    return f"{temperature} °C{default}"

  def _show_temperature_picker(self):
    options = [f"{temperature} °C" for temperature in range(int(MIN_AVERAGE_TEMP_C), int(MAX_AVERAGE_TEMP_C) + 1)]
    current = f"{self._get_temperature()} °C"

    dialog = None

    def handle_selection(result: DialogResult):
      if result == DialogResult.CONFIRM and dialog is not None:
        temperature = int(dialog.selection.removesuffix(" °C"))
        self._params.put("FanControlAmbientTemperatureC", temperature)
        self.set_value(self._temperature_label(temperature))

    dialog = MultiOptionDialog(tr("Average City Temperature (Celsius)"), options, current=current, callback=handle_selection)
    gui_app.push_widget(dialog)


class DeviceLayoutMiciSP(DeviceLayoutMici):
  def __init__(self):
    super().__init__()
    self._scroller.add_widget(FanClimateButton())


class SettingsLayoutSP(OP.SettingsLayout):
  def __init__(self):
    OP.SettingsLayout.__init__(self)

    device_panel = DeviceLayoutMiciSP()
    self._scroller._items[2].set_click_callback(lambda: gui_app.push_widget(device_panel))

    self.icon_offroad_enable = gui_app.texture("../../sunnypilot/selfdrive/assets/icons_mici/always_offroad.png", BIG_ICON_SIZE,
                                               BIG_ICON_SIZE)
    self.icon_offroad_disable = gui_app.texture("../../sunnypilot/selfdrive/assets/icons_mici/disable_offroad.png", BIG_ICON_SIZE,
                                                BIG_ICON_SIZE)
    self.icon_offroad_slider = gui_app.texture("icons_mici/settings/device/lkas.png", BIG_ICON_SIZE, BIG_ICON_SIZE)

    sunnylink_panel = SunnylinkLayoutMici()
    sunnylink_btn = SettingsBigButton(tr("sunnylink"), "", gui_app.texture("icons_mici/settings/developer/ssh.png", 55, 55))
    sunnylink_btn.set_click_callback(lambda: gui_app.push_widget(sunnylink_panel))

    models_panel = ModelsLayoutMici()
    models_btn = SettingsBigButton(tr("models"), "", gui_app.texture("../../sunnypilot/selfdrive/assets/offroad/icon_models.png", ICON_SIZE, ICON_SIZE))
    models_btn.set_click_callback(lambda: gui_app.push_widget(models_panel))

    # onroad: enable button sits at the front (left of toggles)
    self._enable_offroad_btn_onroad = BigCircleButton(self.icon_offroad_enable, red=True)
    self._enable_offroad_btn_onroad.set_click_callback(lambda: self._handle_always_offroad(True))
    self._enable_offroad_btn_onroad.set_visible(lambda: ui_state.started and not ui_state.always_offroad)

    # offroad: enable button sits at the end (right of developer)
    self._enable_offroad_btn_offroad = BigCircleButton(self.icon_offroad_enable, red=True)
    self._enable_offroad_btn_offroad.set_click_callback(lambda: self._handle_always_offroad(True))
    self._enable_offroad_btn_offroad.set_visible(lambda: not ui_state.started and not ui_state.always_offroad)

    self._disable_offroad_btn = BigCircleButton(self.icon_offroad_disable, red=False)
    self._disable_offroad_btn.set_click_callback(lambda: self._handle_always_offroad(False))
    self._disable_offroad_btn.set_visible(lambda: ui_state.always_offroad)

    items = self._scroller._items.copy()

    items.insert(1, sunnylink_btn)
    items.insert(2, models_btn)

    # front slots (only one ever visible at a time): exit-always-offroad, then enable-onroad
    items.insert(0, self._enable_offroad_btn_onroad)
    items.insert(0, self._disable_offroad_btn)
    # end slot: enable-offroad (right of developer)
    items.append(self._enable_offroad_btn_offroad)

    self._scroller._items.clear()
    for item in items:
      self._scroller.add_widget(item)

  def _update_state(self):
    super()._update_state()

  def _handle_always_offroad(self, enable: bool):

    def _set_offroad_status(status: bool):
      if not ui_state.engaged:
        ui_state.params.put_bool("OffroadMode", status)
        ui_state.always_offroad = status

    if not enable:
      dlg = BigConfirmationDialog(tr("slide to exit always offroad"), self.icon_offroad_slider, red=False,
                                  confirm_callback=lambda: _set_offroad_status(False))
    else:
      if ui_state.engaged:
        gui_app.push_widget(BigDialog(tr("disengage to enable always offroad"), "", ))
        return

      dlg = BigConfirmationDialog(tr("slide to force offroad"), self.icon_offroad_slider, red=True,
                                  confirm_callback=lambda: _set_offroad_status(True))
    gui_app.push_widget(dlg)
