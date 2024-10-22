/*
 * Copyright (C) EdgeTX
 *
 * Based on code named
 *   opentx - https://github.com/opentx/opentx
 *   th9x - http://code.google.com/p/th9x
 *   er9x - http://code.google.com/p/er9x
 *   gruvin9x - http://code.google.com/p/gruvin9x
 *
 * License GPLv2: http://www.gnu.org/licenses/gpl-2.0.html
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include "radio_CurrentSensor.h"
#include "radio_calibration.h"
#include "radio_diagkeys.h"
#include "radio_diaganas.h"

#include "menu_screen.h"

#include "menu_model.h"
#include "menu_radio.h"
#include "model_select.h"
#include "model_setup.h"

#include "module_setup.h"
#include "trainer_setup.h"

#include "model_telemetry.h"
#include "view_channels.h"

#include "view_main.h"
//#include "telemetry.h"

#include "edgetx.h"
#include "libopenui.h"
#include "hal/adc_driver.h"
#include "hal/module_port.h"

#include "hw_intmodule.h"
#include "hw_extmodule.h"
#include "hw_serial.h"
#include "hw_inputs.h"

#if defined(BLUETOOTH)
#include "hw_bluetooth.h"
#endif

#if defined(CROSSFIRE)
  #include "mixer_scheduler.h"
#endif

#if defined(CSD203_SENSOR) 
  #include "csd203_sensor.h"
#endif
#if defined(IMU_SENSOR) 
  #include "imu_42627.h"
#endif

#define SET_DIRTY() storageDirty(EE_GENERAL)


RadioCurrentMenu::RadioCurrentMenu(ModelMenu* parent) :
    TabsGroup(ICON_MONITOR), parentMenu(parent)
{
  //_modelTelemetryEnabled = modelTelemetryEnabled();

  addTab(new RadioCurrentPage());
  addTab(new RadioCurrentTotalPage());
  //if (_modelTelemetryEnabled) addTab(new ModelTelemetryPage());

  //parent->setCurrentTab(1);
}

RadioTelemetryMenu::RadioTelemetryMenu(ModelMenu* parent) :
    TabsGroup(ICON_MONITOR), parentMenu(parent)
{
  _modelTelemetryEnabled = modelTelemetryEnabled();
  
  //addTab(new RadioCurrentPage());
  if (_modelTelemetryEnabled) addTab(new ModelTelemetryPage());
}

#if defined(HARDWARE_KEYS)
void RadioCurrentMenu::onPressSYS()
{
    onCancel();
    if (parentMenu) parentMenu->onCancel();
    new RadioMenu();
}
void RadioCurrentMenu::onLongPressSYS()
{
    onCancel();
    if (parentMenu) parentMenu->onCancel();
    // Radio setup
    (new RadioMenu())->setCurrentTab(2);
}
void RadioCurrentMenu::onPressMDL()
{
    onCancel();
    if (!parentMenu) {
      new ModelMenu();
    }
}
void RadioCurrentMenu::onLongPressMDL()
{
    onCancel();
    if (parentMenu) parentMenu->onCancel();
    new ModelLabelsWindow();
}
void RadioCurrentMenu::onPressTELE()
{
    onCancel();
    if (parentMenu) parentMenu->onCancel();
    new RadioTelemetryMenu();
}
void RadioCurrentMenu::onLongPressSENSORVIEW() 
{
  onCancel();
  new RadioCurrentMenu();
}
void RadioCurrentMenu::onLongPressTELEVIEW() 
{
  onCancel();
  new RadioTelemetryMenu();
}
void RadioCurrentMenu::onLongPressCHVIEW() 
{
  onCancel();
  new ChannelsViewMenu();
}

void RadioTelemetryMenu::onPressSYS()
{
    onCancel();
    if (parentMenu) parentMenu->onCancel();
    new RadioMenu();
}
void RadioTelemetryMenu::onLongPressSYS()
{
    onCancel();
    if (parentMenu) parentMenu->onCancel();
    // Radio setup
    (new RadioMenu())->setCurrentTab(2);
}
void RadioTelemetryMenu::onPressMDL()
{
    onCancel();
    if (!parentMenu) {
      new ModelMenu();
    }
}
void RadioTelemetryMenu::onLongPressMDL()
{
    onCancel();
    if (parentMenu) parentMenu->onCancel();
    new ModelLabelsWindow();
}
void RadioTelemetryMenu::onPressTELE()
{
    onCancel();
    if (parentMenu) parentMenu->onCancel();
    new ScreenMenu();
}
void RadioTelemetryMenu::onLongPressSENSORVIEW() 
{
  onCancel();
  new RadioCurrentMenu();
}
void RadioTelemetryMenu::onLongPressTELEVIEW() 
{
  onCancel();
  new RadioTelemetryMenu();
}
void RadioTelemetryMenu::onLongPressCHVIEW() 
{
  onCancel();
  new ChannelsViewMenu();
}
#endif

class MSubScreenButton : public TextButton
{
  public:
    MSubScreenButton(Window* parent, const char* text,
                    std::function<void(void)> pressHandler,
                    std::function<bool(void)> checkActive = nullptr) :
      TextButton(parent, rect_t{}, text, [=]() -> uint8_t {
          pressHandler();
          return 0;
      }),
      m_isActive(std::move(checkActive))
    {
      // Room for two lines of text
      setHeight(62);
      setWidth((LCD_W - 30) / 3);

      lv_obj_set_width(label, lv_pct(100));
      lv_label_set_long_mode(label, LV_LABEL_LONG_WRAP);
      lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
    }

    void checkEvents() override
    {
      check(isActive());
    }

  protected:
    std::function<bool(void)> m_isActive = nullptr;

    virtual bool isActive() { return m_isActive ? m_isActive() : false; }
};
RadioCurrentPage::RadioCurrentPage():
  //PageTab(STR_HARDWARE, ICON_RADIO_CURRENTSENSOR)
  PageTab("HelloRadioSky Sensor", ICON_RADIO_CURRENTSENSOR)
{
}

void RadioCurrentPage::checkEvents()
{
  enableVBatBridge();
}

void RadioCurrentPage::build(Window * window)
{
#if !PORTRAIT_LCD
  static const lv_coord_t col_dsc[] = {LV_GRID_FR(1), LV_GRID_FR(1),
                                       LV_GRID_FR(1), LV_GRID_TEMPLATE_LAST};
#else
  static const lv_coord_t col_dsc[] = {LV_GRID_FR(1), LV_GRID_FR(1),
                                       LV_GRID_TEMPLATE_LAST};
#endif
  static const lv_coord_t row_dsc[] = {LV_GRID_CONTENT, LV_GRID_TEMPLATE_LAST};

  window->setFlexLayout(LV_FLEX_FLOW_COLUMN, PAD_ZERO);

  FlexGridLayout grid(col_dsc, row_dsc, PAD_TINY);

  auto line = window->newLine(grid);
  line->padAll(PAD_TINY);

  // TODO: sub-title?

  // Bat calibration
//  auto line = window->newLine(grid);
  //new StaticText(line, rect_t{}, "All Message:", 0, COLOR_THEME_PRIMARY1);
 // auto box = new FormWindow(line, rect_t{});
//  box->setFlexLayout(LV_FLEX_FLOW_ROW, lv_dpx(8));
 // lv_obj_set_style_grid_cell_x_align(box->getLvObj(), LV_GRID_ALIGN_STRETCH, 0);
 // lv_obj_set_style_flex_cross_place(box->getLvObj(), LV_FLEX_ALIGN_CENTER, 0);

  new Subtitle(window, "System Energy Management");
  //new Subtitle(window, "System Batter:");

  line = window->newLine(grid);
  // Main voltage S1 display
  new StaticText(line, rect_t{}, "Cell1:");
  new DynamicNumber<uint16_t>(
      line, rect_t{},
      [] { return getcsd203BatteryS1Voltage()/10; }, COLOR_THEME_PRIMARY1_INDEX, PREC2,
      "", "V");

  line = window->newLine(grid);
  // Main voltage S2 display
  new StaticText(line, rect_t{}, "Cell2:");
  new DynamicNumber<uint16_t>(
      line, rect_t{},
      [] { return (getCSD203BatteryVoltage()-getcsd203BatteryS1Voltage())/10; }, COLOR_THEME_PRIMARY1_INDEX, PREC2,
      "", "V");

  line = window->newLine(grid);
  // Main voltage total display
  new StaticText(line, rect_t{}, "Total Volt:");
  new DynamicNumber<uint16_t>(
      line, rect_t{},
      [] { return getCSD203BatteryVoltage()/10; }, COLOR_THEME_PRIMARY1_INDEX, PREC2,
      "", "V");


  line = window->newLine(grid);
  //if(usbChargerLed()) {
  if(0){
    // Main current display
    new StaticText(line, rect_t{}, "Change Current:");
    new DynamicNumber<uint16_t>(
      line, rect_t{},
      [] { return (getcsd203MainCurrent()+240)/10; }, COLOR_THEME_PRIMARY1_INDEX, PREC2,
      "", "A");  
  }
  else
  {
    // Main current display
    new StaticText(line, rect_t{}, "Current:");
    new DynamicNumber<uint16_t>(
      line, rect_t{},
      [] { return getcsd203MainCurrent()/10; }, COLOR_THEME_PRIMARY1_INDEX, PREC2,
      "", "A"); 
  }

  line = window->newLine(grid);
  //if(usbChargerLed()) {
  if(0){  
    // Main voltage S2 display
    new StaticText(line, rect_t{}, "Change Power:");
    new DynamicNumber<uint16_t>(
      line, rect_t{},
      [] { return (getCSD203BatteryVoltage()*(getcsd203MainCurrent()+240)/10)/1000; }, COLOR_THEME_PRIMARY1_INDEX, PREC2,
      "", "W"); 
  }
  else
  {
    // Main voltage S2 display
    new StaticText(line, rect_t{}, "System Power:");
    new DynamicNumber<uint16_t>(
      line, rect_t{},
      [] { return (getCSD203BatteryVoltage()*getcsd203MainCurrent()/10)/1000; }, COLOR_THEME_PRIMARY1_INDEX, PREC2,
      "", "W");
  }

  line = window->newLine(grid);
  new Subtitle(window, "Speech recognition Management");
  //new Subtitle(window, "Speech recognition config:");
  //auto serial = new SerialConfigWindow(window, rect_t{});S

  //setFlexLayout();
  //FlexGridLayout grid(col_dsc, row_dsc, 2);

  for (uint8_t port_nr = 1; port_nr < MAX_SERIAL_PORTS-1; port_nr++) {
    auto port = serialGetPort(port_nr);
    if (!port || !port->name) continue;

    //line = newLine(&grid);
    line = window->newLine(grid);
    new StaticText(line, rect_t{}, port->name);

    auto box = new Window(line, rect_t{});
    box->setFlexLayout(LV_FLEX_FLOW_ROW, lv_dpx(8));
    lv_obj_set_style_grid_cell_x_align(box->getLvObj(), LV_GRID_ALIGN_STRETCH, 0);
    lv_obj_set_style_flex_cross_place(box->getLvObj(), LV_FLEX_ALIGN_CENTER, 0);
    
    auto aux = new Choice(
        box, rect_t{}, STR_AUX_SERIAL_MODES, 0, UART_MODE_MAX,
        [=]() { return serialGetMode(port_nr); },
        [=](int value) {
          serialSetMode(port_nr, value);
          serialInit(port_nr, value);
          SET_DIRTY();
        });
    aux->setAvailableHandler(
        [=](int value) { return isSerialModeAvailable(port_nr, value); });

    if (port->set_pwr != nullptr) {
      new StaticText(box, rect_t{}, " ");
      new ToggleSwitch(
          box, rect_t{}, [=] { return serialGetPower(port_nr); },
          [=](int8_t newValue) {
            serialSetPower(port_nr, (bool)newValue);
            SET_DIRTY();
          });
    }
  }
  line = window->newLine(grid);
  //line->padLeft(10);
  new StaticText(line, rect_t{}, "Charge protect");
  new ToggleSwitch(line, rect_t{}, GET_SET_DEFAULT(g_eeGeneral.usbchgprotect));

  line = window->newLine(grid);
  //line->padLeft(10);
  new StaticText(line, rect_t{}, "CH56 Release");
  new ToggleSwitch(line, rect_t{}, GET_SET_DEFAULT(g_eeGeneral.voivech56switch));

  line = window->newLine(grid);
  //line->padLeft(10);
  new StaticText(line, rect_t{}, "Headchasing CH78");
  new ToggleSwitch(line, rect_t{}, GET_SET_DEFAULT(g_eeGeneral.gyroHeadCH78switch));

  line = window->newLine(grid);

  new Subtitle(window, "GYRO Pitch adjust:");
  line = window->newLine(grid);
  // Main volume Pitch rate Roller rate
  new StaticText(line, rect_t{}, "GYRO Pitch dead");
  new Slider(line, lv_pct(50), 40, 200, GET_SET_DEFAULT(g_eeGeneral.imuxdead));

  line = window->newLine(grid);
  // Main volume Pitch rate Roller rate
  new StaticText(line, rect_t{}, "GYRO Pitch P");
  new Slider(line, lv_pct(50), 1, 100, GET_SET_DEFAULT(g_eeGeneral.imuxp));

  line = window->newLine(grid);
  // Main volume Pitch rate Roller rate
  new StaticText(line, rect_t{}, "GYRO Pitch I");
  new Slider(line, lv_pct(50), 1, 100, GET_SET_DEFAULT(g_eeGeneral.imuxi));

  //line = window->newLine(&grid);
  // Main volume Pitch rate Roller rate
  //new StaticText(line, rect_t{}, "GYRO Pitch D", 0, COLOR_THEME_PRIMARY1);
  //new Slider(line, lv_pct(50), 1, 100, GET_SET_DEFAULT(g_eeGeneral.imuxd));

  new Subtitle(window, "GYRO Roller adjust:");
  line = window->newLine(grid);
  // Main volume Pitch rate Roller rate  
  new StaticText(line, rect_t{}, "GYRO Roller dead");
  new Slider(line, lv_pct(50), 40, 200, GET_SET_DEFAULT(g_eeGeneral.imuydead));
  
  line = window->newLine(grid);
  // Main volume Pitch rate Roller rate
  new StaticText(line, rect_t{}, "GYRO Roller P");
  new Slider(line, lv_pct(50), 1, 100, GET_SET_DEFAULT(g_eeGeneral.imuyp));

  line = window->newLine(grid);
  // Main volume Pitch rate Roller rate
  new StaticText(line, rect_t{}, "GYRO Roller I");
  new Slider(line, lv_pct(50), 1, 100, GET_SET_DEFAULT(g_eeGeneral.imuyi));

  //line = window->newLine(&grid);
  // Main volume Pitch rate Roller rate
  //new StaticText(line, rect_t{}, "GYRO Roller D", 0, COLOR_THEME_PRIMARY1);
  //new Slider(line, lv_pct(50), 1, 100, GET_SET_DEFAULT(g_eeGeneral.imuyd));
  new Subtitle(window, "GYRO Rotate adjust:");
  line = window->newLine(grid);
  // Main volume Pitch rate Roller rate  
  new StaticText(line, rect_t{}, "GYRO Rotate dead");
  new Slider(line, lv_pct(50), 40, 200, GET_SET_DEFAULT(g_eeGeneral.imuzdead));
  
  line = window->newLine(grid);
  // Main volume Pitch rate Roller rate
  new StaticText(line, rect_t{}, "GYRO Rotate P");
  new Slider(line, lv_pct(50), 1, 100, GET_SET_DEFAULT(g_eeGeneral.imuzp));

  line = window->newLine(grid);
  // Main volume Pitch rate Roller rate
  new StaticText(line, rect_t{}, "GYRO Rotate I");
  new Slider(line, lv_pct(50), 1, 100, GET_SET_DEFAULT(g_eeGeneral.imuzi));

#if defined(__null__)
  //line = window->newLine(&grid);
  // Main volume Pitch rate Roller rate
  //new StaticText(line, rect_t{}, "GYRO Rotate D", 0, COLOR_THEME_PRIMARY1);
  //new Slider(line, lv_pct(50), 1, 100, GET_SET_DEFAULT(g_eeGeneral.imuzd));
#endif
}

RadioCurrentTotalPage::RadioCurrentTotalPage():
  //PageTab(STR_HARDWARE, ICON_RADIO_CURRENTSENSOR)
  PageTab("HelloRadioSky All Sensor", ICON_RADIO_CURRENTSENSOR)
{
}

void RadioCurrentTotalPage::checkEvents()
{
  enableVBatBridge();
}

void RadioCurrentTotalPage::build(Window * window)
{
  #if !PORTRAIT_LCD
  static const lv_coord_t col_dsc[] = {LV_GRID_FR(1), LV_GRID_FR(1),
                                       LV_GRID_FR(1), LV_GRID_TEMPLATE_LAST};
#else
  static const lv_coord_t col_dsc[] = {LV_GRID_FR(1), LV_GRID_FR(1),
                                       LV_GRID_TEMPLATE_LAST};
#endif
  static const lv_coord_t row_dsc[] = {LV_GRID_CONTENT, LV_GRID_TEMPLATE_LAST};

   window->setFlexLayout(LV_FLEX_FLOW_COLUMN, PAD_ZERO);

  FlexGridLayout grid(col_dsc, row_dsc, PAD_TINY);

  // Bat calibration
  auto line = window->newLine(grid);
  //new StaticText(line, rect_t{}, "All Message:", 0, COLOR_THEME_PRIMARY1);
  auto box = new Window(line, rect_t{});
  box->setFlexLayout(LV_FLEX_FLOW_ROW, lv_dpx(8));
  lv_obj_set_style_grid_cell_x_align(box->getLvObj(), LV_GRID_ALIGN_STRETCH, 0);
  lv_obj_set_style_flex_cross_place(box->getLvObj(), LV_FLEX_ALIGN_CENTER, 0);


  new Subtitle(window, "System Energy Management");
  new Subtitle(window, "System Batter:");

  line = window->newLine(grid);
  // Main voltage S1 display
  new StaticText(line, rect_t{}, "Cell1:");
  new DynamicNumber<uint16_t>(
      line, rect_t{},
      [] { return getcsd203BatteryS1Voltage()/10; }, COLOR_THEME_PRIMARY1_INDEX, PREC2,
      "", "V");

  line = window->newLine(grid);
  // Main voltage S2 display
  new StaticText(line, rect_t{}, "Cell2:");
  new DynamicNumber<uint16_t>(
      line, rect_t{},
      [] { return (getCSD203BatteryVoltage()-getcsd203BatteryS1Voltage())/10; }, COLOR_THEME_PRIMARY1_INDEX, PREC2,
      "", "V");

  line = window->newLine(grid);
  // Main voltage total display
  new StaticText(line, rect_t{}, "Total Volt:");
  new DynamicNumber<uint16_t>(
      line, rect_t{},
      [] { return getCSD203BatteryVoltage()/10; }, COLOR_THEME_PRIMARY1_INDEX, PREC2,
      "", "V");


  line = window->newLine(grid);
  //if(usbChargerLed()) {
  if(0){
    // Main current display
    new StaticText(line, rect_t{}, "Change Current:");
    new DynamicNumber<uint16_t>(
      line, rect_t{},
      [] { return (getcsd203MainCurrent()+240)/10; }, COLOR_THEME_PRIMARY1_INDEX, PREC2,
      "", "A");  
  }
  else
  {
    // Main current display
    new StaticText(line, rect_t{}, "Current:");
    new DynamicNumber<uint16_t>(
      line, rect_t{},
      [] { return getcsd203MainCurrent()/10; }, COLOR_THEME_PRIMARY1_INDEX, PREC2,
      "", "A"); 
  }

  line = window->newLine(grid);
  //if(usbChargerLed()) {
  if(0){  
    // Main voltage S2 display
    new StaticText(line, rect_t{}, "Change Power:");
    new DynamicNumber<uint16_t>(
      line, rect_t{},
      [] { return (getCSD203BatteryVoltage()*(getcsd203MainCurrent()+240)/10)/1000; }, COLOR_THEME_PRIMARY1_INDEX, PREC2,
      "", "W"); 
  }
  else
  {
    // Main voltage S2 display
    new StaticText(line, rect_t{}, "System Power:");
    new DynamicNumber<uint16_t>(
      line, rect_t{},
      [] { return (getCSD203BatteryVoltage()*getcsd203MainCurrent()/10)/1000; }, COLOR_THEME_PRIMARY1_INDEX, PREC2,
      "", "W");
  }

  line = window->newLine(grid);
  new Subtitle(window, "EXTERNAL MODULE:");
  // headline "External module"
  //new StaticText(line, rect_t{}, STR_EXTERNAL_MODULE, 0, COLOR_THEME_PRIMARY1);
  // 
  line = window->newLine(grid);
  auto name=new StaticText(line, rect_t{}, "Module name:");
  name->setText(STR_MODULE_PROTOCOLS[g_model.moduleData[EXTERNAL_MODULE].type]);
  //lv_obj_add_flag(name->getLvObj(), LV_OBJ_FLAG_HIDDEN);

  // CRSF is able to provide status
  if (isModuleCrossfire(EXTERNAL_MODULE)) {
    char statusText[64];

    line = window->newLine(grid);
    auto hz = 1000000 / getMixerSchedulerPeriod();
    // snprintf(statusText, 64, "%d Hz %" PRIu32 " Err", hz, telemetryErrors);
    snprintf(statusText, 64, "%d Hz", hz);
    //status->setText(statusText);
    //lv_obj_clear_flag(module_status_w->getLvObj(), LV_OBJ_FLAG_HIDDEN);
    new StaticText(line, rect_t{}, statusText);
  }
  // MPM is able to provide status
  if (isModuleMultimodule(EXTERNAL_MODULE)) {
    char statusText[64];

    line = window->newLine(grid);
    getMultiModuleStatus(EXTERNAL_MODULE).getStatusString(statusText);
    //name->setText(statusText);
    //lv_obj_clear_flag(name->getLvObj(), LV_OBJ_FLAG_HIDDEN);
    new StaticText(line, rect_t{}, statusText);
  }

  line = window->newLine(grid);
  //ext voltage display
  new StaticText(line, rect_t{}, "Module Voltage:");
  new DynamicNumber<uint16_t>(
      line, rect_t{},
      [] { return (getcsd203extVoltage()/10); }, COLOR_THEME_PRIMARY1_INDEX, PREC2,
      "", "V"); 
  // 
  line = window->newLine(grid);
  //ext current display
  new StaticText(line, rect_t{}, "Module Current:");
  new DynamicNumber<uint16_t>(
      line, rect_t{},
      [] { return (getcsd203extCurrent()/10); }, COLOR_THEME_PRIMARY1_INDEX, PREC2,
      "", "A");

  line = window->newLine(grid);
  // Main voltage S2 display
  new StaticText(line, rect_t{}, "Module Power:");
  new DynamicNumber<uint16_t>(
      line, rect_t{},
      [] { return (getcsd203extVoltage()*getcsd203extCurrent()/10)/1000; }, COLOR_THEME_PRIMARY1_INDEX, PREC2,
      "", "W");

  // Voice check enable
  line = window->newLine(grid);
  //new StaticText(line, rect_t{}, STR_RTC_CHECK, 0, COLOR_THEME_PRIMARY1);
  new StaticText(line, rect_t{}, "Ext Module protect");

  box = new Window(line, rect_t{});
  box->setFlexLayout(LV_FLEX_FLOW_ROW, lv_dpx(8));
  lv_obj_set_style_grid_cell_x_align(box->getLvObj(), LV_GRID_ALIGN_STRETCH, 0);
  lv_obj_set_style_flex_cross_place(box->getLvObj(), LV_FLEX_ALIGN_CENTER, 0);
  new ToggleSwitch(box, rect_t{}, GET_SET_INVERTED(g_eeGeneral.extModuleprotect));

  //new StaticText(line, rect_t{}, "Protect Current 2A", 0, COLOR_THEME_PRIMARY1);

  // 
  line = window->newLine(grid);
  //ext current display
  new StaticText(line, rect_t{}, "Protect Current:");
  new DynamicNumber<uint16_t>(
      line, rect_t{},
      [] { return g_eeGeneral.extmaxcurrent/10; }, COLOR_THEME_PRIMARY1_INDEX, PREC2,
      "", "A");

  line = window->newLine(grid);
  // Main volume Pitch rate Roller rate
  new StaticText(line, rect_t{}, "Protect adjust");
  new Slider(line, lv_pct(50), 200, 3000, GET_SET_DEFAULT(g_eeGeneral.extmaxcurrent));

  //int32_t extmaxcur=800;
  //line = window->newLine(&grid);
  /*new StaticText(line, rect_t{}, "Max Current", 0, COLOR_THEME_PRIMARY1);
  auto ebatMax =
      new NumberEdit(box, rect_t{0, 0, 80, 0}, 5000, 20000,
                     GET_SET_WITH_OFFSET(g_eeGeneral.txVoltageCalibration, 12000), 0, PREC1);
  ebatMax->setSuffix("mA");
  ebatMax->setWindowFlags(REFRESH_ALWAYS);*/


  line = window->newLine(grid);
  new Subtitle(window, "INTERNAL MODULE:");
    //line = window->newLine(&grid);
  // headline "Internal module"
   // new StaticText(line, rect_t{}, STR_INTERNAL_MODULE, 0, COLOR_THEME_PRIMARY1);

  line = window->newLine(grid);
  name=new StaticText(line, rect_t{}, "Module name:");
  name->setText(STR_MODULE_PROTOCOLS[g_model.moduleData[INTERNAL_MODULE].type]);
  //lv_obj_add_flag(module_status_w->getLvObj(), LV_OBJ_FLAG_HIDDEN);

  if (isModuleCrossfire(INTERNAL_MODULE)) {
    char statusText[64];

    line = window->newLine(grid);
    auto hz = 1000000 / getMixerSchedulerPeriod();
    // snprintf(statusText, 64, "%d Hz %" PRIu32 " Err", hz, telemetryErrors);
    snprintf(statusText, 64, "%d Hz", hz);
    //status->setText(statusText);
    //lv_obj_clear_flag(module_status_w->getLvObj(), LV_OBJ_FLAG_HIDDEN);
    new StaticText(line, rect_t{}, statusText);
  }
  // MPM is able to provide status
  if (isModuleMultimodule(INTERNAL_MODULE)) {
    char statusText[64];

    line = window->newLine(grid);
    getMultiModuleStatus(INTERNAL_MODULE).getStatusString(statusText);
    //name->setText(statusText);
    //lv_obj_clear_flag(name->getLvObj(), LV_OBJ_FLAG_HIDDEN);
    new StaticText(line, rect_t{}, statusText);
  }

  line = window->newLine(grid);
  //Int current display
  new StaticText(line, rect_t{}, "Module Current:");
  new DynamicNumber<uint16_t>(
      line, rect_t{},
      [] { return getcsd203IntCurrent()/10; }, COLOR_THEME_PRIMARY1_INDEX, PREC2,
      "", "A");

  line = window->newLine(grid);
  // Main voltage S2 display
  new StaticText(line, rect_t{}, "Module Power:");
  new DynamicNumber<uint16_t>(
      line, rect_t{},
      [] { return (getCSD203BatteryVoltage()*getcsd203IntCurrent()/10)/1000; }, COLOR_THEME_PRIMARY1_INDEX, PREC2,
      "", "W");
  // Voice check enable
  line = window->newLine(grid);
  //new StaticText(line, rect_t{}, STR_RTC_CHECK, 0, COLOR_THEME_PRIMARY1);
  new StaticText(line, rect_t{}, "Int Module protect");

  box = new Window(line, rect_t{});
  box->setFlexLayout(LV_FLEX_FLOW_ROW, lv_dpx(8));
  lv_obj_set_style_grid_cell_x_align(box->getLvObj(), LV_GRID_ALIGN_STRETCH, 0);
  lv_obj_set_style_flex_cross_place(box->getLvObj(), LV_FLEX_ALIGN_CENTER, 0);
  new ToggleSwitch(box, rect_t{}, GET_SET_INVERTED(g_eeGeneral.intModuleprotect));

  new StaticText(line, rect_t{}, "Protect Current 1A");

#if defined(IMU_SENSOR) 

  line = window->newLine(grid);
  new Subtitle(window, "IMU sensor:");

  line = window->newLine(grid);
  uint8_t IMUID=GetIMUID();
  if(IMUID==MU42627ID){
      new StaticText(line, rect_t{}, "ICM42627");
  }
  else if(IMUID==MU6050ID){
      new StaticText(line, rect_t{}, "MPU6050");
  }
  else{
      new StaticText(line, rect_t{}, "NULL");
  }
#endif

  line = window->newLine(grid);
  new Subtitle(window, "Current sensor:");

  if(CSD203MainInitFlag==true) {
      line = window->newLine(grid);
      new StaticText(line, rect_t{}, "System sensor OK");
  }
  if(CSD203ExtInitFlag==true) {
      line = window->newLine(grid);
      new StaticText(line, rect_t{}, "External sensor OK");
  }
  if(CSD203InInitFlag==true) {
      line = window->newLine(grid);
      new StaticText(line, rect_t{}, "Internal sensor OK");
  }

  line = window->newLine(grid);
  //total current display
  new StaticText(line, rect_t{}, "");
  // 
  line = window->newLine(grid);
  //if(usbChargerLed()) {
  if(1){
    //total current display
    new StaticText(line, rect_t{}, "Total Current:");
    new DynamicNumber<uint16_t>(
      line, rect_t{},
      [] { return (getcsd203IntCurrent()+getcsd203extCurrent()+220)/10; }, COLOR_THEME_PRIMARY1_INDEX, PREC2,
      "", "A");
  }
  else{
    //total current display
    new StaticText(line, rect_t{}, "Total Current:");
    new DynamicNumber<uint16_t>(
      line, rect_t{},
      [] { return (getcsd203IntCurrent()+getcsd203extCurrent()+getcsd203MainCurrent())/10; }, COLOR_THEME_PRIMARY1_INDEX, PREC2,
      "", "A");
  }

  line = window->newLine(grid);
  //if(usbChargerLed()) {
  if(1){
    // Main voltage S2 display
    new StaticText(line, rect_t{}, "Total Power:");
    new DynamicNumber<uint16_t>(
      line, rect_t{},
      [] { return (getCSD203BatteryVoltage()*(getcsd203IntCurrent()+getcsd203extCurrent()+220)/10)/1000; }, COLOR_THEME_PRIMARY1_INDEX, PREC2,
      "", "W");
  }
  else{
    // Main voltage S2 display
    new StaticText(line, rect_t{}, "Total Power:");
    new DynamicNumber<uint16_t>(
      line, rect_t{},
      [] { return (getCSD203BatteryVoltage()*(getcsd203IntCurrent()+getcsd203extCurrent()+getcsd203MainCurrent())/10)/1000; }, COLOR_THEME_PRIMARY1_INDEX, PREC2,
      "", "W");
  }

  // 
  //line = window->newLine(grid);
  //total current display
  //new StaticText(line, rect_t{}, "Total power:", 0, COLOR_THEME_PRIMARY1);
  //new DynamicNumber<uint16_t>(line, rect_t{}, [] {
  //    return (((getcsd203IntCurrent()+getcsd203extCurrent()+getcsd203MainCurrent())/1)*getcsd203BatteryVoltage()/1)*10;
  //}, COLOR_THEME_PRIMARY1 | PREC2, nullptr, "mW");

  line = window->newLine(grid);
  new Subtitle(window, "Speech recognition Management");
  //new Subtitle(window, "Speech recognition config:");
  //auto serial = new SerialConfigWindow(window, rect_t{});S

  //setFlexLayout();
  //FlexGridLayout grid(col_dsc, row_dsc, 2);

  for (uint8_t port_nr = 1; port_nr < MAX_SERIAL_PORTS-1; port_nr++) {
    auto port = serialGetPort(port_nr);
    if (!port || !port->name) continue;

    //line = newLine(&grid);
    line = window->newLine(grid);
    new StaticText(line, rect_t{}, port->name);

    box = new Window(line, rect_t{});
    box->setFlexLayout(LV_FLEX_FLOW_ROW, lv_dpx(8));
    lv_obj_set_style_grid_cell_x_align(box->getLvObj(), LV_GRID_ALIGN_STRETCH, 0);
    lv_obj_set_style_flex_cross_place(box->getLvObj(), LV_FLEX_ALIGN_CENTER, 0);
    
    auto aux = new Choice(
        box, rect_t{}, STR_AUX_SERIAL_MODES, 0, UART_MODE_MAX,
        [=]() { return serialGetMode(port_nr); },
        [=](int value) {
          serialSetMode(port_nr, value);
          serialInit(port_nr, value);
          SET_DIRTY();
        });
    aux->setAvailableHandler(
        [=](int value) { return isSerialModeAvailable(port_nr, value); });

    if (port->set_pwr != nullptr) {
      new StaticText(box, rect_t{}, " ");
      new ToggleSwitch(
          box, rect_t{}, [=] { return serialGetPower(port_nr); },
          [=](int8_t newValue) {
            serialSetPower(port_nr, (bool)newValue);
            SET_DIRTY();
          });
    }
  }

  //Voice control mapping
  // Module and receivers versions
  auto btn = new TextButton(window, rect_t{}, "Voice control mapping");
  btn->setPressHandler([=]() -> uint8_t {
    //new VoicemappingDialog(window);
    return 0;
  });
  lv_obj_set_width(btn->getLvObj(), lv_pct(100));

  //FlexGridLayout grid2(col_dsc, row_dsc, 4);
  //line = window->newLine(grid2);
  //line->padTop(8);

  // Modules
  //new MSubScreenButton(line, STR_INTERNALRF, []() { new ModulePage(INTERNAL_MODULE); }, []() { return g_model.moduleData[INTERNAL_MODULE].type > 0; });
  //new MSubScreenButton(line, STR_EXTERNALRF, []() { new ModulePage(EXTERNAL_MODULE); }, []() { return g_model.moduleData[EXTERNAL_MODULE].type > 0; });
  //new MSubScreenButton(line, STR_TRAINER, []() { new TrainerPage(); }, []() { return g_model.trainerData.mode > 0; });

}
