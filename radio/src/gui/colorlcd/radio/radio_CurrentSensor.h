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

#ifndef _RADIO_CURRENT_H_
#define _RADIO_CURRENT_H_

#pragma once

#include "edgetx.h"
#include "tabsgroup.h"

class ModelMenu;

class RadioCurrentPage : public PageTab
{
  void checkEvents() override;

 public:
  RadioCurrentPage();

  void build(FormWindow* window) override;
};
class RadioCurrentTotalPage : public PageTab
{
  void checkEvents() override;

 public:
  RadioCurrentTotalPage();

  void build(FormWindow* window) override;
};

class RadioCurrentMenu: public TabsGroup {
  public:
    RadioCurrentMenu(ModelMenu* parent = nullptr);

  protected:
    ModelMenu* parentMenu = nullptr;

    bool _modelTelemetryEnabled = true;

#if defined(HARDWARE_KEYS)
  void onPressSYS() override;
  void onLongPressSYS() override;
  void onPressMDL() override;
  void onLongPressMDL() override;
  void onPressTELE() override;
  void onLongPressSENSORVIEW() override;
  void onLongPressTELEVIEW() override;
  void onLongPressCHVIEW() override;
#endif
};

class RadioTelemetryMenu: public TabsGroup {
  public:
    RadioTelemetryMenu(ModelMenu* parent = nullptr);

  protected:
    ModelMenu* parentMenu = nullptr;

    bool _modelTelemetryEnabled = true;

#if defined(HARDWARE_KEYS)
  void onPressSYS() override;
  void onLongPressSYS() override;
  void onPressMDL() override;
  void onLongPressMDL() override;
  void onPressTELE() override;
  void onLongPressSENSORVIEW() override;
  void onLongPressTELEVIEW() override;
  void onLongPressCHVIEW() override;
#endif
};

#endif //_RADIO_CURRENT_H_
