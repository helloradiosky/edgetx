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

#include "ext_antenna_settings.h"
#include "choice.h"

#include "edgetx.h"

#define SET_DIRTY() storageDirty(EE_MODEL)

ExtAntennaSettings::ExtAntennaSettings(Window* parent,
                                 const FlexGridLayout& g,
                                 uint8_t moduleIdx) :
    Window(parent, rect_t{}), md(&g_model.moduleData[moduleIdx])
{
  FlexGridLayout grid(g);
  setFlexLayout();

  auto line = newLine(grid);
  new StaticText(line, rect_t{}, STR_ANTENNA);

  if (md->antennaMode != ANTENNA_MODE_INTERNAL &&
      md->antennaMode != ANTENNA_MODE_EXTERNAL) {
    md->antennaMode = ANTENNA_MODE_INTERNAL;
    SET_DIRTY();
  }

  auto antennaChoice = new Choice(
      line, rect_t{}, STR_ANTENNA_MODES, ANTENNA_MODE_INTERNAL,
      ANTENNA_MODE_EXTERNAL,
#if defined(INTMODULE_ANTSEL_GPIO)
      [=]() -> int32_t {
        int8_t global = g_eeGeneral.antennaMode;
        if (global == ANTENNA_MODE_INTERNAL || global == ANTENNA_MODE_EXTERNAL)
          return global;
        int8_t mode = md->antennaMode;
        if (mode == ANTENNA_MODE_INTERNAL || mode == ANTENNA_MODE_EXTERNAL)
          return mode;
        return ANTENNA_MODE_INTERNAL;
      },
#else
      GET_DEFAULT(md->antennaMode),
#endif
      [=](int32_t antenna) -> void {
        setAntennaModeWithConfirm(antenna, EE_MODEL, [=](int8_t mode) {
          md->antennaMode = mode;
#if defined(INTMODULE_ANTSEL_GPIO)
          if (g_eeGeneral.antennaMode != ANTENNA_MODE_PER_MODEL) {
            g_eeGeneral.antennaMode = ANTENNA_MODE_PER_MODEL;
            storageDirty(EE_GENERAL);
          }
#endif
        });
      });

  antennaChoice->setAvailableHandler([=](int8_t mode) {
    return mode == ANTENNA_MODE_INTERNAL || mode == ANTENNA_MODE_EXTERNAL;
  });
}
