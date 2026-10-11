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

#include "hal/gpio.h"
#include "stm32_gpio.h"

#include "board.h"
#if !defined(RADIO_V14) && !defined(RADIO_V14LCD)
#include "hal/usb_driver.h"
#endif

void usbChargerInit()
{
#if defined(RADIO_V14) || defined(RADIO_V14LCD)
  // Dedicated charge jack STAT: active-low while charging (open-drain).
  // Independent of data-port usbPlugged().
  gpio_init(USB_CHARGER_GPIO, GPIO_IN_PU, GPIO_PIN_SPEED_LOW);
#else
  gpio_init(USB_CHARGER_GPIO, GPIO_IN_PU, GPIO_PIN_SPEED_LOW);
#endif
}

bool usbChargerLed()
{
#if defined(RADIO_V14) || defined(RADIO_V14LCD)
  return !gpio_read(USB_CHARGER_GPIO);
#else
  return (!gpio_read(USB_CHARGER_GPIO) && usbPlugged());
#endif
}
