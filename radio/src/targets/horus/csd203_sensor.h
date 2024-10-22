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

#pragma once

void initCSD203(void);
void readCSD203(void);

void Getcsdloop(void);

extern uint16_t getCSD203BatteryVoltage(void);      //BAT S2 Voltage
extern uint16_t getcsd203BatteryS1Voltage(void);    //BAT S1 Voltage
extern int16_t getcsd203MainCurrent(void);         //MAIN SYS CURRENT
extern int16_t getcsd203OldMainCurrent(void);      //MAIN SYS CURRENT
extern uint16_t getcsd203extVoltage(void);          //EXTMODEL Voltage
extern int16_t getcsd203extCurrent(void);          //EXTMODEL CURRENT

extern int16_t getcsd203IntCurrent(void);          //INTMODEL CURRENT

extern uint8_t voiceSwitch;

extern bool ExtModuleprotect;

extern bool CSD203MainInitFlag;
extern bool CSD203InInitFlag;
extern bool CSD203ExtInitFlag;

