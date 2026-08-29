/*
 * Copyright (C) EdgeTX
 *
 * License GPLv2: http://www.gnu.org/licenses/gpl-2.0.html
 */

#pragma once

#if defined(RADIO_V14) || defined(RADIO_V14LCD)

#include <stdint.h>
#include "keys.h"

// True when STAT or charge current says the adapter is charging now.
bool v14ChargePresent();

// True when charge present for several consecutive samples.
bool v14IsCharging();

// True while the full-screen charge overlay is shown.
bool v14ChargeUiActive();

// Runtime poll: detect charging, update overlay state (non-blocking).
void v14ChargeUiPoll();

// Handle keys / voice menu while overlay is active.
void v14ChargeUiHandleEvent(event_t evt);

// Draw the charge overlay (no-op when inactive).
void v14ChargeUiDraw();

// Full-screen charge UI until EXIT or charger gone (power-off path only).
void v14ChargeUiRunUntilExit(bool shutdownMode);

#endif
