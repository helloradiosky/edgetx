/*
 * Copyright (C) EdgeTX
 *
 * License GPLv2: http://www.gnu.org/licenses/gpl-2.0.html
 *
 * Flash V14 ESP modules via the UART ROM download protocol (esptool, no stub):
 *   - internal ELRS TX (ESP32) on the module UART. App-only at 0x10000 if
 *     only the app .bin is present; blank/factory chips need PIO sidecars
 *     in the same folder: bootloader.bin @ 0x1000, partitions.bin @ 0x8000,
 *     optional boot_app0.bin @ 0xe000 (forces dual-OTA to boot the new app0).
 *   - ELRS backpack (ESP8285) via ESP32 UART passthrough (app @ 0x00000).
 */

#pragma once

#include "gui/common/stdlcd/popups_common.h"

/** True if `name` is an ELRS TX image (`.bin` containing `ELRS`, not `BACKUP`). */
bool isInternalElrsAppFirmware(const char* name, const char* fullpath = nullptr);

/** True if `name` is a backpack image (`.bin` containing `BACKUP`). */
bool isInternalElrsBackpackFirmware(const char* name,
                                    const char* fullpath = nullptr);

class Esp32ElrsFirmwareUpdate
{
 public:
  bool flashFirmware(const char* filename, ProgressHandler progressHandler);
};

class Esp32BackpackFirmwareUpdate
{
 public:
  bool flashFirmware(const char* filename, ProgressHandler progressHandler);
};
