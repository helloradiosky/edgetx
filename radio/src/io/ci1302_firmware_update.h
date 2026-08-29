/*
 * Copyright (C) EdgeTX
 *
 * License GPLv2: http://www.gnu.org/licenses/gpl-2.0.html
 *
 * Flash CI1302 / CI-03T voice module via ChipIntelli UART download protocol
 * (same flow as PACK_UPDATE_TOOL / code_program.exe), reading the image from SD.
 */

#pragma once

#include "gui/common/stdlcd/popups_common.h"

/** True if `name` looks like a CI-03T / CI1302 voice firmware `.bin`. */
bool isCi1302VoiceFirmware(const char* name, const char* fullpath = nullptr);

class Ci1302FirmwareUpdate
{
 public:
  bool flashFirmware(const char* filename, ProgressHandler progressHandler);
};
