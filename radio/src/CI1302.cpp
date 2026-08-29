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
#include "edgetx.h"
#include "CI1302.h"
#include "timers_driver.h"
#include "debug.h"
#include "audio.h"
#include "serial.h"
#include "storage/storage.h"

#define VOICE_FRAME_DELAY_US 500

#define VOICE_START_BYTE        0x55
#define VOICE_STARTCN_BYTE      0xAA
#define VOICE_STARTEN_BYTE      0xAB

#define VOICE_FRAME_SIZE        6

#define VOICE_END_BYTE1         0xA5
#define VOICE_END_BYTE2         0x5A

#define VOICE_WKUP                      0x01
#define VOICE_password	                0x02
#define VOICE_Aileron_adjustment	      0x03
#define VOICE_Pitch_adjustment	        0x04
#define VOICE_Rotating_adjustment	      0x05
#define VOICE_Up	                      0x0A
#define VOICE_Down	                    0x0B
#define VOICE_Left_one	                0x14
#define VOICE_Right_one	                0x15
#define VOICE_Confirm                   0x1A
#define VOICE_Cancel                    0x1B
#define VOICE_Model_Menu                0x21
#define VOICE_System_Menu               0x22
#define VOICE_Channel_Setup             0x23
#define VOICE_Channel_Monitor           0X32
#define VOICE_Internal_Module           0X33
#define VOICE_External_Module           0X34
#define VOICE_Aileron_Reverse	          0x4A
#define VOICE_Elevator_Reverse	        0x4B
#define VOICE_Rudder_Reverse	          0x4C
#define VOICE_Motion_Control	          0x55
#define VOICE_Telemetry                 0x5B
#define VOICE_Open_Gear                 0x7E
#define VOICE_Close_Gear                0x80
#define VOICE_Open_Flap                 0x82
#define VOICE_Close_Flap                0x85
#define VOICE_POWEROFF                  0x9E
#define VOICE_Start_recording	          0x88
#define VOICE_End_recording	            0x89
#define VOICE_Photo	                    0x8A
#define VOICE_Take_a_Photo	            0x8B
#define VOICE_go_home	                  0x8C
#define VOICE_GPS_Return	              0x91
#define VOICE_hi_take_off	              0x9F
#define VOICE_hi_land	                  0xA1
#define VOICE_Sensorstatus	            0xA2

struct voice_cmd_inputs_t {
  uint8_t VOICESTARTBYTE1;
  uint8_t VOICELanguage;
  uint8_t VOICECMD;
  uint8_t VOICECMDR;
  uint8_t VOICEENDBYTE1;
  uint8_t VOICEENDBYTE2;
};

bool VoiceRunStatus=true;
bool VoiceGearStatus=false;
uint8_t VoiceFlapStatus=false;
bool VoicePowerStatus=false;

static uint32_t wkup_delay=0;

#define wkup_delayms  10000

#if defined(VOICE_CONTROL_SENSOR)
static void voiceMotionControlEnable();
static void voiceControlGuiHandleCmd(uint8_t voiceCmd);
bool CI1302_voiceSwitchBootGuardActive();
#endif

void processVoiceFrame(uint8_t * voicedata, uint32_t size)
{
  voice_cmd_inputs_t *voiceframe=(voice_cmd_inputs_t*)voicedata;

  if (size != VOICE_FRAME_SIZE || voiceframe->VOICESTARTBYTE1 != VOICE_START_BYTE ||
      voiceframe->VOICEENDBYTE1 != VOICE_END_BYTE1||
      voiceframe->VOICEENDBYTE2 != VOICE_END_BYTE2) {
    return;
  }
  if((voiceframe->VOICECMD+voiceframe->VOICECMDR)!=0xff)return;
#if defined(VOICE_CONTROL_SENSOR)
  // Motion control: no wake word required — call directly anytime.
  if ((voiceframe->VOICELanguage == VOICE_STARTEN_BYTE ||
       voiceframe->VOICELanguage == VOICE_STARTCN_BYTE) &&
      voiceframe->VOICECMD == VOICE_Motion_Control) {
    voiceMotionControlEnable();
    AUDIO_OKAY();
    return;
  }

  if (voiceframe->VOICELanguage == VOICE_STARTEN_BYTE || voiceframe->VOICELanguage == VOICE_STARTCN_BYTE) {
    resetBacklightTimeout();
    if(voiceframe->VOICECMD==VOICE_WKUP){
        wkup_delay=timersGetMsTick();
        AUDIO_HELLORADIO();
      }
      else if (voiceframe->VOICECMD == VOICE_Open_Gear) {
      if (!CI1302_voiceSwitchBootGuardActive()) {
        wkup_delay = timersGetMsTick();
        VoiceGearStatus = true;
      }
    }
    else if (voiceframe->VOICECMD == VOICE_Close_Gear) {
      if (!CI1302_voiceSwitchBootGuardActive()) {
        wkup_delay = timersGetMsTick();
        VoiceGearStatus = false;
      }
    }
    else if (voiceframe->VOICECMD == VOICE_Open_Flap) {
      if (!CI1302_voiceSwitchBootGuardActive()) {
        wkup_delay = timersGetMsTick();
        if (VoiceFlapStatus < 2) {
          VoiceFlapStatus++;
        }
      }
    }
    else if (voiceframe->VOICECMD == VOICE_Close_Flap) {
      if (!CI1302_voiceSwitchBootGuardActive()) {
        wkup_delay = timersGetMsTick();
        if (VoiceFlapStatus > 0) {
          VoiceFlapStatus--;
        }
      }
    }
    else if (voiceframe->VOICECMD == VOICE_Telemetry ||
             voiceframe->VOICECMD == VOICE_Sensorstatus ||
             voiceframe->VOICECMD == VOICE_Channel_Monitor ||
             voiceframe->VOICECMD == VOICE_Internal_Module ||
             voiceframe->VOICECMD == VOICE_External_Module ||
             voiceframe->VOICECMD == VOICE_Model_Menu ||
             voiceframe->VOICECMD == VOICE_System_Menu ||
             voiceframe->VOICECMD == VOICE_Confirm ||
             voiceframe->VOICECMD == VOICE_Cancel) {
      wkup_delay = timersGetMsTick();
      voiceControlGuiHandleCmd(voiceframe->VOICECMD);
    }
  }
#endif
}

extern "C" int GetVoiceInput(uint8_t *rxchar)
{
  auto _getByte = voiceGetByte;
  if (_getByte && (_getByte(rxchar) > 0)) {
    return 0;
  }
  return -1;
}

void processUpdataInput(void)
{
  uint8_t rxchar;
  auto _getByte = dbgGetByte;
  while (_getByte && (_getByte(&rxchar) > 0)) {
    voiceSerialPutc(rxchar);
  }
}

void processVoiceInput(void)
{
  static uint8_t VoiceIndex = 0;
  static uint32_t VoiceTimer;
  static uint8_t VoiceFrame[VOICE_FRAME_SIZE];

  uint32_t active = 0;

  if(VoiceRunStatus==false)return;

  uint8_t rxchar;
  auto _getByte = voiceGetByte;
  while (_getByte && (_getByte(&rxchar) > 0)) {
    active = 1;
    if (VoiceIndex > VOICE_FRAME_SIZE - 1) {
      VoiceIndex = VOICE_FRAME_SIZE - 1;
    }
    VoiceFrame[VoiceIndex++] = rxchar;
    TRACE("0x%x", rxchar);
  }

  if (wkup_delay)
  {
    if ((uint32_t)(timersGetMsTick() - wkup_delay) > wkup_delayms) {
      wkup_delay = 0;
    }
  }

  if (active) {
    VoiceTimer = timersGetUsTick();
    return;
  }
  if (VoiceIndex) {
    if ((uint32_t)(timersGetUsTick() - VoiceTimer) > VOICE_FRAME_DELAY_US) {
      processVoiceFrame(VoiceFrame, VoiceIndex);
      VoiceIndex = 0;
    }
  }
}

#if defined(VOICE_CONTROL_SENSOR)

#include "drivers/CI1302_voice_integration.h"
#include "globals.h"
#include "gyro.h"
#include "input_mapping.h"
#include "keys.h"
#include "myeeprom.h"
#include "storage/yaml/yaml_bits.h"
#include <string.h>

#if defined(COLORLCD)
#include "module_setup.h"
#include "quick_menu.h"
#include "view_channels.h"
#include "view_main.h"
#include "LvglWrapper.h"
#else
#include "gui/common/stdlcd/menus_common.h"
#include "keys.h"
extern uint8_t menuLevel;

static volatile uint8_t s_pendingVoiceGuiCmd = 0;
static volatile event_t s_pendingVoiceKeyEvt = 0;

static void voiceControlGuiQueueCmd(uint8_t voiceCmd)
{
  s_pendingVoiceGuiCmd = voiceCmd;
}

static void voiceControlGuiQueueKeyEvent(event_t evt)
{
  s_pendingVoiceKeyEvt = evt;
}

void CI1302_voiceGuiProcessPending()
{
  uint8_t cmd = s_pendingVoiceGuiCmd;
  if (cmd && menuLevel == 0) {
    s_pendingVoiceGuiCmd = 0;
    switch (cmd) {
      case VOICE_Channel_Monitor:
        pushMenu(menuChannelsView);
        break;
      case VOICE_Telemetry:
      case VOICE_Sensorstatus:
        chainMenu(menuViewTelemetry);
        break;
      case VOICE_Model_Menu:
        pushMenu(menuModelSelect);
        break;
      case VOICE_System_Menu:
        pushMenu(menuTabGeneral[0].menuFunc);
        break;
    }
  }

  event_t kevt = s_pendingVoiceKeyEvt;
  if (kevt) {
    s_pendingVoiceKeyEvt = 0;
    pushEvent(kevt);
  }
}
#endif

// --- motion control ---

bool MotionControlStatus = false;

static constexpr int16_t VOICE_MOTION_RELEASE_THRESHOLD = RESX / 4;
static constexpr uint8_t STICK_ROLE_ELE = 1;
static constexpr uint8_t STICK_ROLE_AIL = 3;

#if defined(IMU)
static int16_t gyroBaselineX = 0;
static int16_t gyroBaselineY = 0;
#endif

static void voiceMotionControlEnable()
{
#if defined(IMU)
  gyroBaselineX = gyroScaledX();
  gyroBaselineY = gyroScaledY();
#endif
  MotionControlStatus = true;
}

void CI1302_voiceMotionControlApplyToInputs(uint8_t pots_offset)
{
#if defined(IMU)
  if (!MotionControlStatus) {
    return;
  }

  const int16_t deltaX = gyroScaledX() - gyroBaselineX;
  const int16_t deltaY = gyroScaledY() - gyroBaselineY;

  for (uint8_t phys = 0; phys < pots_offset; phys++) {
    const uint8_t role = inputMappingConvertMode(phys);
    const int16_t stick = calibratedAnalogs[phys];

    if ((role == STICK_ROLE_AIL || role == STICK_ROLE_ELE) &&
        abs(stick) > VOICE_MOTION_RELEASE_THRESHOLD) {
      MotionControlStatus = false;
      return;
    }
  }

  for (uint8_t phys = 0; phys < pots_offset; phys++) {
    const uint8_t role = inputMappingConvertMode(phys);
    const int16_t stick = calibratedAnalogs[phys];

    if (role == STICK_ROLE_AIL) {
      calibratedAnalogs[phys] =
          limit<int16_t>(-RESX, stick + deltaX, RESX);
    } else if (role == STICK_ROLE_ELE) {
      calibratedAnalogs[phys] =
          limit<int16_t>(-RESX, stick + deltaY, RESX);
    }
  }
#else
  (void)pots_offset;
  MotionControlStatus = false;
#endif
}

// --- virtual voice switches (VGR / VFL) ---

enum VoiceSwitchId {
  VSW_GEAR = 0,
  VSW_FLAP,
};

static int8_t voiceSwitchIdx[VOICE_SWITCH_COUNT] = {
  -2, -2,
};

static const char* const voiceSwitchNames[VOICE_SWITCH_COUNT] = {
  "VGR", "VFL",
};

static constexpr uint32_t VOICE_BOOT_GUARD_MS = 5000;

static uint32_t VoiceBootGuardUntil = 0;

static void initVoiceMappedSwitches()
{
  if (voiceSwitchIdx[VSW_GEAR] != -2) {
    return;
  }

  int8_t base = (int8_t)(switchGetMaxSwitches() + MAX_FLEX_SWITCHES);
  for (uint8_t i = 0; i < VOICE_SWITCH_COUNT; i++) {
    voiceSwitchIdx[i] = base + i;
  }
}

static int8_t voiceSwitchIndex(VoiceSwitchId id)
{
  initVoiceMappedSwitches();
  return voiceSwitchIdx[id];
}

uint8_t CI1302_voiceSwitchExtraCount()
{
  return VOICE_SWITCH_COUNT;
}

bool CI1302_voiceSwitchIsIndex(uint8_t idx)
{
  initVoiceMappedSwitches();
  for (uint8_t i = 0; i < VOICE_SWITCH_COUNT; i++) {
    if ((int8_t)idx == voiceSwitchIdx[i]) {
      return true;
    }
  }
  return false;
}

static VoiceSwitchId voiceSwitchIdFromIndex(uint8_t idx)
{
  initVoiceMappedSwitches();
  for (uint8_t i = 0; i < VOICE_SWITCH_COUNT; i++) {
    if ((int8_t)idx == voiceSwitchIdx[i]) {
      return (VoiceSwitchId)i;
    }
  }
  return VSW_GEAR;
}

static bool isTwoPosVoiceSwitch(VoiceSwitchId id)
{
  return id != VSW_FLAP;
}

bool CI1302_voiceSwitchIsPositionAvailable(uint8_t idx, uint8_t position)
{
  if (!CI1302_voiceSwitchIsIndex(idx)) {
    return false;
  }

  if (isTwoPosVoiceSwitch(voiceSwitchIdFromIndex(idx)) && position == 1) {
    return false;
  }

  return true;
}

static SwitchHwPos voiceSwitchPosition(uint8_t idx)
{
  switch (voiceSwitchIdFromIndex(idx)) {
    case VSW_GEAR:
      return VoiceGearStatus ? SWITCH_HW_DOWN : SWITCH_HW_UP;

    case VSW_FLAP: {
      uint8_t flapPos = (VoiceFlapStatus >= 2) ? 2 : VoiceFlapStatus;
      return (SwitchHwPos)flapPos;
    }

    default:
      return SWITCH_HW_UP;
  }
}

bool CI1302_voiceSwitchTryGetPosition(uint8_t idx, SwitchHwPos* pos)
{
  if (!CI1302_voiceSwitchIsIndex(idx)) {
    return false;
  }

  *pos = voiceSwitchPosition(idx);
  return true;
}

const char* CI1302_voiceSwitchTryGetName(uint8_t idx)
{
  initVoiceMappedSwitches();
  for (uint8_t i = 0; i < VOICE_SWITCH_COUNT; i++) {
    if ((int8_t)idx == voiceSwitchIdx[i]) {
      return voiceSwitchNames[i];
    }
  }
  return nullptr;
}

int8_t CI1302_voiceSwitchTryGetIndexFromName(const char* name)
{
  for (uint8_t i = 0; i < VOICE_SWITCH_COUNT; i++) {
    if (strcmp(name, voiceSwitchNames[i]) == 0) {
      return voiceSwitchIndex((VoiceSwitchId)i);
    }
  }
  return -1;
}

bool CI1302_voiceSwitchTryGetHwType(uint8_t idx, SwitchHwType* type)
{
  if (!CI1302_voiceSwitchIsIndex(idx)) {
    return false;
  }

  *type = SWITCH_HW_3POS;
  return true;
}

bool CI1302_voiceSwitchTryIsAvailable(int swtch, int context, bool* available)
{
  int idx = swtch;

  if (idx < 0) {
    idx = -idx;
  }

  if (idx < SWSRC_FIRST_SWITCH || idx > SWSRC_LAST_SWITCH) {
    return false;
  }

  idx -= SWSRC_FIRST_SWITCH;
  div_t swinfo = div(idx, 3);

  if (!CI1302_voiceSwitchIsIndex(swinfo.quot)) {
    return false;
  }

  if (context == GeneralCustomFunctionsContext) {
    *available = false;
    return true;
  }

  *available = CI1302_voiceSwitchIsPositionAvailable(swinfo.quot, swinfo.rem);
  return true;
}

static bool voiceSwitchTryParseYamlTag(const char* val, uint8_t val_len,
                                       const char* tag, VoiceSwitchId id,
                                       int32_t* ival)
{
  const size_t tagLen = strlen(tag);
  if (val_len <= tagLen || strncmp(val, tag, tagLen) != 0) {
    return false;
  }

  if (val[tagLen] < '0' || val[tagLen] > '2') {
    return false;
  }

  int32_t sw = voiceSwitchIndex(id) * 3;
  if (sw < 0) {
    return false;
  }

  *ival = sw + yaml_str2int(val + tagLen, val_len - tagLen) + SWSRC_FIRST_SWITCH;
  return true;
}

bool CI1302_voiceSwitchTryParseYamlSrc(const char* val, uint8_t val_len, int32_t* ival)
{
  for (uint8_t i = 0; i < VOICE_SWITCH_COUNT; i++) {
    if (voiceSwitchTryParseYamlTag(val, val_len, voiceSwitchNames[i],
                                   (VoiceSwitchId)i, ival)) {
      return true;
    }
  }
  return false;
}

void CI1302_voiceSwitchInitBootGuard()
{
  VoiceBootGuardUntil = timersGetMsTick() + VOICE_BOOT_GUARD_MS;
}

bool CI1302_voiceSwitchBootGuardActive()
{
  return VoiceBootGuardUntil != 0 && timersGetMsTick() < VoiceBootGuardUntil;
}

bool CI1302_voiceSwitchSuppressMovedAudio(uint8_t idx, bool startup)
{
  if (!CI1302_voiceSwitchIsIndex(idx)) {
    return false;
  }

  return startup || CI1302_voiceSwitchBootGuardActive();
}

bool CI1302_voiceSwitchIsSwitchSource(swsrc_t swtch)
{
  int idx = swtch;
  if (idx < 0) {
    idx = -idx;
  }

  if (idx < SWSRC_FIRST_SWITCH || idx > SWSRC_LAST_SWITCH) {
    return false;
  }

  div_t swinfo = div(idx - SWSRC_FIRST_SWITCH, 3);
  return CI1302_voiceSwitchIsIndex(swinfo.quot);
}

bool CI1302_voiceSwitchShouldSuppressFunctionAudio(swsrc_t swtch, bool risingEdge)
{
  if (!risingEdge || !CI1302_voiceSwitchBootGuardActive()) {
    return false;
  }

  return CI1302_voiceSwitchIsSwitchSource(swtch);
}

// --- voice menu navigation ---

#if defined(COLORLCD)
static bool voiceControlGuiOnMainScreen()
{
  auto viewMain = ViewMain::instance();
  if (!viewMain) {
    return true;
  }

  Window* top = Window::topWindow();
  if (!top) {
    return true;
  }

  return top == viewMain || top == viewMain->getTopbar();
}
#endif

static void voiceControlGuiHandleCmd(uint8_t voiceCmd)
{
#if defined(COLORLCD)
  switch (voiceCmd) {
    case VOICE_Channel_Monitor:
      AUDIO_OKAY();
      new ChannelsViewMenu();
      return;

    case VOICE_Telemetry:
    case VOICE_Sensorstatus:
      AUDIO_OKAY();
      QuickMenu::openPage(QM_MODEL_TELEMETRY);
      return;

    case VOICE_Internal_Module:
      AUDIO_OKAY();
      new ModulePage(INTERNAL_MODULE);
      return;

    case VOICE_External_Module:
      AUDIO_OKAY();
      new ModulePage(EXTERNAL_MODULE);
      return;

    case VOICE_Model_Menu:
      AUDIO_OKAY();
      QuickMenu::openPage(QM_MODEL_SETUP);
      return;

    case VOICE_System_Menu:
      AUDIO_OKAY();
      QuickMenu::openPage(QM_RADIO_SETUP);
      return;
  }
#else
  switch (voiceCmd) {
    case VOICE_Channel_Monitor:
    case VOICE_Telemetry:
    case VOICE_Sensorstatus:
    case VOICE_Model_Menu:
    case VOICE_System_Menu:
      if (menuLevel == 0) {
        AUDIO_OKAY();
        voiceControlGuiQueueCmd(voiceCmd);
      }
      return;

    case VOICE_Internal_Module:
    case VOICE_External_Module:
      AUDIO_OKAY();
      return;
  }
#endif

  switch (voiceCmd) {
    case VOICE_Confirm:
      AUDIO_OKAY();
#if defined(COLORLCD)
      if (voiceControlGuiOnMainScreen()) {
        pushEvent(EVT_KEY_BREAK(KEY_ENTER));
      } else {
        lvglRequestKeyboardEnter();
      }
#else
      voiceControlGuiQueueKeyEvent(EVT_KEY_BREAK(KEY_ENTER));
#endif
      break;

    case VOICE_Cancel:
#if defined(COLORLCD)
      if (voiceControlGuiOnMainScreen()) {
        break;
      }
#endif
      AUDIO_OKAY();
      MotionControlStatus = false;
#if defined(COLORLCD)
      pushEvent(EVT_KEY_BREAK(KEY_EXIT));
#else
      voiceControlGuiQueueKeyEvent(EVT_KEY_BREAK(KEY_EXIT));
#endif
      break;
  }
}

// --- shared-code integration hooks ---

void CI1302_voiceIntegrationPer10ms()
{
  // Voice UART is polled from mixerTask execMixerFrequentActions()
  // (same as hrs_v14) for timely wake-frame assembly.
}

void CI1302_voiceIntegrationOnFlightReset()
{
  CI1302_voiceSwitchInitBootGuard();
}

bool CI1302_voiceIntegrationMixSrcValue(mixsrc_t i, getvalue_t* val)
{
  if (i == MIXSRC_VGR) {
    *val = VoiceGearStatus ? 1024 : -1024;
    return true;
  }

  if (i == MIXSRC_VFL) {
    switch (VoiceFlapStatus) {
      case 0:
        *val = -1024;
        break;
      case 1:
        *val = 0;
        break;
      default:
        *val = 1024;
        break;
    }
    return true;
  }

  return false;
}

const char* CI1302_voiceIntegrationMixSrcName(mixsrc_t idx)
{
  if (idx == MIXSRC_VGR) {
    return "VGR";
  }

  if (idx == MIXSRC_VFL) {
    return "VFL";
  }

  return nullptr;
}

bool CI1302_voiceIntegrationMixSrcParseYaml(const char* val, uint8_t val_len, uint32_t* out)
{
  if (val_len == 3 && val[0] == 'V' && val[1] == 'G' && val[2] == 'R') {
    *out = MIXSRC_VGR;
    return true;
  }

  if (val_len == 3 && val[0] == 'V' && val[1] == 'F' && val[2] == 'L') {
    *out = MIXSRC_VFL;
    return true;
  }

  return false;
}

bool CI1302_voiceIntegrationMixSrcWriteYaml(uint32_t val, const char** str)
{
  if (val == MIXSRC_VGR) {
    *str = "VGR";
    return true;
  }

  if (val == MIXSRC_VFL) {
    *str = "VFL";
    return true;
  }

  return false;
}

bool CI1302_voiceIntegrationSwitchSrcParseYaml(const char* val, uint8_t val_len, int32_t* ival)
{
  return CI1302_voiceSwitchTryParseYamlSrc(val, val_len, ival);
}

bool CI1302_voiceIntegrationModelGetSwitchType(uint8_t n, SwitchConfig* out)
{
  if (!CI1302_voiceSwitchIsIndex(n)) {
    return false;
  }

  *out = SWITCH_3POS;
  return true;
}

bool CI1302_voiceIntegrationModelSetSwitchType(uint8_t n)
{
  return CI1302_voiceSwitchIsIndex(n);
}

bool CI1302_voiceIntegrationIsSwitchAvailable(int swtch, int context, bool* available)
{
  return CI1302_voiceSwitchTryIsAvailable(swtch, context, available);
}

bool CI1302_voiceIntegrationIsSwitchSwitchAvailable(int swtch, bool* available)
{
  return CI1302_voiceSwitchTryIsAvailable(swtch + SWSRC_FIRST_SWITCH, 0, available);
}

#endif // VOICE_CONTROL_SENSOR

// --- V14 / V14LCD AUX serial hardware (ISP + runtime via serial "Voice" port) ---

#if defined(AUX_SERIAL)

#include "serial.h"
#include "hal/serial_driver.h"
#include "hal/serial_port.h"
#include "hal/gpio.h"
#include "stm32_gpio.h"
#include "board.h"

void voiceSerialSetSendCb(void* ctx, void (*cb)(void*, uint8_t));
void voiceSerialGetSendCb(void* crx, int (*cb)(void*, uint8_t*));

static void* _voice_isp_ctx = nullptr;
static int _voice_saved_mode = -1;

extern "C" void voiceControlPowerOn(void)
{
#if defined(AUX_SERIAL_PWR_GPIO)
  gpio_init(AUX_SERIAL_PWR_GPIO, GPIO_OUT, GPIO_PIN_SPEED_LOW);
  gpio_set(AUX_SERIAL_PWR_GPIO);
#endif
  VoicePowerStatus = true;
}

extern "C" void voiceControlPowerOff(void)
{
#if defined(AUX_SERIAL_PWR_GPIO)
  gpio_init(AUX_SERIAL_PWR_GPIO, GPIO_OUT, GPIO_PIN_SPEED_LOW);
  gpio_clear(AUX_SERIAL_PWR_GPIO);
#endif
  VoicePowerStatus = false;
}

static void voiceIspBindCallbacks(void* ctx, const etx_serial_port_t* port)
{
  if (!port || !port->uart) return;
  voiceSerialSetSendCb(ctx, port->uart->sendByte);
  voiceSerialGetSendCb(ctx, port->uart->getByte);
}

extern "C" void voiceControlUartInit(uint32_t baud)
{
  const etx_serial_port_t* port = auxSerialGetPort(SP_AUX1);
  if (!port || !port->uart || !port->uart->init) return;

  etx_serial_init cfg = {
      .baudrate = baud,
      .encoding = ETX_Encoding_8N1,
      .direction = ETX_Dir_TX_RX,
      .polarity = ETX_Pol_Normal,
  };

  if (!_voice_isp_ctx) {
    if (_voice_saved_mode < 0) {
      _voice_saved_mode = serialGetMode(SP_AUX1);
    }
    serialStop(SP_AUX1);
    _voice_isp_ctx = port->uart->init(port->hw_def, &cfg);
    voiceIspBindCallbacks(_voice_isp_ctx, port);
  } else if (port->uart->setBaudrate) {
    port->uart->setBaudrate(_voice_isp_ctx, baud);
  }
}

extern "C" void voiceControlUartReopen(uint32_t baud)
{
  const etx_serial_port_t* port = auxSerialGetPort(SP_AUX1);
  if (_voice_isp_ctx && port && port->uart && port->uart->deinit) {
    port->uart->deinit(_voice_isp_ctx);
    _voice_isp_ctx = nullptr;
  }
  voiceControlUartInit(baud);
}

extern "C" void voiceControlWaitTxCompleted(void)
{
  const etx_serial_port_t* port = auxSerialGetPort(SP_AUX1);
  if (_voice_isp_ctx && port && port->uart && port->uart->waitForTxCompleted) {
    port->uart->waitForTxCompleted(_voice_isp_ctx);
  }
}

extern "C" void voiceControlSetBaudrate(uint32_t baud)
{
  voiceControlPowerOn();
  voiceControlUartInit(baud);
}

void voiceControlInit(void)
{
#if (defined(RADIO_V14) || defined(RADIO_V14LCD)) && defined(AUX_SERIAL)
  if (!voiceModulePowerUiEnabled()) {
    voiceControlPowerOff();
    VoiceRunStatus = false;
    return;
  }
#endif

  // Must run after storageReadAll()/initSerialPorts(): EEPROM may leave AUX
  // powered off or in UART_MODE_NONE, which kills CI1302 RX / wake-up.
  VoiceRunStatus = true;
  voiceControlPowerOn();

  const etx_serial_port_t* port = auxSerialGetPort(SP_AUX1);
  if (_voice_isp_ctx && port && port->uart && port->uart->deinit) {
    port->uart->deinit(_voice_isp_ctx);
    _voice_isp_ctx = nullptr;
  }

  _voice_saved_mode = -1;
  if (!serialGetPower(SP_AUX1))
    serialSetPower(SP_AUX1, true);
  serialSetMode(SP_AUX1, UART_MODE_DEBUG);
  serialInit(SP_AUX1, UART_MODE_DEBUG);
  serialSetBaudrate(SP_AUX1, VOICE_CTRL_BAUD);

#if defined(VOICE_CONTROL_SENSOR)
  CI1302_voiceSwitchInitBootGuard();
#endif
}

#if (defined(RADIO_V14) || defined(RADIO_V14LCD)) && defined(AUX_SERIAL)

bool voiceModulePowerUiEnabled()
{
  const uint32_t portMask = 0xFFu << (SP_AUX1 * SERIAL_CONF_BITS_PER_PORT);
  const uint32_t cfg = g_eeGeneral.serialPort & portMask;
  // Unconfigured EEPROM (legacy) defaults to voice module ON.
  if (cfg == 0) return true;
  return serialGetPower(SP_AUX1);
}

void voiceModuleSetPower(bool on)
{
  if (on) {
    serialSetPower(SP_AUX1, true);
    voiceControlInit();
  } else {
    serialSetMode(SP_AUX1, UART_MODE_DEBUG);
    serialSetPower(SP_AUX1, false);
    VoiceRunStatus = false;
    voiceControlPowerOff();
  }
  storageDirty(EE_GENERAL);
}

#endif

extern "C" void voiceSerialPutstr(uint8_t* data, uint32_t len)
{
  if (!data || !len) return;

  const etx_serial_port_t* port = auxSerialGetPort(SP_AUX1);
  if (_voice_isp_ctx && port && port->uart && port->uart->sendBuffer) {
    constexpr uint32_t kChunk = 256;
    while (len) {
      uint32_t n = len > kChunk ? kChunk : len;
      port->uart->sendBuffer(_voice_isp_ctx, data, n);
      if (port->uart->waitForTxCompleted) {
        port->uart->waitForTxCompleted(_voice_isp_ctx);
      }
      data += n;
      len -= n;
    }
    return;
  }

  while (len--) {
    voiceSerialPutc((char)*data++);
  }
}

#else  // !AUX_SERIAL

extern "C" void voiceControlPowerOn(void) {}
extern "C" void voiceControlPowerOff(void) {}
extern "C" void voiceControlUartInit(uint32_t baud) { (void)baud; }
extern "C" void voiceControlUartReopen(uint32_t baud) { (void)baud; }
extern "C" void voiceControlWaitTxCompleted(void) {}
extern "C" void voiceControlSetBaudrate(uint32_t baud) { (void)baud; }
void voiceControlInit(void)
{
#if defined(VOICE_CONTROL_SENSOR)
  CI1302_voiceSwitchInitBootGuard();
#endif
}
extern "C" void voiceSerialPutstr(uint8_t* data, uint32_t len)
{
  (void)data;
  (void)len;
}

#endif  // AUX_SERIAL
