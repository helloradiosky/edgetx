/*
 * Copyright (C) EdgeTX
 *
 * License GPLv2: http://www.gnu.org/licenses/gpl-2.0.html
 *
 * CI1302 / CI-03T voice module: UART helpers (SD ISP) + runtime voice control.
 */

#ifndef _CI1302_H_
#define _CI1302_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void voiceSerialPutc(char c);
int voiceGetByte(uint8_t* byte);
void voiceSerialPutstr(uint8_t* byte, uint32_t len);
int dbgGetByte(uint8_t* byte);

void voiceControlPowerOn(void);
void voiceControlPowerOff(void);
void voiceControlUartInit(uint32_t baud);
void voiceControlUartReopen(uint32_t baud);
void voiceControlWaitTxCompleted(void);
void voiceControlSetBaudrate(uint32_t baud);
void voiceControlInit(void);

#ifdef __cplusplus
}
#endif

#if defined(RADIO_V14) || defined(RADIO_V14LCD)
bool voiceModulePowerUiEnabled();
void voiceModuleSetPower(bool on);
#endif

#ifdef __cplusplus

extern bool VoiceGearStatus;
extern uint8_t VoiceFlapStatus;
extern bool VoicePowerStatus;
extern bool VoiceRunStatus;
extern bool MotionControlStatus;

void processUpdataInput(void);
void processVoiceInput(void);

#endif

#ifndef VOICE_CTRL_BAUD
#define VOICE_CTRL_BAUD 115200
#endif

#endif
