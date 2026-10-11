/*
 * Copyright (C) EdgeTX
 *
 * License GPLv2: http://www.gnu.org/licenses/gpl-2.0.html
 *
 * ChipIntelli CI1302 / CI-03T UART ISP (same framing as code_program.exe).
 * Frame: A5 0F | len(u16 LE) | type | cmd | flags | payload | crc16 | FF
 *
 * Boot/sync at 115200, then probe ISP baud ladder 460800 -> 230400 -> 115200.
 * After a failed baud attempt, power-cycle the module before the next try.
 */

#include "ci1302_firmware_update.h"

#include <ctype.h>
#include <string.h>

#include "edgetx.h"
#include "hal/watchdog_driver.h"
#include "lib_file.h"
#include "os/sleep.h"
#include "pulses/pulses.h"
#include "sdcard.h"
#include "strhelpers.h"
#include "tasks/mixer_task.h"
#include "timers_driver.h"

#if defined(VOICE_SENSOR) && defined(AUX_SERIAL)
#include "CI1302.h"
#endif

// Implemented in io/ci1302_zlib.cpp
extern "C" int stbi_zlib_decode_buffer(char* obuffer, int olen, const char* ibuffer,
                                       int ilen);

#include "ci1302_upgrade_agent.inc"

#if defined(SIMU)

bool isCi1302VoiceFirmware(const char* name, const char* fullpath)
{
  (void)fullpath;
  if (!name) return false;
  const char* ext = strrchr(name, '.');
  if (!ext || strcasecmp(ext, ".bin") != 0) return false;
  char lower[80];
  size_t n = strlen(name);
  if (n >= sizeof(lower)) n = sizeof(lower) - 1;
  for (size_t i = 0; i < n; i++) lower[i] = (char)tolower((unsigned char)name[i]);
  lower[n] = 0;
  return strstr(lower, "ci_03") || strstr(lower, "ci-03") ||
         strstr(lower, "ci1302") || strstr(lower, "jx_ci");
}

bool Ci1302FirmwareUpdate::flashFirmware(const char* filename,
                                         ProgressHandler progressHandler)
{
  for (int i = 0; i <= 100; i += 10) {
    progressHandler(getBasename(filename), STR_WRITING, i, 100);
    sleep_ms(20);
  }
  return true;
}

#elif !(defined(VOICE_SENSOR) && defined(AUX_SERIAL))

bool isCi1302VoiceFirmware(const char* name, const char* fullpath)
{
  (void)name;
  (void)fullpath;
  return false;
}

bool Ci1302FirmwareUpdate::flashFirmware(const char* filename,
                                         ProgressHandler progressHandler)
{
  (void)filename;
  (void)progressHandler;
  POPUP_WARNING("CI1302 not supported");
  return false;
}

#else

namespace
{

constexpr uint32_t CI_BOOT_BAUD = 115200;
constexpr uint16_t CI_MAX_CHUNK = 0x1000;  // PC tool WRITE chunks
constexpr uint16_t CI_PART_TABLE_SIZE = 0x116;
constexpr uint32_t CI_MAX_FW_SIZE = 2 * 1024 * 1024;

/** Probe list: try high speed first, fall back to boot baud. */
constexpr uint32_t kIspBaudLadder[] = {
    460800, 230400, CI_BOOT_BAUD,
};

constexpr uint8_t CI_TYPE_REQ = 0xA0;
constexpr uint8_t CI_TYPE_WAIT = 0xA1;
constexpr uint8_t CI_TYPE_ACK = 0xA2;

constexpr uint8_t CI_CMD_READ_PART = 0x04;
constexpr uint8_t CI_CMD_READY5 = 0x05;
constexpr uint8_t CI_CMD_ERASE = 0x06;
constexpr uint8_t CI_CMD_ERASE_BLK = 0x07;
constexpr uint8_t CI_CMD_WRITE = 0x08;
constexpr uint8_t CI_CMD_WRITE_DONE = 0x09;
constexpr uint8_t CI_CMD_VERIFY = 0x0A;
constexpr uint8_t CI_CMD_BAUD = 0x0B;
constexpr uint8_t CI_CMD_0C = 0x0C;
constexpr uint8_t CI_CMD_0E = 0x0E;
constexpr uint8_t CI_CMD_MSG = 0x11;
constexpr uint8_t CI_CMD_SET_VER = 0x12;
constexpr uint8_t CI_CMD_WAIT_RST = 0x83;
constexpr uint8_t CI_CMD_84 = 0x84;
constexpr uint8_t CI_CMD_READY85 = 0x85;
constexpr uint8_t CI_CMD_VERIFY_SRAM = 0x8A;
constexpr uint8_t CI_CMD_RESET = 0xA1;

/** Official tool with "Erase NV": wipe 32KB at end of 2M flash. */
constexpr uint32_t CI_NV_ADDR = 0x001F8000;
constexpr uint32_t CI_NV_SIZE = 0x8000;

struct CiPacket {
  uint16_t len;
  uint8_t type;
  uint8_t cmd;
  uint8_t flags;
  uint8_t payload[CI_MAX_CHUNK + 16];
};

// ISP buffers are large (~40KB with agent). Do NOT park them in permanent
// .ram -- that starves the Lua heap (ELRS script: "not enough memory").
// Allocate only for the duration of flashFirmware().
struct CiIspWorkspace {
  CiPacket pendingPkt;
  CiPacket rxPkt;
  uint8_t lastAck[4 + CI_MAX_CHUNK];
  uint8_t writeBody[4 + CI_MAX_CHUNK];
  uint8_t crcBuf[0x1000];
  uint8_t agentRam[CI1302_UPGRADE_AGENT_SIZE];
};

CiIspWorkspace* s_ws = nullptr;

ProgressHandler s_progress = nullptr;
const char* s_label = nullptr;
FIL* s_file = nullptr;
uint32_t s_fileSize = 0;
char s_errDetail[64] = {0};
char s_stageMsg[48] = {0};

bool s_havePendingPkt = false;

void clearPendingPkt() { s_havePendingPkt = false; }

bool recvPacket(CiPacket* out, uint32_t timeoutMs);  // defined below

bool recvPacketOrPending(CiPacket* out, uint32_t timeoutMs)
{
  if (s_havePendingPkt) {
    *out = s_ws->pendingPkt;
    s_havePendingPkt = false;
    return true;
  }
  return recvPacket(out, timeoutMs);
}

/** Current ISP host UART baud (boot or probed high-speed). */
uint32_t s_ispBaud = CI_BOOT_BAUD;

/** Full reopen ? required so GPIO speed is recomputed for high baud. */
void reopenIspUart(uint32_t baud)
{
  s_ispBaud = baud;
  voiceControlUartReopen(baud);
}

void ensureIspBootBaud()
{
  reopenIspUart(CI_BOOT_BAUD);
}

const char* setStageError(const char* fmt, uint32_t arg)
{
  snprintf(s_errDetail, sizeof(s_errDetail), fmt, arg);
  return s_errDetail;
}

void holdWatchdog()
{
  // units are 10 ms
  watchdogSuspend(6000);
}

void progress(const char* status, int cur, int tot)
{
  holdWatchdog();
  if (s_progress) s_progress(s_label ? s_label : "", status, cur, tot);
}

uint16_t crc16Update(uint16_t crc, const uint8_t* data, uint32_t len)
{
  for (uint32_t i = 0; i < len; i++) {
    uint8_t idx = (uint8_t)(((crc >> 8) ^ data[i]) & 0xFF);
    crc = (uint16_t)((crc << 8) ^ CI1302_CRC16_TAB[idx]);
  }
  return crc;
}

void putLe32(uint8_t* p, uint32_t v)
{
  p[0] = (uint8_t)v;
  p[1] = (uint8_t)(v >> 8);
  p[2] = (uint8_t)(v >> 16);
  p[3] = (uint8_t)(v >> 24);
}

uint32_t getLe32(const uint8_t* p)
{
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) |
         ((uint32_t)p[3] << 24);
}

void uartFlushRx()
{
  uint8_t b;
  while (voiceGetByte(&b)) {
  }
}

int uartReadByte(uint32_t timeoutMs)
{
  const tmr10ms_t start = get_tmr10ms();
  uint8_t b;
  while (true) {
    if (voiceGetByte(&b)) return b;
    if ((uint32_t)(get_tmr10ms() - start) * 10 >= timeoutMs) return -1;
    WDG_RESET();
    sleep_ms(1);
  }
}

bool sendPacket(uint8_t cmd, uint8_t type, uint8_t flags, const uint8_t* payload,
                uint16_t len)
{
  if (len > CI_MAX_CHUNK + 4) return false;

  uint8_t hdr[7];
  hdr[0] = 0xA5;
  hdr[1] = 0x0F;
  hdr[2] = (uint8_t)(len & 0xFF);
  hdr[3] = (uint8_t)((len >> 8) & 0xFF);
  hdr[4] = type;
  hdr[5] = cmd;
  hdr[6] = flags;

  uint16_t crc = crc16Update(0, &hdr[4], 3);
  if (len && payload) crc = crc16Update(crc, payload, len);

  uint8_t tail[3] = {(uint8_t)(crc & 0xFF), (uint8_t)((crc >> 8) & 0xFF), 0xFF};

  voiceSerialPutstr(hdr, 7);
  if (len && payload) voiceSerialPutstr((uint8_t*)payload, len);
  voiceSerialPutstr(tail, 3);
  voiceControlWaitTxCompleted();
  return true;
}

bool recvPacket(CiPacket* out, uint32_t timeoutMs)
{
  if (!out) return false;
  out->len = 0;

  int b0 = -1;
  const tmr10ms_t start = get_tmr10ms();
  while ((uint32_t)(get_tmr10ms() - start) * 10 < timeoutMs) {
    WDG_RESET();
    int b = uartReadByte(40);
    if (b < 0) continue;
    if (b0 < 0) {
      if (b == 0xA5) b0 = b;
      continue;
    }
    if (b == 0x0F) break;
    b0 = (b == 0xA5) ? b : -1;
  }
  if (b0 != 0xA5) return false;

  int l0 = uartReadByte(300);
  int l1 = uartReadByte(300);
  int type = uartReadByte(300);
  int cmd = uartReadByte(300);
  int flags = uartReadByte(300);
  if (l0 < 0 || l1 < 0 || type < 0 || cmd < 0 || flags < 0) return false;

  uint16_t len = (uint16_t)(l0 | (l1 << 8));
  if (len > sizeof(out->payload)) return false;

  for (uint16_t i = 0; i < len; i++) {
    int b = uartReadByte(800);
    if (b < 0) return false;
    out->payload[i] = (uint8_t)b;
  }

  int c0 = uartReadByte(300);
  int c1 = uartReadByte(300);
  int ff = uartReadByte(300);
  if (c0 < 0 || c1 < 0 || ff != 0xFF) return false;

  uint8_t th[3] = {(uint8_t)type, (uint8_t)cmd, (uint8_t)flags};
  uint16_t crc = crc16Update(0, th, 3);
  if (len) crc = crc16Update(crc, out->payload, len);
  if (crc != (uint16_t)(c0 | (c1 << 8))) return false;

  out->len = len;
  out->type = (uint8_t)type;
  out->cmd = (uint8_t)cmd;
  out->flags = (uint8_t)flags;
  return true;
}

bool waitCmd(uint8_t expectCmd, uint8_t expectType /*0xFF=any*/, uint32_t timeoutMs,
             CiPacket* resp = nullptr)
{
  const tmr10ms_t start = get_tmr10ms();
  while ((uint32_t)(get_tmr10ms() - start) * 10 < timeoutMs) {
    WDG_RESET();
    if (!recvPacket(&s_ws->rxPkt, 200)) continue;
    if (s_ws->rxPkt.cmd == CI_CMD_MSG) continue;
    if (s_ws->rxPkt.cmd != expectCmd) continue;
    if (expectType != 0xFF && s_ws->rxPkt.type != expectType) continue;
    if (resp) *resp = s_ws->rxPkt;
    return true;
  }
  return false;
}

bool cmdRoundtrip(uint8_t cmd, uint8_t type, uint8_t flags, const uint8_t* payload,
                  uint16_t len, uint8_t expectCmd, uint8_t expectType,
                  uint32_t timeoutMs, CiPacket* resp = nullptr)
{
  if (!sendPacket(cmd, type, flags, payload, len)) return false;
  return waitCmd(expectCmd, expectType, timeoutMs, resp);
}

bool pollReady(uint8_t readyCmd)
{
  return cmdRoundtrip(readyCmd, CI_TYPE_REQ, 1, nullptr, 0, readyCmd, 0xFF, 2000);
}

/** PC wait_reset: cmd 0x83 type A1, expect cmd 0x83 type A2 (100 ms recv window). */
bool pollWaitResetOnce()
{
  if (!sendPacket(CI_CMD_WAIT_RST, CI_TYPE_WAIT, 0, nullptr, 0)) return false;
  return waitCmd(CI_CMD_WAIT_RST, CI_TYPE_ACK, 100);
}

bool waitReady(uint8_t readyCmd, uint32_t timeoutMs, const char* stage)
{
  const tmr10ms_t start = get_tmr10ms();
  int attempt = 0;
  while ((uint32_t)(get_tmr10ms() - start) * 10 < timeoutMs) {
    holdWatchdog();
    if (stage) {
      uint32_t elapsed = (uint32_t)(get_tmr10ms() - start) * 10;
      int pct = (int)((uint64_t)elapsed * 99 / timeoutMs);
      if (pct > 99) pct = 99;
      progress(stage, pct, 100);
    }
    if (pollReady(readyCmd)) return true;
    attempt++;
    sleep_ms(50);
  }
  (void)attempt;
  return false;
}

/**
 * Match PACK_UPDATE_TOOL: UART already open at 115200, power-cycle module,
 * then hammer cmd 0x83 until the ISP ROM answers (type A2).
 */
bool enterBootloaderMode()
{
  progress("Power off", 0, 100);
  voiceControlPowerOff();
  sleep_ms(2000);
  ensureIspBootBaud();
  uartFlushRx();
  voiceControlPowerOn();
  sleep_ms(30);

  const tmr10ms_t start = get_tmr10ms();
  while ((uint32_t)(get_tmr10ms() - start) * 10 < 20000) {
    holdWatchdog();
    uint32_t elapsed = (uint32_t)(get_tmr10ms() - start) * 10;
    progress("BL sync", (int)(elapsed * 99 / 20000), 100);

    if (pollWaitResetOnce()) return true;
    if (pollReady(CI_CMD_READY85)) return true;

    sleep_ms(30);
  }
  return false;
}

void progressWait(const char* msg, tmr10ms_t start, uint32_t timeoutMs)
{
  uint32_t elapsed = (uint32_t)(get_tmr10ms() - start) * 10;
  int pct = timeoutMs ? (int)((uint64_t)elapsed * 99 / timeoutMs) : 0;
  if (pct > 99) pct = 99;
  progress(msg, pct, 100);
}

void progressEraseWait(uint32_t addr, tmr10ms_t start, uint32_t timeoutMs)
{
  snprintf(s_stageMsg, sizeof(s_stageMsg), "Erase 0x%X", (unsigned)addr);
  progressWait(s_stageMsg, start, timeoutMs);
}

bool handleMsgProgress(const CiPacket& pkt, tmr10ms_t start, uint32_t timeoutMs)
{
  if (pkt.cmd != CI_CMD_MSG || pkt.len == 0) return false;

  char msg[64];
  size_t n = pkt.len;
  if (n >= sizeof(msg)) n = sizeof(msg) - 1;
  memcpy(msg, pkt.payload, n);
  msg[n] = 0;

  uint32_t elapsed = (uint32_t)(get_tmr10ms() - start) * 10;
  int pct = timeoutMs ? (int)((uint64_t)elapsed * 99 / timeoutMs) : 0;
  if (pct > 99) pct = 99;
  progress(msg, pct, 100);
  (void)timeoutMs;
  return true;
}

uint32_t eraseTimeoutMs(uint32_t eraseSize, bool isFlash)
{
  if (!isFlash) {
    // SRAM agent erase is only a session start ??should ACK in seconds.
    return 20000;
  }
  // Large voice image (~1.5 MB) can take several minutes to erase.
  uint32_t ms = 90000 + eraseSize / 256;
  if (ms < 180000) ms = 180000;
  if (ms > 600000) ms = 600000;
  return ms;
}

bool waitEraseAck(uint8_t expectCmd, uint32_t addr, uint32_t eraseSize, uint16_t crcPad,
                  bool withBlock, uint32_t block, bool isFlash)
{
  uint8_t p[12];
  uint16_t plen;
  if (withBlock) {
    putLe32(p + 0, addr);
    putLe32(p + 4, eraseSize);
    putLe32(p + 8, block);
    plen = 12;
  } else {
    putLe32(p + 0, addr);
    putLe32(p + 4, eraseSize);
    p[8] = (uint8_t)(crcPad & 0xFF);
    p[9] = (uint8_t)((crcPad >> 8) & 0xFF);
    plen = 10;
  }

  const uint32_t timeoutMs = eraseTimeoutMs(eraseSize, isFlash);
  uartFlushRx();
  if (!sendPacket(withBlock ? CI_CMD_ERASE_BLK : CI_CMD_ERASE, CI_TYPE_REQ, 0, p, plen))
    return false;

  const tmr10ms_t start = get_tmr10ms();
  tmr10ms_t lastUi = start;
  // PC tool: only a few erase re-sends on silence, then keep listening.
  int resendLeft = 3;

  while ((uint32_t)(get_tmr10ms() - start) * 10 < timeoutMs) {
    holdWatchdog();
    if ((uint32_t)(get_tmr10ms() - lastUi) >= 20) {
      progressEraseWait(addr, start, timeoutMs);
      lastUi = get_tmr10ms();
    }

    if (!recvPacket(&s_ws->rxPkt, 1000)) {
      if (resendLeft-- > 0)
        sendPacket(withBlock ? CI_CMD_ERASE_BLK : CI_CMD_ERASE, CI_TYPE_REQ, 0, p, plen);
      continue;
    }

    if (handleMsgProgress(s_ws->rxPkt, start, timeoutMs)) continue;
    if (s_ws->rxPkt.cmd == expectCmd) {
      progress(isFlash ? STR_FLASH_ERASE : STR_WRITING, 100, 100);
      return true;
    }
    // Device may skip erase ACK and go straight to WRITE ??keep that packet.
    if (s_ws->rxPkt.cmd == CI_CMD_WRITE || s_ws->rxPkt.cmd == CI_CMD_WRITE_DONE) {
      s_ws->pendingPkt = s_ws->rxPkt;
      s_havePendingPkt = true;
      progress(isFlash ? STR_FLASH_ERASE : STR_WRITING, 100, 100);
      return true;
    }
  }
  return false;
}

bool erasePartition(uint32_t addr, uint32_t size, uint16_t crcPad, bool isFlash)
{
  return waitEraseAck(CI_CMD_ERASE, addr, size, crcPad, false, 0, isFlash);
}

bool eraseBlock(uint32_t addr, uint32_t size, uint32_t block)
{
  return waitEraseAck(CI_CMD_ERASE_BLK, addr, size, 0, true, block, true);
}

bool writePartitionLoop(bool fromMem, const uint8_t* mem, uint32_t size,
                        uint16_t* outCrc, uint32_t timeoutMs)
{
  uint16_t crc = outCrc ? *outCrc : 0;
  const tmr10ms_t start = get_tmr10ms();
  tmr10ms_t lastProgress = start;
  tmr10ms_t lastWrite = start;
  int idleRetries = 0;
  int resendLeft = 3;

  // Never allocate 4KB WRITE chunks on the menus stack.
  uint16_t lastAckLen = 0;
  bool haveLastAck = false;

  while ((uint32_t)(get_tmr10ms() - start) * 10 < timeoutMs) {
    holdWatchdog();
    if (!recvPacketOrPending(&s_ws->rxPkt, 5000)) {
      // PC tool: on timeout re-send the last WRITE ack (up to 3 times).
      if (haveLastAck && resendLeft-- > 0) {
        sendPacket(CI_CMD_WRITE, CI_TYPE_ACK, 0, s_ws->lastAck, lastAckLen);
        continue;
      }
      if (++idleRetries > 3) return false;
      if (lastWrite > start &&
          (uint32_t)(get_tmr10ms() - lastWrite) * 10 > 30000)
        return false;
      progressWait(STR_WRITING, start, timeoutMs);
      continue;
    }
    idleRetries = 0;
    resendLeft = 3;

    if (s_ws->rxPkt.cmd == CI_CMD_MSG) {
      if ((uint32_t)(get_tmr10ms() - lastProgress) >= 20) {
        progressWait(STR_WRITING, start, timeoutMs);
        lastProgress = get_tmr10ms();
      }
      continue;
    }
    if (s_ws->rxPkt.cmd == CI_CMD_WRITE_DONE) {
      if (outCrc) *outCrc = crc;
      return true;
    }
    if (s_ws->rxPkt.cmd != CI_CMD_WRITE || s_ws->rxPkt.len < 8) continue;

    lastWrite = get_tmr10ms();

    const uint32_t offset = getLe32(s_ws->rxPkt.payload);
    uint32_t remain = getLe32(s_ws->rxPkt.payload + 4);
    if (remain > CI_MAX_CHUNK) remain = CI_MAX_CHUNK;

    putLe32(s_ws->writeBody, offset);
    memset(s_ws->writeBody + 4, 0xFF, remain);

    if (offset < size) {
      uint32_t got = size - offset;
      if (got > remain) got = remain;
      if (fromMem) {
        // mem==nullptr: keep the 0xFF fill (NV erase pad, no 32KB buffer).
        if (mem) memcpy(s_ws->writeBody + 4, mem + offset, got);
      } else {
        UINT br = 0;
        if (f_lseek(s_file, offset) != FR_OK) return false;
        if (f_read(s_file, s_ws->writeBody + 4, got, &br) != FR_OK) return false;
        if (br < got) memset(s_ws->writeBody + 4 + br, 0xFF, got - br);
      }
    }

    if (offset == 0) crc = 0;
    crc = crc16Update(crc, s_ws->writeBody + 4, remain);

    const uint16_t bodyLen = (uint16_t)(remain + 4);
    if (!sendPacket(CI_CMD_WRITE, CI_TYPE_ACK, 0, s_ws->writeBody, bodyLen)) return false;

    memcpy(s_ws->lastAck, s_ws->writeBody, bodyLen);
    lastAckLen = bodyLen;
    haveLastAck = true;

    if (size) {
      progress(STR_WRITING, (int)((offset + remain) * 100ull / size), 100);
      lastProgress = get_tmr10ms();
    }
  }
  return false;
}

bool verifyPartition(bool isFlash, uint32_t addr, uint32_t eraseSize, uint16_t crc)
{
  if (!isFlash) {
    if (!sendPacket(CI_CMD_VERIFY_SRAM, CI_TYPE_REQ, 0, nullptr, 0)) return false;
    const tmr10ms_t start = get_tmr10ms();
    while ((uint32_t)(get_tmr10ms() - start) * 10 < 15000) {
      if (!recvPacket(&s_ws->rxPkt, 2000)) continue;
      if (s_ws->rxPkt.cmd == CI_CMD_MSG) continue;
      if (s_ws->rxPkt.cmd == CI_CMD_VERIFY_SRAM) {
        if (s_ws->rxPkt.len == 0 || s_ws->rxPkt.payload[0] == 1) return true;
        return false;
      }
    }
    return false;
  }
  uint8_t p[10];
  putLe32(p + 0, addr);
  putLe32(p + 4, eraseSize);
  p[8] = (uint8_t)(crc & 0xFF);
  p[9] = (uint8_t)((crc >> 8) & 0xFF);
  return cmdRoundtrip(CI_CMD_VERIFY, CI_TYPE_REQ, 0, p, 10, CI_CMD_VERIFY, 0xFF,
                      60000);
}

bool updatePartition(bool isFlash, uint32_t addr, const uint8_t* mem, uint32_t size,
                    uint32_t writeTimeoutMs, bool fillFF = false)
{
  s_errDetail[0] = 0;
  const uint32_t eraseSize = (size + 0xFFF) & ~0xFFFu;
  uint16_t crc = 0;

  if (!isFlash) {
    crc = crc16Update(0, mem, size);
    for (uint32_t i = size; i < eraseSize; i++) {
      const uint8_t ff = 0xFF;
      crc = crc16Update(crc, &ff, 1);
    }
  }

  progress(isFlash ? STR_FLASH_ERASE : STR_WRITING, 0, 100);
  clearPendingPkt();
  // PC tool still sends erase(0x06) for SRAM (session start + CRC), even though
  // the log says "No need to erase". If the erase ACK is missing but WRITE
  // already started, waitEraseAck treats that as success.
  if (!erasePartition(addr, eraseSize, crc, isFlash)) {
    if (isFlash) {
      setStageError("Erase TO@0x%X", addr);
      return false;
    }
    // SRAM: continue -- device may already be in write mode without ACK.
    progress("SRAM write", 5, 100);
  }
  if (isFlash && !eraseBlock(addr, eraseSize, 0x1000)) {
    setStageError("Erase blk@0x%X", addr);
    return false;
  }

  // Do NOT uartFlushRx() here -- device often sends WRITE REQ immediately
  // after erase ACK (same as PC capture cmd#70); flushing drops it and
  // causes Write timeout@addr.
  uint16_t wcrc = isFlash ? 0 : crc;
  // fromMem: real buffer, or fillFF (mem ignored, write 0xFF chunks).
  if (!writePartitionLoop(fillFF || mem != nullptr, fillFF ? nullptr : mem, size,
                          &wcrc, writeTimeoutMs)) {
    setStageError("Write TO@0x%X", addr);
    return false;
  }
  if (!verifyPartition(isFlash, addr, eraseSize, wcrc)) {
    setStageError("Verify@0x%X", addr);
    return false;
  }
  return true;
}

bool readPartitionTable(uint8_t* table)
{
  if (!sendPacket(CI_CMD_READ_PART, CI_TYPE_REQ, 1, nullptr, 0)) return false;

  const tmr10ms_t start = get_tmr10ms();
  while ((uint32_t)(get_tmr10ms() - start) * 10 < 15000) {
    WDG_RESET();
    if (!recvPacket(&s_ws->rxPkt, 1000)) continue;
    if (s_ws->rxPkt.cmd == CI_CMD_MSG) continue;
    if ((s_ws->rxPkt.cmd == CI_CMD_READ_PART || s_ws->rxPkt.type == CI_TYPE_ACK) &&
        s_ws->rxPkt.len >= 8 + CI_PART_TABLE_SIZE) {
      memcpy(table, s_ws->rxPkt.payload + 8, CI_PART_TABLE_SIZE);
      return true;
    }
  }
  return false;
}

/**
 * Brief power-cycle + ISP ROM re-sync at 115200.
 * Needed between baud probes: a failed high-speed switch leaves the device
 * on an unknown rate; must power off/on before the next baud attempt.
 */
bool ispPowerCycleAndSync()
{
  progress("Baud PWR", 0, 100);
  voiceControlPowerOff();
  sleep_ms(1000);
  ensureIspBootBaud();
  uartFlushRx();
  voiceControlPowerOn();
  sleep_ms(50);

  const tmr10ms_t start = get_tmr10ms();
  while ((uint32_t)(get_tmr10ms() - start) * 10 < 12000) {
    holdWatchdog();
    uint32_t elapsed = (uint32_t)(get_tmr10ms() - start) * 10;
    progress("Baud sync", (int)(elapsed * 99 / 12000), 100);
    if (pollWaitResetOnce()) return true;
    if (pollReady(CI_CMD_READY85)) return true;
    sleep_ms(30);
  }
  return false;
}

bool sendCmd0c()
{
  uint8_t c0c[4] = {0x10, 0x00, 0x00, 0x00};
  return cmdRoundtrip(CI_CMD_0C, CI_TYPE_REQ, 0, c0c, 4, CI_CMD_0C, CI_TYPE_ACK,
                      3000);
}

/** Ask device to switch baud (host still on s_ispBaud), then reopen host. */
bool tryIspBaud(uint32_t baud)
{
  uint8_t baudLe[4];
  putLe32(baudLe, baud);
  if (!cmdRoundtrip(CI_CMD_BAUD, CI_TYPE_REQ, 0, baudLe, 4, CI_CMD_BAUD,
                    CI_TYPE_ACK, 3000))
    return false;

  if (baud != s_ispBaud) {
    sleep_ms(40);
    uartFlushRx();
    reopenIspUart(baud);
    sleep_ms(40);
    uartFlushRx();
  }

  return sendCmd0c();
}

/**
 * Probe baud ladder 460800 -> 230400 -> 115200.
 * After each failed attempt, power-cycle so the next try starts clean @115200.
 */
bool ispSelectBaud()
{
  if (s_ispBaud != CI_BOOT_BAUD) {
    reopenIspUart(CI_BOOT_BAUD);
    sleep_ms(20);
    uartFlushRx();
  }

  for (size_t i = 0; i < sizeof(kIspBaudLadder) / sizeof(kIspBaudLadder[0]); i++) {
    const uint32_t baud = kIspBaudLadder[i];

    if (i > 0) {
      if (!ispPowerCycleAndSync()) continue;
    }

    char msg[28];
    snprintf(msg, sizeof(msg), "Baud %lu", (unsigned long)baud);
    progress(msg, 0, 100);

    // Fresh ROM session after power-on needs CMD84 again.
    (void)cmdRoundtrip(CI_CMD_84, CI_TYPE_REQ, 0, nullptr, 0, CI_CMD_84,
                       CI_TYPE_ACK, 2000);

    if (tryIspBaud(baud)) {
      snprintf(s_stageMsg, sizeof(s_stageMsg), "ISP %lu", (unsigned long)baud);
      progress(s_stageMsg, 100, 100);
      return true;
    }
  }
  return false;
}

/**
 * After agent is in SRAM: only re-confirm the selected baud ? never power-cycle
 * (that would wipe the agent).
 */
bool ispEnsureFlashBaud()
{
  char msg[28];
  snprintf(msg, sizeof(msg), "Baud %lu", (unsigned long)s_ispBaud);
  progress(msg, 0, 100);

  uint8_t baudLe[4];
  putLe32(baudLe, s_ispBaud);
  if (cmdRoundtrip(CI_CMD_BAUD, CI_TYPE_REQ, 0, baudLe, 4, CI_CMD_BAUD,
                   CI_TYPE_ACK, 3000) &&
      sendCmd0c())
    return true;

  reopenIspUart(s_ispBaud);
  sleep_ms(50);
  uartFlushRx();
  putLe32(baudLe, s_ispBaud);
  return cmdRoundtrip(CI_CMD_BAUD, CI_TYPE_REQ, 0, baudLe, 4, CI_CMD_BAUD,
                      CI_TYPE_ACK, 3000) &&
         sendCmd0c();
}

/** Official tool: CMD84 ("Mrom") -> BAUD probe -> CMD0C before agent download. */
bool ispRomPreamble(const char* stage)
{
  if (stage) progress(stage, 0, 100);
  // CMD84 is issued inside ispSelectBaud() (again after each power-cycle).
  return ispSelectBaud();
}

/** After updater is running: BAUD -> CMD0C (no power-cycle). */
bool ispUpdaterPreamble(const char* stage)
{
  if (stage) progress(stage, 0, 100);
  return ispEnsureFlashBaud();
}

const char* transportAgent()
{
  ensureIspBootBaud();

  if (!waitReady(CI_CMD_READY85, 8000, "ISP ready"))
    return "ISP ready timeout";

  if (!ispRomPreamble("ISP prep"))
    return "ISP prep failed";

  // Agent is zlib-compressed in flash; expand into heap workspace
  // (allocated for flash only), not permanent .ram / CCM.
  progress("Inflate", 10, 100);
  const int got = stbi_zlib_decode_buffer(
      (char*)s_ws->agentRam, (int)CI1302_UPGRADE_AGENT_SIZE,
      (const char*)CI1302_UPGRADE_AGENT_Z, (int)CI1302_UPGRADE_AGENT_Z_SIZE);
  if (got != (int)CI1302_UPGRADE_AGENT_SIZE) return "Inflate failed";

  progress("Agent", 20, 100);
  // Official tool: erase/write agent at 0x1FF7A000, size 0x5000.
  if (!updatePartition(false, CI1302_UPGRADE_AGENT_ADDR, s_ws->agentRam,
                       CI1302_UPGRADE_AGENT_SIZE, 120000))
    return s_errDetail[0] ? s_errDetail : "Agent write failed";
  return nullptr;
}

const char* doFlash(FIL* file, const char* label, ProgressHandler progressFn)
{
  s_progress = progressFn;
  s_label = label;
  s_file = file;
  s_fileSize = f_size(file);
  if (s_fileSize < 64 * 1024 || s_fileSize > CI_MAX_FW_SIZE)
    return STR_DEVICE_FILE_ERROR;

  progress("CRC", 0, 100);

  // Precompute image CRC (same as PC tool before ISP)
  uint16_t fileCrc = 0;
  {
    uint32_t off = 0;
    while (off < s_fileSize) {
      holdWatchdog();
      UINT br = 0;
      uint32_t n = s_fileSize - off;
      if (n > sizeof(s_ws->crcBuf)) n = sizeof(s_ws->crcBuf);
      if (f_lseek(file, off) != FR_OK) return STR_DEVICE_FILE_ERROR;
      if (f_read(file, s_ws->crcBuf, n, &br) != FR_OK || br != n)
        return STR_DEVICE_FILE_ERROR;
      fileCrc = crc16Update(fileCrc, s_ws->crcBuf, br);
      off += br;
      progress("CRC", (int)(off * 100ull / s_fileSize), 100);
    }
  }
  (void)fileCrc;

  VoiceRunStatus = false;
  ensureIspBootBaud();

  if (!enterBootloaderMode()) {
    // One more power cycle -- same as PC tool user retrying reset.
    if (!enterBootloaderMode()) return "BL entry failed";
  }

  {
    const char* err = transportAgent();
    if (err) return err;
  }

  // After agent: try selected flash baud first, then boot baud.
  // Do NOT power-cycle here ? that would wipe the SRAM agent.
  progress("Sync", 0, 100);
  const uint32_t flashBaud = s_ispBaud;
  uartFlushRx();
  reopenIspUart(flashBaud);
  sleep_ms(100);
  if (!waitReady(CI_CMD_READY5, 5000, "Sync")) {
    reopenIspUart(CI_BOOT_BAUD);
    sleep_ms(50);
    if (!waitReady(CI_CMD_READY5, 20000, "Sync"))
      return "Updater not ready";
  }

  {
    uint8_t ver = 2;
    if (!cmdRoundtrip(CI_CMD_SET_VER, CI_TYPE_REQ, 1, &ver, 1, CI_CMD_SET_VER,
                      CI_TYPE_ACK, 2000))
      return "Set ver failed";
  }

  // Official tool reads partition for UI only -- flash is still a full image @0.
  {
    uint8_t part[CI_PART_TABLE_SIZE];
    progress("Part table", 25, 100);
    (void)readPartitionTable(part);
  }

  // PC tool: BAUD+CMD0C, then erase/write file at flash address 0 (not userAddr).
  if (!ispUpdaterPreamble("Flash prep"))
    return "Flash prep failed";

  progress(STR_WRITING, 30, 100);
  if (!updatePartition(true, 0x00000000, nullptr, s_fileSize, 600000))
    return s_errDetail[0] ? s_errDetail : "FW write failed";

  // Match PACK_UPDATE "Erase NV": program 0xFF over 0x1F8000..+32K.
  // fillFF avoids a permanent/temporary 32KB 0xFF pad buffer.
  progress("Erase NV", 90, 100);
  if (!updatePartition(true, CI_NV_ADDR, nullptr, CI_NV_SIZE, 120000, true))
    return s_errDetail[0] ? s_errDetail : "NV erase failed";

  // Official tool sends 0x0E after last verify (session finalize).
  (void)cmdRoundtrip(CI_CMD_0E, CI_TYPE_REQ, 0, nullptr, 0, CI_CMD_0E, 0xFF, 2000);

  progress(STR_DEVICE_RESET, 95, 100);
  (void)cmdRoundtrip(CI_CMD_RESET, CI_TYPE_REQ, 0, nullptr, 0, CI_CMD_RESET,
                     0xFF, 2000);

  // Cold-boot voice module so it runs the new image at runtime baud.
  voiceControlPowerOff();
  sleep_ms(200);
  voiceControlPowerOn();
  sleep_ms(300);
  voiceControlSetBaudrate(VOICE_CTRL_BAUD);
  voiceControlInit();
  VoiceRunStatus = true;
  progress(STR_WRITING, 100, 100);
  return nullptr;
}

}  // namespace

bool isCi1302VoiceFirmware(const char* name, const char* fullpath)
{
  (void)fullpath;
  if (!name) return false;
  const char* ext = strrchr(name, '.');
  if (!ext || strcasecmp(ext, ".bin") != 0) return false;
  // Avoid stealing ESP / ELRS images
  char lower[96];
  size_t n = strlen(name);
  if (n >= sizeof(lower)) n = sizeof(lower) - 1;
  for (size_t i = 0; i < n; i++) lower[i] = (char)tolower((unsigned char)name[i]);
  lower[n] = 0;
  if (strstr(lower, "elrs") || strstr(lower, "esp32") || strstr(lower, "esp8266") ||
      strstr(lower, "backpack") || strstr(lower, "backup"))
    return false;
  return strstr(lower, "ci_03") || strstr(lower, "ci-03") ||
         strstr(lower, "ci1302") || strstr(lower, "jx_ci") ||
         strstr(lower, "jx_firm");
}

bool Ci1302FirmwareUpdate::flashFirmware(const char* filename,
                                         ProgressHandler progressHandler)
{
  FIL file;
  if (f_open(&file, filename, FA_OPEN_EXISTING | FA_READ) != FR_OK) {
    POPUP_WARNING(STR_DEVICE_FILE_ERROR);
    return false;
  }

  s_ws = (CiIspWorkspace*)malloc(sizeof(CiIspWorkspace));
  if (!s_ws) {
    f_close(&file);
    POPUP_WARNING(STR_FIRMWARE_UPDATE_ERROR, "Out of memory");
    return false;
  }
  memset(s_ws, 0, sizeof(*s_ws));

  progressHandler(getBasename(filename), STR_DEVICE_RESET, 0, 100);

  mixerTaskStop();
  pulsesStop();
  holdWatchdog();

  const char* err = doFlash(&file, getBasename(filename), progressHandler);
  f_close(&file);

  free(s_ws);
  s_ws = nullptr;

  VoiceRunStatus = true;
  voiceControlSetBaudrate(VOICE_CTRL_BAUD);
  voiceControlInit();

  AUDIO_PLAY(AU_SPECIAL_SOUND_BEEP1);
  BACKLIGHT_ENABLE();

  // Clear leftover ENTER from the confirm dialog so it cannot dismiss
  // POPUP_INFORMATION immediately on the next GUI pass.
  killAllEvents();
  waitKeysReleased();

  if (err) {
    POPUP_WARNING(STR_FIRMWARE_UPDATE_ERROR, err);
  } else {
    POPUP_INFORMATION(STR_FIRMWARE_UPDATE_SUCCESS);
  }

  watchdogSuspend(500);
  mixerTaskStart();
  pulsesStart();
  return err == nullptr;
}

#endif
