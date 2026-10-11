/*
 * Copyright (C) EdgeTX
 *
 * License GPLv2: http://www.gnu.org/licenses/gpl-2.0.html
 *
 * ESP ROM bootloader (esptool) flasher for HelloRadioSky V14:
 *   - internal ELRS TX (ESP32-D0WDQ6) on the module UART
 *   - ELRS backpack (ESP8285) via ESP32 UART passthrough
 *
 * TX sequence: power OFF -> BOOT -> power ON -> UART flash ->
 * power OFF -> BOOT release -> power ON (via pulsesStart).
 *
 * TX: app @ 0x10000. Blank chips need PIO sidecars beside the .bin:
 *   bootloader.bin @ 0x1000, partitions.bin @ 0x8000,
 *   optional boot_app0.bin @ 0xe000 (forces dual-OTA to boot app0).
 * ESP8285 backpack app @ 0x00000.
 */

#include "edgetx.h"
#include "esp32_firmware_update.h"
#include "board.h"
#include "hal/gpio.h"
#include "hal/module_port.h"
#include "hal/watchdog_driver.h"
#include "pulses/pulses.h"
#include "sdcard.h"
#include "timers_driver.h"
#include "lib_file.h"
#include "os/sleep.h"
#include "stm32_gpio.h"

#include <string.h>

namespace {

constexpr uint8_t ESP_IMAGE_MAGIC = 0xE9;
constexpr uint32_t ESP_APP_OFFSET = 0x10000;
constexpr uint32_t ESP8266_APP_OFFSET = 0x00000;
constexpr uint32_t ESP32_IDF_BOOT_OFFSET = 0x1000;
constexpr uint32_t ESP32_PTABLE_OFFSET = 0x8000;
constexpr uint32_t ESP32_BOOT_APP0_OFFSET = 0xe000;
constexpr uint32_t ESP_FLASH_BLOCK = 0x400;  // ROM write size (no stub)
constexpr uint32_t ESP_FLASH_SECTOR = 0x1000;
constexpr uint32_t ESP_MIN_APP_SIZE = 64 * 1024;
constexpr uint32_t ESP_MAX_APP_SIZE = 3 * 1024 * 1024;
constexpr uint32_t ESP_SYNC_BAUD = 115200;
constexpr uint32_t ESP_FAST_BAUD = 921600;
constexpr uint32_t ESP_MID_BAUD = 460800;

constexpr uint8_t ESP_FLASH_BEGIN = 0x02;
constexpr uint8_t ESP_FLASH_DATA = 0x03;
constexpr uint8_t ESP_FLASH_END = 0x04;
constexpr uint8_t ESP_SYNC = 0x08;
constexpr uint8_t ESP_READ_REG = 0x0A;
constexpr uint8_t ESP_SPI_SET_PARAMS = 0x0B;
constexpr uint8_t ESP_SPI_ATTACH = 0x0D;
constexpr uint8_t ESP_CHANGE_BAUDRATE = 0x0F;
constexpr uint32_t ESP_FLASH_SIZE = 4 * 1024 * 1024;
constexpr uint32_t EFUSE_BLK0_RDATA3 = 0x3FF5A00C;
constexpr uint32_t EFUSE_BLK0_RDATA5 = 0x3FF5A014;
constexpr uint32_t EFUSE_BLK0_RDATA4 = 0x3FF5A010;
constexpr uint32_t RTCCALICFG1 = 0x3FF5F06C;
constexpr uint32_t ESP_CHIP_DETECT_MAGIC = 0x40001000;
constexpr uint32_t ESP32_CHIP_MAGIC = 0x00F01D83;
constexpr uint8_t ESP_CHECKSUM_MAGIC = 0xEF;
constexpr uint8_t SLIP_END = 0xC0;
constexpr uint8_t SLIP_ESC = 0xDB;
constexpr uint8_t SLIP_ESC_END = 0xDC;
constexpr uint8_t SLIP_ESC_ESC = 0xDD;

constexpr uint32_t SLIP_TX_MAX = 2560;
constexpr uint32_t SLIP_RX_MAX = 256;
constexpr uint32_t ESP_CMD_MAX = 8 + 16 + ESP_FLASH_BLOCK;

uint8_t g_slipTx[SLIP_TX_MAX];
uint8_t g_cmdPkt[ESP_CMD_MAX];
uint8_t g_slipRx[SLIP_RX_MAX];

void putLe32(uint8_t* p, uint32_t v)
{
  p[0] = (uint8_t)(v);
  p[1] = (uint8_t)(v >> 8);
  p[2] = (uint8_t)(v >> 16);
  p[3] = (uint8_t)(v >> 24);
}

uint32_t getLe32(const uint8_t* p)
{
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) |
         ((uint32_t)p[3] << 24);
}

uint32_t packSpiPins(uint32_t clk, uint32_t q, uint32_t d, uint32_t hd, uint32_t cs)
{
  auto enc = [](uint32_t gpio) -> uint32_t {
    if (gpio == 32) return 30;
    if (gpio == 33) return 31;
    return gpio & 0x3F;
  };
  // esptool: (HD << 24) | (CS << 18) | (D << 12) | (Q << 6) | CLK
  return enc(clk) | (enc(q) << 6) | (enc(d) << 12) | (enc(cs) << 18) | (enc(hd) << 24);
}

uint8_t checksum(const uint8_t* data, uint32_t len)
{
  uint8_t s = ESP_CHECKSUM_MAGIC;
  for (uint32_t i = 0; i < len; i++) s ^= data[i];
  return s;
}

bool nameHasTag(const char* name, const char* tag)
{
  if (!name || !tag || !tag[0]) return false;
  const char* base = getBasename(name);
  const size_t tagLen = strlen(tag);
  for (const char* p = base; *p; ++p) {
    if (strncasecmp(p, tag, tagLen) == 0) return true;
  }
  return false;
}

bool isBinFile(const char* name)
{
  const char* ext = getFileExtension(getBasename(name));
  // Reject .bin.gz renamed to .bin ??still require content check below.
  return ext && strcasecmp(ext, ".bin") == 0;
}

bool binSizeOk(const char* fullpath, uint32_t minSize = ESP_MIN_APP_SIZE)
{
  if (!fullpath) return true;
  FILINFO info;
  if (f_stat(fullpath, &info) != FR_OK) return false;
  return info.fsize >= minSize && info.fsize <= ESP_MAX_APP_SIZE;
}

// ESP8266/ESP32 app image: magic 0xE9. WiFi OTA .bin.gz starts with 1F 8B.
bool isEspAppImageHeader(const uint8_t* hdr, unsigned n)
{
  if (!hdr || n < 1) return false;
  if (hdr[0] == 0x1F && n >= 2 && hdr[1] == 0x8B) return false;  // gzip
  if (hdr[0] != ESP_IMAGE_MAGIC) return false;
  // Segment count (byte 1): typically 1??6 for ESP8266/ESP32 images.
  if (n >= 2 && (hdr[1] == 0 || hdr[1] > 16)) return false;
  return true;
}

bool binHasEspImageMagic(const char* fullpath)
{
  if (!fullpath) return true;
  FIL f;
  if (f_open(&f, fullpath, FA_READ) != FR_OK) return false;
  uint8_t hdr[4] = {};
  UINT n = 0;
  const FRESULT fr = f_read(&f, hdr, sizeof(hdr), &n);
  f_close(&f);
  if (fr != FR_OK || n < 1) return false;
  return isEspAppImageHeader(hdr, n);
}

const char* checkEspAppImageFile(FIL* file)
{
  if (!file) return STR_DEVICE_FILE_ERROR;
  uint8_t hdr[4] = {};
  UINT n = 0;
  if (f_lseek(file, 0) != FR_OK || f_read(file, hdr, sizeof(hdr), &n) != FR_OK ||
      n < 1)
    return STR_DEVICE_FILE_ERROR;
  if (!isEspAppImageHeader(hdr, n)) return STR_DEVICE_FILE_WRONG_SIG;
  if (f_lseek(file, 0) != FR_OK) return STR_DEVICE_FILE_ERROR;
  return nullptr;
}

void makeSidecarPath(char* out, size_t outLen, const char* appPath, const char* name)
{
  if (!out || outLen == 0) return;
  out[0] = 0;
  if (!appPath || !name) return;
  const char* base = getBasename(appPath);
  const size_t dirLen = (size_t)(base - appPath);
  const size_t nameLen = strlen(name);
  if (dirLen + nameLen + 1 > outLen) return;
  memcpy(out, appPath, dirLen);
  memcpy(out + dirLen, name, nameLen + 1);
}

bool sidecarExists(const char* path)
{
  if (!path || !path[0]) return false;
  FILINFO info;
  return f_stat(path, &info) == FR_OK && info.fsize > 0;
}

bool resolvePartitionTablePath(char* out, size_t outLen, const char* appPath)
{
  makeSidecarPath(out, outLen, appPath, "partitions.bin");
  if (sidecarExists(out)) return true;
  makeSidecarPath(out, outLen, appPath, "partition-table.bin");
  return sidecarExists(out);
}

class Esp32RomFlasher
{
 public:
  const char* flash(FIL* file, const char* appPath, const char* label,
                    ProgressHandler progress)
  {
#if defined(SIMU)
    for (int i = 0; i <= 100; i += 5) {
      progress(label, STR_WRITING, i, 100);
      sleep_ms(20);
    }
    return nullptr;
#else
    const uint32_t imageSize = f_size(file);
    if (imageSize < ESP_MIN_APP_SIZE || imageSize > ESP_MAX_APP_SIZE)
      return STR_DEVICE_FILE_ERROR;

    if (const char* sig = checkEspAppImageFile(file)) return sig;

    auto* desc = modulePortGetModuleDescription(INTERNAL_MODULE);
    if (!desc || !desc->set_pwr) return STR_DEVICE_NO_RESPONSE;

    static const etx_serial_init serialCfg = {
        .baudrate = ESP_SYNC_BAUD,
        .encoding = ETX_Encoding_8N1,
        .direction = ETX_Dir_TX_RX,
        .polarity = ETX_Pol_Normal,
    };

    char bootPath[FF_MAX_LFN + 1];
    char ptablePath[FF_MAX_LFN + 1];
    char bootApp0Path[FF_MAX_LFN + 1];
    makeSidecarPath(bootPath, sizeof(bootPath), appPath, "bootloader.bin");
    const bool havePtable =
        resolvePartitionTablePath(ptablePath, sizeof(ptablePath), appPath);
    makeSidecarPath(bootApp0Path, sizeof(bootApp0Path), appPath, "boot_app0.bin");
    const bool fullFactoryLayout =
        sidecarExists(bootPath) && havePtable;

    progressFn = progress;
    progressLabel = label;
    holdWatchdogMs(60000);
    progress(label, STR_DEVICE_RESET, 0, 100);

    modSt = modulePortInitSerial(INTERNAL_MODULE, ETX_MOD_PORT_UART, &serialCfg,
                                 false);
    if (!modSt) {
      leave(false);
      return "Port error";
    }
    drv = modulePortGetSerialDrv(modSt->tx);
    ctx = modulePortGetCtx(modSt->tx);
    if (!drv || !ctx) {
      leave(false);
      return "Port error";
    }

    // PWR uses gpio_set = ON / gpio_clear = OFF.
    // V14 BOOT default is 1 (run); download needs GPIO0 low - try 0 then 1.
    const char* err = enterDownloadAndSync(0);
    if (err) err = enterDownloadAndSync(1);
    if (err) {
      leave(false);
      return "ELRS sync fail";
    }

    err = spiAttach();
    if (err) {
      leave(false);
      return err;
    }

    // ROM flashchip defaults are often 0 / wrong; FLASH_BEGIN then hangs
    // in erase instead of returning. esptool always sets these first.
    err = spiSetParams(ESP_FLASH_SIZE);
    if (err) {
      leave(false);
      return "SPI params fail";
    }

    // ROM sync is always 115200; raise UART before the bulk write.
    // Keep going at 115200 if baud change fails.
    if (changeBaud(ESP_FAST_BAUD)) (void)changeBaud(ESP_MID_BAUD);

    if (fullFactoryLayout) {
      progress(label, "Bootloader", 0, 100);
      err = flashPathAt(bootPath, ESP32_IDF_BOOT_OFFSET, true);
      if (err) {
        leave(false);
        return err;
      }
      progress(label, "Partitions", 25, 100);
      err = flashPathAt(ptablePath, ESP32_PTABLE_OFFSET, false);
      if (err) {
        leave(false);
        return err;
      }
      progress(label, STR_WRITING, 50, 100);
      err = flashOpenedFileAt(file, ESP_APP_OFFSET);
    } else {
      err = flashImage(file, label, progress);
    }
    if (err) {
      leave(false);
      return err;
    }

    // Dual-OTA: if otadata still selects app1, writing app0 "succeeds" but
    // the module keeps booting the old image. Force app0 when present.
    if (sidecarExists(bootApp0Path)) {
      progress(label, "boot_app0", 95, 100);
      err = flashPathAt(bootApp0Path, ESP32_BOOT_APP0_OFFSET, false);
      if (err) {
        leave(false);
        return err;
      }
    }

    (void)flashEndStay();
    leave(true);
    return nullptr;
#endif
  }

  const char* flashBackpack(FIL* file, const char* label, ProgressHandler progress)
  {
#if defined(SIMU)
    for (int i = 0; i <= 100; i += 5) {
      progress(label, STR_WRITING, i, 100);
      sleep_ms(20);
    }
    return nullptr;
#else
    // V14 backpack is ESP8285 (ESP8266 ROM), bridged by the ESP32 TX.
    chipEsp8266 = true;
    appOffset = ESP8266_APP_OFFSET;
    const uint32_t imageSize = f_size(file);
    if (imageSize < 8 * 1024 || imageSize > ESP_MAX_APP_SIZE)
      return STR_DEVICE_FILE_ERROR;
    // UART/SD needs uncompressed ESP image (0xE9). Reject WiFi OTA .bin.gz
    // bytes even if the user renamed the extension to .bin.
    if (const char* sig = checkEspAppImageFile(file)) return sig;

    auto* desc = modulePortGetModuleDescription(INTERNAL_MODULE);
    if (!desc || !desc->set_pwr) return STR_DEVICE_NO_RESPONSE;

    progressFn = progress;
    progressLabel = label;
    holdWatchdogMs(60000);
    progress(label, STR_DEVICE_RESET, 0, 100);

    // GPIO0 must be held while ELRS is still running so the TX enters
    // backpack UART passthrough (same flow as V15 / ELRS startPassthrough).
    const char* err = enterBackpackPassthroughAndSync();
    if (err) {
      leave(false);
      return err;
    }

    // Keep UI off the UART path ??460800 double-bridge is as flaky as USB ETX.
    quietIo = true;
    err = flashImage(file, label, progress);
    quietIo = false;
    if (err) {
      leave(false);
      return err;
    }

    (void)flashEndStay();
    leave(true);
    return nullptr;
#endif
  }


 private:
  etx_module_state_t* modSt = nullptr;
  const etx_serial_driver_t* drv = nullptr;
  void* ctx = nullptr;
  ProgressHandler progressFn = nullptr;
  const char* progressLabel = nullptr;
  const char* waitMsg = nullptr;
  uint32_t progressLastMs = 0;
  int waitCur = 0;
  int waitTot = 0;
  bool inRomErase = false;
  uint32_t romEraseStartMs = 0;
  uint32_t romEraseTimeoutMs = 0;
  int downloadBootLevel = -1;
  int backpackRunBoot = 0;  // set_bootcmd(0) = normal ELRS run
  bool chipEsp8266 = false;  // ESP8285 backpack ROM
  bool quietIo = false;      // skip progress redraw during tight UART sync
  uint32_t appOffset = ESP_APP_OFFSET;

  void rfPowerOff() { INTERNAL_MODULE_OFF(); }  // gpio_clear(PWR)
  void rfPowerOn() { INTERNAL_MODULE_ON(); }    // gpio_set(PWR)

  void bootWrite(int level)
  {
#if defined(INTMODULE_BOOTCMD_GPIO)
    gpio_write(INTMODULE_BOOTCMD_GPIO, level ? 1 : 0);
#endif
  }

  // EdgeTX CLI "bootpin N" / set_bootcmd ??respects INTMODULE_BOOTCMD_DEFAULT invert.
  // V16 DEFAULT=1: bootCmd(1) ??GPIO0 low (passthrough / download strap).
  void bootCmd(uint8_t enable)
  {
    auto* desc = modulePortGetModuleDescription(INTERNAL_MODULE);
    if (desc && desc->set_bootcmd)
      desc->set_bootcmd(enable);
    else
      bootWrite(enable ? !INTMODULE_BOOTCMD_DEFAULT : INTMODULE_BOOTCMD_DEFAULT);
  }


  // watchdogSuspend() units are 10 ms; keep the IWDG fed for long AUX1 flash work.
  void holdWatchdogMs(uint32_t ms)
  {
    watchdogSuspend((ms / 10u) + 500u);
  }

  void wdgSleep(uint32_t ms)
  {
    const uint32_t end = timersGetMsTick() + ms;
    while (timersGetMsTick() < end) {
      WDG_RESET();
      sleep_ms(10);
    }
  }


  void leave(bool success)
  {
    // ESP32 GPIO0 sampled at reset: download level stays in ROM, so Lua/CRSF
    // never start. Release BOOT to the opposite of whatever entered download.
    int normalBoot = INTMODULE_BOOTCMD_DEFAULT;
    if (chipEsp8266)
      normalBoot = backpackRunBoot;
    else if (downloadBootLevel == 0)
      normalBoot = 1;
    else if (downloadBootLevel == 1)
      normalBoot = 0;

    holdWatchdogMs(10000);
    if (chipEsp8266)
      bootWrite(backpackRunBoot);
    else
      bootWrite(normalBoot);
    wdgSleep(50);

    if (drv && ctx && drv->clearRxBuffer) drv->clearRxBuffer(ctx);
    if (modSt) {
      modulePortDeInit(modSt);
      modSt = nullptr;
    }
    rfPowerOff();
    wdgSleep(400);
    if (chipEsp8266)
      bootWrite(backpackRunBoot);
    else
      bootWrite(normalBoot);
    drv = nullptr;
    ctx = nullptr;
    (void)success;
  }

  const char* enterDownloadAndSync(int bootLevel)
  {
    progress(STR_DEVICE_RESET, 0, 1);
    waitMsg = STR_DEVICE_RESET;
    holdWatchdogMs(30000);

    // Power off, assert BOOT (download), then power on - ESP32 samples GPIO0 at reset.
    rfPowerOff();
    wdgSleep(500);

    // V14 INTMODULE_BOOTCMD_DEFAULT=1 (run). Download needs GPIO0 low - try 0 first.
    bootWrite(bootLevel);
    wdgSleep(100);

    rfPowerOn();
    wdgSleep(800);

    if (drv->clearRxBuffer) drv->clearRxBuffer(ctx);
    const char* err = syncRom();
    if (!err) downloadBootLevel = bootLevel;
    return err;
  }


  bool openUart(uint32_t baud)
  {
    if (modSt) {
      if (drv && ctx && drv->clearRxBuffer) drv->clearRxBuffer(ctx);
      modulePortDeInit(modSt);
      modSt = nullptr;
      drv = nullptr;
      ctx = nullptr;
    }
    etx_serial_init serialCfg = {
        .baudrate = baud,
        .encoding = ETX_Encoding_8N1,
        .direction = ETX_Dir_TX_RX,
        .polarity = ETX_Pol_Normal,
    };
    modSt = modulePortInitSerial(INTERNAL_MODULE, ETX_MOD_PORT_UART, &serialCfg,
                                 false);
    if (!modSt) return false;
    drv = modulePortGetSerialDrv(modSt->tx);
    ctx = modulePortGetCtx(modSt->tx);
    return drv && ctx;
  }

  void closeUart()
  {
    if (drv && ctx && drv->clearRxBuffer) drv->clearRxBuffer(ctx);
    if (modSt) {
      modulePortDeInit(modSt);
      modSt = nullptr;
    }
    drv = nullptr;
    ctx = nullptr;
  }

  bool talkingToTxRom()
  {
    uint32_t magic = 0;
    if (readReg(ESP_CHIP_DETECT_MAGIC, &magic)) return false;
    return magic == ESP32_CHIP_MAGIC;
  }

  const char* syncBackpackAt(uint32_t baud, bool* hitTxRom)
  {
    if (hitTxRom) *hitTxRom = false;
    if (!openUart(baud)) return "Port error";
    // Let the ELRS UART bridge settle; flush any CRSF/garbage.
    sleep_ms(150);
    if (drv->clearRxBuffer) drv->clearRxBuffer(ctx);
    sleep_ms(50);
    if (drv->clearRxBuffer) drv->clearRxBuffer(ctx);

    // Bridge + ESP8266 auto-baud needs a longer first-command window.
    const char* err = syncRom(baud == ESP_MID_BAUD ? 2500 : 1200,
                              baud == ESP_MID_BAUD ? 600 : 300);
    if (err) return err;
    if (talkingToTxRom()) {
      if (hitTxRom) *hitTxRom = true;
      return "TX ROM (no PT)";
    }
    return nullptr;
  }

  const char* tryBackpackBauds()
  {
    // V16 hardware.json: passthrough_baud unset ??debug_backpack_baud 460800.
    const uint32_t bauds[] = {ESP_MID_BAUD, ESP_SYNC_BAUD, 420000u, 400000u,
                              230400u};
    const char* last = STR_DEVICE_NO_RESPONSE;
    for (uint32_t baud : bauds) {
      watchdogSuspend(600);
      progress(STR_DEVICE_RESET, (int)(baud / 100), (int)(ESP_MID_BAUD / 100));
      bool hitTxRom = false;
      last = syncBackpackAt(baud, &hitTxRom);
      if (!last) return nullptr;
      if (hitTxRom) return last;
    }
    return last;
  }

  const char* syncBackpackOnce()
  {
    const char* err = "Port error";
    quietIo = true;
#if !defined(SIMU)
    vTaskSuspendAll();
#endif
    if (openUart(ESP_MID_BAUD)) {
      const uint32_t tSettle = timersGetMsTick();
      while ((timersGetMsTick() - tSettle) < 300) WDG_RESET();
      if (drv->clearRxBuffer) drv->clearRxBuffer(ctx);
      // USB flasher also needs many connect retries ??give a long window.
      err = syncRom(10000, 500);
      if (!err && talkingToTxRom()) err = "TX ROM (no PT)";
    }
#if !defined(SIMU)
    xTaskResumeAll();
#endif
    quietIo = false;
    return err;
  }

  const char* enterBackpackPassthroughAndSync()
  {
    progress(STR_DEVICE_RESET, 0, 1);
    waitMsg = STR_DEVICE_RESET;
    watchdogSuspend(12000);

    // USB ETX backpack flash works but is flaky on V14. Match `--before etx`
    // and retry the full sequence with several bootpin hold times.
    backpackRunBoot = INTMODULE_BOOTCMD_DEFAULT;
    pulsesStop();

    // bootpin hold ms: short (official 100) ??longer (debounce)
    const uint32_t holds[] = {100u, 200u, 500u, 1000u, 2000u};
    const char* last = STR_DEVICE_NO_RESPONSE;

    for (unsigned attempt = 0; attempt < sizeof(holds) / sizeof(holds[0]);
         attempt++) {
      watchdogSuspend(12000);
      progress(STR_DEVICE_RESET, (int)(attempt + 1),
               (int)(sizeof(holds) / sizeof(holds[0])));
      closeUart();
      bootCmd(0);

      modulePortSetPower(INTERNAL_MODULE, false);
      rfPowerOff();
      wdgSleep(200);

      rfPowerOn();
      modulePortSetPower(INTERNAL_MODULE, true);
      wdgSleep(2500);  // ELRS loop()

      bootCmd(1);
      wdgSleep(holds[attempt]);
      bootCmd(0);
      wdgSleep(100);

      last = syncBackpackOnce();
      if (!last) return nullptr;
    }
    return last;
  }

  void progress(const char* msg, int cur, int tot)
  {
    waitMsg = msg;
    waitCur = cur;
    waitTot = tot;
    if (!progressFn || quietIo) {
      WDG_RESET();
      return;
    }
    const uint32_t now = timersGetMsTick();
    const bool force = (cur <= 0) || (tot > 0 && cur >= tot);
    if (!force && (now - progressLastMs) < 250) {
      WDG_RESET();
      return;
    }
    progressLastMs = now;
    WDG_RESET();
    progressFn(progressLabel ? progressLabel : "", msg, cur, tot > 0 ? tot : 100);
    WDG_RESET();
  }

  void tickProgress()
  {
    if (!progressFn || quietIo) return;
    const uint32_t now = timersGetMsTick();
    if ((now - progressLastMs) < 250) return;
    progressLastMs = now;
    WDG_RESET();
    if (inRomErase && romEraseTimeoutMs > 0) {
      const uint32_t elapsed = now - romEraseStartMs;
      int pct = (int)((uint64_t)elapsed * 100u / romEraseTimeoutMs);
      if (pct > 99) pct = 99;
      progressFn(progressLabel ? progressLabel : "", STR_FLASH_ERASE, pct, 100);
      return;
    }
    progressFn(progressLabel ? progressLabel : "",
               waitMsg ? waitMsg : STR_DEVICE_RESET, waitCur,
               waitTot > 0 ? waitTot : 100);
  }

  bool getByte(uint8_t& b, uint32_t timeoutMs)
  {
    const uint32_t t0 = timersGetMsTick();
    while ((timersGetMsTick() - t0) < timeoutMs) {
      if (drv->getByte(ctx, &b) > 0) return true;
      WDG_RESET();
      tickProgress();
      WDG_RESET();
    }
    return false;
  }

  bool readSlip(uint8_t* out, uint32_t outMax, uint32_t& outLen, uint32_t timeoutMs)
  {
    outLen = 0;
    uint8_t b = 0;
    const uint32_t t0 = timersGetMsTick();

    // Wait for frame start
    while ((timersGetMsTick() - t0) < timeoutMs) {
      if (!getByte(b, 20)) continue;
      if (b == SLIP_END) break;
      WDG_RESET();
    }
    if (b != SLIP_END) return false;

    bool esc = false;
    const uint32_t byteWaitMs = 50u;
    while ((timersGetMsTick() - t0) < timeoutMs) {
      if (!getByte(b, byteWaitMs)) continue;
      if (!esc) {
        if (b == SLIP_END) {
          if (outLen == 0) continue;  // extra 0xC0 / empty frame
          return true;
        }
        if (b == SLIP_ESC) {
          esc = true;
          continue;
        }
      } else {
        if (b == SLIP_ESC_END)
          b = SLIP_END;
        else if (b == SLIP_ESC_ESC)
          b = SLIP_ESC;
        else
          return false;
        esc = false;
      }
      if (outLen >= outMax) return false;
      out[outLen++] = b;
      WDG_RESET();
    }
    return false;
  }

  void sendSlip(const uint8_t* src, uint32_t len)
  {
    uint32_t n = 0;
    auto put = [&](uint8_t b) {
      if (n < SLIP_TX_MAX) g_slipTx[n++] = b;
    };
    put(SLIP_END);
    for (uint32_t i = 0; i < len; i++) {
      if (src[i] == SLIP_END) {
        put(SLIP_ESC);
        put(SLIP_ESC_END);
      } else if (src[i] == SLIP_ESC) {
        put(SLIP_ESC);
        put(SLIP_ESC_ESC);
      } else {
        put(src[i]);
      }
    }
    put(SLIP_END);
    if (drv->waitForTxCompleted) {
      drv->waitForTxCompleted(ctx);
    }
    drv->sendBuffer(ctx, g_slipTx, n);
    if (drv->waitForTxCompleted) {
      drv->waitForTxCompleted(ctx);
    }
  }

  const char* command(uint8_t cmd, const uint8_t* data, uint16_t dataLen,
                      uint8_t chk, uint32_t timeoutMs, uint32_t* valueOut = nullptr)
  {
    uint8_t hdr[8];
    hdr[0] = 0x00;
    hdr[1] = cmd;
    hdr[2] = (uint8_t)(dataLen);
    hdr[3] = (uint8_t)(dataLen >> 8);
    putLe32(&hdr[4], chk);

    // Pack header+payload into pkt for a single SLIP frame.
    if ((uint32_t)8 + dataLen > sizeof(g_cmdPkt)) return STR_DEVICE_WRONG_REQUEST;
    memcpy(g_cmdPkt, hdr, 8);
    if (dataLen && data) memcpy(g_cmdPkt + 8, data, dataLen);

    if (drv->clearRxBuffer) drv->clearRxBuffer(ctx);
    sendSlip(g_cmdPkt, 8 + dataLen);

    uint32_t rlen = 0;
    if (!readSlip(g_slipRx, sizeof(g_slipRx), rlen, timeoutMs) || rlen < 10)
      return STR_DEVICE_NO_RESPONSE;
    if (g_slipRx[0] != 0x01 || g_slipRx[1] != cmd) return STR_DEVICE_NO_RESPONSE;

    const uint16_t size = (uint16_t)g_slipRx[2] | ((uint16_t)g_slipRx[3] << 8);
    if ((uint32_t)8 + size > rlen) return STR_DEVICE_NO_RESPONSE;
    if (valueOut) *valueOut = getLe32(&g_slipRx[4]);

    // Status bytes are the last 2 of the payload (ESP32 ROM).
    // ESP8266/ESP8285 through the TX bridge sometimes returns a different
    // trailer; accept the frame if the command echoed correctly.
    if (size >= 2 && !chipEsp8266) {
      const uint8_t* st = &g_slipRx[8 + size - 2];
      if (st[0] != 0) return STR_DEVICE_WRONG_REQUEST;
    }
    return nullptr;
  }

  const char* syncRom(uint32_t windowMs = 4000, uint32_t cmdTimeoutMs = 120)
  {
    uint8_t sync[36];
    sync[0] = 0x07;
    sync[1] = 0x07;
    sync[2] = 0x12;
    sync[3] = 0x20;
    memset(sync + 4, 0x55, 32);

    const uint32_t t0 = timersGetMsTick();
    while ((timersGetMsTick() - t0) < windowMs) {
      WDG_RESET();
      const char* err = command(ESP_SYNC, sync, sizeof(sync), 0, cmdTimeoutMs);
      if (!err) {
        // Drain extra sync replies.
        uint32_t extra = 0;
        for (int i = 0; i < 8; i++) {
          if (!readSlip(g_slipRx, sizeof(g_slipRx), extra, 30)) break;
        }
        return nullptr;
      }
    }
    return STR_DEVICE_NO_RESPONSE;
  }

  const char* readReg(uint32_t addr, uint32_t* out)
  {
    uint8_t p[4];
    putLe32(p, addr);
    return command(ESP_READ_REG, p, 4, 0, 300, out);
  }

  const char* spiAttach()
  {
    auto tryAttach = [&](uint32_t arg) -> const char* {
      uint8_t z[8];
      memset(z, 0, sizeof(z));
      putLe32(z, arg);
      const char* err = command(ESP_SPI_ATTACH, z, sizeof(z), 0, 800);
      if (!err) return nullptr;
      return command(ESP_SPI_ATTACH, z, 4, 0, 800);
    };

    // ESP8266/ESP8285 ROM: default attach is unsupported ??FLASH_BEGIN enables SPI.
    // Keep this branch for shared call sites; backpack path skips spiAttach().
    if (chipEsp8266) {
      return nullptr;
    }

    // ESP32-D0WDQ6: prefer eFuse pads, else ROM default (attach 0),
    // else WROOM SPI pins. Do NOT use PICO-D4 pins (6,17,8,11,16).
    uint32_t arg = 0;
    uint32_t r5 = 0;
    uint32_t r3 = 0;
    if (!readReg(EFUSE_BLK0_RDATA5, &r5) && !readReg(EFUSE_BLK0_RDATA3, &r3)) {
      const uint32_t clk = r5 & 0x1F;
      const uint32_t q = (r5 >> 5) & 0x1F;
      const uint32_t d = (r5 >> 10) & 0x1F;
      const uint32_t cs = (r5 >> 15) & 0x1F;
      const uint32_t hd = (r3 >> 4) & 0x1F;
      if (clk | q | d | cs | hd) arg = packSpiPins(clk, q, d, hd, cs);
    }

    if (arg) {
      const char* err = tryAttach(arg);
      if (!err) return nullptr;
    }

    if (!tryAttach(0)) return nullptr;

    // Datasheet default SPI: CLK=6 Q=7 D=8 HD=9 CS=11
    if (!tryAttach(packSpiPins(6, 7, 8, 9, 11))) return nullptr;

    return "SPI attach fail";
  }

  const char* spiSetParams(uint32_t flashSize)
  {
    uint8_t p[24];
    putLe32(p + 0, 0);             // flash id
    putLe32(p + 4, flashSize);     // total size
    putLe32(p + 8, 64 * 1024);     // block
    putLe32(p + 12, ESP_FLASH_SECTOR);
    putLe32(p + 16, 256);          // page
    putLe32(p + 20, 0xFFFF);       // status mask
    return command(ESP_SPI_SET_PARAMS, p, sizeof(p), 0, 500);
  }

  uint32_t esp32RomBaudArg(uint32_t baud)
  {
    // ESP32 ROM CK_8M drift workaround (esptool ESP32ROM.change_baud).
    uint32_t cali = 0;
    uint32_t efuse4 = 0;
    if (readReg(RTCCALICFG1, &cali) || readReg(EFUSE_BLK0_RDATA4, &efuse4))
      return baud;
    const uint32_t caliVal = (cali >> 7) & 0x01FFFFFF;
    const uint32_t clk8m = efuse4 & 0xFF;
    if (!caliVal || !clk8m) return baud;
    const uint32_t romHz =
        (uint32_t)((uint64_t)caliVal * 15625ULL * (uint64_t)clk8m / 40ULL);
    const uint32_t xtalHz = romHz > 33000000u ? 40000000u : 26000000u;
    return (uint32_t)((uint64_t)baud * (uint64_t)romHz / (uint64_t)xtalHz);
  }

  const char* changeBaud(uint32_t baud)
  {
    if (!drv || !drv->setBaudrate) return STR_DEVICE_NO_RESPONSE;
    // ESP8266 ROM has no reliable CHANGE_BAUDRATE over the TX bridge.
    if (chipEsp8266) return nullptr;
    uint8_t p[8];
    putLe32(p + 0, esp32RomBaudArg(baud));
    putLe32(p + 4, 0);
    const char* err = command(ESP_CHANGE_BAUDRATE, p, sizeof(p), 0, 500);
    if (err) return err;
    wdgSleep(30);
    drv->setBaudrate(ctx, baud);
    if (drv->clearRxBuffer) drv->clearRxBuffer(ctx);
    uint32_t dummy = 0;
    return readReg(EFUSE_BLK0_RDATA5, &dummy);
  }

  const char* flashBegin(uint32_t offset, uint32_t size)
  {
    const uint32_t aligned =
        (size + ESP_FLASH_BLOCK - 1) & ~(ESP_FLASH_BLOCK - 1);
    const uint32_t blocks = aligned / ESP_FLASH_BLOCK;
    uint8_t p[20];
    putLe32(p + 0, aligned);
    putLe32(p + 4, blocks);
    putLe32(p + 8, ESP_FLASH_BLOCK);
    putLe32(p + 12, offset);
    // ESP32 classic + ESP8266: 16-byte FLASH_BEGIN (no encrypted flag).
    const uint16_t plen = 16;
    // ROM erases the region up front before returning (~30s/MB on-chip).
    uint32_t timeoutMs = 1000;
    if (size) {
      timeoutMs = 3000u + (uint32_t)((uint64_t)aligned * 35000u / (1024u * 1024u));
      if (timeoutMs < 20000u) timeoutMs = 20000u;
      if (timeoutMs > 120000u) timeoutMs = 120000u;
    }
    inRomErase = size != 0;
    romEraseStartMs = timersGetMsTick();
    romEraseTimeoutMs = timeoutMs;
    watchdogSuspend((timeoutMs / 10) + 500);
    const char* err = command(ESP_FLASH_BEGIN, p, plen, 0, timeoutMs);
    inRomErase = false;
    return err;
  }

  const char* writeBlocks(FIL* file, uint32_t nbytes, uint32_t /*fileTot*/)
  {
    uint8_t block[16 + ESP_FLASH_BLOCK];
    uint32_t seq = 0;
    uint32_t left = nbytes;
    const uint32_t totalBlocks =
        (nbytes + ESP_FLASH_BLOCK - 1) / ESP_FLASH_BLOCK;
    UINT got = 0;
    while (left) {
      // Sparse UI updates while quietIo (high baud / AUX1 ? avoid lv_refr each block)
      if (!quietIo || (seq & 0x0F) == 0) {
        const bool q = quietIo;
        quietIo = false;
        progress(STR_WRITING, (int)seq, (int)totalBlocks);
        quietIo = q;
      }
      memset(block, 0xFF, sizeof(block));
      const uint32_t want = left > ESP_FLASH_BLOCK ? ESP_FLASH_BLOCK : left;
      if (f_read(file, block + 16, want, &got) != FR_OK)
        return STR_DEVICE_FILE_ERROR;

      putLe32(block + 0, ESP_FLASH_BLOCK);
      putLe32(block + 4, seq);
      putLe32(block + 8, 0);
      putLe32(block + 12, 0);
      const uint8_t chk = checksum(block + 16, ESP_FLASH_BLOCK);
      const uint32_t dataTimeout = 2000u;
      holdWatchdogMs(dataTimeout + 2000u);
      const char* err = nullptr;
      const int maxAttempts = 1;
      for (int attempt = 0; attempt < maxAttempts; attempt++) {
        if (attempt > 0) {
          if (drv->clearRxBuffer) drv->clearRxBuffer(ctx);
          wdgSleep(20);
        }
        err = command(ESP_FLASH_DATA, block, (uint16_t)sizeof(block), chk,
                      dataTimeout);
        if (!err) break;
      }
      if (err) return err;
      seq++;
      left -= want;
      WDG_RESET();
    }
    return nullptr;
  }

  const char* flashOpenedFileAt(FIL* file, uint32_t offset)
  {
    if (f_lseek(file, 0) != FR_OK) return STR_DEVICE_FILE_ERROR;
    const uint32_t imageSize = f_size(file);
    const uint32_t aligned =
        (imageSize + ESP_FLASH_BLOCK - 1) & ~(ESP_FLASH_BLOCK - 1);
    progress(STR_FLASH_ERASE, 0, 100);
    const char* err = flashBegin(offset, aligned);
    if (err) return err;
    err = writeBlocks(file, aligned, aligned);
    if (err) return err;
    const uint32_t totalBlocks =
        (aligned + ESP_FLASH_BLOCK - 1) / ESP_FLASH_BLOCK;
    progress(STR_WRITING, (int)totalBlocks, (int)totalBlocks);
    return nullptr;
  }

  const char* flashPathAt(const char* path, uint32_t offset, bool requireImageMagic)
  {
    FIL f;
    if (f_open(&f, path, FA_READ) != FR_OK) return STR_DEVICE_FILE_ERROR;
    const uint32_t sz = f_size(&f);
    if (!sz || sz > ESP_MAX_APP_SIZE) {
      f_close(&f);
      return STR_DEVICE_FILE_ERROR;
    }
    if (requireImageMagic) {
      uint8_t magic = 0;
      UINT n = 0;
      if (f_read(&f, &magic, 1, &n) != FR_OK || n != 1 ||
          magic != ESP_IMAGE_MAGIC) {
        f_close(&f);
        return STR_DEVICE_FILE_WRONG_SIG;
      }
    }
    const char* err = flashOpenedFileAt(&f, offset);
    f_close(&f);
    return err;
  }

  const char* flashImage(FIL* file, const char* label, ProgressHandler /*progress*/)
  {
    (void)label;
    return flashOpenedFileAt(file, appOffset);
  }

  const char* flashEndStay()
  {
    uint8_t p[4];
    // ESP32: 1 = stay in bootloader. ESP8266: 0 = reboot (we power-cycle anyway).
    putLe32(p, chipEsp8266 ? 0u : 1u);
    return command(ESP_FLASH_END, p, sizeof(p), 0, 1000);
  }
};

}  // namespace

bool nameOrPathHasTag(const char* name, const char* fullpath, const char* tag)
{
  if (nameHasTag(name, tag)) return true;
  if (fullpath && nameHasTag(fullpath, tag)) return true;
  return false;
}

bool isInternalElrsAppFirmware(const char* name, const char* fullpath)
{
  // TX image: ".bin" with "ELRS", not "BACKUP" (case-insensitive).
  // Prefer V14 tag when present; still accept ELRS-only names.
  if (!name || !isBinFile(name)) return false;
  if (nameOrPathHasTag(name, fullpath, "BACKUP")) return false;
  if (!nameOrPathHasTag(name, fullpath, "ELRS")) return false;
  return binSizeOk(fullpath) && binHasEspImageMagic(fullpath);
}

bool isInternalElrsBackpackFirmware(const char* name, const char* fullpath)
{
  // Backpack image: ".bin" whose name contains "BACKUP" (HRS packaging keyword).
  if (!name || !isBinFile(name)) return false;
  if (!nameOrPathHasTag(name, fullpath, "BACKUP")) return false;
  if (!fullpath) return true;
  FILINFO info;
  if (f_stat(fullpath, &info) != FR_OK) return false;
  if (info.fsize < 8 * 1024 || info.fsize > ESP_MAX_APP_SIZE) return false;
  // Must be raw ESP8266 image (0xE9), not gzip WiFi package (1F 8B).
  return binHasEspImageMagic(fullpath);
}

bool Esp32ElrsFirmwareUpdate::flashFirmware(const char* filename,
                                            ProgressHandler progressHandler)
{
  FIL file;
  if (f_open(&file, filename, FA_READ) != FR_OK) {
    POPUP_WARNING(STR_DEVICE_FILE_ERROR);
    return false;
  }

  pulsesStop();
  progressHandler(getBasename(filename), STR_DEVICE_RESET, 0, 100);

  Esp32RomFlasher flasher;
  const char* result =
      flasher.flash(&file, filename, getBasename(filename), progressHandler);
  f_close(&file);

  AUDIO_PLAY(AU_SPECIAL_SOUND_BEEP1);
  BACKLIGHT_ENABLE();

  if (result) {
    POPUP_WARNING(STR_FIRMWARE_UPDATE_ERROR, result);
  } else {
    POPUP_INFORMATION(STR_FIRMWARE_UPDATE_SUCCESS);
  }

  watchdogSuspend(500);
  WDG_RESET();
  pulsesStart();  // power module on in normal (non-boot) mode
  return result == nullptr;
}

bool Esp32BackpackFirmwareUpdate::flashFirmware(const char* filename,
                                                ProgressHandler progressHandler)
{
  FIL file;
  if (f_open(&file, filename, FA_READ) != FR_OK) {
    POPUP_WARNING(STR_DEVICE_FILE_ERROR);
    return false;
  }

  progressHandler(getBasename(filename), STR_DEVICE_RESET, 0, 100);

  Esp32RomFlasher flasher;
  const char* result =
      flasher.flashBackpack(&file, getBasename(filename), progressHandler);
  f_close(&file);

  AUDIO_PLAY(AU_SPECIAL_SOUND_BEEP1);
  BACKLIGHT_ENABLE();

  if (result) {
    POPUP_WARNING(STR_FIRMWARE_UPDATE_ERROR, result);
  } else {
    POPUP_INFORMATION(STR_FIRMWARE_UPDATE_SUCCESS);
  }

  watchdogSuspend(500);
  WDG_RESET();
  pulsesStart();
  return result == nullptr;
}

