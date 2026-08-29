/*
 * Copyright (C) EdgeTX
 *
 * License GPLv2: http://www.gnu.org/licenses/gpl-2.0.html
 */

#include "v14_charge_ui.h"

#if defined(RADIO_V14) || defined(RADIO_V14LCD)

#include "board.h"
#include "csd203_sensor.h"
#include "delays_driver.h"
#include "edgetx.h"
#include "hal/adc_driver.h"
#include "hal/key_driver.h"
#include "keys.h"
#include "lcd.h"
#include "timers_driver.h"
#include "gui/common/stdlcd/menus_common.h"

#if !defined(SIMU)
#include "hal/watchdog_driver.h"
#endif

extern bool suspendI2CTasks;

#ifndef V14_CHARGE_CONFIRM_SAMPLES
#define V14_CHARGE_CONFIRM_SAMPLES 3
#endif

#ifndef V14_CHARGE_CURRENT_MIN_MA
#define V14_CHARGE_CURRENT_MIN_MA 50
#endif

#ifndef V14_CHARGE_FULL_VBAT
#define V14_CHARGE_FULL_VBAT 825
#endif

// Leave overlay / allow a new session after charger is gone this long.
#ifndef V14_CHARGE_ABSENT_MS
#define V14_CHARGE_ABSENT_MS 400
#endif

static bool s_dismissed = false;
static bool s_wasCharging = false;
static bool s_chargeFull = false;
static uint8_t s_confirmCount = 0;
static bool s_inChargeUi = false;
static bool s_absentTiming = false;
static uint32_t s_absentStartMs = 0;
static uint8_t s_chargeProgress = 0;
static tmr10ms_t s_chargeLastAnim = 0;

static bool s_telemFiltInit = false;
static int32_t s_filtVbat = 0;
static int32_t s_filtIma = 0;
static uint8_t s_dispSoc = 0;

static void v14ChargeTelemReset()
{
  s_telemFiltInit = false;
  s_filtVbat = 0;
  s_filtIma = 0;
  s_dispSoc = 0;
}

static void v14ChargeSessionReset()
{
  s_wasCharging = false;
  s_chargeFull = false;
  s_confirmCount = 0;
  s_absentTiming = false;
}

static void v14ChargeUiLeave()
{
  s_inChargeUi = false;
  v14ChargeTelemReset();
}

static void v14ChargeUiDismiss()
{
  s_dismissed = true;
  s_confirmCount = 0;
  s_wasCharging = true;
  killEvents(KEY_EXIT);
  v14ChargeUiLeave();
}

static int16_t v14ChargeSenseCurrentMa()
{
  return getCSD203MainCurrent();
}

static bool v14CurrentIsCharging()
{
  // Positive pack current = charging. STAT alone must not keep the UI up
  // after the adapter is unplugged.
  return v14ChargeSenseCurrentMa() >= (int16_t)V14_CHARGE_CURRENT_MIN_MA;
}

static void v14ChargeSenseTick()
{
  if (v14CurrentIsCharging()) {
    s_absentTiming = false;
    return;
  }

  if (!s_absentTiming) {
    s_absentTiming = true;
    s_absentStartMs = timersGetMsTick();
  }
}

static void v14ChargeTickSensors(bool fullCycle)
{
#if !defined(SIMU)
  // One CSD203 device per readCSD203() call; a full cycle updates V and I.
  const int n = fullCycle ? 3 : 1;
  for (int i = 0; i < n; i++) {
    readCSD203();
  }
#endif
  v14ChargeSenseTick();
}

static bool v14ChargeAbsentStable()
{
  if (!s_absentTiming)
    return false;
  return (timersGetMsTick() - s_absentStartMs) >= V14_CHARGE_ABSENT_MS;
}

static bool v14ChargeActiveRaw()
{
  return v14CurrentIsCharging();
}

bool v14ChargePresent()
{
  return v14ChargeActiveRaw();
}

bool v14IsCharging()
{
  if (s_chargeFull || s_dismissed)
    return false;

  if (!v14ChargeActiveRaw()) {
    s_confirmCount = 0;
    return false;
  }

  if (s_confirmCount < V14_CHARGE_CONFIRM_SAMPLES)
    s_confirmCount++;

  return s_confirmCount >= V14_CHARGE_CONFIRM_SAMPLES;
}

bool v14ChargeUiActive()
{
  return s_inChargeUi;
}

static uint8_t v14EstimateSocPercent(uint16_t vbat10mv)
{
  constexpr uint16_t empty = 660;
  constexpr uint16_t full = 835;
  if (vbat10mv <= empty) return 0;
  if (vbat10mv >= full) return 100;
  return (uint8_t)(((uint32_t)(vbat10mv - empty) * 100) / (full - empty));
}

static void v14ChargeUpdateTelem(uint16_t vRaw, int16_t iRaw, uint16_t& vOut, int16_t& iOut,
                                 uint8_t& socOut)
{
  if (!s_telemFiltInit) {
    s_filtVbat = ((int32_t)vRaw) << 3;
    s_filtIma = ((int32_t)iRaw) << 3;
    s_dispSoc = v14EstimateSocPercent(vRaw);
    s_telemFiltInit = true;
  } else {
    s_filtVbat += ((int32_t)vRaw - (s_filtVbat >> 3));
    s_filtIma += ((int32_t)iRaw - (s_filtIma >> 3));
  }

  vOut = (uint16_t)((s_filtVbat + 4) >> 3);
  iOut = (int16_t)((s_filtIma + 4) >> 3);

  const uint8_t socRaw = v14EstimateSocPercent(vOut);
  if (socRaw >= (uint8_t)(s_dispSoc + 2) || s_dispSoc >= (uint8_t)(socRaw + 2)) {
    s_dispSoc = socRaw;
  }
  socOut = s_dispSoc;
}

static void v14DrawBoltIcon(coord_t x, coord_t y)
{
  // 8x7 filled lightning (MSB = left); last row omitted so it sits above the header line
  static const uint8_t kBolt[7] = {
      0b00011100,
      0b00111000,
      0b01110000,
      0b11111110,
      0b00011100,
      0b00111000,
      0b01110000,
  };
  for (uint8_t row = 0; row < 7; row++) {
    uint8_t bits = kBolt[row];
    for (uint8_t col = 0; col < 8; col++) {
      if (bits & (0x80 >> col))
        lcdDrawPoint(x + col, y + row);
    }
  }
}

static void v14DrawChargeScreen(uint8_t progress)
{
  constexpr coord_t BAT_X = 2;
  constexpr coord_t BAT_Y = 12;
  constexpr coord_t BAT_W = 84;
  constexpr coord_t BAT_H = 14;
  constexpr coord_t TIP_W = 3;
  constexpr coord_t TIP_H = 6;
  constexpr uint8_t SEGS = 5;
  constexpr coord_t SEG_GAP = 1;

  lcdClear();

  uint16_t vbat = 0;
  int16_t ima = 0;
  uint8_t soc = 0;
  v14ChargeUpdateTelem(getBatteryVoltage(), v14ChargeSenseCurrentMa(), vbat, ima, soc);

  const int32_t pPrec2 = ((int32_t)vbat * (int32_t)ima) / 1000;
  const int32_t pShow = (pPrec2 < 0) ? -pPrec2 : pPrec2;
  const int16_t iAbs = (ima < 0) ? -ima : ima;

  lcdDrawText(1, 0, "CHARGE");
  v14DrawBoltIcon(LCD_W - 3 * FW - 10, 0);
  lcdDrawText(LCD_W - 3 * FW - 1, 0, "USB");
  lcdDrawSolidHorizontalLine(0, 9, LCD_W);

  lcdDrawRect(BAT_X, BAT_Y, BAT_W, BAT_H);
  lcdDrawSolidFilledRect(BAT_X + BAT_W, BAT_Y + (BAT_H - TIP_H) / 2, TIP_W, TIP_H);

  {
    const coord_t pad = 2;
    const coord_t innerW = BAT_W - 2 * pad;
    const coord_t innerH = BAT_H - 2 * pad;
    const coord_t segW = (innerW - (SEGS - 1) * SEG_GAP) / SEGS;
    uint8_t lit = (uint8_t)(((uint16_t)soc * SEGS + 50) / 100);
    if (lit > SEGS) lit = SEGS;

    for (uint8_t i = 0; i < SEGS; i++) {
      coord_t sx = BAT_X + pad + i * (segW + SEG_GAP);
      coord_t sy = BAT_Y + pad;
      if (i < lit) {
        lcdDrawSolidFilledRect(sx, sy, segW, innerH);
      } else if (i == lit && lit < SEGS && ((progress & 1) == 0)) {
        lcdDrawFilledRect(sx, sy, segW, innerH, DOTTED);
      }
    }
  }

  {
    uint8_t digits = (soc >= 100) ? 3 : ((soc >= 10) ? 2 : 1);
    coord_t tw = digits * FWNUM + FW;
    coord_t x = BAT_X + (BAT_W - tw) / 2;
    lcdDrawNumber(x, BAT_Y + BAT_H + 2, soc, LEFT);
    lcdDrawText(lcdNextPos, BAT_Y + BAT_H + 2, "%");
  }

  {
    constexpr coord_t y0 = 12;
    constexpr coord_t row = 9;
    constexpr coord_t unitR = LCD_W - 1;

    // RIGHT-aligned number then unit: draw unit first, number ends at lcdLastLeftPos.
    lcdDrawText(unitR, y0, "V", RIGHT);
    lcdDrawNumber(lcdLastLeftPos, y0, vbat, PREC2 | LEADING0 | RIGHT, 3);

    lcdDrawText(unitR, y0 + row, "A", RIGHT);
    lcdDrawNumber(lcdLastLeftPos, y0 + row, ima / 10, PREC2 | RIGHT);

    lcdDrawText(unitR, y0 + 2 * row, "W", RIGHT);
    lcdDrawNumber(lcdLastLeftPos, y0 + 2 * row, pShow, PREC2 | RIGHT);
  }

  lcdDrawSolidHorizontalLine(0, 40, LCD_W);

  {
    constexpr coord_t barY = 44;
    constexpr coord_t barH = 9;
    constexpr coord_t barX = 2;
    constexpr coord_t barW = 90;
    constexpr uint8_t ticks = 22;
    constexpr coord_t gap = 1;
    const coord_t tickW = (barW - (ticks - 1) * gap) / ticks;
    uint8_t filled = (uint8_t)(((uint16_t)soc * ticks + 50) / 100);
    if (filled > ticks) filled = ticks;
    if (filled < ticks && (progress & 1) == 0)
      filled++;

    for (uint8_t i = 0; i < ticks; i++) {
      coord_t tx = barX + i * (tickW + gap);
      if (i < filled)
        lcdDrawSolidFilledRect(tx, barY, tickW, barH);
      else
        lcdDrawRect(tx, barY, tickW, barH);
    }

    const char* profile;
    if (iAbs >= 1800)
      profile = "9V/2A";
    else if (iAbs >= 900)
      profile = "5V/2A";
    else
      profile = "5V/1A";
    lcdDrawText(LCD_W - 1, barY + 1, profile, RIGHT);
  }

  lcdDrawText(2, LCD_H - 8, "EXIT", INVERS);

  lcdRefresh();
}

static void v14ChargeUiEnter()
{
  s_inChargeUi = true;
  s_chargeProgress = 0;
  s_chargeLastAnim = get_tmr10ms();
  v14ChargeTelemReset();
  BACKLIGHT_ENABLE();
}

static bool v14ChargeUiCheckAutoExit()
{
  if (!v14ChargeAbsentStable())
    return false;

  if (getBatteryVoltage() >= V14_CHARGE_FULL_VBAT)
    s_chargeFull = true;

  return true;
}

void v14ChargeUiHandleEvent(event_t evt)
{
  if (!s_inChargeUi)
    return;

  if (IS_KEY_EVT(evt, KEY_EXIT)) {
    v14ChargeUiDismiss();
    return;
  }

  if (menuLevel > 0) {
    v14ChargeUiDismiss();
  }
}

void v14ChargeUiDraw()
{
  if (!s_inChargeUi)
    return;

  lcdRefreshWait();

  tmr10ms_t now = get_tmr10ms();
  if ((tmr10ms_t)(now - s_chargeLastAnim) >= 50) {
    s_chargeLastAnim = now;
    s_chargeProgress = (s_chargeProgress + 1) % 5;
  }

  v14DrawChargeScreen(s_chargeProgress);
}

void v14ChargeUiPoll()
{
  v14ChargeTickSensors(s_inChargeUi);

  if (!v14ChargeActiveRaw()) {
    if (s_inChargeUi && v14ChargeUiCheckAutoExit())
      v14ChargeUiLeave();
    if (v14ChargeAbsentStable()) {
      s_dismissed = false;
      v14ChargeSessionReset();
    }
    return;
  }

  if (s_inChargeUi)
    return;

  if (s_chargeFull || s_dismissed)
    return;

  if (!v14IsCharging()) {
    s_wasCharging = false;
    return;
  }

  if (!s_wasCharging) {
    s_wasCharging = true;
    v14ChargeUiEnter();
  }
}

void v14ChargeUiRunUntilExit(bool shutdownMode)
{
  // edgeTxClose() sets suspendI2CTasks before boardOff(); without this,
  // CSD203 never updates and the shutdown charge UI shows frozen numbers.
  const bool i2cWasSuspended = suspendI2CTasks;
  suspendI2CTasks = false;

  v14ChargeUiEnter();

  while (true) {
#if !defined(SIMU)
    WDG_RESET();
#endif

    v14ChargeTickSensors(true);

    if (v14ChargeUiCheckAutoExit())
      break;

    event_t evt = getEvent();
    if (IS_KEY_EVT(evt, KEY_EXIT)) {
      if (!shutdownMode)
        v14ChargeUiDismiss();
      else
        v14ChargeUiLeave();
      break;
    }

    v14ChargeUiDraw();

#if !defined(SIMU)
    delay_ms(20);
#endif
  }

  suspendI2CTasks = i2cWasSuspended;
  v14ChargeUiLeave();
  lcdClear();
  lcdRefresh();
}

#endif
