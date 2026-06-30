#include "batsenser.h"

#if defined(COLORLCD) && defined(MODULE_BATTERY_SENSOR)

#include "edgetx.h"
#include "fonts.h"
#include "mainwindow.h"
#include "static.h"

#include <stdio.h>

#include <string>

namespace {

constexpr uint16_t BATSENSER_MIN_MV = 3300;
constexpr uint16_t BATSENSER_FULL_MV = 4200;
constexpr uint16_t BATSENSER_LOW_MV = 3500;
constexpr uint16_t BATSENSER_DELTA_WARN_MV = 50;
constexpr uint16_t BATSENSER_ANALYZE_MIN_MV = 3000;
constexpr uint16_t BATSENSER_QUALITY_LOW_MV = 3600;
constexpr uint16_t BATSENSER_QUALITY_DELTA_MV = 50;
constexpr uint16_t BATSENSER_STABLE_JITTER_MV = 20;
constexpr uint8_t BATSENSER_STABLE_SAMPLE_COUNT = 10;
constexpr uint8_t BATSENSER_INVALID_HOLD_SAMPLES = 20;
constexpr tmr10ms_t BATSENSER_ANNOUNCE_DELAY_10MS = 50;  // 0.5s
constexpr uint8_t BATSENSER_ANNOUNCE_PLAY_ID = ID_PLAY_FROM_SD_MANAGER;

// Layout (480x272 target)
constexpr coord_t DASH_MARGIN = 6;
constexpr coord_t HDR_H = 30;
constexpr coord_t MAIN_TOP = DASH_MARGIN + HDR_H;
constexpr coord_t LEFT_W = 154;
constexpr coord_t LEFT_X = DASH_MARGIN;
constexpr coord_t RIGHT_X = LEFT_X + LEFT_W + 6;
constexpr coord_t RIGHT_W = LCD_W - RIGHT_X - DASH_MARGIN;
constexpr coord_t CELL_AREA_H = 138;
constexpr coord_t PACK_SECTION_H = 56;
constexpr coord_t BAR_TRACK_W = 16;
constexpr coord_t BAR_TRACK_H = 74;
constexpr coord_t BAR_BORDER_W = 1;
constexpr coord_t BAR_FILL_MARGIN =
    1; /* gap between inner edge of frame border and SOC column */
constexpr coord_t BAR_FILL_W =
    BAR_TRACK_W - 2 * BAR_BORDER_W - 2 * BAR_FILL_MARGIN;
constexpr coord_t BAR_FILL_ZONE_H =
    BAR_TRACK_H - 2 * BAR_BORDER_W - 2 * BAR_FILL_MARGIN;
constexpr coord_t FOOTER_H = 26;
/** Width taken from dV band so VBAT label fits one line at FONT(XS). */
constexpr coord_t FOOTER_VBAT_EXTRA_W = 30;

static const char TITLE_VOLT_MON[] = "Battery Meter";
static const char LBL_TOTAL_V[] = "Pack Vtot";
static const char LBL_CELL_RANGE[] = "Cell V span";
static const char LBL_CELL_PANEL[] = "Cells (1-6S)";
static const char LBL_PACK_BAR[] = "Pack SOC bar";

static inline lv_color_t colNeonGreen()
{
  return lv_color_hex(0x39ff14);
}
static inline lv_color_t colElecBlue()
{
  return lv_color_hex(0x00b4ff);
}
static inline lv_color_t colDashBg()
{
  return lv_color_hex(0x0a100e);
}
static inline lv_color_t colCardBg()
{
  return lv_color_hex(0x121a17);
}
static inline lv_color_t colCardBorder()
{
  return lv_color_hex(0x2a3d34);
}
static inline lv_color_t colMuted()
{
  return lv_color_hex(0x8aa898);
}
static inline lv_color_t colWarn()
{
  return lv_color_hex(0xffcc00);
}
static inline lv_color_t colDanger()
{
  return lv_color_hex(0xff3355);
}

static lv_color_t cellBarColor(uint16_t voltageMv)
{
  if (voltageMv < BATSENSER_LOW_MV) {
    return colDanger();
  }
  if (voltageMv < BATSENSER_MIN_MV + 80) {
    return colWarn();
  }
  return colNeonGreen();
}

static void lockObjectInteractions(lv_obj_t* obj)
{
  lv_obj_clear_flag(
      obj,
      LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_CHAIN_HOR |
          LV_OBJ_FLAG_SCROLL_CHAIN_VER | LV_OBJ_FLAG_GESTURE_BUBBLE |
          LV_OBJ_FLAG_SCROLL_MOMENTUM | LV_OBJ_FLAG_SCROLL_ELASTIC);
  lv_obj_set_scrollbar_mode(obj, LV_SCROLLBAR_MODE_OFF);
  lv_obj_set_scroll_dir(obj, LV_DIR_NONE);
}

static void styleDashboardBg(lv_obj_t* obj)
{
  lv_obj_set_style_bg_color(obj, colDashBg(), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(obj, LV_OPA_COVER, LV_PART_MAIN);
}

static void styleCard(lv_obj_t* obj)
{
  lv_obj_set_style_bg_color(obj, colCardBg(), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(obj, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_set_style_border_color(obj, colCardBorder(), LV_PART_MAIN);
  lv_obj_set_style_border_width(obj, 1, LV_PART_MAIN);
  lv_obj_set_style_radius(obj, 4, LV_PART_MAIN);
  lv_obj_set_style_pad_all(obj, 2, LV_PART_MAIN);
}

static void setLabelRgb(StaticText* st, lv_color_t c)
{
  if (st && st->getLvObj()) {
    lv_obj_set_style_text_color(st->getLvObj(), c, LV_PART_MAIN);
  }
}

class Battery6SDialog : public Window
{
 public:
  Battery6SDialog() :
      Window(MainWindow::instance(), {0, 0, LCD_W, LCD_H})
  {
    setWindowFlag(OPAQUE);
    lockObjectInteractions(lvobj);
    styleDashboardBg(lvobj);

    pushLayer();

    buildHeader();
    buildLeftSummary();
    buildCellColumns();
    buildPackBarSection();
    buildFooter();

    closeCondition = []() { return !v15BatteryCell1Present(); };
    batteryInsertedTick = get_tmr10ms();
    refreshValues();
  }

 protected:
  std::function<bool(void)> closeCondition;

  StaticText* hdrDecorLeft = nullptr;
  StaticText* hdrTitle = nullptr;
  StaticText* hdrDecorRight = nullptr;

  Window* leftCard = nullptr;
  StaticText* lblTotalTitle = nullptr;
  StaticText* lblTotalVolts = nullptr;
  StaticText* lblTotalPct = nullptr;
  StaticText* lblCellRange = nullptr;
  StaticText* lblCellRangeDetail = nullptr;
  StaticText* lblRefPack = nullptr;
  StaticText* lblRefCell = nullptr;

  Window* cellStrip = nullptr;
  Window* cellCards[6] = {nullptr};
  StaticText* cellIdx[6] = {nullptr};
  StaticText* cellVolt[6] = {nullptr};
  StaticText* cellPct[6] = {nullptr};
  lv_obj_t* cellBarTrack[6] = {nullptr};
  lv_obj_t* cellBarFill[6] = {nullptr};

  Window* packStrip = nullptr;
  StaticText* lblPackTitle = nullptr;
  lv_obj_t* packBarBg = nullptr;
  lv_obj_t* packMarker = nullptr;
  StaticText* lblPackPctBig = nullptr;
  StaticText* lblPackScale = nullptr;

  StaticText* summaryCells = nullptr;
  StaticText* summaryPack = nullptr;
  StaticText* summarySystem = nullptr;
  StaticText* summaryDelta = nullptr;

  bool qualityAnnounced = false;
  bool hasStableBaseline = false;
  uint8_t stableSamples = 0;
  uint8_t invalidSamples = 0;
  uint8_t baselineCells = 0;
  uint16_t baselineCellMv[6] = {0};
  tmr10ms_t batteryInsertedTick = 0;

  void buildHeader()
  {
    auto* hdr = new Window(this, {0, DASH_MARGIN, LCD_W, HDR_H});
    lockObjectInteractions(hdr->getLvObj());
    lv_obj_set_style_bg_opa(hdr->getLvObj(), LV_OPA_TRANSP, LV_PART_MAIN);

    hdrDecorLeft =
      new StaticText(hdr,
                     {LEFT_X, 2, 36, EdgeTxStyles::STD_FONT_HEIGHT},
                     "//>", COLOR_THEME_PRIMARY2_INDEX, FONT(STD));
    hdrTitle = new StaticText(
        hdr,
        {LEFT_X + 40, 0, LCD_W - 80 - LEFT_X * 2, HDR_H}, TITLE_VOLT_MON,
        COLOR_THEME_PRIMARY2_INDEX, CENTERED | FONT(L));
    hdrDecorRight =
      new StaticText(hdr,
                     {LCD_W - LEFT_X - 36, 2, 36, EdgeTxStyles::STD_FONT_HEIGHT},
                     "<//", COLOR_THEME_PRIMARY2_INDEX, RIGHT | FONT(STD));

    lockObjectInteractions(hdrDecorLeft->getLvObj());
    lockObjectInteractions(hdrTitle->getLvObj());
    lockObjectInteractions(hdrDecorRight->getLvObj());
    setLabelRgb(hdrDecorLeft, colElecBlue());
    setLabelRgb(hdrDecorRight, colElecBlue());
    setLabelRgb(hdrTitle, colElecBlue());
  }

  void buildLeftSummary()
  {
    const coord_t h = LCD_H - MAIN_TOP - FOOTER_H - DASH_MARGIN;
    leftCard = new Window(this, {LEFT_X, MAIN_TOP, LEFT_W, h});
    lockObjectInteractions(leftCard->getLvObj());
    styleCard(leftCard->getLvObj());

    coord_t y = 4;
    lblTotalTitle =
      new StaticText(leftCard, {4, y, LEFT_W - 8, 14}, LBL_TOTAL_V,
                     COLOR_THEME_PRIMARY2_INDEX, FONT(XS));
    y += 14;
    lblTotalVolts =
      new StaticText(leftCard, {4, y, LEFT_W - 8, 36}, "--.-- V",
                     COLOR_THEME_PRIMARY2_INDEX, FONT(XL));
    y += 38;
    lblTotalPct =
      new StaticText(leftCard, {4, y, LEFT_W - 8, 22}, "--%",
                     COLOR_THEME_PRIMARY2_INDEX, FONT(L));
    y += 26;
    lblCellRange =
      new StaticText(leftCard, {4, y, LEFT_W - 8, 14}, LBL_CELL_RANGE,
                     COLOR_THEME_PRIMARY2_INDEX, FONT(XS));
    y += 16;
    lblCellRangeDetail =
      new StaticText(leftCard, {4, y, LEFT_W - 8, 28}, "--.--V ~ --.--V",
                     COLOR_THEME_PRIMARY2_INDEX, FONT(STD));
    y += 32;

    lblRefPack =
      new StaticText(leftCard, {4, y, LEFT_W - 8, 28},
                     "SOC ref (6S eq.): \n19.8V-25.2V",
                     COLOR_THEME_PRIMARY2_INDEX, FONT(XS));
    y += 32;
    lblRefCell = new StaticText(
        leftCard, {4, y, LEFT_W - 8, 28},
        "SOC ref (cell): \n3.30V-4.20V", COLOR_THEME_PRIMARY2_INDEX, FONT(XS));

    lockObjectInteractions(lblTotalTitle->getLvObj());
    lockObjectInteractions(lblTotalVolts->getLvObj());
    lockObjectInteractions(lblTotalPct->getLvObj());
    lockObjectInteractions(lblCellRange->getLvObj());
    lockObjectInteractions(lblCellRangeDetail->getLvObj());
    lockObjectInteractions(lblRefPack->getLvObj());
    lockObjectInteractions(lblRefCell->getLvObj());
    setLabelRgb(lblTotalTitle, colMuted());
    setLabelRgb(lblTotalVolts, colNeonGreen());
    setLabelRgb(lblTotalPct, colNeonGreen());
    setLabelRgb(lblCellRange, colElecBlue());
    setLabelRgb(lblCellRangeDetail, colNeonGreen());
    setLabelRgb(lblRefPack, colMuted());
    setLabelRgb(lblRefCell, colMuted());
  }

  void buildCellColumns()
  {
    cellStrip =
      new Window(this, {RIGHT_X, MAIN_TOP, RIGHT_W, CELL_AREA_H});
    lockObjectInteractions(cellStrip->getLvObj());
    lv_obj_set_style_bg_opa(cellStrip->getLvObj(), LV_OPA_TRANSP, LV_PART_MAIN);

    auto* stripTitle =
      new StaticText(cellStrip, {0, 0, RIGHT_W, 14}, LBL_CELL_PANEL,
                     COLOR_THEME_PRIMARY2_INDEX, FONT(XS));
    lockObjectInteractions(stripTitle->getLvObj());
    setLabelRgb(stripTitle, colElecBlue());

    const coord_t cellW = (RIGHT_W - 8) / 6;
    const coord_t rowTop = 18;
    constexpr coord_t CELL_VOLT_Y = 0;
    constexpr coord_t CELL_VOLT_H = 14;
    constexpr coord_t CELL_BAR_Y = 18;
    constexpr coord_t CELL_IDX_H = 13;
    constexpr coord_t CELL_ROW_GAP = 2;
    constexpr coord_t CELL_IDX_TO_PCT_GAP = 1;
    constexpr coord_t CELL_PCT_UP = 2;
    constexpr coord_t CELL_IDX_PCT_UP = 2;
    constexpr coord_t CELL_CONTENT_UP =
        2; /* shift volt / bar / idx / pct inside card; rowTop unchanged */

    const coord_t voltTop = (coord_t)(CELL_VOLT_Y - CELL_CONTENT_UP);
    const coord_t barTop = (coord_t)(CELL_BAR_Y - CELL_CONTENT_UP);

    for (uint8_t i = 0; i < 6; ++i) {
      const coord_t x = 4 + static_cast<coord_t>(i) * cellW;
      cellCards[i] =
        new Window(cellStrip, {x, rowTop, cellW - 2, CELL_AREA_H - rowTop});
      lockObjectInteractions(cellCards[i]->getLvObj());
      styleCard(cellCards[i]->getLvObj());

      lv_obj_t* cardObj = cellCards[i]->getLvObj();

      cellVolt[i] = new StaticText(
          cellCards[i],
          {0, voltTop, cellW - 4, CELL_VOLT_H}, "--",
          COLOR_THEME_PRIMARY2_INDEX, CENTERED | FONT(STD));

      lv_coord_t barLeft =
          (lv_coord_t)((cellW - 2 - BAR_TRACK_W) / 2);
      cellBarTrack[i] = lv_obj_create(cardObj);
      lv_obj_set_size(cellBarTrack[i], BAR_TRACK_W, BAR_TRACK_H);
      lv_obj_set_pos(cellBarTrack[i], barLeft, barTop);
      lv_obj_set_style_bg_color(cellBarTrack[i], colDashBg(), LV_PART_MAIN);
      lv_obj_set_style_border_color(cellBarTrack[i], colCardBorder(),
                                    LV_PART_MAIN);
      lv_obj_set_style_border_width(cellBarTrack[i], BAR_BORDER_W, LV_PART_MAIN);
      lv_obj_set_style_radius(cellBarTrack[i], 2, LV_PART_MAIN);
      lv_obj_clear_flag(cellBarTrack[i], LV_OBJ_FLAG_SCROLLABLE);

      cellBarFill[i] = lv_obj_create(cellBarTrack[i]);
      lv_obj_set_width(cellBarFill[i], BAR_FILL_W);
      lv_obj_set_style_radius(cellBarFill[i], 1, LV_PART_MAIN);
      lv_obj_set_style_border_width(cellBarFill[i], 0, LV_PART_MAIN);
      lv_obj_align(cellBarFill[i], LV_ALIGN_BOTTOM_MID, 0,
                   -(BAR_BORDER_W + BAR_FILL_MARGIN));
      lv_obj_clear_flag(cellBarFill[i], LV_OBJ_FLAG_SCROLLABLE);

      const coord_t idxY = (coord_t)(barTop + BAR_TRACK_H + CELL_ROW_GAP -
                                      CELL_IDX_PCT_UP);
      cellIdx[i] = new StaticText(cellCards[i],
                                  {0, idxY, cellW - 4, CELL_IDX_H}, "",
                                  COLOR_THEME_PRIMARY2_INDEX,
                                  CENTERED | FONT(XS));

      cellPct[i] = new StaticText(
          cellCards[i],
          {0,
           (coord_t)(idxY + CELL_IDX_H + CELL_IDX_TO_PCT_GAP - CELL_PCT_UP),
           cellW - 4, 14},
          "--%", COLOR_THEME_PRIMARY2_INDEX, CENTERED | FONT(XS));

      lockObjectInteractions(cellIdx[i]->getLvObj());
      lockObjectInteractions(cellVolt[i]->getLvObj());
      lockObjectInteractions(cellPct[i]->getLvObj());
      setLabelRgb(cellIdx[i], colMuted());
    }
  }

  void buildPackBarSection()
  {
    const coord_t y = MAIN_TOP + CELL_AREA_H + 4;
    packStrip = new Window(this, {RIGHT_X, y, RIGHT_W, PACK_SECTION_H});
    lockObjectInteractions(packStrip->getLvObj());
    styleCard(packStrip->getLvObj());

    lblPackTitle =
      new StaticText(packStrip, {4, 2, RIGHT_W - 8, 14}, LBL_PACK_BAR,
                     COLOR_THEME_PRIMARY2_INDEX, FONT(XS));
    lockObjectInteractions(lblPackTitle->getLvObj());
    setLabelRgb(lblPackTitle, colElecBlue());

    packBarBg = lv_obj_create(packStrip->getLvObj());
    lv_obj_set_size(packBarBg, RIGHT_W - 80, 14);
    lv_obj_set_pos(packBarBg, 4, 22);
    lv_obj_set_style_radius(packBarBg, 6, LV_PART_MAIN);
    lv_obj_set_style_bg_color(packBarBg, colNeonGreen(), LV_PART_MAIN);
    lv_obj_set_style_bg_grad_color(packBarBg, lv_color_hex(0xff2200),
                                   LV_PART_MAIN);
    lv_obj_set_style_bg_grad_dir(packBarBg, LV_GRAD_DIR_HOR, LV_PART_MAIN);
    lv_obj_clear_flag(packBarBg, LV_OBJ_FLAG_SCROLLABLE);

    packMarker = lv_obj_create(packBarBg);
    lv_obj_set_size(packMarker, 7, 10);
    lv_obj_set_style_radius(packMarker, 0, LV_PART_MAIN);
    lv_obj_set_style_bg_color(packMarker, lv_color_white(), LV_PART_MAIN);
    lv_obj_set_style_border_width(packMarker, 0, LV_PART_MAIN);
    lv_obj_align(packMarker, LV_ALIGN_TOP_MID, 0, -11);

    lblPackPctBig =
      new StaticText(packStrip,
                     {(coord_t)(RIGHT_W - 68), 18, 60, 22}, "--%",
                     COLOR_THEME_PRIMARY2_INDEX, RIGHT | FONT(L));
    lblPackScale = new StaticText(
        packStrip, {4, (coord_t)(38), RIGHT_W - 8, 14},
        "8.4V 12.6V 16.8V 21.0V 25.2V", COLOR_THEME_PRIMARY2_INDEX,
        FONT(XS));

    lockObjectInteractions(lblPackPctBig->getLvObj());
    lockObjectInteractions(lblPackScale->getLvObj());
    setLabelRgb(lblPackPctBig, colNeonGreen());
    setLabelRgb(lblPackScale, colMuted());
  }

  void buildFooter()
  {
    const coord_t fy = LCD_H - FOOTER_H - DASH_MARGIN;
    auto* footerPanel =
        new Window(this, {DASH_MARGIN, fy, LCD_W - 2 * DASH_MARGIN, FOOTER_H});
    lockObjectInteractions(footerPanel->getLvObj());
    lv_obj_set_style_bg_opa(footerPanel->getLvObj(), LV_OPA_TRANSP,
                            LV_PART_MAIN);

    const coord_t shiftLeft =
        static_cast<coord_t>(getTextWidth("888888888888", 12, FONT(XS)));
    const coord_t shiftDvVbat =
        shiftLeft -
        static_cast<coord_t>(getTextWidth("88", 2, FONT(XS)));
    const coord_t shiftVbatRight =
        static_cast<coord_t>(getTextWidth("888888", 6, FONT(XS)));
    const coord_t wFoot = footerPanel->width();
    const coord_t packFootW =
        (coord_t)(121 - shiftDvVbat);  /* up to 1px before dV column */

    summaryCells =
      new StaticText(footerPanel,
                     {0, 4, 72, EdgeTxStyles::STD_FONT_HEIGHT}, "",
                     COLOR_THEME_PRIMARY2_INDEX, FONT(XS));
    summaryPack =
      new StaticText(footerPanel,
                     {76, 4, packFootW, EdgeTxStyles::STD_FONT_HEIGHT}, "",
                     COLOR_THEME_PRIMARY2_INDEX, FONT(XS));
    summarySystem =
      new StaticText(
          footerPanel,
          {(coord_t)(wFoot - 104 - shiftDvVbat - FOOTER_VBAT_EXTRA_W +
                     shiftVbatRight),
           4,
           (coord_t)(102 + FOOTER_VBAT_EXTRA_W),
           EdgeTxStyles::STD_FONT_HEIGHT},
          "", COLOR_THEME_PRIMARY2_INDEX, RIGHT | FONT(XS));
    summaryDelta =
      new StaticText(
          footerPanel,
          {(coord_t)(198 - shiftDvVbat), 4,
           (coord_t)(wFoot - 198 - 108 - FOOTER_VBAT_EXTRA_W),
           EdgeTxStyles::STD_FONT_HEIGHT},
          "", COLOR_THEME_PRIMARY2_INDEX, CENTERED | FONT(XS));

    lv_label_set_long_mode(summarySystem->getLvObj(), LV_LABEL_LONG_CLIP);
    lv_label_set_long_mode(summaryPack->getLvObj(), LV_LABEL_LONG_CLIP);

    lockObjectInteractions(summaryCells->getLvObj());
    lockObjectInteractions(summaryPack->getLvObj());
    lockObjectInteractions(summarySystem->getLvObj());
    lockObjectInteractions(summaryDelta->getLvObj());

    setLabelRgb(summaryCells, colMuted());
    setLabelRgb(summaryPack, colMuted());
    setLabelRgb(summarySystem, colMuted());
    setLabelRgb(summaryDelta, colMuted());
  }

  static uint8_t voltageToPercent(uint16_t voltage)
  {
    if (voltage <= BATSENSER_MIN_MV) {
      return 0;
    }
    if (voltage >= BATSENSER_FULL_MV) {
      return 100;
    }
    return (uint8_t)(((voltage - BATSENSER_MIN_MV) * 100) /
                     (BATSENSER_FULL_MV - BATSENSER_MIN_MV));
  }

  void resetQualityStability()
  {
    hasStableBaseline = false;
    stableSamples = 0;
    baselineCells = 0;
    invalidSamples = 0;
  }

  bool isReasonableMeasurement(uint8_t cells, uint16_t minCell,
                                  uint16_t maxCell) const
  {
    return (cells > 0 && minCell >= BATSENSER_ANALYZE_MIN_MV &&
            maxCell <= BATSENSER_FULL_MV + 200);
  }

  bool isMeasurementStable(const uint16_t* cellVoltages, uint8_t cells)
  {
    if (!hasStableBaseline || baselineCells != cells) {
      for (uint8_t i = 0; i < cells; ++i) {
        baselineCellMv[i] = cellVoltages[i];
      }
      baselineCells = cells;
      hasStableBaseline = true;
      stableSamples = 1;
      return false;
    }

    bool stable = true;
    for (uint8_t i = 0; i < cells; ++i) {
      int16_t diff = static_cast<int16_t>(cellVoltages[i]) -
                     static_cast<int16_t>(baselineCellMv[i]);
      if (diff < 0) {
        diff = -diff;
      }
      if (diff > static_cast<int16_t>(BATSENSER_STABLE_JITTER_MV)) {
        stable = false;
      }
      baselineCellMv[i] = cellVoltages[i];
    }

    if (stable) {
      if (stableSamples < 255) {
        ++stableSamples;
      }
    } else {
      stableSamples = 1;
    }

    return (stableSamples >= BATSENSER_STABLE_SAMPLE_COUNT);
  }

  bool playBatteryResultFile(const char* wavName)
  {
    auto tryPlayPath = [](const char* fullPath) {
      STOP_PLAY(BATSENSER_ANNOUNCE_PLAY_ID);
      PLAY_FILE(fullPath, 0, BATSENSER_ANNOUNCE_PLAY_ID);
      return IS_PLAYING(BATSENSER_ANNOUNCE_PLAY_ID);
    };

    char path[AUDIO_FILENAME_MAXLEN + 1];
    char* str = getAudioPath(path);
    strcpy(str, SYSTEM_SUBDIR "/");
    strcat(path, wavName);
    if (tryPlayPath(path)) {
      return true;
    }
    return false;
  }

  void analyzeAndAnnounceQuality(uint16_t minCell, uint16_t delta)
  {
    if (qualityAnnounced) {
      return;
    }

    bool played = false;
    if (minCell < BATSENSER_QUALITY_LOW_MV) {
      played = playBatteryResultFile("batterlow.wav");
    } else if (delta >= BATSENSER_QUALITY_DELTA_MV) {
      played = playBatteryResultFile("batternc.wav");
    } else {
      played = playBatteryResultFile("battergood.wav");
    }

    if (played) {
      qualityAnnounced = true;
    }
  }

  void refreshValues()
  {
    char line[40] = {0};
    uint16_t cellVoltages[6] = {0};

    uint8_t cells = v15BatteryDetectedCells();
    uint16_t minCell = 0xFFFF;
    uint16_t maxCell = 0;
    uint32_t totalVoltageMv = 0;
    uint32_t sumCellPct = 0;

    for (uint8_t i = 1; i <= 6; ++i) {
      const uint8_t idx = i - 1;

      snprintf(line, sizeof(line), "%u", static_cast<unsigned>(i));
      cellIdx[idx]->setText(line);

      if (i <= cells) {
        uint16_t voltage = v15BatteryCellVoltage(i);
        cellVoltages[idx] = voltage;
        uint8_t percent = voltageToPercent(voltage);
        sumCellPct += static_cast<uint32_t>(percent);
        lv_color_t barCol = cellBarColor(voltage);

        if (voltage < minCell) {
          minCell = voltage;
        }
        if (voltage > maxCell) {
          maxCell = voltage;
        }
        totalVoltageMv += voltage;

        uint16_t voltageCv = static_cast<uint16_t>((voltage + 5) / 10);
        snprintf(line, sizeof(line), "%u.%02uV", voltageCv / 100,
                 voltageCv % 100);
        cellVolt[idx]->setText(line);
        setLabelRgb(cellVolt[idx], barCol);

        snprintf(line, sizeof(line), "%u%%", static_cast<unsigned>(percent));
        cellPct[idx]->setText(line);
        setLabelRgb(cellPct[idx], barCol);

        lv_coord_t fillH =
            (lv_coord_t)((BAR_FILL_ZONE_H * percent) / 100);
        if (fillH < 2) {
          fillH = (percent > 0) ? 2 : 1;
        }
        lv_obj_set_height(cellBarFill[idx], fillH);
        lv_obj_set_style_bg_color(cellBarFill[idx], barCol, LV_PART_MAIN);
        lv_obj_set_style_bg_opa(cellBarFill[idx], LV_OPA_COVER, LV_PART_MAIN);
      } else {
        cellVolt[idx]->setText("--");
        cellPct[idx]->setText("--");
        setLabelRgb(cellVolt[idx], colMuted());
        setLabelRgb(cellPct[idx], colMuted());
        lv_obj_set_height(cellBarFill[idx], 2);
        lv_obj_set_style_bg_color(cellBarFill[idx], colCardBorder(),
                                  LV_PART_MAIN);
      }
    }

    if (cells > 0) {
      uint32_t totalVoltageCv = (totalVoltageMv + 5) / 10;
      snprintf(line, sizeof(line), "%lu.%02lu V",
               static_cast<unsigned long>(totalVoltageCv / 100),
               static_cast<unsigned long>(totalVoltageCv % 100));
      lblTotalVolts->setText(line);

      uint8_t packPct = static_cast<uint8_t>(
          (sumCellPct + static_cast<uint32_t>(cells) / 2U) /
          static_cast<uint32_t>(cells));
      snprintf(line, sizeof(line), "%u%%", static_cast<unsigned>(packPct));
      lblTotalPct->setText(line);
      lblPackPctBig->setText(line);
      setLabelRgb(lblTotalVolts, colNeonGreen());
      setLabelRgb(lblTotalPct, colNeonGreen());
      setLabelRgb(lblPackPctBig, colNeonGreen());

      if (minCell != 0xFFFF && maxCell >= minCell) {
        uint16_t minCv = static_cast<uint16_t>((minCell + 5) / 10);
        uint16_t maxCv = static_cast<uint16_t>((maxCell + 5) / 10);
        snprintf(line, sizeof(line), "%u.%02uV ~ %u.%02uV", minCv / 100,
                 minCv % 100, maxCv / 100, maxCv % 100);
      } else {
        snprintf(line, sizeof(line), "--.--V ~ --.--V");
      }
      lblCellRangeDetail->setText(line);
      setLabelRgb(lblCellRangeDetail, colNeonGreen());

      const lv_coord_t barW = lv_obj_get_width(packBarBg);
      const lv_coord_t mx =
          4 + (lv_coord_t)((barW - 12) * packPct / 100);
      lv_obj_set_x(packMarker, mx);
    } else {
      lblTotalVolts->setText("--.-- V");
      lblTotalPct->setText("--%");
      lblPackPctBig->setText("--%");
      lblCellRangeDetail->setText("--.--V ~ --.--V");
      lv_obj_set_x(packMarker, 4);
      setLabelRgb(lblTotalVolts, colMuted());
      setLabelRgb(lblTotalPct, colMuted());
      setLabelRgb(lblPackPctBig, colMuted());
      setLabelRgb(lblCellRangeDetail, colMuted());
    }

    uint16_t sys = v15BatterySystemVoltage();
    int16_t sysCurrent = v15BatterySystemCurrent();
    uint16_t sysCurrentAbs =
      (sysCurrent < 0) ? static_cast<uint16_t>(-sysCurrent)
                       : static_cast<uint16_t>(sysCurrent);
    uint16_t delta = 0;
    bool deltaValid = (cells >= 2 && minCell != 0xFFFF);
    if (deltaValid) {
      delta = maxCell - minCell;
    }

    snprintf(line, sizeof(line), "%u/6S", static_cast<unsigned>(cells));
    summaryCells->setText(line);

    if (cells > 0) {
      uint32_t totalVoltageCv = (totalVoltageMv + 5) / 10;
      snprintf(line, sizeof(line), "%lu.%02luV",
               static_cast<unsigned long>(totalVoltageCv / 100),
               static_cast<unsigned long>(totalVoltageCv % 100));
    } else {
      snprintf(line, sizeof(line), "--.--V");
    }
    summaryPack->setText(line);

    uint16_t sysCv = static_cast<uint16_t>((sys + 5) / 10);
    snprintf(line, sizeof(line), "VBAT %u.%02uV %s%u.%02uA", sysCv / 100,
             sysCv % 100, (sysCurrent < 0) ? "-" : " ",
             sysCurrentAbs / 1000, (sysCurrentAbs % 1000) / 10);
    summarySystem->setText(line);

    if (deltaValid) {
      uint16_t deltaCv = static_cast<uint16_t>((delta + 5) / 10);
      snprintf(line, sizeof(line), "dV %u.%02uV", deltaCv / 100,
               deltaCv % 100);
      setLabelRgb(summaryDelta, delta >= BATSENSER_DELTA_WARN_MV ? colDanger()
                                                                 : colWarn());
    } else {
      snprintf(line, sizeof(line), "dV --");
      setLabelRgb(summaryDelta, colMuted());
    }
    summaryDelta->setText(line);

    bool announceDelayElapsed =
      ((tmr10ms_t)(get_tmr10ms() - batteryInsertedTick) >=
       BATSENSER_ANNOUNCE_DELAY_10MS);

    bool hasAnyCellMeasurement = (cells > 0 && minCell != 0xFFFF);
    bool deltaAnomaly = (deltaValid && delta >= BATSENSER_QUALITY_DELTA_MV);
    bool readyForQuality =
      announceDelayElapsed && hasAnyCellMeasurement;

    if (!qualityAnnounced && readyForQuality) {
      if (deltaAnomaly) {
        analyzeAndAnnounceQuality(minCell, delta);
        return;
      }

      if (isReasonableMeasurement(cells, minCell, maxCell)) {
        invalidSamples = 0;
        if (isMeasurementStable(cellVoltages, cells)) {
          analyzeAndAnnounceQuality(minCell, deltaValid ? delta : 0);
        }
      } else {
        if (invalidSamples < 255) {
          ++invalidSamples;
        }
        if (invalidSamples >= BATSENSER_INVALID_HOLD_SAMPLES) {
          resetQualityStability();
        }
      }
    } else if (!qualityAnnounced) {
      if (!announceDelayElapsed) {
        resetQualityStability();
      }
    }
  }

  void onCancel() override { deleteLater(); }

  void checkEvents() override
  {
    Window::checkEvents();

    if (closeCondition && closeCondition()) {
      deleteLater();
    } else {
      refreshValues();
    }
  }
};

Battery6SDialog* s_dialog = nullptr;

}  // namespace

void v15BatteryUiTask(void)
{
  if (s_dialog && s_dialog->deleted()) {
    s_dialog = nullptr;
  }

  if (!v15BatteryCell1Present()) {
    if (s_dialog) {
      s_dialog->deleteLater();
      s_dialog = nullptr;
    }
    return;
  }

  if (!s_dialog && v15BatteryUiTriggerPending()) {
    s_dialog = new Battery6SDialog();
    s_dialog->setCloseHandler([]() { s_dialog = nullptr; });
    v15BatteryUiAcknowledgeTrigger();
  }
}

#else

void v15BatteryUiTask(void) {}

#endif
