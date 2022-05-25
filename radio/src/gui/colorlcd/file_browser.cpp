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

#include "file_browser.h"
#include "libopenui_file.h"
#include "font.h"

#include <list>
#include <string>

#define CELL_CTRL_DIR  LV_TABLE_CELL_CTRL_CUSTOM_1
#define CELL_CTRL_FILE LV_TABLE_CELL_CTRL_CUSTOM_2

static const char* getFullPath(const char* filename)
{
  static char full_path[FF_MAX_LFN + 1];
  f_getcwd((TCHAR*)full_path, FF_MAX_LFN);
  strcat(full_path, "/");
  strcat(full_path, filename);
  return full_path;
}

static const char* getCurrentPath()
{
  static char path[FF_MAX_LFN + 1];
  f_getcwd((TCHAR*)path, FF_MAX_LFN);
  return path;
}

static int scan_files(std::list<std::string>& files,
                      std::list<std::string>& directories)
{
  FILINFO fno;
  DIR dir;

  FRESULT res = f_opendir(&dir, "."); // Open the directory
  if (res != FR_OK) return -1;

  // read all entries
  bool firstTime = true;
  for (;;) {
    res = sdReadDir(&dir, &fno, firstTime);

    if (res != FR_OK || fno.fname[0] == 0)
      break; // Break on error or end of dir
    // if (strlen((const char*)fno.fname) > SD_SCREEN_FILE_LENGTH)
    //   continue;
    if (fno.fname[0] == '.' && fno.fname[1] != '.')
      continue; // Ignore hidden files under UNIX, but not ..

    if (fno.fattrib & AM_DIR) {
      directories.push_back((char*)fno.fname);
    } else {
      files.push_back((char*)fno.fname);
    }
  }

  directories.sort(compare_nocase);
  files.sort(compare_nocase);

  return 0;
}

FileBrowser::FileBrowser(Window* parent, const rect_t& rect, const char* dir) :
    TableField(parent, rect)
{
  setColumnCount(1);
  setColumnWidth(0, rect.w);

  f_chdir(dir);
}

void FileBrowser::setFileAction(FileAction fct) { fileAction = std::move(fct); }

void FileBrowser::refresh()
{
  std::list<std::string> files;
  std::list<std::string> directories;
  if (scan_files(files, directories) < 0) return;

  setRowCount(files.size() + directories.size());

  uint16_t row = 0;
  for (const auto& name: directories) {

    // LV_SYMBOL_DIRECTORY
    lv_table_set_cell_value(lvobj, row, 0, name.c_str());
    lv_table_add_cell_ctrl(lvobj, row, 0, CELL_CTRL_DIR);
    row++;
  }

  for (const auto& name: files) {
    // LV_SYMBOL_FILE
    lv_table_set_cell_value(lvobj, row, 0, name.c_str());
    lv_table_clear_cell_ctrl(lvobj, row, 0, CELL_CTRL_DIR);
    row++;
  }

  select(0, 0);
}

void FileBrowser::onPress(uint16_t row, uint16_t col)
{
  bool is_dir = lv_table_has_cell_ctrl(lvobj, row, col, CELL_CTRL_DIR);
  onPress(lv_table_get_cell_value(lvobj, row, col), is_dir);  
}

void FileBrowser::onDrawBegin(uint16_t row, uint16_t col, lv_obj_draw_part_dsc_t* dsc)
{
  lv_coord_t cell_left = lv_obj_get_style_pad_left(lvobj, LV_PART_ITEMS);
  dsc->label_dsc->ofs_x = getFontHeight(FONT(STD)) + cell_left;
}

void FileBrowser::onDrawEnd(uint16_t row, uint16_t col, lv_obj_draw_part_dsc_t* dsc)
{
  const char* sym = nullptr;
  if (lv_table_has_cell_ctrl(lvobj, row, 0, CELL_CTRL_DIR)) {
    // dir
    const char* dir = lv_table_get_cell_value(lvobj, row, 0);
    if (dir[0] == '.')
      sym = LV_SYMBOL_LEFT;
    else
      sym = LV_SYMBOL_DIRECTORY;
  } else {
    // file
    sym = LV_SYMBOL_FILE;
  }

  lv_area_t coords;
  lv_coord_t area_h = lv_area_get_height(dsc->draw_area);

  lv_coord_t cell_left = lv_obj_get_style_pad_left(lvobj, LV_PART_ITEMS);
  lv_coord_t font_h = getFontHeight(FONT(STD));

  coords.x1 = dsc->draw_area->x1 + cell_left;
  coords.x2 = coords.x1 + dsc->label_dsc->ofs_x - cell_left;
  coords.y1 = dsc->draw_area->y1 + (area_h - font_h) / 2;
  coords.y2 = coords.y1 + font_h - 1;

  dsc->label_dsc->ofs_x = 0;
  lv_draw_label(dsc->draw_ctx, dsc->label_dsc, &coords, sym, nullptr);
}

void FileBrowser::onPress(const char* name, bool is_dir)
{
  const char* path = getCurrentPath();
  const char* fullpath = getFullPath(name);  
  if (is_dir) {
    f_chdir(fullpath);
    refresh();
  } else if (fileAction){
    fileAction(path, name, fullpath);
  }
}