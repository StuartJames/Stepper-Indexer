/*
*              Copyright (c) 2019-2021 HydraSystems.
*
*  This software is copyrighted by and is the sole property of
*  HydraSystems.  All rights, title, ownership, or other interests
*  in the software remain the property of HydraSystems.
*  This software may only be used in accordance with the corresponding
*  license agreement.  Any unauthorised use, duplication, transmission,
*  distribution, or disclosure of this software is expressly forbidden.
*
*  This Copyright notice may not be removed or modified without prior
*  written consent of HydraSystems.
*
*  HydraSystems, reserves the right to modify this software without
*  notice.
*
* =====================================================================
*
* This file contains code to implement font display.
*
* Edit              Date/Ver     Edit Description
* ==============  ============  ==============================================
* Ersn Duchan     2009/07/24    Original
* Baoshi Zhu      2015/01/03    Modified
* Stuart James    2019/12/15    Modified for Heltec ESP32 WiFi Dev Card
*
*/

#pragma once

#include "stdint.h"

typedef struct _FontCharDesc
{
  uint8_t width;                                // Character width in pixel
  uint16_t offset;                              // Offset of this character in bitmap
} FontCharDesc_t;


typedef struct _FontInfo
{
    uint8_t height;                             // Character height in pixel, all characters have same height
    uint8_t c;                                  // Simulation of "C" width in TrueType term, the space between adjacent characters
    char char_start;                            // First character
    char char_end;                              // Last character
    const FontCharDesc_t* char_descriptors;     // descriptor for each character
    const uint8_t *bitmap;                      // Character bitmap
} FontInfo_t;


enum font_list_e{
  FONT_LCD_5x7 = 0,
  FONT_CALIBRI_8PT,
  FONT_CALIBRI_12PT,
  FONT_CALIBRI_18PT,
  FONT_SANSSERIF_8PT,
  FONT_SANSSERIF_12PT,
  FONT_SEVENSEG_10PT,
  FONT_TAHOMA_8PT,
  FONT_TAHOMA_12PT,
  FONT_VERDANA_8PT,
  FONT_VERDANA_12PT,
  FONT_VERDANA_18PT,
  FONT_MAX                             // Number of built-in fonts
} ;


extern const FontInfo_t * InstalledFonts[FONT_MAX];    // Built-in fonts
