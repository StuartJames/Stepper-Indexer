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
* Baoshi Zhu      2015/01/03    Original
* Stuart James    2019/12/15    Modified for Heltec ESP32 WiFi Dev Card
*
*/

#include "FontInfo.h"

extern const FontInfo_t LCD_5x7_FontInfo;
extern const FontInfo_t Calibri8ptFontInfo;
extern const FontInfo_t Calibri12ptFontInfo;
extern const FontInfo_t Calibri18ptFontInfo;
extern const FontInfo_t SansSerif8ptFontInfo;
extern const FontInfo_t SansSerif12ptFontInfo;
extern const FontInfo_t SevenSegment10ptFontInfo;
extern const FontInfo_t Tahoma8ptFontInfo;
extern const FontInfo_t Tahoma12ptFontInfo;
extern const FontInfo_t Verdana8ptFontInfo;
extern const FontInfo_t Verdana12ptFontInfo;
extern const FontInfo_t Verdana18ptFontInfo;

const FontInfo_t * InstalledFonts[FONT_MAX] =
{
    &LCD_5x7_FontInfo,
    &Calibri8ptFontInfo,
    &Calibri12ptFontInfo,
    &Calibri18ptFontInfo,
    &SansSerif8ptFontInfo,
    &SansSerif12ptFontInfo,
    &SevenSegment10ptFontInfo,
    &Tahoma8ptFontInfo,
    &Tahoma12ptFontInfo,
    &Verdana8ptFontInfo,
    &Verdana12ptFontInfo,
    &Verdana18ptFontInfo,
};
