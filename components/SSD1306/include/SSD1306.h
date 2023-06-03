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
* This file contains code to implement an interface to the SSD1306 display
* driver IC.
*
* Much of the code here is based on Adafruit SSD1306 and GFX Arduino library.
*
* Edit              Date/Ver     Edit Description
* ==============  ============  =====================================================
* Baoshi Zhu      2015/01/03    Original
* Stuart James    2019/12/15    Modified for single display
*
*/

#pragma once

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>

#include "FontInfo.h"

#define SSD1306_NONE          0  //!< not used
#define SSD1306_128x64        1  //!< 128x32 panel
#define SSD1306_128x32        2  //!< 128x64 panel

// Control byte
#define OLED_CONTROL_BYTE_CMD_SINGLE    0x00    // The following byte is a single command
#define OLED_CONTROL_BYTE_DATA_BYTE     0x40    // The following byte is a single data value
#define OLED_CONTROL_BYTE_DATA_STREAM   0x80    // The following bytes are a data stream

// Fundamental commands (pg.28)
#define SSD1306_SET_CONTRAST           0x81    // follow with 0x7F
#define SSD1306_DISPLAY_RAM            0xA4
#define SSD1306_DISPLAY_ALLON          0xA5
#define SSD1306_DISPLAY_NORMAL         0xA6
#define SSD1306_DISPLAY_INVERTED       0xA7
#define SSD1306_DISPLAY_OFF            0xAE
#define SSD1306_DISPLAY_ON             0xAF

// Addressing Command Table (pg.30)
#define SSD1306_SET_MEMORY_ADDR_MODE   0x20    // follow with 0x00 = HORZ mode = Behave like a KS108 graphic LCD
#define SSD1306_SET_COLUMN_RANGE       0x21    // can be used only in HORZ/VERT mode - follow with 0x00 and 0x7F = COL127
#define SSD1306_SET_PAGE_RANGE         0x22    // can be used only in HORZ/VERT mode - follow with 0x00 and 0x07 = PAGE7
#define SSD1306_DEACTIVATE_SCROLL      0x2E

// Hardware Config (pg.31)
#define SSD1306_SET_DISPLAY_START_LINE 0x40
#define SSD1306_SET_SEGMENT_REMAP      0xA1
#define SSD1306_SET_MUX_RATIO          0xA8    // follow with 0x3F = 64 MUX
#define SSD1306_SET_COM_SCAN_MODE      0xC8
#define SSD1306_SET_DISPLAY_OFFSET     0xD3    // follow with 0x00
#define SSD1306_SET_COM_PIN_MAP        0xDA    // follow with 0x12
#define SSD1306_NOP                    0xE3    // NOP

// Timing and Driving Scheme (pg.32)
#define SSD1306_SET_DISPLAY_CLK_DIV    0xD5    // follow with 0x80
#define SSD1306_SET_PRECHARGE          0xD9    // follow with 0xF1
#define SSD1306_SET_VCOMH_DESELCT      0xDB    // follow with 0x30

// Charge Pump (pg.62)
#define SSD1306_SET_CHARGE_PUMP        0x8D    // follow with 0x14

// external status
#define STATUS_DISPLAY_ON             0x01
#define STATUS_DISPLAY_INVERTED       0x04
#define STATUS_BUFFER_CHANGED         0x02

// ASCII characters that receive special processing
#define ASCII_BEL                     0x07
#define ASCII_BS                      0x08
#define ASCII_TAB                     0x09
#define ASCII_LF                      0x0A
#define ASCII_VTAB                    0x0B
#define ASCII_CR                      0x0D
#define ASCII_XON                     0x11
#define ASCII_XOFF                    0x13

#define DEFAULT_LINESPACE             2
#define DEFAULT_TABSTOP               4             // tab stop characters

////////////////////////////////////////////////////////////////////////////////

typedef struct _pos
{
  uint8_t x;
  uint8_t y;
} pos_t;


typedef struct _oled_i2c_ctx
{
  uint8_t status;
  uint8_t type;               // Panel type
  uint8_t *buffer;            // display buffer
  uint8_t width;              // panel width (128)
  uint8_t height;             // panel height (32 or 64)
  uint8_t linespace;          // line spacing, pixels between text lines
  uint8_t tabstop;            // calculated tab stop in pixels
  uint8_t tabs;               // tab stop in characters
  pos_t   textpos;
  uint8_t refresh_top;        // "Dirty" window
  uint8_t refresh_left;
  uint8_t refresh_right;
  uint8_t refresh_bottom;
  const FontInfo_t* Font;     // current font
} oled_i2c_ctx;


typedef enum
{
  SSD1306_COLOR_TRANSPARENT = -1, // Transparent (not drawing)
  SSD1306_COLOR_BLACK = 0,        // Black (pixel off)
  SSD1306_COLOR_WHITE,            // White (or blue, yellow, pixel on)
  SSD1306_COLOR_INVERT,           // Invert pixel (XOR)
} ssd1306_color_t;

////////////////////////////////////////////////////////////////////////////////

bool      DC_Init(uint8_t scl_pin, uint8_t sda_pin, uint8_t rst_pin, uint8_t i2c_address);
uint8_t   DC_GetStatus(uint8_t flags);                                                                                      // Returns the status flags
uint8_t   DC_GetWidth(void);                                                                                                // Return OLED panel width
uint8_t   DC_GetHeight(void);                                                                                               // Return OLED panel height
void      DC_On(bool flag);                                                                                                 // Display on/off
void      DC_ClearBuffer(void);                                                                                             // Clear display buffer (fill with black)
void      DC_Invert(bool invert);                                                                                           // Set normal or inverted display
void      DC_Refresh(bool force);                                                                                           // Refresh display (send display buffer to the panel)
void      DC_DrawPixel(int8_t x, int8_t y, ssd1306_color_t color);                                                          // Draw one pixel
void      DC_DrawHLine(int8_t x, int8_t y, uint8_t w, ssd1306_color_t color);                                               // Draw horizontal line
void      DC_DrawVLine(int8_t x, int8_t y, uint8_t h, ssd1306_color_t color);                                               // Draw vertical line
void      DC_DrawRectangle(int8_t x, int8_t y, uint8_t w, uint8_t h, ssd1306_color_t color);                                // Draw a rectangle
void      DC_DrawRectFilled(int8_t x, int8_t y, uint8_t w, uint8_t h, ssd1306_color_t color);                               // Draw a filled rectangle
void      DC_DrawCircle(int8_t x0, int8_t y0, uint8_t r, ssd1306_color_t color);                                            // Draw a circle
void      DC_DrawCircleFilled(int8_t x0, int8_t y0, uint8_t r, ssd1306_color_t color);                                      // Draw a filled circle
void      DC_SetTextPos(uint8_t x, uint8_t y);                                                                              // Set the text origin
void      DC_SelectFont(uint8_t fid);                                                                                       // Select font for drawing
uint8_t   DC_GetFontHeight(uint8_t fid);                                                                                    // Get the height of current selected font
uint8_t   DC_GetFontC(void);                                                                                                // Get the "C" value (space between adjacent characters)
uint8_t   DC_SetInterline(uint8_t lspace);                                                                                  // Set distance between bottom of current character line and top of next
void      DC_SetTabs(uint8_t tabset);                                                                                       // Set the tab increment in characters
uint8_t   DC_PutChar(uint8_t x, uint8_t y,unsigned char c, ssd1306_color_t foreground, ssd1306_color_t background);         // Draw one character using currently selected font
void      DC_PutString(const char *str, ssd1306_color_t foreground, ssd1306_color_t background);                            // Draw a string. note no checks are made
uint8_t   DC_PlaceString(uint8_t x, uint8_t y, const char *str, ssd1306_color_t foreground, ssd1306_color_t background);    // Print a string using currently selected font
uint8_t   DC_PrintString(ssd1306_color_t foreground, ssd1306_color_t background, const char *lpStrFmt, ...);                // Print a formated string at a specified origin
uint8_t   DC_MeasureString(char *str);                                                                                      // Measure width of string with current selected font
void      DC_BufferFill(uint8_t* data, uint16_t length);                                                                    // Direct update display buffer
