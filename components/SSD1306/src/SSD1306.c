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


#include <driver/gpio.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "Softi2c.h"
#include "SSD1306.h"


#define OLED_TYPE                   SSD1306_128x64
#define RESET_SETTLING_TIME         5                                           // milliseconds

oled_i2c_ctx *_ctxs = { NULL };

static uint8_t  SendCommand(uint8_t c);
static const char *TAG = "[SSDD]";

////////////////////////////////////////////////////////////////////////////////

bool DC_Init(uint8_t scl_pin, uint8_t sda_pin, uint8_t rst_pin, uint8_t i2c_address)
{
gpio_config_t io_conf;
oled_i2c_ctx *ctx = NULL;

  if(rst_pin < 255){
    io_conf.intr_type = GPIO_INTR_DISABLE;                                  // disable interrupt
    io_conf.mode = GPIO_MODE_OUTPUT;                                            // set as output mode
    io_conf.pin_bit_mask = (1ULL << rst_pin);                                   // bit mask of the pins
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;                               // disable pull-down mode
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;                                    // enable pull-up mode
    gpio_config(&io_conf);                                                      // configure GPIO with the given settings
    gpio_set_level(rst_pin, true);
    vTaskDelay(pdMS_TO_TICKS(RESET_SETTLING_TIME));
    gpio_set_level(rst_pin, false);
    vTaskDelay(pdMS_TO_TICKS(RESET_SETTLING_TIME));
    gpio_set_level(rst_pin, true);
  }
  Softi2cInit(i2c_address, scl_pin, sda_pin);
  ctx = malloc(sizeof(oled_i2c_ctx));
  if(ctx == NULL){
    ESP_LOGI(TAG, "Alloc display context failed.");
    goto oled_init_fail;
  }
  memset(ctx, 0, sizeof(oled_i2c_ctx));
  ctx->type = OLED_TYPE;
  ctx->buffer = malloc(1024); // 128 * 64 / 8
  ctx->width = 128;                                                             // width in pixels
  ctx->height = 64;                                                             // height in pixels
  ctx->linespace = DEFAULT_LINESPACE;                                           // default pixels between text lines
  ctx->tabs = DEFAULT_TABSTOP;                                                  // default character tab spacing
  if(ctx->buffer == NULL){
    ESP_LOGI(TAG, "Alloc display buffer failed.");
    goto oled_init_fail;
  }
  SendCommand(SSD1306_DISPLAY_OFF);                                             // SSD1306_DISPLAYOFF
  SendCommand(SSD1306_SET_DISPLAY_CLK_DIV);                                     // SSD1306_SETDISPLAYCLOCKDIV
  SendCommand(0x80);                                                            // Suggested value 0x80
  SendCommand(SSD1306_SET_MUX_RATIO);                                           // SSD1306_SETMULTIPLEX
  SendCommand(0x3f);                                                            // 1/64
  SendCommand(SSD1306_SET_DISPLAY_OFFSET);                                      // SSD1306_SETDISPLAYOFFSET
  SendCommand(0x00);                                                            // 0 no offset
  SendCommand(SSD1306_SET_DISPLAY_START_LINE);                                  // SSD1306_SETSTARTLINE line #0
  SendCommand(SSD1306_SET_CHARGE_PUMP);                                         // SSD1306_CHARGEPUMP
  SendCommand(0x14);                                                            // Charge pump on
  SendCommand(SSD1306_SET_MEMORY_ADDR_MODE);                                    // SSD1306_MEMORYMODE
  SendCommand(0x00);                                                            // 0x0 act like ks0108
  SendCommand(SSD1306_SET_SEGMENT_REMAP);                                       // SSD1306_SEGREMAP
  SendCommand(SSD1306_SET_COM_SCAN_MODE);                                       // SSD1306_COMSCANDEC
  SendCommand(SSD1306_SET_COM_PIN_MAP);                                         // SSD1306_SETCOMPINS
  SendCommand(0x12);
  SendCommand(SSD1306_SET_CONTRAST);                                            // SSD1306_SETCONTRAST
  SendCommand(0xcf);
  SendCommand(SSD1306_SET_PRECHARGE);                                           // SSD1306_SETPRECHARGE
  SendCommand(0xf1);
  SendCommand(SSD1306_SET_VCOMH_DESELCT);                                       // SSD1306_SETVCOMDETECT
  SendCommand(0x40);
  SendCommand(SSD1306_DEACTIVATE_SCROLL);                                       // SSD1306_DEACTIVATE_SCROLL
  SendCommand(SSD1306_DISPLAY_RAM);                                             // SSD1306_DISPLAYALLON_RESUME
  SendCommand(SSD1306_DISPLAY_NORMAL);                                          // SSD1306_NORMALDISPLAY
  SendCommand(SSD1306_DISPLAY_ON);                                              // SSD1306_DISPLAYON
  _ctxs = ctx;
  DC_SelectFont(0);
  DC_ClearBuffer();
  DC_Refresh(true);
  _ctxs->status = STATUS_DISPLAY_ON;
  return true;

oled_init_fail:
  if (ctx && ctx->buffer) free(ctx->buffer);
  if (ctx) free(ctx);
  ESP_LOGI(TAG, "Display driver initialisation failed.");
  return false;
}

////////////////////////////////////////////////////////////////////////////////

static inline uint8_t SendCommand(uint8_t c)
{
  Softi2cStartTransfer();                                             // includes address
  Softi2cSendByte(OLED_CONTROL_BYTE_CMD_SINGLE);                      // Co = 0, D/C = 0
  Softi2cSendByte(c);
  Softi2cEndTransfer();
  return 1;
}

////////////////////////////////////////////////////////////////////////////////

void DC_Refresh(bool force)
{
uint8_t i, j;
uint16_t k = 0;
uint8_t page_start, page_end;

  if(_ctxs == NULL) return;
  if(!(_ctxs->status & STATUS_BUFFER_CHANGED) && !force) return;
  if(force){
    if(_ctxs->type == SSD1306_128x64){
      SendCommand(SSD1306_SET_COLUMN_RANGE);                          // SSD1306_COLUMNADDR
      SendCommand(0);                                                 // column start
      SendCommand(127);                                               // column end
      SendCommand(SSD1306_SET_PAGE_RANGE);                            // SSD1306_PAGEADDR
      SendCommand(0);                                                 // page start
      SendCommand(7);                                                 // page end (8 pages for 64 rows OLED)
      while(k < 1024){
        Softi2cStartTransfer();
        Softi2cSendByte(OLED_CONTROL_BYTE_DATA_BYTE);
        for(j = 0; j < 16; ++j){
          Softi2cSendByte(_ctxs->buffer[k]);
          ++k;
        }
        Softi2cEndTransfer();
      }
    }
    else if(_ctxs->type == SSD1306_128x32){
      SendCommand(SSD1306_SET_COLUMN_RANGE);                          // SSD1306_COLUMNADDR
      SendCommand(0);                                                 // column start
      SendCommand(127);                                               // column end
      SendCommand( SSD1306_SET_PAGE_RANGE);                           // SSD1306_PAGEADDR
      SendCommand(0);                                                 // page start
      SendCommand(3);                                                 // page end (4 pages for 32 rows OLED)
      while(k < 512){
        Softi2cStartTransfer();
        Softi2cSendByte(OLED_CONTROL_BYTE_DATA_BYTE);
        for(j = 0; j < 16; ++j){
          Softi2cSendByte(_ctxs->buffer[k]);
          ++k;
        }
        Softi2cEndTransfer();
      }
    }
  }
  else{
    if((_ctxs->refresh_top <= _ctxs->refresh_bottom) && (_ctxs->refresh_left <= _ctxs->refresh_right)){
      page_start = _ctxs->refresh_top / 8;
      page_end = _ctxs->refresh_bottom / 8;
      SendCommand(SSD1306_SET_COLUMN_RANGE);                          // SSD1306_COLUMNADDR
      SendCommand(_ctxs->refresh_left);                               // column start
      SendCommand(_ctxs->refresh_right);                              // column end
      SendCommand(SSD1306_SET_PAGE_RANGE);                            // SSD1306_PAGEADDR
      SendCommand(page_start);                                        // page start
      SendCommand(page_end);                                          // page end
      for(i = page_start; i <= page_end; ++i){
        for(j = _ctxs->refresh_left; j <= _ctxs->refresh_right; ++j){
          if(k == 0){
            Softi2cStartTransfer();
            Softi2cSendByte(OLED_CONTROL_BYTE_DATA_BYTE);
          }
          Softi2cSendByte(_ctxs->buffer[i * _ctxs->width + j]);
          ++k;
          if(k == 16){
            Softi2cEndTransfer();
            k = 0;
          }
        }
      }
      if(k != 0) Softi2cEndTransfer();                                // for last batch if stop was not sent
    }
  }
  _ctxs->refresh_top = 255;                                           // reset dirty area
  _ctxs->refresh_left = 255;
  _ctxs->refresh_right = 0;
  _ctxs->refresh_bottom = 0;
  _ctxs->status &= ~STATUS_BUFFER_CHANGED;
}

////////////////////////////////////////////////////////////////////////////////

uint8_t DC_GetStatus(uint8_t flags)
{
  if(_ctxs == NULL) return 0;
  return (_ctxs->status & flags);
}

////////////////////////////////////////////////////////////////////////////////

uint8_t DC_GetWidth(void)
{
  if(_ctxs == NULL) return 0;
  return _ctxs->width;
}

////////////////////////////////////////////////////////////////////////////////

uint8_t DC_GetHeight(void)
{
  if(_ctxs == NULL) return 0;
  return _ctxs->height;
}

////////////////////////////////////////////////////////////////////////////////

void DC_On(bool flag)
{
  if(flag){
    SendCommand(SSD1306_DISPLAY_ON);
    _ctxs->status |= STATUS_DISPLAY_ON;
  }
  else{
    SendCommand(SSD1306_DISPLAY_OFF);
    _ctxs->status &= ~STATUS_DISPLAY_ON;
  }
}

////////////////////////////////////////////////////////////////////////////////

void DC_ClearBuffer(void)
{
  if(_ctxs == NULL) return;
  if(_ctxs->type == SSD1306_128x64){
    memset(_ctxs->buffer, 0, 1024);
  }
  else if(_ctxs->type == SSD1306_128x32){
    memset(_ctxs->buffer, 0, 512);
  }
  _ctxs->refresh_right = _ctxs->width - 1;
  _ctxs->refresh_bottom = _ctxs->height - 1;
  _ctxs->refresh_top = 0;
  _ctxs->refresh_left = 0;
  _ctxs->status |= STATUS_BUFFER_CHANGED;
  _ctxs->textpos.x = 0;
  _ctxs->textpos.y = 0;
}

////////////////////////////////////////////////////////////////////////////////

void DC_Invert(bool invert)
{
  if(invert){
    SendCommand(SSD1306_DISPLAY_INVERTED);
    _ctxs->status |= STATUS_DISPLAY_INVERTED;
  }
  else{
    SendCommand(SSD1306_DISPLAY_NORMAL);
    _ctxs->status &= ~STATUS_DISPLAY_INVERTED;
  }
}

////////////////////////////////////////////////////////////////////////////////

void DC_BufferFill(uint8_t *data, uint16_t length)                    // write directly to the screen buffer
{
  if(_ctxs == NULL) return;
  if(_ctxs->type == SSD1306_128x64){
    memcpy(_ctxs->buffer, data, (length < 1024) ? length : 1024);
  }
  else if(_ctxs->type == SSD1306_128x32){
    memcpy(_ctxs->buffer, data, (length < 512) ? length : 512);
  }
  _ctxs->refresh_top = 0;
  _ctxs->refresh_left = 0;
  _ctxs->refresh_right = _ctxs->width - 1;
  _ctxs->refresh_bottom = _ctxs->height - 1;
}

////////////////////////////////////////////////////////////////////////////////

void DC_SetPixel(int8_t x, int8_t y, ssd1306_color_t color)
{
uint16_t index;

  if(_ctxs == NULL) return;
  if((x >= _ctxs->width) || (x < 0) || (y >= _ctxs->height) || (y < 0)) return;
  index = x + (y / 8) * _ctxs->width;
  switch(color){
    case SSD1306_COLOR_WHITE:
      _ctxs->buffer[index] |= (1 << (y & 7));
      break;
    case SSD1306_COLOR_BLACK:
      _ctxs->buffer[index] &= ~(1 << (y & 7));
      break;
    case SSD1306_COLOR_INVERT:
      _ctxs->buffer[index] ^= (1 << (y & 7));
      break;
    default:
      break;
  }
  if(_ctxs->refresh_left > x) _ctxs->refresh_left = x;
  if(_ctxs->refresh_right < x) _ctxs->refresh_right = x;
  if(_ctxs->refresh_top > y) _ctxs->refresh_top = y;
  if(_ctxs->refresh_bottom < y) _ctxs->refresh_bottom = y;
  _ctxs->status |= STATUS_BUFFER_CHANGED;
}

////////////////////////////////////////////////////////////////////////////////

inline void DC_SetTextPos(uint8_t x, uint8_t y)
{
  if(_ctxs == NULL) return;
  _ctxs->textpos.x = x;
  _ctxs->textpos.y = y;
}

////////////////////////////////////////////////////////////////////////////////

void DC_SelectFont(uint8_t fid)
{

  if(_ctxs == NULL) return;
  if(fid < FONT_MAX) _ctxs->Font = InstalledFonts[fid];
  else _ctxs->Font = InstalledFonts[0];
  _ctxs->tabstop = _ctxs->tabs * _ctxs->Font->char_descriptors[' ' - _ctxs->Font->char_start].width;        // use the space character width as the tab width
//  ESP_LOGI(TAG, "Font:%d-%d.", _ctxs->Font->char_descriptors['e' - _ctxs->Font->char_start].width, _ctxs->tabstop);
}

////////////////////////////////////////////////////////////////////////////////

uint8_t DC_GetFontHeight(uint8_t fid)
{
  if(_ctxs == NULL) return 0;
  if(fid >= FONT_MAX){                                                // use current font
    if(_ctxs->Font == NULL) return 0;
    return _ctxs->Font->height;
  }
  return InstalledFonts[fid]->height;                                 // use specified font
}

////////////////////////////////////////////////////////////////////////////////

inline uint8_t DC_GetFontC(void)                                      // get pixel space between characters
{
  if(_ctxs == NULL) return 0;
  if(_ctxs->Font == NULL) return 0;
  return (_ctxs->Font->c);
}

////////////////////////////////////////////////////////////////////////////////

uint8_t DC_SetInterline(uint8_t lspace)                               // set number of pixel rows between text lines
{
  if(_ctxs == NULL) return 0;
  uint8_t oldlspace = _ctxs->linespace;
  _ctxs->linespace = lspace;
  return oldlspace;
}

////////////////////////////////////////////////////////////////////////////////

inline void DC_SetTabs(uint8_t tabset)                                // set number of tab characters
{
  if(_ctxs == NULL) return;
  _ctxs->tabs = tabset;
}

////////////////////////////////////////////////////////////////////////////////

uint8_t DC_PutChar(uint8_t x, uint8_t y, unsigned char c, ssd1306_color_t foreground, ssd1306_color_t background)
{
uint8_t i, j;
const uint8_t *bitmap;
uint8_t line = 0;

  if(_ctxs == NULL) return 0;
  if(_ctxs->Font == NULL) return 0;
  if((c < _ctxs->Font->char_start) || (c > _ctxs->Font->char_end)) c = ' ';   // replace with ' ' if not in the font set
  c = c - _ctxs->Font->char_start;                                            // c now becomes index to tables
  bitmap = _ctxs->Font->bitmap + _ctxs->Font->char_descriptors[c].offset;
  for(j = 0; j < _ctxs->Font->height; ++j){
    for(i = 0; i < _ctxs->Font->char_descriptors[c].width; ++i){
      if(i % 8 == 0){
        line = bitmap[(_ctxs->Font->char_descriptors[c].width + 7) / 8 * j + i / 8];  // line data
      }
      if(line & 0x80){
        DC_SetPixel(x + i, y + j, foreground);
      }
      else{
        switch(background){
        case SSD1306_COLOR_TRANSPARENT:
          break;                                                      // Not drawing for transparent background
        case SSD1306_COLOR_WHITE:
        case SSD1306_COLOR_BLACK:
          DC_SetPixel(x + i, y + j, background);
          break;
        case SSD1306_COLOR_INVERT:
           break;                                                     // I don't know why I need invert background
       }
      }
      line = line << 1;
    }
  }
  return (_ctxs->Font->char_descriptors[c].width);                    // return character width
}

////////////////////////////////////////////////////////////////////////////////

void DC_PutString(const char *str, ssd1306_color_t foreground, ssd1306_color_t background)
{
  while(*str){
    switch(*str){
      case ASCII_LF:
        _ctxs->textpos.x = 0;
        _ctxs->textpos.y += (_ctxs->Font->height + _ctxs->linespace);
        break;
      case ASCII_CR:
        _ctxs->textpos.x = 0;
        break;
      case ASCII_TAB:
        if((_ctxs->textpos.x / _ctxs->tabstop) < ((_ctxs->width - 1) / _ctxs->tabstop)){
          _ctxs->textpos.x += _ctxs->tabstop - _ctxs->textpos.x % _ctxs->tabstop;
        }
        break;
      default:                                                        // un-trapped control characters get replaced with ' '
        _ctxs->textpos.x += DC_PutChar(_ctxs->textpos.x, _ctxs->textpos.y, *str, foreground, background);
        if(*str) _ctxs->textpos.x += _ctxs->Font->c;
        break;
    }
    str++;
  }
}

////////////////////////////////////////////////////////////////////////////////

uint8_t DC_PrintString(ssd1306_color_t foreground, ssd1306_color_t background, const char *lpStrFmt, ...)
{
va_list  argptr;
char str[80];

  if(_ctxs == NULL) return 0;
  if(_ctxs->Font == NULL) return 0;
  va_start(argptr, lpStrFmt);                                         // Initialise va_ functions
  vsprintf(str, lpStrFmt, argptr);                                    // prints string to buffer
  va_end(argptr);                                                     // Close va_ functions
  DC_PutString(str, foreground, background);
  return _ctxs->textpos.x;
}

////////////////////////////////////////////////////////////////////////////////

uint8_t DC_PrintStringAt(uint8_t x, uint8_t y, ssd1306_color_t foreground, ssd1306_color_t background, const char *lpStrFmt, ...)
{
va_list  argptr;
char str[80];

  if(_ctxs == NULL) return 0;
  if(_ctxs->Font == NULL) return 0;
  _ctxs->textpos.x = x;
  _ctxs->textpos.y = y;
  va_start(argptr, lpStrFmt);                                         // Initialise va_ functions
  vsprintf(str, lpStrFmt, argptr);                                    // prints string to buffer
  va_end(argptr);                                                     // Close va_ functions
  DC_PutString(str, foreground, background);
  return _ctxs->textpos.x;
}

////////////////////////////////////////////////////////////////////////////////

uint8_t DC_MeasureString(char *str)
{
uint8_t w = 0;
unsigned char c;

  if(_ctxs == NULL) return 0;
  if(_ctxs->Font == NULL) return 0;
  while(*str){
    c = *str;
    if((c < _ctxs->Font->char_start) || (c > _ctxs->Font->char_end)) c = ' ';     // replace with ' ' if not in the font set
    c = c - _ctxs->Font->char_start;                                              // c now become index to tables
    w += _ctxs->Font->char_descriptors[c].width;
    ++str;
    if(*str) w += _ctxs->Font->c;
  }
  return w;                                                           // return width of string
}

////////////////////////////////////////////////////////////////////////////////

void DC_DrawHLine(int8_t x, int8_t y, uint8_t w, ssd1306_color_t color)
{
uint16_t index;
uint8_t mask, t;

  if(_ctxs == NULL) return;
  if((x >= _ctxs->width) || (x < 0) || (y >= _ctxs->height) || (y < 0)) return;  // boundary check
  if(w == 0) return;
  if(x + w > _ctxs->width) w = _ctxs->width - x;
  t = w;
  index = x + (y / 8) * _ctxs->width;
  mask = 1 << (y & 7);
  switch(color){
    case SSD1306_COLOR_WHITE:
      while(t--){
        _ctxs->buffer[index] |= mask;
        ++index;
      }
      break;
    case SSD1306_COLOR_BLACK:
      mask = ~mask;
      while(t--){
        _ctxs->buffer[index] &= mask;
        ++index;
      }
      break;
    case SSD1306_COLOR_INVERT:
      while(t--){
        _ctxs->buffer[index] ^= mask;
        ++index;
      }
      break;
    default:
      break;
  }
  if(_ctxs->refresh_left > x) _ctxs->refresh_left = x;
  if(_ctxs->refresh_right < x + w - 1) _ctxs->refresh_right = x + w - 1;
  if(_ctxs->refresh_top > y) _ctxs->refresh_top = y;
  if(_ctxs->refresh_bottom < y) _ctxs->refresh_bottom = y;
  _ctxs->status |= STATUS_BUFFER_CHANGED;
}

////////////////////////////////////////////////////////////////////////////////

void DC_DrawVLine(int8_t x, int8_t y, uint8_t h, ssd1306_color_t color)
{
uint16_t index;
uint8_t mask, mod, t;

  if(_ctxs == NULL) return;
  if((x >= _ctxs->width) || (x < 0) || (y >= _ctxs->height) || (y < 0)) return;             // boundary check
  if(h == 0) return;
  if(y + h > _ctxs->height) h = _ctxs->height - y;
  t = h;
  index = x + (y / 8) * _ctxs->width;
  mod = y & 7;
  if(mod){                                                                                  // partial line that does not fit into byte at top
    mod = 8 - mod;                                                                          // Magic from Adafruit
    static const uint8_t premask[8] = {0x00, 0x80, 0xC0, 0xE0, 0xF0, 0xF8, 0xFC, 0xFE};
    mask = premask[mod];
    if(t < mod) mask &= (0xFF >> (mod - t));
    switch(color){
      case SSD1306_COLOR_WHITE:
        _ctxs->buffer[index] |= mask;
        break;
      case SSD1306_COLOR_BLACK:
        _ctxs->buffer[index] &= ~mask;
        break;
      case SSD1306_COLOR_INVERT:
        _ctxs->buffer[index] ^= mask;
        break;
      default:
        break;
    }
    if(t < mod) goto draw_vline_finish;
    t -= mod;
    index += _ctxs->width;
  }
  if(t >= 8){                                                         // byte aligned line at middle
    switch(color){
      case SSD1306_COLOR_WHITE:
        do{
          _ctxs->buffer[index] = 0xff;
          index += _ctxs->width;
          t -= 8;
        }
        while(t >= 8);
        break;
      case SSD1306_COLOR_BLACK:
        do{
          _ctxs->buffer[index] = 0x00;
          index += _ctxs->width;
          t -= 8;
        }
        while(t >= 8);
        break;
      case SSD1306_COLOR_INVERT:
        do{
          _ctxs->buffer[index] = ~_ctxs->buffer[index];
          index += _ctxs->width;
          t -= 8;
        }
        while(t >= 8);
        break;
      default:
        break;
    }
  }
  if(t){                                                              // partial line at bottom
    mod = t & 7;
    static const uint8_t postmask[8] = {0x00, 0x01, 0x03, 0x07, 0x0F, 0x1F, 0x3F, 0x7F};
    mask = postmask[mod];
    switch(color){
      case SSD1306_COLOR_WHITE:
        _ctxs->buffer[index] |= mask;
        break;
      case SSD1306_COLOR_BLACK:
        _ctxs->buffer[index] &= ~mask;
        break;
      case SSD1306_COLOR_INVERT:
        _ctxs->buffer[index] ^= mask;
        break;
      default:
        break;
    }
  }
draw_vline_finish:
  if(_ctxs->refresh_left > x) _ctxs->refresh_left = x;
  if(_ctxs->refresh_right < x) _ctxs->refresh_right = x;
  if(_ctxs->refresh_top > y) _ctxs->refresh_top = y;
  if(_ctxs->refresh_bottom < y + h - 1) _ctxs->refresh_bottom = y + h - 1;
  _ctxs->status |= STATUS_BUFFER_CHANGED;
  return;
}

////////////////////////////////////////////////////////////////////////////////

void DC_DrawRectangle(int8_t x, int8_t y, uint8_t w, uint8_t h, ssd1306_color_t color)
{
  DC_DrawHLine(x, y, w, color);
  DC_DrawHLine(x, y + h - 1, w, color);
  DC_DrawVLine(x, y, h, color);
  DC_DrawVLine(x + w - 1, y, h, color);
}

////////////////////////////////////////////////////////////////////////////////

void DC_DrawRectFilled(int8_t x, int8_t y, uint8_t w, uint8_t h, ssd1306_color_t color)
{                                                                     // Can be optimised?
uint8_t i;

  for(i = x; i < x + w; ++i) DC_DrawVLine(i, y, h, color);
}

////////////////////////////////////////////////////////////////////////////////

void DC_DrawCircle(int8_t x0, int8_t y0, uint8_t r, ssd1306_color_t color)
{                                                                     // Refer to http://en.wikipedia.org/wiki/Midpoint_circle_algorithm for the algorithm
int8_t x = r;
int8_t y = 1;
int16_t radius_err = 1 - x;

  if(_ctxs == NULL) return;
  if(r == 0) return;
  DC_SetPixel(x0 - r, y0, color);
  DC_SetPixel(x0 + r, y0, color);
  DC_SetPixel(x0, y0 - r, color);
  DC_SetPixel(x0, y0 + r, color);
  while(x >= y){
    DC_SetPixel(x0 + x, y0 + y, color);
    DC_SetPixel(x0 - x, y0 + y, color);
    DC_SetPixel(x0 + x, y0 - y, color);
    DC_SetPixel(x0 - x, y0 - y, color);
    if(x != y){                                                       // Otherwise the 4 drawings below are the same as above, causing problem when colour is INVERT
      DC_SetPixel(x0 + y, y0 + x, color);
      DC_SetPixel(x0 - y, y0 + x, color);
      DC_SetPixel(x0 + y, y0 - x, color);
      DC_SetPixel(x0 - y, y0 - x, color);
    }
    ++y;
    if(radius_err < 0){
      radius_err += 2 * y + 1;
    }
    else{
      --x;
      radius_err += 2 * (y - x + 1);
    }

  }
}

////////////////////////////////////////////////////////////////////////////////

void DC_DrawCircleFilled(int8_t x0, int8_t y0, uint8_t r, ssd1306_color_t color)
{
int8_t x = 1;
int8_t y = r;
int16_t radius_err = 1 - y;
int8_t x1;

  if(_ctxs == NULL) return;
  if(r == 0) return;
  DC_DrawVLine(x0, y0 - r, 2 * r + 1, color);                         // Centre vertical line
  while(y >= x){
    DC_DrawVLine(x0 - x, y0 - y, 2 * y + 1, color);
    DC_DrawVLine(x0 + x, y0 - y, 2 * y + 1, color);
    if(color != SSD1306_COLOR_INVERT){
      DC_DrawVLine(x0 - y, y0 - x, 2 * x + 1, color);
      DC_DrawVLine(x0 + y, y0 - x, 2 * x + 1, color);
    }
    ++x;
    if(radius_err < 0){
      radius_err += 2 * x + 1;
    }
    else{
      --y;
      radius_err += 2 * (x - y + 1);
    }
  }
  if(color == SSD1306_COLOR_INVERT){
    x1 = x;                                                           // Save where we stopped
    y = 1;
    x = r;
    radius_err = 1 - x;
    DC_DrawHLine(x0 + x1, y0, r - x1 + 1, color);
    DC_DrawHLine(x0 - r, y0, r - x1 + 1, color);
    while(x >= y){
      DC_DrawHLine(x0 + x1, y0 - y, x - x1 + 1, color);
      DC_DrawHLine(x0 + x1, y0 + y, x - x1 + 1, color);
      DC_DrawHLine(x0 - x, y0 - y, x - x1 + 1, color);
      DC_DrawHLine(x0 - x, y0 + y, x - x1 + 1, color);
      ++y;
      if(radius_err < 0){
        radius_err += 2 * y + 1;
      }
      else{
        --x;
        radius_err += 2 * (y - x + 1);
      }
    }
  }
}

