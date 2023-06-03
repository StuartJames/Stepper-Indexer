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
* This file contains code to implement a i2c interface.
*
* Edit              Date/Ver     Edit Description
* ==============  ============  ==============================================
* Baoshi Zhu      2015/01/03    Original
* Stuart James    2019/12/15    Modified for Heltec ESP32 WiFi Dev Card
*
*/

#include "Softi2c.h"
#include "rom/ets_sys.h"

//! Delay amount in-between bits, with os_delay_us(1) I get ~300kHz I2C clock
#define _DELAY_S ets_delay_us(2)
#define _DELAY_L ets_delay_us(10)

static uint8_t Address;
static uint8_t g_scl_pin;
static uint8_t g_sda_pin;
static bool i2c_started;

static void Softi2c_Init(void);
static void Softi2c_Write_byte(uint8_t b);
static void Softi2c_Read_bit(void);
static void Softi2c_Write_bit(uint8_t val);
static void Softi2c_Stop(void);
static void Softi2c_Start(void);
static inline void Softi2c_Clear_sda(void) SSD1306_ALWAYSINLINE;
static inline void Softi2c_Read_sda(void) SSD1306_ALWAYSINLINE;
static inline void Softi2c_Clear_scl(void) SSD1306_ALWAYSINLINE;
static inline void Softi2c_Read_scl_delay(void) SSD1306_ALWAYSINLINE;
static void Softi2c_DelayShort(void) SSD1306_NOINLINE;
static void Softi2c_DelayLong(void) SSD1306_NOINLINE;

/* software i2c, ignores ACK response (which is anyway not provided by some displays) also does not allow reading from the device */

////////////////////////////////////////////////////////////////////////////////

void Softi2cInit(uint8_t address, uint8_t scl_pin, uint8_t sda_pin)
{
gpio_config_t io_conf;

  Address = address;
  g_scl_pin = scl_pin;
  g_sda_pin = sda_pin;
  io_conf.intr_type = GPIO_INTR_DISABLE;                                    // disable interrupt
  io_conf.mode = GPIO_MODE_INPUT_OUTPUT_OD;                                     // set as output mode
  io_conf.pin_bit_mask = ((1ULL << g_scl_pin) | (1ULL << g_sda_pin));           // bit mask of the pins
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;                                 // disable pull-down mode
  io_conf.pull_up_en = GPIO_PULLUP_ENABLE;                                      // enable pull-up mode
  gpio_config(&io_conf);                                                        // configure GPIO with the given settings
  Softi2c_Init();
}

////////////////////////////////////////////////////////////////////////////////

void Softi2cSendBytes(uint8_t cnt, uint8_t *pData)
{
  while(cnt > 0){
    Softi2c_Write_byte(*pData);
    pData++;
    cnt--;
  }
}

////////////////////////////////////////////////////////////////////////////////

void Softi2cSendByte(uint8_t byte)
{
  Softi2c_Write_byte(byte);
}

////////////////////////////////////////////////////////////////////////////////

void Softi2cStartTransfer(void)
{
  Softi2c_Start();
  Softi2c_Write_byte(Address);
}

////////////////////////////////////////////////////////////////////////////////

void Softi2cEndTransfer(void)
{
  Softi2c_Stop();
}

////////////////////////////////////////////////////////////////////////////////

static void Softi2c_DelayShort(void)
{
  _DELAY_S;
}

////////////////////////////////////////////////////////////////////////////////

static void Softi2c_DelayLong(void)
{
  _DELAY_L;
}

////////////////////////////////////////////////////////////////////////////////

static void Softi2c_Init(void)
{
  gpio_set_level(g_scl_pin, 1);
  gpio_set_level(g_sda_pin, 1);;
  Softi2c_DelayShort();
}

////////////////////////////////////////////////////////////////////////////////

/* actually, the scl line is not observed, so this procedure does not return a value */
static inline void Softi2c_Read_scl_delay(void)
{
  gpio_set_level(g_scl_pin, 1);                                                 // set as input (line will be high)
  Softi2c_DelayShort();
}

////////////////////////////////////////////////////////////////////////////////

static inline void Softi2c_Clear_scl(void)
{
  gpio_set_level(g_scl_pin, 0);
}

////////////////////////////////////////////////////////////////////////////////

static inline void Softi2c_Read_sda(void)
{
  gpio_set_level(g_sda_pin, 1);                                                 // set as input (line will be high)
}

////////////////////////////////////////////////////////////////////////////////

static inline void Softi2c_Clear_sda(void)
{
  gpio_set_level(g_sda_pin, 0);                                                 // set open collector and drive low
}

////////////////////////////////////////////////////////////////////////////////

static void Softi2c_Start(void)
{
  if(i2c_started != false){                                                     // if already started: do restart
    Softi2c_Read_sda();                                                         // SDA = 1
    Softi2c_DelayShort();
    Softi2c_Read_scl_delay();
  }
  Softi2c_Read_sda();
  Softi2c_Clear_sda();                                                          // send the start condition, both lines go from 1 to 0
  Softi2c_DelayShort();
  Softi2c_Clear_scl();
  i2c_started = false;
}

////////////////////////////////////////////////////////////////////////////////

static void Softi2c_Stop(void)
{
  Softi2c_Clear_sda();                                                          // set SDA to 0
  Softi2c_DelayShort();
  Softi2c_Read_scl_delay();                                                     // now release all lines
  Softi2c_Read_sda();                                                           // set SDA to 1
  Softi2c_DelayShort();
  i2c_started = false;
}

////////////////////////////////////////////////////////////////////////////////

static void Softi2c_Write_bit(uint8_t val)
{
  if(val) Softi2c_Read_sda();
  else Softi2c_Clear_sda();
  Softi2c_DelayShort();
  Softi2c_Read_scl_delay();
  Softi2c_Clear_scl();
}

////////////////////////////////////////////////////////////////////////////////

static void Softi2c_Read_bit(void)
{
  Softi2c_Read_sda();                                                           // do not drive SDA
  Softi2c_DelayShort();
  Softi2c_Read_scl_delay();
  Softi2c_Read_sda();
  Softi2c_DelayShort();
  Softi2c_Clear_scl();
}

////////////////////////////////////////////////////////////////////////////////

static void Softi2c_Write_byte(uint8_t b)
{
  Softi2c_Write_bit(b & 128);
  Softi2c_Write_bit(b & 64);
  Softi2c_Write_bit(b & 32);
  Softi2c_Write_bit(b & 16);
  Softi2c_Write_bit(b & 8);
  Softi2c_Write_bit(b & 4);
  Softi2c_Write_bit(b & 2);
  Softi2c_Write_bit(b & 1);
  Softi2c_Read_bit();                                                           // read ack from client.  0: ack was given by client. 1: nothing happend during ack cycle
  Softi2c_DelayLong();
}
