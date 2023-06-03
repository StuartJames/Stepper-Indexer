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

#pragma once

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#include <freertos/FreeRTOS.h>
#include <driver/gpio.h>

#define SSD1306_NOINLINE __attribute__((noinline))
#define SSD1306_ALWAYSINLINE __attribute__((always_inline))

#define SSD1306_MSG_INIT               21
#define SSD1306_MSG_SEND_BYTE          22
#define SSD1306_MSG_SEND_STREAM        23
#define SSD1306_MSG_SET_DC             24
#define SSD1306_MSG_START_TRANSFER     25
#define SSD1306_MSG_END_TRANSFER       26
#define SSD1306_MSG_DELAY_I2C          27

//typedef enum {
//    I2C_MASTER_WRITE = 0,   /*!< I2C write data */
//    I2C_MASTER_READ,        /*!< I2C read data */
//} i2c_rw_t;

void        Softi2cInit(uint8_t address, uint8_t scl_pin, uint8_t sda_pin);
void        Softi2cSendBytes(uint8_t cnt, uint8_t *data);
void        Softi2cSendByte(uint8_t byte);
void        Softi2cStartTransfer(void);
void        Softi2cEndTransfer(void);
