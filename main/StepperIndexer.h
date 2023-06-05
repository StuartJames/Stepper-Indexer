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
* This file contains code to implement the control FSM code and pressure
* digitisation.
*
* Edit     Date     Version       Edit Description
* ====  ==========  ======= =====================================================
* SJ    2023/05/25   1.a.1   Original
*
*/

#pragma once

#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <ctype.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/event_groups.h>
#include <esp_freertos_hooks.h>
#include <driver/gpio.h>
#include <driver/gptimer.h>
#include <driver/uart.h>
#include <nvs_flash.h>
#include <esp_bt.h>
#include <esp_vfs.h>
#include <esp_spiffs.h>
#include <esp_log.h>
#include <esp_check.h>
#include <cJSON.h>

#include "SSD1306.h"
#include "FontInfo.h"

#ifdef __cplusplus
extern "C" {
#endif

///////////////////////////////Change the following configurations according to your board//////////////////////////////

#define OLED_ON                   0
#define IND_ON                    0
#define IND_OFF                   1
#define MOTOR_ENABLE              0
#define MOTOR_DISABLE             !MOTOR_ENABLE
#define SPIN_DIR_CW               0
#define SPIN_DIR_CCW              !SPIN_DIR_CW

#define SECONDS_TO_TICKS(xTimeInS) ((TickType_t)(xTimeInS) * configTICK_RATE_HZ)
#define TICKS_TO_SECONDS(xTicks) ((TickType_t)(xTicks) / configTICK_RATE_HZ)

#define SYSFLAG_DISPLAY_READY     0x0001
#define SYSFLAG_ENCODERS_READY    0x0002
#define SYSFLAG_UART_READY        0x0004
#define SYSFLAG_SPP_PROC_READY    0x0008
#define SYSFLAG_SYSTEM_UP         0x0010
#define SYSFLAG_DISPLAY_UPDATE    0x0020
#define SYSFLAG_BUTTON_CHANGED    0x0040
#define SYSFLAG_UART_TX_READY     0x0080
#define SYSFLAG_UART_RX_EVENT     0x0100
#define SYSFLAG_BT_RX_EVENT       0x0200
#define SYSFLAG_SLEEPTIMER_EXP    0x1000

#define STEP_MOTOR_RESOLUTION_HZ 1000000 // 1MHz resolution

#define GPIO_VEXT_OLED            GPIO_NUM_21
#define STEP_MOTOR_GPIO_EN        CONFIG_INX_ENABLE_PIN
#define STEP_MOTOR_GPIO_DIR       CONFIG_INX_DIR_PIN
#define STEP_MOTOR_GPIO_STEP      CONFIG_INX_STEP_PIN
#define GPIO_LED_IND              GPIO_NUM_25
#define GPIO_OUTPUT_PIN_SEL       ((1ULL << GPIO_VEXT_OLED) | (1ULL << GPIO_LED_IND) | (1ULL << STEP_MOTOR_GPIO_EN) | (1ULL << STEP_MOTOR_GPIO_DIR) | (1ULL << STEP_MOTOR_GPIO_STEP))

#define GPIO_JOG_CW               GPIO_NUM_14
#define GPIO_JOG_CCW              GPIO_NUM_27
#define GPIO_INPUT_PIN_SEL        ((1ULL << GPIO_NUM_14) | (1ULL << GPIO_NUM_27))

#define I2C_MASTER_SCL_IO         15      // gpio number for I2C master clock
#define I2C_MASTER_SDA_IO         04      // gpio number for I2C master data

#define SLEEP_TIMER_ID            0
#define INPUT_TIMER_ID            1

#define DEM_INC                   4
#define BASE_SPEED                50000
#define BASESP_MAX                10000
#define MAX_SPEED                 125 // steps per second
#define DEF_STOP_CNT              4096
#define DEF_BASE_SPEED            40
#define DEF_MAX_SPEED             6000

#define FILTER_CW_BIT             0x01
#define FILTER_CCW_BIT            0x02

#define FILTER_JOG_CW             0x00
#define FILTER_JOG_CCW            0x01

#define NIBLE_TO_BINARY_PATTERN   %c%c%c%c
#define NIBLE_TO_BINARY(byte)  \
  (byte & 0x08 ? '1' : '0'), \
  (byte & 0x04 ? '1' : '0'), \
  (byte & 0x02 ? '1' : '0'), \
  (byte & 0x01 ? '1' : '0')

#define CNTRL_ENABLE_BIT          0x01
#define CNTRL_MOVE_BIT            0x02
#define CNTRL_AUTOMOVE_BIT        0x04
#define CNTRL_MANUAL_BIT          0x08
#define CNTRL_IO0_BIT             0x10
#define CNTRL_IO1_BIT             0x20
#define CNTRL_IO2_BIT             0x40
#define CNTRL_IO3_BIT             0x80

#define UART_TX_MSG_SIZE          132
#define RECV_QUEUE_SIZE           10
#define SEND_QUEUE_SIZE           10
#define QUEUE_MAXDELAY            512

///////////////////////////////////////////////////////////////////////////////////////

typedef enum  MotorStates_e{
  MOTOR_IDLE,
  MOTOR_RAMP_UP,
  MOTOR_RAMP_MAX,
  MOTOR_RAMP_DOWN,
  MOTOR_RAMP_LAST,
  MOTOR_OFF
} MotorStates_e;

typedef enum UserCommands_e {
  UC_HELP = 0,
  UC_MOVEA,
  UC_MOVEI,
  UC_MOVEJ,
  UC_STOP,
  UC_BASE,
  UC_SPEED,
  UC_HOME,
  UC_SETPOS,
  UC_GETPOS,
  UC_MAX
} UserCommands_e;

typedef enum CommsDest_e {
  CD_GATT = 0,
  CD_UART,
  CD_MAX
} CommsDest_e;

typedef struct CmdObject_t{
  uint8_t Srce;
  void    *pData;
  int     Length;
} CmdObject_t;

typedef struct SendObject_t{
  uint8_t Type;
  void    *pData;
  int     Length;
} SendObject_t;
///////////////////////////////////////////////////////////////////////////////////////

#ifdef _MAIN_

const char      *TitleStr[] = {"STEPPER INDEXER V1.a.1", "STATUS:", "BASE SPD:", "RUN SPD:","POSITION:"};
const char      *StatusStr[] = {"IDLE", "RAMPUP", "RAMPMAX", "RAMPDN", "RAMPLST", "OFF"};
gptimer_handle_t m_hTimer = NULL;
EventGroupHandle_t   m_SystemFlags;       // Bit control flags (see above for definition)
QueueHandle_t   m_CmdQueue = NULL;
QueueHandle_t   m_UARTSendQueue = NULL;
QueueHandle_t   m_BTSendQueue = NULL;
TimerHandle_t   m_hInputTimer;
MotorStates_e   m_MotorState = MOTOR_IDLE;
long            m_CurrentPosition = 0;                                                    // current motor position
bool            m_CurrentDirection = 1;                                                   // 1 for forward, 0 for reverse
uint16_t        m_MaximumSpeed = 2000;
uint16_t        m_Acceleration;
uint16_t        m_TargetSpeed;
uint16_t        m_BaseSpeed = 20;
long            m_NewPosition = 0;                                                        // used to store required new position
uint16_t        m_DelayCount = 1;                                                         // integer delay count
uint32_t        m_StepNumber = 0;                                                         // progress of move
uint32_t        m_StepDown = 0;                                                           // start of down-ramp
uint32_t        m_StepsToMove = 0;                                                        // total steps to move
uint32_t        m_MidPoint = 0;                                                           // midpoint of move
uint32_t        m_SlopeEnd = 0;                                                           // Current point on accel slope used for decel stop
uint32_t        m_WorkingDelay;                                                       // 24.8 fixed point delay count
long            m_Denominator = 0;                                                        // 4.n+1 in ramp algo
bool            m_MotorEnabled = true;                                                       // Disabled flag
bool            m_DisablePending = false;                                             // waiting for motor to stop
bool            m_MovePending = false;                                                // delayed move flag
bool            m_JogMove = false;                                                    // Set when continuous move required
bool            m_JogPending = false;
uint8_t         m_FilteredState[2] = {0};
uint8_t         m_LatchedSWState = 0;

#else

extern const char     *TitleStr[];
extern const char     *StatusStr[];
extern gptimer_handle_t m_hTimer;
extern EventGroupHandle_t m_SystemFlags;
extern QueueHandle_t   m_CmdQueue;
extern QueueHandle_t   m_UARTSendQueue;
extern QueueHandle_t   m_BTSendQueue;
extern TimerHandle_t  m_hInputTimer;
extern MotorStates_e  m_MotorState;
extern long           m_CurrentPosition;
extern bool           m_CurrentDirection;
extern uint16_t       m_MaximumSpeed;
extern uint16_t       m_Acceleration;
extern uint16_t       m_TargetSpeed;
extern uint16_t       m_BaseSpeed;
extern long           m_NewPosition;
extern uint16_t       m_DelayCount;
extern uint32_t       m_StepNumber;
extern uint32_t       m_StepDown;
extern uint32_t       m_StepsToMove;
extern uint32_t       m_MidPoint;
extern uint32_t       m_SlopeEnd;
extern uint32_t       m_WorkingDelay;
extern long           m_Denominator;
extern bool           m_MotorEnabled;                                                       // Disabled flag
extern bool           m_DisablePending;                                                     // waiting for motor to stop
extern bool           m_MovePending;                                                        // delayed move flag
extern bool           m_JogMove;                                                            // Set when continuous move required
extern bool           m_JogPending;
extern uint8_t        m_FilteredState[2];
extern uint8_t        m_LatchedSWState;

#endif

///////////////////////////////////////////////////////////////////////////////////////

void FreePtr(void *pPtr);

void DisplayTask(void *pArg);
void IndicatorTask(void *pArg);
void ProcessSPPMessage(void *pArg);

void ConfigureIndexer(void);
void BluetoothInit(void);
esp_err_t UARTInit(gpio_num_t TxPin, gpio_num_t RxPin, int Baud);
void SendMessage(uint8_t Dest, const char *pFmt, ...);

void Move(signed long target);
void HaltMotor(void);                                                                    // performs an immediate stop
void StopMotor(void);

#ifdef __cplusplus
}
#endif
