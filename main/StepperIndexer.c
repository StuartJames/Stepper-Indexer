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

#define _MAIN_

#include "StepperIndexer.h"

static const char *TAG = "[MAIN ]";
const char CmdTokens[UC_MAX][10]  = {"HELP", "MOVEA", "MOVEI", "MOVEJ", "STOP", "BASE", "SPEED", "HOME", "SETPOS", "GETPOS"};
static uint8_t Clock1 = 0, Clock0 = 0;

///////////////////////////////////////////////////////////////////////////////////////

inline void FreePtr(void *pPtr)
{
  if(pPtr != NULL) free(pPtr);
  pPtr = NULL;
}

///////////////////////////////////////////////////////////////////////////////////
/* 2-bit cyclic vertical counters count the 4 samples. As long as there is no
  change, the counters are held in the reset state of 00b. When a change is detected
  between the current sample and the filtered or debounced sample, the counters
  are incremented. The counting sequence is 00,01,10,11,00... When the counters
  roll over from 11b to 00b, the debounced state is updated. If the input changes
  back to the filtered state while the counters are counting, then the counters
  are re-initialised to the reset state and the filtered state is unaffected. */

uint8_t Debounce(uint8_t Bits)
{
uint8_t Delta;
uint8_t Changes;

  Delta = Bits ^ m_FilteredState[0];
  Clock1 ^= Clock0;
  Clock0 = ~Clock0;
  Clock1 &= Delta;
  Clock0 &= Delta;
  Changes = ~(~Delta | Clock1 | Clock0);
  m_FilteredState[0] ^= Changes;
  return Changes;
}

///////////////////////////////////////////////////////////////////////////////////////

void GetInputStateCB(TimerHandle_t Timer)
{
  uint8_t PortBits = ((gpio_get_level(GPIO_JOG_CW) > 0) << FILTER_JOG_CW);
  PortBits |= ((gpio_get_level(GPIO_JOG_CCW) > 0) << FILTER_JOG_CCW);
  Debounce(PortBits);
  if(m_FilteredState[1] != m_FilteredState[0]){
    xEventGroupSetBits(m_SystemFlags, SYSFLAG_BUTTON_CHANGED);                              // flag changes
    m_FilteredState[1] = m_FilteredState[0];
  }
}

///////////////////////////////////////////////////////////////////////////////////

esp_err_t CheckNVS(void)
{
  esp_err_t Res = nvs_flash_init();
  if(Res == ESP_ERR_NVS_NO_FREE_PAGES || Res == ESP_ERR_NVS_NEW_VERSION_FOUND){   // NVS partition was truncated and needs to be erased Retry nvs_flash_init
    ESP_ERROR_CHECK(nvs_flash_erase());
    Res = nvs_flash_init();
  }
  return Res;
}

///////////////////////////////////////////////////////////////////////////////////////

void app_main(void)
{
gpio_config_t io_conf;

  ESP_LOGI(TAG, "***************************");
  ESP_LOGI(TAG, "* %s  *", TitleStr[0]);
  ESP_LOGI(TAG, "*  (c) HydraSystems 2023  *");
  ESP_LOGI(TAG, "* Written by Stuart James *");
  ESP_LOGI(TAG, "***************************");

  CheckNVS();

  m_SystemFlags = xEventGroupCreate();
  if(m_SystemFlags == NULL){
    ESP_LOGI(TAG, "failed to create system flags.");
    while(true) vTaskDelay(pdMS_TO_TICKS(1000));
  }

  io_conf.intr_type = GPIO_INTR_DISABLE;
  io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;                                               // bit mask of the pins, use GPIO17/5/23 here
  io_conf.mode = GPIO_MODE_OUTPUT;
  gpio_config(&io_conf);

  gpio_set_level(STEP_MOTOR_GPIO_EN, MOTOR_DISABLE);                                        // make sure drive is disabled. some drives place full current through coils when enabled

  io_conf.intr_type = GPIO_INTR_DISABLE;                                                    // no interrupts
  io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;                                                // bit mask of the pins, use GPIO14/27 here
  io_conf.mode = GPIO_MODE_INPUT;                                                           // set as input mode
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;                                             // disable pull-down mode
  io_conf.pull_up_en = GPIO_PULLUP_ENABLE;                                                  // enable pull-up mode
  gpio_config(&io_conf);

  uint32_t io_num = INPUT_TIMER_ID;
  m_hInputTimer = xTimerCreate("InputTimer", pdMS_TO_TICKS(50), pdTRUE, (void*)&io_num, GetInputStateCB);
  if(m_hInputTimer == NULL){
    ESP_LOGE(TAG, "failed to create input timer.");
    while(true) vTaskDelay(pdMS_TO_TICKS(1000));
  }
  xTimerStart(m_hInputTimer, 0);

  m_CmdQueue = xQueueCreate(RECV_QUEUE_SIZE, sizeof(CmdObject_t));                            // create command queue
  if(m_CmdQueue == NULL){
    ESP_LOGE(TAG, "failed to create command queue.");
    while(true) vTaskDelay(pdMS_TO_TICKS(1000));
  }

  m_UARTSendQueue = xQueueCreate(SEND_QUEUE_SIZE, sizeof(SendObject_t));
  if(m_UARTSendQueue == NULL){
    ESP_LOGE(TAG, "failed to create UART send queue.");
    while(true) vTaskDelay(pdMS_TO_TICKS(1000));
  }

  m_BTSendQueue = xQueueCreate(SEND_QUEUE_SIZE, sizeof(SendObject_t));
  if(m_BTSendQueue == NULL){
    ESP_LOGE(TAG, "failed to create BT send queue.");
    while(true) vTaskDelay(pdMS_TO_TICKS(1000));
  }

  BaseType_t id = xTaskCreate(IndicatorTask, "Indicators", 1024, NULL, 10, NULL);           // start the LED flash task
  if(id == 0L){
    ESP_LOGI(TAG, "failed to create indicators thread.");
    while(true) vTaskDelay(pdMS_TO_TICKS(1000));
  }

  id = xTaskCreate(DisplayTask, "Display", 3072, NULL, 10, NULL);                           // start the OLED display task
  if(id == 0L){
    ESP_LOGI(TAG, "failed to create display thread.");
    while(true) vTaskDelay(pdMS_TO_TICKS(1000));
  }

  id = xTaskCreate(ProcessSPPMessage, "SPP Process", 3072, NULL, 10, NULL);                 // start the LED flash task
  if(id == 0L){
    ESP_LOGI(TAG, "failed to create SPP process thread.");
    while(true) vTaskDelay(pdMS_TO_TICKS(1000));
  }

  ConfigureIndexer();

  BluetoothInit();                                                                          // initialise the Bluetooth stack
  vTaskDelay(50);                                                                           // allow BT to finish up

  if(UARTInit(CONFIG_UART_TXD_PIN, CONFIG_UART_RXD_PIN, CONFIG_UART_BAUDRATE) == ESP_FAIL){
    ESP_LOGE(TAG, "Comms system failed.");
    while(1) vTaskDelay(pdMS_TO_TICKS(1000));
  }

  xEventGroupWaitBits(m_SystemFlags, SYSFLAG_DISPLAY_READY | SYSFLAG_ENCODERS_READY | SYSFLAG_UART_READY | SYSFLAG_SPP_PROC_READY, pdFALSE, pdTRUE, portMAX_DELAY);  // wait for threads to become ready
  ESP_LOGI(TAG, "System Up.");
  xEventGroupSetBits(m_SystemFlags, SYSFLAG_SYSTEM_UP);
  SendMessage(CD_UART, "Stepper Indexer ready\r\n");

  while(1){
    EventBits_t Flags = xEventGroupWaitBits(m_SystemFlags, SYSFLAG_BUTTON_CHANGED, pdTRUE, pdFALSE, portMAX_DELAY);
    if(Flags & SYSFLAG_BUTTON_CHANGED){
//      ESP_LOGI(TAG, "Got button input: %c%c%c%c.", NIBLE_TO_BINARY(m_FilteredState[0]));
      if(((m_FilteredState[0] & FILTER_CW_BIT) == 0) && !m_MovePending){
        m_NewPosition = m_CurrentPosition - 100;
        m_MovePending = true;
        m_JogPending = true;
      }
      if(((m_FilteredState[0] & FILTER_CCW_BIT) == 0) && !m_MovePending){
        m_NewPosition = m_CurrentPosition + 100;
        m_MovePending = true;
        m_JogPending = true;
      }
      if(((m_FilteredState[0] & FILTER_CW_BIT)) && (m_FilteredState[0] & FILTER_CCW_BIT) && m_JogMove) StopMotor();
      xEventGroupSetBits(m_SystemFlags, SYSFLAG_DISPLAY_UPDATE);
    }
    vTaskDelay(20);
  }
}

///////////////////////////////////////////////////////////////////////////////////

void SendMessage(uint8_t Dest, const char *pFmt, ...)
{
char *pMsg;
va_list args;
QueueHandle_t Queue = 0;

  pMsg = (char*)malloc(UART_TX_MSG_SIZE * sizeof(char));                                    // size applies to GATT as well
  va_start(args, pFmt);
  vsprintf(pMsg, pFmt, args);
  va_end(args);
  SendObject_t SendObj = {.Type = 0, .pData = (uint8_t*)pMsg, .Length = (int)strlen(pMsg)};
  if(Dest == CD_UART) Queue = m_UARTSendQueue;                                              // reply to the correct interface
  else Queue = m_BTSendQueue;
  if(xQueueSend(Queue, &SendObj, QUEUE_MAXDELAY) != pdTRUE) ESP_LOGW(TAG, "Write to %d queue fail.", Dest);
}

//////////////////////////////////////////////////////////////////////////////////////////////

int16_t FindCommand(char *pInput)
{
int16_t i = 0;

  ESP_LOGI(TAG, "In %s, %d", pInput, strlen(pInput));
  while(i < UC_MAX){
    if((strlen(pInput) == strlen(CmdTokens[i])) && (!strncmp(pInput, CmdTokens[i], strlen(CmdTokens[i])))) return i;
    ++i;
  }
  return -1;
}

///////////////////////////////////////////////////////////////////////////////////////

void ProcessSPPMessage(void *pArg)
{
uint8_t Dest = CD_UART;
int16_t i, CommandIndex = 0;
char *pToken = NULL, *pMsg;
long CommandArg = 0;
CmdObject_t RecObj;

  xEventGroupSetBits(m_SystemFlags, SYSFLAG_SPP_PROC_READY);                          // Tell system we're ready
  ESP_LOGI(TAG, "SPP Process Initialised.");
  xEventGroupWaitBits(m_SystemFlags, SYSFLAG_SYSTEM_UP, pdFALSE, pdFALSE, portMAX_DELAY);
  while(true){
    xQueueReceive(m_CmdQueue, &RecObj, portMAX_DELAY);
    Dest = RecObj.Srce;
    pMsg = (char*)RecObj.pData;
    for(i = 0; i < strlen(pMsg); ++i){
      if((pMsg[i] == '\n') || (pMsg[i] == '\r')) pMsg[i] = '\0';
      else pMsg[i] = toupper(pMsg[i]);
    }
    if((pToken = strpbrk(pMsg, "\r\n")) != NULL) pToken = '\0';
    pToken = strtok(pMsg, " ,;");                                                     // get a token
    if(pToken == NULL) return;
    CommandIndex = FindCommand(pToken);                                               // see if it's a command
    ESP_LOGI(TAG, "Find(%d)", CommandIndex);
    switch(CommandIndex){
      case UC_HELP:{
        SendMessage(Dest, "Base n  - Set base speed\r\n");
        SendMessage(Dest, "Home    - Set home position\r\n");
        SendMessage(Dest, "MoveA n - Move to absolute position nnn\r\n");
        SendMessage(Dest, "MoveI n - Move incrementally to position + nnn\r\n");
        SendMessage(Dest, "MoveJ n - Jog move using +/- values of nnn\r\n");
        SendMessage(Dest, "Speed n - Set maximum speed\r\n");
        SendMessage(Dest, "Set n   - Set position\r\n");
        SendMessage(Dest, "Stop    - Stop move\r\n");
        break;
      }
      case UC_MOVEA:{
        if(m_MotorState == MOTOR_IDLE){
          pToken = strtok(NULL, " ,;");
          if(pToken == NULL){
            SendMessage(Dest, "Error: no target position specified\r\n");
          }
          CommandArg = atol(pToken);
          m_NewPosition = CommandArg;
          m_MovePending = true;
          m_JogPending = false;
          SendMessage(Dest, "Moving to position %ld\r\n", m_NewPosition);
        }
        break;
      }
      case UC_MOVEI:{
        if(m_MotorState == MOTOR_IDLE){
          pToken = strtok(NULL, " ,;");
          if(pToken == NULL){
            SendMessage(Dest, "Error: no target position specified\r\n");
          }
          CommandArg = atol(pToken);
          m_NewPosition = m_CurrentPosition + CommandArg;
          m_MovePending = true;
          m_JogPending = false;
          SendMessage(Dest, "Moving to position %ld\r\n", m_NewPosition);
        }
        break;
      }
      case UC_MOVEJ:{
        if(m_MotorState == MOTOR_IDLE){
          pToken = strtok(NULL, " ,;");
          if(pToken == NULL){
            SendMessage(Dest, "Error: no direction value specified\r\n");
          }
          CommandArg = atol(pToken);
          m_NewPosition = (CommandArg >= 0) ? m_CurrentPosition + 100 : m_CurrentPosition - 100;
          m_MovePending = true;
          m_JogPending = true;
          SendMessage(Dest, "Moving continuously. Type 'stop' to end\r\n");
        }
        break;
      }
      case UC_STOP:{
        if(m_MotorState != MOTOR_IDLE) StopMotor();
        SendMessage(Dest, "Stopping\r\n", m_BaseSpeed);
        break;
      }
      case UC_BASE:{
        if(m_MotorState == MOTOR_IDLE){
          pToken = strtok(NULL, " ,;");
          if(pToken == NULL){
            SendMessage(Dest, "Error: no base speed specified\r\n");
          }
          CommandArg = atol(pToken);
          m_BaseSpeed = 1000000 / CommandArg;
          m_BaseSpeed = (m_BaseSpeed < m_MaximumSpeed) ? m_MaximumSpeed : (m_BaseSpeed < BASESP_MAX) ? BASESP_MAX : m_BaseSpeed;
          SendMessage(Dest, "Base speed set to %ld\r\n", m_BaseSpeed);
        }
        break;
      }
      case UC_SPEED:{
        if(m_MotorState == MOTOR_IDLE){
          pToken = strtok(NULL, " ,;");
          if(pToken == NULL){
            SendMessage(Dest, "Error: no target speed specified\r\n");
          }
          CommandArg = atol(pToken);
          m_MaximumSpeed = 4010000 / CommandArg;
          if(m_MaximumSpeed < MAX_SPEED) m_MaximumSpeed = MAX_SPEED;
          SendMessage(Dest, "Running speed set to %ld\r\n", m_MaximumSpeed);
        }
        break;
      }
      case UC_HOME:{
        if(m_MotorState == MOTOR_IDLE){
          m_NewPosition = 0;
          m_CurrentPosition = 0;
        }
        break;
      }
      case UC_SETPOS:{
        if(m_MotorState == MOTOR_IDLE){
          pToken = strtok(NULL, " ,;");
          if(pToken == NULL){
            SendMessage(Dest, "Error: no position value specified\r\n");
          }
          m_CurrentPosition = atol(pToken);
          SendMessage(Dest, "Current position set to %ld\r\n", m_CurrentPosition);
        }
        break;
      }
      case UC_GETPOS:{
        CommandArg = m_CurrentPosition;
        SendMessage(Dest, "Current position is %ld\r\n", CommandArg);
        break;
      }
      default:{
        SendMessage(Dest, "Error: No or invalid command. Type 'Help' for command list\r\n");
      }
    }
    free(pMsg);
  }
}

