
/*  distribution, or disclosure of this software is expressly forbidden.
 *
 *  This Copyright notice may not be removed or modified without prior
 *  written consent of HydraSystems.
 *
 *  HydraSystems, reserves the right to modify this software without
 *  notice.
 *
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
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

#include "StepperIndexer.h"

#define PACKET_READ_TICS  100 / portTICK_RATE_MS

#define UART_TASK_STACK_SIZE    2048
#define UART_TASK_PRIO          10

const int UART_Port         = CONFIG_UART_NUM;

static const char *TAG = "[UART ]";

static QueueHandle_t UART0_Queue = NULL;
static uint8_t* pRxBuffer;
static uint8_t* pTxBuffer;

///////////////////////////////////////////////////////////////////////////////////////

static void UART_Tx(void *arg)
{
SendObject_t SendObj;

  xEventGroupWaitBits(m_SystemFlags, SYSFLAG_SYSTEM_UP, pdFALSE, pdFALSE, portMAX_DELAY);
  while(true){
    xQueueReceive(m_UARTSendQueue, &SendObj, portMAX_DELAY);
    if(SendObj.pData != NULL){
      while(uart_wait_tx_done(UART_Port, 10) != ESP_OK);                                            // wait for out buffer to become free
      if(uart_write_bytes(UART_Port, SendObj.pData, SendObj.Length) != SendObj.Length){               // send the data to the driver
        ESP_LOGI(TAG, "Failed to transmit all data.");
      }
      FreePtr(SendObj.pData);
    }
  }
}

///////////////////////////////////////////////////////////////////////////////////////

static void UART_Rx(void *arg)
{
uart_event_t event;
uint16_t FillCount = 0;
CmdObject_t RecObj = {.Srce = CD_UART, .pData = NULL, .Length = 0};

  xEventGroupWaitBits(m_SystemFlags, SYSFLAG_SYSTEM_UP, pdFALSE, pdFALSE, portMAX_DELAY);
  while(true){
    if(xQueueReceive(UART0_Queue, (void *)&event, (TickType_t)portMAX_DELAY)){
      switch(event.type) {
        case UART_DATA:{                                                                    // UART receiving data
          uart_read_bytes(UART_Port, &pRxBuffer[FillCount], event.size, portMAX_DELAY);
          FillCount += ((SPP_MESSAGE_SIZE - FillCount) >= event.size) ? event.size : SPP_MESSAGE_SIZE - FillCount;
          if(strchr((char*)pRxBuffer, 10)){
            uint8_t *pCmdBuff = NULL;
            pCmdBuff = (uint8_t*)malloc(FillCount * sizeof(uint8_t));
            if(pCmdBuff == NULL){
              ESP_LOGE(TAG, "%s malloc failed", __func__);
              bzero(pRxBuffer, SPP_MESSAGE_SIZE);
              FillCount = 0;
              break;
            }
            memcpy(pCmdBuff, pRxBuffer, FillCount);
            RecObj.pData = pCmdBuff;
            RecObj.Length = FillCount;
            xQueueSend(m_CmdQueue, &RecObj, 10 / portTICK_PERIOD_MS);
            bzero(pRxBuffer, SPP_MESSAGE_SIZE);
            FillCount = 0;
            break;
          }
          break;
        }
        case UART_FIFO_OVF:{                                                                // HW FIFO overflow detected
          uart_flush_input(UART_Port);                                                      // the ISR has already reset the Rx FIFO,
          xQueueReset(UART0_Queue);                                                         // we directly flush the Rx buffer
          break;
        }
        case UART_BUFFER_FULL:{                                                             // UART ring buffer full
          uart_flush_input(UART_Port);
          xQueueReset(UART0_Queue);                                                         // we directly flush the Rx buffer
          break;
        }
        case UART_BREAK:                                                                    // UART RX break detected
        case UART_PARITY_ERR:                                                               // UART parity check error
        case UART_FRAME_ERR:{                                                               // UART frame error
          ESP_LOGI(TAG, " Frame or parity error.");
          break;
        }
        default:{
          break;
        }
      }
    }
  }
}

///////////////////////////////////////////////////////////////////////////////////////

esp_err_t UARTInit(gpio_num_t TxPin, gpio_num_t RxPin, int Baud)
{
 const uart_config_t UARTConfig = {
     .baud_rate = Baud,
     .data_bits = UART_DATA_8_BITS,
     .parity = UART_PARITY_DISABLE,
     .stop_bits = UART_STOP_BITS_1,
     .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
     .rx_flow_ctrl_thresh = 122,
     .source_clk = UART_SCLK_DEFAULT,
 };

  if(gpio_set_direction(TxPin, GPIO_MODE_OUTPUT)) return ESP_FAIL;
  if(gpio_set_direction(RxPin, GPIO_MODE_INPUT)) return ESP_FAIL;
  if(gpio_set_pull_mode(RxPin, GPIO_PULLUP_ONLY)) return ESP_FAIL;
  if(uart_param_config(UART_Port, &UARTConfig)) return ESP_FAIL;                                      //Configure UART1 parameters
  if(uart_set_pin(UART_Port, TxPin, RxPin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE)) return ESP_FAIL;   //Set UART1 pins(TX, RX, RTS, CTS)
  if(uart_driver_install(UART_Port, SPP_MESSAGE_SIZE * 2, SPP_MESSAGE_SIZE * 2, 20, &UART0_Queue, 0)) return ESP_FAIL;
  if(xTaskCreate(UART_Tx, "UART Tx", UART_TASK_STACK_SIZE, NULL, UART_TASK_PRIO, NULL) == 0L) return ESP_FAIL;
  if(xTaskCreate(UART_Rx, "UART Rx", UART_TASK_STACK_SIZE, NULL, UART_TASK_PRIO, NULL) == 0L) return ESP_FAIL;
  pRxBuffer = (uint8_t*) malloc(SPP_MESSAGE_SIZE);
  pTxBuffer = (uint8_t*) malloc(SPP_MESSAGE_SIZE);
  bzero(pRxBuffer, SPP_MESSAGE_SIZE);
  bzero(pTxBuffer, SPP_MESSAGE_SIZE);
  xEventGroupSetBits(m_SystemFlags, SYSFLAG_UART_READY);                          // Tell system we're ready
  ESP_LOGI(TAG, "UART Comms ready");
  return ESP_OK;
}





