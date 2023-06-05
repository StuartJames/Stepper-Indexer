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
* This file contains code to implement the OLED display code.
*
* Edit     Date     Version       Edit Description
* ====  ==========  ======= =====================================================
* SJ    2019/11/26   1.a.1   Original
* SJ    2021/05/30   1.b.1   Modified pressure sampling and indicator display
* SJ    2021/07/31   1.c.1   Added button input de-bounce function
*
*/

#define FIRST_DISPLAY

#include "StepperIndexer.h"

////////////////////////////////////////////////////////////////////////////////

#ifdef FIRST_DISPLAY
#define DISPLAY_ADDR                  0x3c                                                  // Display address for panel A
#else
#define DISPLAY_ADDR                  0x3d                                                  // Display address for panel B
#endif

// SLA (0x3C) + WRITE_MODE (0x00) =  0x78 (0b01111000)
#define I2C_ADDR                    (DISPLAY_ADDR << 1)                                     // I2C address

#define GPIO_I2C_SCL                CONFIG_OLED_CLOCK
#define GPIO_I2C_SDA                CONFIG_OLED_DATA
#define GPIO_OLED_RST               CONFIG_OLED_RESET

static const char *TAG = "[DISP ]";

///////////////////////////////////////////////////////////////////////////////////////

void SleepCallback(TimerHandle_t pxTimer)
{

  xEventGroupSetBits(m_SystemFlags, SYSFLAG_SLEEPTIMER_EXP);                                // display should go to sleep
}

///////////////////////////////////////////////////////////////////////////////////////

void IndicatorTask(void *pArg)
{
bool LedInd = false;
int LedTime = 1, DispTime = 5;

  xEventGroupWaitBits(m_SystemFlags, SYSFLAG_SYSTEM_UP, pdFALSE, pdFALSE, portMAX_DELAY);
  while(true){
    vTaskDelay(pdMS_TO_TICKS(100));                                                         // 100mS loop resolution
    if(--LedTime <= 0){
      gpio_set_level(GPIO_LED_IND, LedInd);                                                 // toggle the on-board LED
      if(LedInd) LedTime = 1;                                                               // on for 100mS
      else LedTime = 19;                                                                    // off for 1.9 seconds
      LedInd = !LedInd;                                                                     // toggle the led state on each pass
    }
    if(m_MotorState == MOTOR_IDLE){
      if(--DispTime <= 0){
        DispTime = 5;
        xEventGroupSetBits(m_SystemFlags, SYSFLAG_DISPLAY_UPDATE);
      }
    }
    else xEventGroupSetBits(m_SystemFlags, SYSFLAG_DISPLAY_UPDATE);
  }
}

///////////////////////////////////////////////////////////////////////////////////////

void DisplayTask(void *pArg)
{
  char buff[15];
TimerHandle_t hSleepTimer;
uint32_t TimerID;
EventBits_t flags;

  DC_Init(GPIO_I2C_SCL, GPIO_I2C_SDA, GPIO_OLED_RST, I2C_ADDR);
  DC_SetTabs(4);                                                                                              // set the tab spacing to 3 characters
  TimerID = SLEEP_TIMER_ID;
  hSleepTimer = xTimerCreate("SleepTimer", SECONDS_TO_TICKS(120), pdFALSE, (void*)&TimerID, SleepCallback);   // 10 minute timeout
  if(hSleepTimer == NULL){
    ESP_LOGI(TAG, "failed to create sleep timer.");
    while(1);
  }
  xEventGroupSetBits(m_SystemFlags, SYSFLAG_DISPLAY_READY | SYSFLAG_DISPLAY_UPDATE);                          // Tell system we're ready
  ESP_LOGI(TAG, "Display Initialised.");
  xEventGroupWaitBits(m_SystemFlags, SYSFLAG_SYSTEM_UP, pdFALSE, pdFALSE, portMAX_DELAY);
  while(true){
    flags = xEventGroupWaitBits(m_SystemFlags, SYSFLAG_DISPLAY_UPDATE | SYSFLAG_SLEEPTIMER_EXP, pdTRUE, pdFALSE, portMAX_DELAY);
    if(flags & SYSFLAG_SLEEPTIMER_EXP){
      DC_On(false);                                                                                           // turn display off
    }
    if(flags & SYSFLAG_DISPLAY_UPDATE){                                                                       // the display should update
      if(m_MotorState == MOTOR_IDLE){                                                    // but if the system is in idle/error mode check the sleep timer has started
        if(!xTimerIsTimerActive(hSleepTimer) && DC_GetStatus(STATUS_DISPLAY_ON)) xTimerReset(hSleepTimer, 50); // start or restart sleep timer
      }
      else{                                                                                                   // Not in Idle mode so make sure...
        if(xTimerIsTimerActive(hSleepTimer)) xTimerStop(hSleepTimer, 0);                                      // the sleep timer is stopped
        if(!DC_GetStatus(STATUS_DISPLAY_ON)) DC_On(true);                                                     // and the display is on
      }
      if(DC_GetStatus(STATUS_DISPLAY_ON)){                                                                    // only if the display is active
        DC_ClearBuffer();
        DC_SelectFont(FONT_SANSSERIF_8PT);
        DC_PrintString(SSD1306_COLOR_WHITE, SSD1306_COLOR_BLACK, "%s\n", TitleStr[0]);
        DC_SelectFont(FONT_CALIBRI_8PT);
        DC_PrintString(SSD1306_COLOR_WHITE, SSD1306_COLOR_BLACK, "%s\t\t\t%s\n", TitleStr[1], StatusStr[m_MotorState]);
        sprintf(buff, "%6d", m_BaseSpeed);
        DC_PrintString(SSD1306_COLOR_WHITE, SSD1306_COLOR_BLACK, "%s\t\t%s\n", TitleStr[2], buff);
        sprintf(buff, "%6d", m_MaximumSpeed);
        DC_PrintString(SSD1306_COLOR_WHITE, SSD1306_COLOR_BLACK, "%s\t\t%s\n", TitleStr[3], buff);
        sprintf(buff, "%6ld", m_CurrentPosition);
        DC_PrintString(SSD1306_COLOR_WHITE, SSD1306_COLOR_BLACK, "%s\t\t%s\n", TitleStr[4], buff);
        DC_Refresh(false);
//        ESP_LOGI(TAG, "%s-%s-%s.", StatesStr[m_Status], ModesStr[m_Mode], buff);
      }
    }
  }
}

