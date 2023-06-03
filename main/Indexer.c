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

static const char *TAG = "[INDEX]";

static bool StepState = false;

///////////////////////////////////////////////////////////////////////////////////////

static bool IRAM_ATTR TimerOnAlarmCB(gptimer_handle_t Timer, const gptimer_alarm_event_data_t *pEventData, void *pUserData)
{
  gptimer_alarm_config_t alarm_config = {.alarm_count = pEventData->alarm_value + (m_DelayCount >> 1)}; // add step delay count
  gptimer_set_alarm_action(Timer, &alarm_config);
  gpio_set_level(STEP_MOTOR_GPIO_STEP, StepState);                                  // set the pin state
  if(StepState) StepState = false;                                                  // only adjust timer on each complete pulse
  else{
    StepState = true;
    switch(m_MotorState){
      case MOTOR_RAMP_UP:{                                                          // acceleration
        m_SlopeEnd = m_StepNumber;
        if(m_StepNumber >= m_MidPoint &&  !m_JogMove){                              // midpoint: don't acceleration anymore
          m_MotorState = MOTOR_RAMP_DOWN;
          m_Denominator = ((m_StepNumber - m_StepsToMove) << 2) + 1;
          if(!(m_StepsToMove & 1)){                                                 // even move: repeat last delay before decel
            m_Denominator += DEM_INC;
            break;
          }
        }
        m_Denominator += DEM_INC;
        m_WorkingDelay -= (uint32_t)(((long)m_WorkingDelay << 1) / m_Denominator);  // ramp algorithm
        m_DelayCount = (m_WorkingDelay + 128) >> 8;                                 // round 24.8 format->int16
        if(m_DelayCount <= m_MaximumSpeed){                                         // go to constant speed
          m_MotorState = MOTOR_RAMP_MAX;
          m_StepDown = m_StepsToMove - m_StepNumber;
          m_DelayCount = m_MaximumSpeed;
          break;
        }
        break;
      }
      case MOTOR_RAMP_DOWN:{                                                        // deceleration
        if(m_StepNumber >= m_StepsToMove - 1 && !m_JogMove){                        // next IRQ is cleanup (no step)
          m_MotorState = MOTOR_RAMP_LAST;
          break;
        }
        m_Denominator += DEM_INC;
        m_WorkingDelay -= (uint32_t)(((long)m_WorkingDelay << 1) / m_Denominator);  // ramp algorithm
        m_DelayCount = (m_WorkingDelay + 128) >> 8;                                 // round 24.8 format->int16
        if((m_DelayCount <= m_MaximumSpeed) && m_JogMove){                          // go to constant speed
          m_MotorState = MOTOR_RAMP_MAX;
          m_StepDown = m_StepsToMove - m_StepNumber;
          m_DelayCount = m_MaximumSpeed;
          break;
        }
        break;
      }
      case MOTOR_RAMP_MAX:{                                                         // constant speed
        if(m_StepNumber >= m_StepDown && !m_JogMove){                               // start deceleration
          m_MotorState = MOTOR_RAMP_DOWN;
          m_Denominator = ((m_StepNumber - m_StepsToMove) << 2) + 5;
        }
        break;
      }
      default:{                                                                     // last step: cleanup
        m_MotorState = MOTOR_IDLE;
        gptimer_stop(Timer);                                                        // stop timer immediately
        gpio_set_level(STEP_MOTOR_GPIO_EN, MOTOR_DISABLE);
        return false;
      }
    }
    if(m_CurrentDirection) m_CurrentPosition++;                                     // Add a step to the current position in the correct direction
    else m_CurrentPosition--;
    ++m_StepNumber;
  }
  return false;
}

///////////////////////////////////////////////////////////////////////////////////////

void Move(signed long target)
{
  m_MovePending = false;
  m_JogMove = m_JogPending;
  if(!m_MotorEnabled){
    ESP_LOGW(TAG, "Motor not enabled");
    return;
  }
  if(target > m_CurrentPosition){
    m_CurrentDirection = true;                                                        // set forward flag
    m_StepsToMove = target - m_CurrentPosition;                                       // subtract smaller (CurrentPosition)
  }
  else{
    m_CurrentDirection = false;                                                       // clear forward flag
    m_StepsToMove = m_CurrentPosition - target;                                       // subtract smaller (target)
  }
  if(m_StepsToMove > 1){                                                              // If we are moving more than one step
    gpio_set_level(STEP_MOTOR_GPIO_EN, MOTOR_ENABLE);
    m_MidPoint = (m_StepsToMove - 1) >> 1;
    m_DelayCount = m_BaseSpeed;
    m_WorkingDelay = (uint32_t)m_DelayCount << 8;                                     // keep c in 24.8 fixed-point format for ramp calculations
    m_StepNumber = 0;                                                                 // step counter
    m_Denominator = 1;                                                                // 4.n+1, n=0
    m_MotorState = MOTOR_RAMP_UP;                                                     // start ramp state-machine
    if(m_CurrentDirection)  gpio_set_level(STEP_MOTOR_GPIO_DIR, SPIN_DIR_CW);
    else gpio_set_level(STEP_MOTOR_GPIO_DIR, SPIN_DIR_CCW);
    uint64_t RawCount;
    gptimer_get_raw_count(m_hTimer, &RawCount);                                       // get the timer count as is
    gptimer_alarm_config_t AlarmConfig = {.alarm_count = RawCount + 1000};            // set the alarm to now + initial clock period
    ESP_ERROR_CHECK(gptimer_set_alarm_action(m_hTimer, &AlarmConfig));
    ESP_ERROR_CHECK(gptimer_start(m_hTimer));
  }
}

///////////////////////////////////////////////////////////////////////////////////////

void HaltMotor(void)                                                                    // performs an immediate stop (not good for the hardware)
{
  gptimer_stop(m_hTimer);                                                               // stop timer immediately
  m_MotorState = MOTOR_IDLE;
  m_StepNumber = m_StepsToMove;
  m_MovePending = false;
  gpio_set_level(STEP_MOTOR_GPIO_EN, MOTOR_DISABLE);
}

///////////////////////////////////////////////////////////////////////////////////////

void StopMotor(void)
{
  if(m_SlopeEnd < 10) HaltMotor();
  else if((m_MotorState == MOTOR_RAMP_UP) ||  (m_MotorState == MOTOR_RAMP_MAX)){          // only if ramping up or at max speed
    gptimer_disable(m_hTimer);
    m_JogMove = false;
    m_MotorState = MOTOR_RAMP_DOWN;
    m_StepDown = m_StepNumber;                                                            // set deceleration start
    m_StepsToMove = m_StepNumber + m_SlopeEnd;                                            // set deceleration end
    m_Denominator = ((m_StepNumber - m_StepsToMove) << 2);                                // calculate slope
    gptimer_enable(m_hTimer);
  }
  m_MovePending = false;
}

///////////////////////////////////////////////////////////////////////////////////////

void MotorTask(void *arg)
{
  xEventGroupWaitBits(m_SystemFlags, SYSFLAG_SYSTEM_UP, pdFALSE, pdFALSE, portMAX_DELAY);
  while(true){
    if((m_MotorState == MOTOR_IDLE) && (m_MovePending == true)) Move(m_NewPosition);      // initiate move to new position
    vTaskDelay(pdMS_TO_TICKS(100));                                                       // 100mS loop resolution
  }
}

///////////////////////////////////////////////////////////////////////////////////////

void ConfigureIndexer(void)
{
  m_BaseSpeed = 1000000 / DEF_BASE_SPEED;
  m_MaximumSpeed = 4010000 / DEF_MAX_SPEED;
  gptimer_config_t TimerConfig = {.clk_src = GPTIMER_CLK_SRC_DEFAULT, .direction = GPTIMER_COUNT_UP, .resolution_hz = 1000000}; // 1MHz, 1 tick=1us
  ESP_ERROR_CHECK(gptimer_new_timer(&TimerConfig, &m_hTimer));
  gptimer_event_callbacks_t cbs = {.on_alarm = TimerOnAlarmCB};
  ESP_ERROR_CHECK(gptimer_register_event_callbacks(m_hTimer, &cbs, NULL));
  ESP_ERROR_CHECK(gptimer_enable(m_hTimer));
  BaseType_t id = xTaskCreate(MotorTask, "Indexer", 1024, NULL, 10, NULL);           // start the LED flash task
  if(id == 0L){
    ESP_LOGI(TAG, "failed to create indexer thread.");
    return;
  }
  xEventGroupSetBits(m_SystemFlags, SYSFLAG_ENCODERS_READY);                          // Tell system we're ready
  ESP_LOGI(TAG, "Indexer ready");
}
