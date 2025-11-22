/**
*******************************************************************************
* @file      :main_task.cpp
* @brief     :
* @history   :
*  Version     Date            Author          Note
*  V0.9.0      yyyy-mm-dd      <author>        1. <note>
*******************************************************************************
* @attention :
*******************************************************************************
*  Copyright (c) 2024 Hello World Team，Zhejiang University.
*  All Rights Reserved.
*******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "main_task.hpp"
#include "system_user.hpp"

#include "DT7.hpp"
#include "HW_fdcan.hpp"
#include "dm4310_drv.hpp"
#include "iwdg.h"
#include "math.h"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

uint32_t tick = 0;

namespace remote_control = hello_world::devices::remote_control;
static const uint8_t kRxBufLen = remote_control::kRcRxDataLen;
static uint8_t rx_buf[kRxBufLen] __attribute__((section(".RAM_D1")));
remote_control::DT7 *rc_ptr;

void RobotInit(void) { rc_ptr = new remote_control::DT7(); }

void MainInit(void) {
  RobotInit();

  // 开启FDCAN
  FdcanFilter_Init(&hfdcan1);
  HAL_FDCAN_Start(&hfdcan1);
  HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

  FdcanFilter_Init(&hfdcan2);
  HAL_FDCAN_Start(&hfdcan2);
  HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

  FdcanFilter_Init(&hfdcan3);
  HAL_FDCAN_Start(&hfdcan3);
  HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);

  // 开启遥控器接收
  HAL_UARTEx_ReceiveToIdle_DMA(&huart5, rx_buf, kRxBufLen);

  // 开启定时器
  HAL_TIM_Base_Start_IT(&htim6);
}

void MainTask(void) { tick++; }

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

  if (htim == &htim6) {
    MainTask();
  }
}
uint8_t rx_data = 0;
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
  if (huart == &huart5) {
    if (Size == remote_control::kRcRxDataLen) {
      // TODO:在这里进行看门狗刷新

      rc_ptr->decode(rx_buf);
    }

    HAL_UARTEx_ReceiveToIdle_DMA(&huart5, rx_buf, kRxBufLen);
  }
}
