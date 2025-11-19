/*
 * @Author: Redbamboo 550362161@qq.com
 * @Date: 2025-10-27 18:33:12
 * @LastEditors: Redbamboo 550362161@qq.com
 * @LastEditTime: 2025-11-20 01:17:55
 * @FilePath: \FinalTask\Tasks\main_task.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
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
#include "HW_can.hpp"
#include "dm4310_drv.hpp"
#include "iwdg.h"
#include "math.h"
#include "MotorControl.hpp"
#include "dm4310_drv.hpp"
#include "imu_task.hpp"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

uint32_t tick = 0;

namespace remote_control = hello_world::devices::remote_control;
static const uint8_t kRxBufLen = remote_control::kRcRxDataLen;
static uint8_t rx_buf[kRxBufLen];
remote_control::DT7 *rc_ptr;

void RobotInit(void) { rc_ptr = new remote_control::DT7(); }

void MainInit(void) {
  RobotInit();

  // 开启CAN1和CAN2
  CanFilter_Init(&hcan1);
  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

  CanFilter_Init(&hcan2);
  HAL_CAN_Start(&hcan2);
  HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);

  // 开启遥控器接收
  HAL_UARTEx_ReceiveToIdle_DMA(&huart3, rx_buf, kRxBufLen);

  // 开启定时器
  HAL_TIM_Base_Start_IT(&htim6);

  enable_motor_mode(yaw.get_hcan(), yaw.get_id(), MIT_MODE); // 使能云台电机

  ImuInit();
}

void emergencyStop()
{
    uint8_t data[8] = {0};

    for (int i = 0; i < 8; i++)
    {
      data[i] = 0;
    }
    
    mit_ctrl(yaw.get_hcan(), yaw.get_id(), 0, 0, 0, 0, 0);
    CAN_Send_Msg(&hcan2, data, 0x200, 8);
}

int currentMode = 0;

//debug
double debug1 = yaw.targetSpeed;
double debug2 = yaw.realSpeed;

#define MAX_SPEED 1.2 // 最大速度，单位m/s
#define MAX_YAW_SPEED 500 // 最大云台旋转速度，无单位

void getInput() 
{
  if (rc_ptr->rc_l_switch() == remote_control::kSwitchStateUp) 
  {
    emergencyStop();
  }
  else
  {
    // 正常获取遥控器速度指令
    double speed_X = rc_ptr->rc_lh() * MAX_SPEED;  // 左右速度，范围[-1, 1]
    double speed_Y = rc_ptr->rc_lv() * MAX_SPEED;  // 前后速度，范围[-1, 1]
    if (rc_ptr->rc_r_switch() == remote_control::kSwitchStateUp)  //分离
    {
      double omega = 0;
      motor.compute(speed_X, speed_Y, omega, (motor.realPosition - yaw.realPosition) / 180.0 * 3.1415926);  //底盘解算
      motor.setSpeed();
 
      double speed = 100; //单位应该是度每秒
      yaw.targetPosition += rc_ptr->rc_rh() * -speed / 1000; //更改目标位置
      if (yaw.targetPosition > 180)
      {
        yaw.targetPosition -= 360;
      }
      if (yaw.targetPosition < -180)
      {
        yaw.targetPosition += 360;
      }
      yaw.setPosition();
    }
    else if (rc_ptr->rc_r_switch() == remote_control::kSwitchStateMid)  //跟随
    {
      motor.targetPosition = yaw.targetPosition;
      double omega = motor.OmegaCompute();
      motor.compute(speed_X, speed_Y, omega, (motor.realPosition - yaw.realPosition) / 180.0 * 3.1415926);  //底盘解算
      motor.setSpeed();

      double speed = 100; //单位应该是度每秒
      yaw.targetPosition += rc_ptr->rc_rh() * -speed / 1000; //更改目标位置
      if (yaw.targetPosition > 180)
      {
        yaw.targetPosition -= 360;
      }
      if (yaw.targetPosition < -180)
      {
        yaw.targetPosition += 360;
      }
      yaw.setPosition();
    }
    else if (rc_ptr->rc_r_switch() == remote_control::kSwitchStateDown) //小陀螺
    {
      double omega = 6;
      motor.compute(speed_X, speed_Y, omega, (motor.realPosition - yaw.realPosition) / 180.0 * 3.1415926);  //底盘解算
      motor.setSpeed();

      double speed = 100; //单位应该是度每秒
      yaw.targetPosition += rc_ptr->rc_rh() * -speed / 1000; //更改目标位置
      if (yaw.targetPosition > 180)
      {
        yaw.targetPosition -= 360;
      }
      if (yaw.targetPosition < -180)
      {
        yaw.targetPosition += 360;
      }
      yaw.setPosition();
    }
  }
}

int floatToInt8(float value)
{
  // 钳制到[-1, 1]范围
    if (value < -1.0) 
    {
      value = -1.0;
    }
    if (value > 1.0) 
    {
      value = 1.0;
    }
    
    // 线性映射：[-1, 1] -> [0, 255]
    // value + 1 将范围变为 [0, 2]
    // 除以2得到 [0, 1]，再乘以255得到 [0, 255]
    double normalized = (value + 1.0) / 2.0 * 255.0;
    
    return (uint8_t)(normalized);
}

void connect()  //底盘和云台连接
{
  uint8_t data[8] = {0};
  data[0] = rc_ptr->rc_l_switch();
  data[1] = floatToInt8(rc_ptr->rc_rv());
    
  uint16_t id = 0x328;
  CAN_Send_Msg(&hcan1, data, id, 8);
}

double yaw_drift = 0;
double yaw_drift_speed = 0;

double yaw_drift_start = 0;
double yaw_drift_end = 0;

void UpdateYawDrift()
{
  yaw_drift += yaw_drift_speed;
  if (yaw_drift > 3.1415926535)
  {
    yaw_drift -= 3.1415926535 * 2;
  }
  if (yaw_drift < -3.1415926535)
  {
    yaw_drift += 3.1415926535 * 2;
  }
}

void MainTask(void) {
  tick++; 
  if (tick > 2000) {
    getInput();

    motor.GetFeedback((euler_angles[0] - yaw_drift) / 3.1415926 * 180);

    yaw.GetFeedback(motor.realPosition);

    ImuUpdate();

    UpdateYawDrift();

    connect();
  }
  else if (tick >= 1000)
  {
    ImuUpdate();  //准备计算零漂

    if (tick == 1100)
    {
      yaw_drift_start = euler_angles[0];
    }
    if (tick == 2000)
    {
      yaw_drift_end = euler_angles[0];
      yaw_drift_speed = (yaw_drift_end - yaw_drift_start) / (900 - 10);

      yaw_drift = euler_angles[0];
    }
    emergencyStop();
  }
  else
  {
    emergencyStop();
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim == &htim6) {
    MainTask();
  }
}

uint8_t rx_data = 0;
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
  if (huart == &huart3) {
    if (Size == remote_control::kRcRxDataLen) {
      //在这里刷新看门狗
      HAL_IWDG_Refresh(&hiwdg);

      rc_ptr->decode(rx_buf);
    }

    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, rx_buf, kRxBufLen);
  }
}
