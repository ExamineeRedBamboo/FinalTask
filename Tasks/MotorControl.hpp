/*
 * @Author: Redbamboo 550362161@qq.com
 * @Date: 2025-10-26 23:30:23
 * @LastEditors: Redbamboo 550362161@qq.com
 * @LastEditTime: 2025-11-20 01:03:11
 * @FilePath: \Task\mycode\CanInit.HPP
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef _CanInit_H_
#define _CanInit_H_

#include "HW_can.hpp"
#include "stm32f407xx.h"
#include "iwdg.h"
#include "tim.h"
#include "stdint.h"
#include "math.h"
#include "DT7.hpp"
#include "dm4310_drv.hpp"

#ifdef __cplusplus
extern "C" {
#endif

void MainInit(void);

#ifdef __cplusplus
}

class PID 
{
private:
    double kp_;
    double ki_;
    double kd_;

    double min_; //解决误差周期性问题
    double max_;

    bool circular_enabled_ = false;  //周期最短路径模式

    double integral = 0.0;
    double last_error = 0.0;
public:
    PID(double kp, double ki, double kd, double min, double max, bool circular_enabled);
    double compute(double setpoint, double measured, double dt);
};

class G3508_Motor 
{
private:
    CAN_HandleTypeDef* hcan_;
    
    PID speedPID[4] = {
        PID(10, 0.0, 0.0, -16384, 16384, false),
        PID(10, 0.0, 0.0, -16384, 16384, false),
        PID(10, 0.0, 0.0, -16384, 16384, false),
        PID(10, 0.0, 0.0, -16384, 16384, false)
    };

    PID positionPID = PID(0.05, 0, 0, -180, 180, true);

public:
    int current[4] = {0, 0, 0, 0};
    double realSpeeds[4] = {0, 0, 0, 0};
    double targetSpeeds[4] = {0, 0, 0, 0};

    double targetPosition = 0;
    double realPosition = 0;
    
    G3508_Motor(CAN_HandleTypeDef* hcan);

    void compute(double speed_X, double speed_Y, double target_omega, double joint_angle);

    void setSpeed();

    double OmegaCompute();  //算出目标角速度

    void GetFeedback(double angle);
};

class DM4310_Motor 
{
private:
    CAN_HandleTypeDef* hcan_;
    uint16_t id_;

    PID speedPID = PID(0.012, 0.0, 0, -16384, 16384, false);
    PID positionPID = PID(10, 0.0, 0, -180.0, 180.0, true); //位置环，周期模式开

public:
    double targetPosition = 0.0;
    double realPosition = 0.0;

    double targetSpeed = 0.0;
    double realSpeed = 0.0;

    int16_t fbData[2] = {0, 0}; //反馈数据

    DM4310_Motor(CAN_HandleTypeDef* hcan, uint16_t id);

    void setSpeed();

    void setPosition();

    void GetFeedback(double chassisPosition);

    CAN_HandleTypeDef* get_hcan();
    uint16_t get_id();
};

extern G3508_Motor motor;

extern DM4310_Motor yaw;

#endif

#endif