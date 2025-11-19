/*
 * @Author: Redbamboo 550362161@qq.com
 * @Date: 2025-10-26 23:30:14
 * @LastEditors: Redbamboo 550362161@qq.com
 * @LastEditTime: 2025-11-19 22:11:16
 * @FilePath: \Task\mycode\MotorControl.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "MotorControl.hpp"

PID::PID(double kp, double ki, double kd, double min, double max, bool circular_enabled = false)
{
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
    min_ = min;
    max_ = max;

    circular_enabled_ = circular_enabled;  //周期模式
}
double PID::compute(double setpoint, double measured, double dt)
{
    double error = setpoint - measured;

    if (max_ > min_) 
    {  
        double range = max_ - min_;
        if (circular_enabled_)
        {
            // 周期模式下，选择最短路径
            while (error > range / 2) 
            {
                error -= range;
            } 
            while (error < -range / 2)
            {
                error += range;
            }
        }
        else
        {
            // 非周期模式下，进行限幅
            if (error > max_) 
            {
                error = max_;
            }
            if (error < min_) 
            {
                error = min_;
            }
        }
    }

    integral += error * dt;

    double derivative = (error - last_error) / dt;
    last_error = error;


    return kp_ * error + ki_ * integral + kd_ * derivative;
}

G3508_Motor::G3508_Motor(CAN_HandleTypeDef* hcan)
{
    hcan_ = hcan;
}

#define WHEEL_DISTANCE 0.21691  // 轮子中心到旋转中心的距离：216.91mm = 0.21691m
#define WHEEL_RADIUS 0.07786    // 轮子半径：77.86mm = 0.07786m
#define REDUCTION_RATIO 14.0    // 减速比
#define RPM_CONVERT (60.0f / (2 * 3.1415926))  // rad/s 转 rpm 的系数

void G3508_Motor::compute(double speed_X, double speed_Y, double target_omega, double joint_angle)
{
    double speed_cal[4];  // 用于存储4个轮子的计算速度（浮点数）
    double sin_ang = sin(joint_angle);  // 底盘当前角度的正弦值（角度需为弧度制）
    double cos_ang = cos(joint_angle);  // 底盘当前角度的余弦值（角度需为弧度制）

    // 计算4个轮子的速度（基于运动学推导公式，结合绝对坐标系速度与底盘自转角速度）
    speed_cal[0] = ((-cos_ang + sin_ang) * speed_X + (-sin_ang - cos_ang) * speed_Y + WHEEL_DISTANCE * target_omega) / sqrt(2);
    speed_cal[1] = ((cos_ang + sin_ang) * speed_X + (sin_ang - cos_ang) * speed_Y + WHEEL_DISTANCE * target_omega) / sqrt(2);
    speed_cal[2] = ((cos_ang - sin_ang) * speed_X + (sin_ang + cos_ang) * speed_Y + WHEEL_DISTANCE * target_omega) / sqrt(2);
    speed_cal[3] = ((-cos_ang - sin_ang) * speed_X + (-sin_ang + cos_ang) * speed_Y + WHEEL_DISTANCE * target_omega) / sqrt(2);
    
    // 将浮点数计算速度转换为整数输出速度
    for (int i = 0; i < 4; i++)
    {
        targetSpeeds[i] = speed_cal[i] / WHEEL_RADIUS * REDUCTION_RATIO * RPM_CONVERT;
    }
}

void G3508_Motor::setSpeed()
{
    double dt = 0.001;  //控制的周期

    for (int i = 0; i < 4; i++)
    {
        current[i] = (int)speedPID[i].compute(targetSpeeds[i], realSpeeds[i], dt);
    }
    
    uint8_t data[8] = {0};

    data[0] = (current[0] >> 8) & 0xFF; 
    data[1] = current[0] & 0xFF;        
    data[2] = (current[1] >> 8) & 0xFF; 
    data[3] = current[1] & 0xFF; 
    data[4] = (current[2] >> 8) & 0xFF; 
    data[5] = current[2] & 0xFF;
    data[6] = (current[3] >> 8) & 0xFF; 
    data[7] = current[3] & 0xFF;
    
    uint16_t id = 0x200;
    CAN_Send_Msg(hcan_, data, id, 8);
}

double G3508_Motor::OmegaCompute()
{
    double dt = 0.001;  //控制周期
    double targetOmega = positionPID.compute(targetPosition, realPosition, dt);
    return targetOmega;
}

void G3508_Motor::GetFeedback(double angle)
{
    realPosition = angle;
}

DM4310_Motor::DM4310_Motor(CAN_HandleTypeDef* hcan, uint16_t id)
{
    hcan_ = hcan;
    id_ = id;
}

double int_to_degree(int int_angle)
{
    const uint16_t cal_0 = 18700;    // 0度基准值，是手动测出来的，枪口对准前方，逆时针为正

    // 调整编码器值，处理溢出
    int32_t adjusted = int_angle - cal_0;
    if (adjusted < 0) {
        adjusted += 65536;  // 16位编码器最大值+1
    }
    
    // 计算角度 (65536个计数对应360度)
    float angle = (float)adjusted / 65536.0f * 360.0f;
    
    // 确保角度在范围内
    if (angle < -180) 
    {
        angle += 360.0f;
    }
    if (angle >= 180.0f) 
    {
        angle -= 360.0f;
    }
    return angle;
}

int degree_to_int(double angle)
{
    const uint16_t cal_0 = 18700;   // 0度基准值，是手动测出来的，枪口对准前方，逆时针为正
    
    // 规范化角度到[0, 360)范围
    angle = fmod(angle, 360.0);
    if (angle < 0) angle += 360.0;
    
    // 计算编码器值
    int32_t int_angle = (int32_t)round((angle / 360.0) * 65536.0) + cal_0;
    
    // 处理16位溢出
    int_angle %= 65536;
    if (int_angle < 0) int_angle += 65536;
    
    return (int)int_angle;
}

void DM4310_Motor::setSpeed()
{
    double dt = 0.001;  //控制周期
    double torq = (int)speedPID.compute(targetSpeed, realSpeed, dt);

    if (torq > 7)
    {
        torq = 7;
    }
    if (torq < -7)
    {
        torq = -7;
    }
    mit_ctrl(hcan_, id_, 0, 0, 0, 0, torq);
}

void DM4310_Motor::setPosition()
{
    double dt = 0.001;  //控制周期
    targetSpeed = positionPID.compute(targetPosition, realPosition, dt);
    setSpeed();
}

void DM4310_Motor::GetFeedback(double chassisPosition)
{
    realPosition = int_to_degree(fbData[0]) + chassisPosition;
    realSpeed = fbData[1] - 2048;   //速度为0时，返回2048
}

CAN_HandleTypeDef* DM4310_Motor::get_hcan()
{
    return hcan_;
}

uint16_t DM4310_Motor::get_id()
{
    return id_;
}

G3508_Motor motor = G3508_Motor(&hcan2);

DM4310_Motor yaw = DM4310_Motor(&hcan1, 2);