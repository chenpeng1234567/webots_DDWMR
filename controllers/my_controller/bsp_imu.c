#include "bsp_imu.h"

wb_imu_data_t wb_imu_data;

void Read_Roll_Pitch_Yaw(WbDeviceTag Tag, wb_imu_data_t *wb_imu_data)
{
    wb_imu_data->imu_val = wb_inertial_unit_get_roll_pitch_yaw(Tag);
}
//横滚角、俯仰角和偏航角

//横滚角表示设备绕其 x 轴的旋转角度，间隔为 [-π，π]。当 InertialUnit 水平时，即当其 y 轴与重力方向相反时，滚动角度为零（WorldInfo 定义坐标系）。
//俯仰角表示设备的旋转角度是 z 轴，间隔为 [-π/2，π/2]。当 InertialUnit 水平时，即当其 y 轴与重力方向相反时，俯仰角为零。如果惯性单元以标准方向放置在机器人上，则机器人下降时俯仰角为负，机器人上升时俯仰角为正。
//偏航角表示相对于 WorldInfo 的单位方向，在区间 [-π，π] 内。 coordinateSystem .当惯性单位的 x 轴与北方向对齐时，偏航角为零，当单位向东时偏航角为 π/2，当单位朝向西方时，偏航角为 -π/2。偏航角可用作指南针。
//依次为roll, pitch and yaw
const double Get_IMU_Roll()
{
    return wb_imu_data.imu_val[0];
}

const double Get_IMU_Pitch()
{
    return wb_imu_data.imu_val[1];
}

const double Get_IMU_Yaw()
{
    return wb_imu_data.imu_val[2];
}

