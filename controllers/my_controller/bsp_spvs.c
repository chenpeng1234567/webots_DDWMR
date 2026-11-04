#include "bsp_spvs.h"

wb_spvs_data_t wb_spvs_data;

//前三个分别是 x、y 和 z 方向的线速度。最后三个分别是围绕 x、y 和 z 轴的角速度。
void Read_SPVS_Val(wb_spvs_data_t *wb_spvs_data)
{
    wb_spvs_data->node_name = wb_supervisor_node_get_self();
    wb_spvs_data->node_vel = wb_supervisor_node_get_velocity(wb_spvs_data->node_name);
}

const double Get_SPVS_X_Vel()
{
    return wb_spvs_data.node_vel[0];
}

const double Get_SPVS_Y_Vel()
{
    return wb_spvs_data.node_vel[1];
}

//转化米为单位
const double Get_SPVS_X_Vel_mm()
{
    return wb_spvs_data.node_vel[0] * 1000.0f;
}

const double Get_SPVS_Y_Vel_mm()
{
    return wb_spvs_data.node_vel[1] * 1000.0f;
}

const double Get_SPVS_Z_Angular_Vel()
{
    return wb_spvs_data.node_vel[5];
}