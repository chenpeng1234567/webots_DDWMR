#ifndef _CHASSIS_H
#define _CHASSIS_H

#include <webots/motor.h>
#include <webots/robot.h>
#include "bsp_imu.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define T		    0.064

#define ANGLE2RAD(x) (x / 180.0f * M_PI) //角度转弧度
#define RAD2ANGLE(x) (x / M_PI * 180.0f) //弧度转角度

#define DIS_WHEEL2CENTER 0.56568542494923801952067548968388
#define WHEEL_BASE  0.09       // 轮间距（单位：米）
#define WHEEL_RADIUS 0.025     // 轮子半径（单位：米）

typedef struct
{
    float ct_vx;
    float ct_vy;
    float ct_wz;
}chassis_ct_t;

typedef struct 
{
    float ct_fdb_vx;
    float ct_fdb_vy;
    float ct_fdb_wz;
}chassis_fdb_ct_t;

typedef enum
{
	GLOBAL_COORDINATE,	//基于全局坐标系控制
	CHASSIS_COORDINATE,	//基于机器人底盘坐标系控制
	LOCK_CHASSIS
}chassis_cor_t;

typedef struct
{
    float angle_actual;
    float angle_target;
    float VN2X; //轮系线速度与X轴的夹角 -180° ~ 180°
    float v_target_left; //期望
    float v_target_right;
    float VC;
}helm_wheel_t;

typedef struct
{
	//速度大小
	float vel;
	//速度方向
	float direction;
	//角速度大小
	float omega;
}robotVel_t;

typedef struct
{
    chassis_ct_t chassis_ct;
    chassis_fdb_ct_t chassis_fdb_ct;
    helm_wheel_t wheel1;
    helm_wheel_t wheel2;
    
    float left_raw;
    float right_raw;
    float wheel_limit;
    float wheel_scale;
    float body_v;
    float dir_chassis;
    chassis_cor_t chassis_cor;
}helm_chassis_t;

/** Extern **/
extern helm_chassis_t helm_chassis;

/* Function */
//void Chassis_Wb_Init(void);
//void Chassis_Self_Spin(void);
int Helm_Chassis_Init(void);
void Helm_Chassis_Ctrl(float vx_input, float vy_input, float wz_input, helm_chassis_t *chassis,  float yaw);
void Helm_Wheel_Ctrl(float vx, float vy, float wz, helm_wheel_t *wheel_ptr);
void V_Dir2Wheel_Angle(float VX, float VY, helm_wheel_t *wheel, float dir);
void Angle_Limit(float *angle);
//void V_Dir2Wheel_Angle(helm_wheel_t *, float);
//void DiffDrive_Ctrl(float vx_input, float vy_input, float wz_input, float yaw, diff_drive_t *drive);


#endif
