#ifndef PID_NAV_H
#define PID_NAV_H

#include "stdint.h"
#include "fuzzy_pid.h"
#include "path.h"

#define NAV_PATH_DONE (10)
#define NAV_PATH_NOT_DONE (11)
#define NAV_TRACK_FINAL (12)

#define TRACK_POINT_DONE (20)
#define TRACK_POINT_NOT_DONE (21)

#define ANGLE2RAD(x) (x / 180.0f * M_PI) //角度转弧度
#define RAD2ANGLE(x) (x / M_PI * 180.0f) //弧度转角度
#define DEG_TO_RAD (3.1415926f / 180.0f)
#define RAD_TO_DEG (180.0f / 3.1415926f)

#define WZ_MAX 2.0f  // 最大角速度（单位：rad/s），可根据实际需求调整

typedef struct
{
    float x_set;
    float y_set;
    float theta_set;

    float v_set;
    float v_plan;

    float tgt_theta;
    float now_theta;

    uint8_t path_index;
    uint8_t point_index;
    uint8_t track_state; //点跟踪状态 DONE/NOT DONE
    uint8_t nav_state;   //轨迹跟踪状态 DONE/ NOT DONE

    float err_cir_x;
    float err_cir_y;

    float vx_limit;
    float vy_limit;

    /* Diagnostics: distances in m, angles in rad, speeds in SI units. */
    float distance_m;
    float cross_track_m;
    float segment_progress;
    float bearing_rad;
    float heading_error_rad;
    float bearing_error_rad;
    float angular_raw;
    float corner_angle_rad;
    float waypoint_radius_m;
    float corner_speed_limit_m_s;
    float heading_speed_scale;
    int rotating;
    int heading_initialized;

    // point_t now_point;
    // point_t plan_point;
}pid_nav_t;


typedef struct 
{
    float x;
    float y;
}point_t;

typedef struct 
{
    float vx;
    float vy;
    float ang_w;
}vel_t;

typedef struct
{
	float g_theta;
}robotVel_t1;

extern pid_nav_t pid_nav;
extern fzy_pid_t fzy_pid_x;
extern fzy_pid_t fzy_pid_y;
vel_t PID_Nav_Func(pid_nav_t *pid_nav, fzy_pid_t *fzy_ptr, fzy_pid_t *fzy_ptr2, path_t *path);
void PID_Nav_Init(pid_nav_t *pid_nav, fzy_pid_t *fzy_ptr1, fzy_pid_t *fzy_ptr2);
vel_t PID_Nav_Final_Track(pid_nav_t *pid_nav, fzy_pid_t *fzy_ptr, fzy_pid_t *fzy_ptr2, path_t *path);

void PID_Nav_Point_Handler(pid_nav_t *pid_nav, path_t *path);
vel_t PID_Nav_Point_Tracker(pid_nav_t *pid_nav, fzy_pid_t *fzy_ptr1, fzy_pid_t *fzy_ptr2, path_t *path);

float Navigation_Angle_Ctrl(float anglePresent,float angleTarget,float kp,float kd);
void Angle_Limit(float *angle);
float Nav_Wrap_Pi(float angle);
#endif