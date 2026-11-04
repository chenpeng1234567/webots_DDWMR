#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "PID_Nav.h"
#include "chassis.h"
static double x,y,yaw;
const double Get_GPS_X(void){return x;}
const double Get_GPS_Y(void){return y;}
const double Get_IMU_Yaw(void){return yaw;}
int main(void){
 assert(PATH_1_NUM_DEF==61 && path_1[60].y==3000);
 assert(fabsf(Nav_Wrap_Pi(-358*DEG_TO_RAD)-2*DEG_TO_RAD)<1e-5);
 PID_Nav_Init(&pid_nav,&fzy_pid_x,&fzy_pid_y);
 assert(Navigation_Angle_Ctrl(179*DEG_TO_RAD,-179*DEG_TO_RAD,4,.256f)>0);
 Helm_Chassis_Init(); Helm_Chassis_Ctrl(.1,0,0,&helm_chassis,0);
 assert(fabsf(helm_chassis.wheel1.v_target_right-4)<1e-5);
 Helm_Chassis_Ctrl(0,0,1,&helm_chassis,0);
 assert(helm_chassis.wheel1.v_target_right<0 && helm_chassis.wheel2.v_target_left>0);
 Helm_Chassis_Ctrl(1,0,1,&helm_chassis,0);
 assert(helm_chassis.wheel_scale<1 && fabsf(helm_chassis.wheel2.v_target_left-15)<1e-5);
 pid_nav.heading_initialized=1;pid_nav.tgt_theta=2;
 PID_Nav_Point_Handler(&pid_nav,path_1);
 assert(pid_nav.point_index==1 && pid_nav.tgt_theta==2);
 PID_Nav_Init(&pid_nav,&fzy_pid_x,&fzy_pid_y);
 pid_nav.point_index=40;
 PID_Nav_Point_Handler(&pid_nav,path_1);
 assert(pid_nav.corner_angle_rad>40*DEG_TO_RAD && fabsf(pid_nav.waypoint_radius_m-.020f)<1e-6);
 PID_Nav_Init(&pid_nav,&fzy_pid_x,&fzy_pid_y);Helm_Chassis_Init();
 int steps, rotation_samples=0;double max_cte=0;
 for(steps=0;steps<30000;++steps){
  PID_Nav_Point_Handler(&pid_nav,path_1);
  vel_t v=PID_Nav_Point_Tracker(&pid_nav,&fzy_pid_x,&fzy_pid_y,path_1);
  Helm_Chassis_Ctrl(v.vx,v.vy,v.ang_w,&helm_chassis,yaw);
  rotation_samples += pid_nav.rotating;
  double l=helm_chassis.wheel1.v_target_right,r=helm_chassis.wheel2.v_target_left;
  assert(isfinite(l)&&isfinite(r)&&fabs(l)<=15.0001&&fabs(r)<=15.0001);
  if(pid_nav.nav_state==NAV_PATH_DONE){assert(l==0&&r==0);break;}
  double speed=(l+r)*WHEEL_RADIUS*.5,w=(r-l)*WHEEL_RADIUS/WHEEL_BASE;
  x+=speed*cos(yaw+w*T*.5)*T;y+=speed*sin(yaw+w*T*.5)*T;yaw=Nav_Wrap_Pi(yaw+w*T);
  if(fabs(pid_nav.cross_track_m)>max_cte)max_cte=fabs(pid_nav.cross_track_m);
 }
 printf("ideal rollout: steps=%d time=%.1fs final=(%.4f,%.4f)m max_active_segment_cte=%.4fm state=%u\n",steps,steps*T,x,y,max_cte,pid_nav.nav_state);
 assert(pid_nav.nav_state==NAV_PATH_DONE && hypot(x,y-3)<=.0101);
 printf("rotation samples=%d\n",rotation_samples);
 puts("PASS: seam, waypoint heading, wheel units, saturation, full path, terminal stop");return 0;
}
