/* Differential tracking controller. Detailed logs describe state at sample t;
   commands on that row are applied for the NEXT simulation interval. */
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/keyboard.h>
#include <webots/pen.h>
#include <webots/position_sensor.h>
#include <math.h>
#include <stdio.h>
#include <time.h>
#include <direct.h>
#include <io.h>
#include <fcntl.h>
#include <sys/stat.h>
#include "bsp_imu.h"
#include "bsp_gps.h"
#include "bsp_spvs.h"
#include "chassis.h"
#include "PID_Nav.h"

#define TIME_STEP 64
#define RUN_VERSION "tracking-fix-v2"

int main(void) {
    wb_robot_init();
    WbDeviceTag left = wb_robot_get_device("left_motor");
    WbDeviceTag right = wb_robot_get_device("right_motor");
    WbDeviceTag gps = wb_robot_get_device("gps");
    WbDeviceTag imu = wb_robot_get_device("imu");
    WbDeviceTag ls = wb_robot_get_device("left wheel sensor");
    WbDeviceTag rs = wb_robot_get_device("right wheel sensor");
    WbDeviceTag pen = wb_robot_get_device("pen");
    wb_motor_set_position(left, INFINITY);
    wb_motor_set_position(right, INFINITY);
    wb_motor_set_velocity(left, 0);
    wb_motor_set_velocity(right, 0);
    wb_gps_enable(gps, TIME_STEP);
    wb_inertial_unit_enable(imu, TIME_STEP);
    wb_position_sensor_enable(ls, TIME_STEP);
    wb_position_sensor_enable(rs, TIME_STEP);
    wb_keyboard_enable(TIME_STEP);
    wb_pen_set_ink_color(pen, 0x2060d0, 1);
    wb_pen_write(pen, 1);
    Helm_Chassis_Init();
    helm_chassis.wheel_limit = fmin(wb_motor_get_max_velocity(left), wb_motor_get_max_velocity(right));
    PID_Nav_Init(&pid_nav, &fzy_pid_x, &fzy_pid_y);

    _mkdir("logs");
    time_t now = time(NULL);
    struct tm *local = localtime(&now);
    char stem[80], filename[160];
    strftime(stem, sizeof(stem), "logs/run_%Y%m%d_%H%M%S", local);
    /* Exclusive creation prevents overwriting a prior run, even in the same second. */
    FILE *log = NULL;
    for (int suffix = 0; suffix < 1000 && !log; ++suffix) {
        snprintf(filename, sizeof(filename), "%s_%03d.csv", stem, suffix);
        int fd = _open(filename, _O_WRONLY | _O_CREAT | _O_EXCL | _O_TEXT,
                       _S_IREAD | _S_IWRITE);
        if (fd >= 0) {
            log = _fdopen(fd, "w");
            if (!log) _close(fd);
        }
    }
    if (!log) {
        fprintf(stderr, "Cannot create diagnostic log; controller stopped.\n");
        wb_robot_cleanup();
        return 1;
    }
    setvbuf(log, NULL, _IOLBF, 0);
    fprintf(log, "# version=%s,dt_s=0.064,path_points=%d,wheel_radius_m=%.6f,wheel_base_m=%.6f,wheel_limit_rad_s=%.6f,final_radius_m=0.010,adaptive_corner_radius=1,commands_apply_next_interval=1\n",
            RUN_VERSION, PATH_1_NUM_DEF, WHEEL_RADIUS, WHEEL_BASE, helm_chassis.wheel_limit);
    fprintf(log, "time_s,dt_s,index,state,event,sensor_ok,x_m,y_m,yaw_rad,target_x_m,target_y_m,distance_m,cross_track_m,segment_progress,bearing_rad,target_heading_rad,heading_error_rad,bearing_error_rad,rotating,v_plan_m_s,v_cmd_m_s,w_raw_rad_s,w_cmd_rad_s,left_raw_rad_s,right_raw_rad_s,wheel_scale,left_cmd_rad_s,right_cmd_rad_s,vx_actual_m_s,vy_actual_m_s,v_body_actual_m_s,w_actual_rad_s,left_actual_rad_s,right_actual_rad_s,corner_angle_rad,waypoint_radius_m,corner_speed_limit_m_s,heading_speed_scale\n");
    FILE *positions = fopen("data.txt", "w");
    printf("[%s] Log: %s\n", RUN_VERSION, filename);
    printf("Path: %d points; cruise 0.1375 m/s. Y: pen on; X: pen off.\n", PATH_1_NUM_DEF);

    int previous_index = -1, previous_state = -1, previous_rotating = -1;
    int tick = 0, invalid_reported = 0;
    double previous_time = NAN, previous_l = NAN, previous_r = NAN;
    while (wb_robot_step(TIME_STEP) != -1) {
        int key = wb_keyboard_get_key();
        if (key == 'Y') wb_pen_write(pen, 1);
        if (key == 'X') wb_pen_write(pen, 0);
        Read_Roll_Pitch_Yaw(imu, &wb_imu_data);
        Read_GPS_Val(gps, &wb_gps_data);
        Read_SPVS_Val(&wb_spvs_data);
        double t = wb_robot_get_time();
        double dt = isfinite(previous_time) ? t - previous_time : TIME_STEP*.001;
        double lpos = wb_position_sensor_get_value(ls), rpos = wb_position_sensor_get_value(rs);
        double la = isfinite(previous_l) && dt > 0 ? (lpos-previous_l)/dt : NAN;
        double ra = isfinite(previous_r) && dt > 0 ? (rpos-previous_r)/dt : NAN;
        previous_time = t; previous_l = lpos; previous_r = rpos;
        double x = Get_GPS_X(), y = Get_GPS_Y(), yaw = Get_IMU_Yaw();
        double avx = Get_SPVS_X_Vel(), avy = Get_SPVS_Y_Vel(), aw = Get_SPVS_Z_Angular_Vel();
        int valid = isfinite(x) && isfinite(y) && isfinite(yaw) && isfinite(avx) && isfinite(avy) && isfinite(aw) && isfinite(lpos) && isfinite(rpos);
        vel_t command = {0, 0, 0};
        if (valid) {
            PID_Nav_Point_Handler(&pid_nav, path_1);
            command = PID_Nav_Point_Tracker(&pid_nav, &fzy_pid_x, &fzy_pid_y, path_1);
            Helm_Chassis_Ctrl(command.vx, command.vy, command.ang_w, &helm_chassis, yaw);
        } else {
            Helm_Chassis_Ctrl(0, 0, 0, &helm_chassis, 0);
            if (!invalid_reported) fprintf(stderr, "Invalid sensor reading at %.3f s; motors stopped, see CSV.\n", t);
            invalid_reported = 1;
        }
        wb_motor_set_velocity(left, helm_chassis.wheel1.v_target_right);
        wb_motor_set_velocity(right, helm_chassis.wheel2.v_target_left);
        const char *event = !valid ? "sensor_invalid" :
            previous_state != pid_nav.nav_state ? (pid_nav.nav_state == NAV_PATH_DONE ? "done" : "state_change") :
            previous_index != pid_nav.point_index ? "waypoint_change" :
            previous_rotating != pid_nav.rotating ? "rotation_change" :
            helm_chassis.wheel_scale < .999f ? "wheel_saturation" : "step";
        fprintf(log,
            "%.6f,%.6f,%u,%u,%s,%d,%.9f,%.9f,%.9f,%.6f,%.6f,%.6f,%.6f,%.6f,%.9f,%.9f,%.9f,%.9f,%d,%.6f,%.6f,%.9f,%.9f,%.6f,%.6f,%.6f,%.6f,%.6f,%.9f,%.9f,%.9f,%.9f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
            t, dt, pid_nav.point_index, pid_nav.nav_state, event, valid, x, y, yaw,
            pid_nav.x_set*.001, pid_nav.y_set*.001, pid_nav.distance_m, pid_nav.cross_track_m,
            pid_nav.segment_progress, pid_nav.bearing_rad, pid_nav.tgt_theta,
            pid_nav.heading_error_rad, pid_nav.bearing_error_rad, pid_nav.rotating,
            pid_nav.v_plan, valid ? pid_nav.v_set : 0, valid ? pid_nav.angular_raw : 0, command.ang_w,
            helm_chassis.left_raw, helm_chassis.right_raw, helm_chassis.wheel_scale,
            helm_chassis.wheel1.v_target_right, helm_chassis.wheel2.v_target_left,
            avx, avy, avx*cos(yaw)+avy*sin(yaw), aw, la, ra,
            pid_nav.corner_angle_rad, pid_nav.waypoint_radius_m,
            pid_nav.corner_speed_limit_m_s, pid_nav.heading_speed_scale);
        fflush(log);
        if (positions && valid) { fprintf(positions, "%.9f, %.9f\n", x, y); fflush(positions); }
        if (tick++ % 16 == 0 || previous_index != pid_nav.point_index || previous_state != pid_nav.nav_state) {
            printf("t=%.2f index=%u/%d state=%u dist=%.3fm cte=%.3fm heading_err=%.1fdeg v=%.3f w=%.3f event=%s\n",
                   t, pid_nav.point_index, PATH_1_NUM_DEF-1, pid_nav.nav_state, pid_nav.distance_m,
                   pid_nav.cross_track_m, pid_nav.bearing_error_rad*RAD_TO_DEG,
                   valid ? pid_nav.v_set : 0, command.ang_w, event);
        }
        previous_index = pid_nav.point_index;
        previous_state = pid_nav.nav_state;
        previous_rotating = pid_nav.rotating;
    }
    fclose(log);
    if (positions) fclose(positions);
    wb_robot_cleanup();
    return 0;
}
