#include "PID_Nav.h"
#include "bsp_spvs.h"
#include "bsp_gps.h"
#include "bsp_imu.h"
#include "chassis.h"
#include <math.h>
#include <string.h>

/* Path coordinates remain mm. Controller velocities use m/s and rad/s. */
#define WAYPOINT_RADIUS_M 0.07f
#define FINAL_RADIUS_M 0.010f
#define MAX_LINEAR_MPS 0.18f
#define MAX_ANGULAR_RPS 1.0f
#define HEADING_SLEW_RPS (5.0f * DEG_TO_RAD / T)

pid_nav_t pid_nav;
fzy_pid_t fzy_pid_x;
fzy_pid_t fzy_pid_y;
fzy_pid_t fzy_pid_v;
static float last_angle_error;
static float filtered_derivative;
static int angle_initialized;
static float angle_raw;

float Nav_Wrap_Pi(float angle) {
    return atan2f(sinf(angle), cosf(angle));
}

void Angle_Limit(float *angle) {
    *angle = Nav_Wrap_Pi(*angle);
}

void PID_Nav_Init(pid_nav_t *nav, fzy_pid_t *px, fzy_pid_t *py) {
    memset(nav, 0, sizeof(*nav));
    fuzzy_init(px, -250, 250, -100, 100, .015f, .018f, 0, 0, .010f, .012f, 0, 50);
    fuzzy_init(py, -250, 250, -100, 100, .015f, .018f, 0, 0, .010f, .012f, 0, 50);
    nav->nav_state = NAV_PATH_NOT_DONE;
    nav->track_state = TRACK_POINT_NOT_DONE;
    nav->err_cir_x = nav->err_cir_y = WAYPOINT_RADIUS_M * 1000;
    nav->vx_limit = nav->vy_limit = MAX_LINEAR_MPS;
    last_angle_error = filtered_derivative = angle_raw = 0;
    angle_initialized = 0;
}

/* kd is in seconds: old 4 * delta_error is 0.256 * delta_error / 0.064. */
float Navigation_Angle_Ctrl(float present, float target, float kp, float kd) {
    float error = Nav_Wrap_Pi(target - present);
    float derivative = angle_initialized ? Nav_Wrap_Pi(error - last_angle_error) / T : 0;
    filtered_derivative = .5f * derivative + .5f * filtered_derivative;
    last_angle_error = error;
    angle_initialized = 1;
    angle_raw = kp * error + kd * filtered_derivative;
    return fmaxf(-MAX_ANGULAR_RPS, fminf(MAX_ANGULAR_RPS, angle_raw));
}

/* Projection diagnostics are against the active incoming path segment. */
static void update_geometry(pid_nav_t *nav, path_t *path) {
    unsigned i = nav->point_index;
    nav->x_set = path[i].x;
    nav->y_set = path[i].y;
    nav->v_plan = path[i].plan_v;
    float x = Get_GPS_X(), y = Get_GPS_Y();
    float tx = path[i].x * .001f, ty = path[i].y * .001f;
    nav->distance_m = hypotf(tx - x, ty - y);
    nav->cross_track_m = 0;
    nav->segment_progress = 0;
    nav->corner_angle_rad = 0;
    nav->waypoint_radius_m = WAYPOINT_RADIUS_M;
    if (i > 0 && i < PATH_1_NUM_DEF - 1) {
        float incoming = atan2f(path[i].y-path[i-1].y, path[i].x-path[i-1].x);
        float outgoing = atan2f(path[i+1].y-path[i].y, path[i+1].x-path[i].x);
        nav->corner_angle_rad = fabsf(Nav_Wrap_Pi(outgoing-incoming));
        /* Sharp corners must be reached more closely before switching. */
        if (nav->corner_angle_rad > 40*DEG_TO_RAD) nav->waypoint_radius_m = .020f;
        else if (nav->corner_angle_rad > 20*DEG_TO_RAD) nav->waypoint_radius_m = .035f;
    }
    nav->err_cir_x = nav->err_cir_y = nav->waypoint_radius_m*1000;
    if (i > 0) {
        float ax = path[i-1].x * .001f, ay = path[i-1].y * .001f;
        float dx = tx - ax, dy = ty - ay;
        float len = hypotf(dx, dy);
        if (len > 1e-6f) {
            nav->segment_progress = ((x-ax)*dx + (y-ay)*dy) / (len*len);
            nav->cross_track_m = (dx*(y-ay) - dy*(x-ax)) / len;
        }
    }
}

void PID_Nav_Point_Handler(pid_nav_t *nav, path_t *path) {
    if (nav->nav_state == NAV_PATH_DONE) return;
    if (!nav->heading_initialized) {
        nav->tgt_theta = Get_IMU_Yaw();
        nav->heading_initialized = 1;
    }
    update_geometry(nav, path);
    /* Skip reached points, or passed points only when close to their segment. */
    while (nav->point_index < PATH_1_NUM_DEF - 1 &&
           (nav->distance_m <= nav->waypoint_radius_m ||
            (nav->segment_progress >= 1 && fabsf(nav->cross_track_m) < nav->waypoint_radius_m))) {
        nav->point_index++;
        nav->track_state = TRACK_POINT_DONE;
        update_geometry(nav, path);
        /* Preserve tgt_theta across switches; path theta is not a steering command. */
    }
    if (nav->point_index == PATH_1_NUM_DEF - 1) {
        nav->nav_state = NAV_TRACK_FINAL;
        if (nav->distance_m <= FINAL_RADIUS_M) {
            nav->nav_state = NAV_PATH_DONE;
            nav->rotating = 0;
        }
    }
}

vel_t PID_Nav_Point_Tracker(pid_nav_t *nav, fzy_pid_t *px, fzy_pid_t *py, path_t *path) {
    vel_t out = {0, 0, 0};
    (void)px; (void)py; (void)path;
    nav->v_set = nav->angular_raw = 0;
    nav->heading_speed_scale = 0;
    nav->heading_error_rad = nav->bearing_error_rad = 0;
    if (nav->nav_state == NAV_PATH_DONE) return out;

    float dx = nav->x_set * .001f - Get_GPS_X();
    float dy = nav->y_set * .001f - Get_GPS_Y();
    float yaw = Get_IMU_Yaw();
    nav->bearing_rad = atan2f(dy, dx);
    float change = Nav_Wrap_Pi(nav->bearing_rad - nav->tgt_theta);
    float max_change = HEADING_SLEW_RPS * T;
    nav->tgt_theta = Nav_Wrap_Pi(nav->tgt_theta + fmaxf(-max_change, fminf(max_change, change)));
    nav->heading_error_rad = Nav_Wrap_Pi(nav->tgt_theta - yaw);
    nav->bearing_error_rad = Nav_Wrap_Pi(nav->bearing_rad - yaw);

    /* Hysteresis avoids toggling near the rotation threshold. Use true bearing,
       so smoothing cannot conceal a large error after switching waypoints. */
    if (fabsf(nav->bearing_error_rad) > 45 * DEG_TO_RAD) nav->rotating = 1;
    else if (fabsf(nav->bearing_error_rad) < 15 * DEG_TO_RAD) nav->rotating = 0;

    float speed = fminf(nav->v_plan, MAX_LINEAR_MPS);
    nav->corner_speed_limit_m_s = MAX_LINEAR_MPS;
    if (nav->point_index < PATH_1_NUM_DEF - 1) {
        /* Braking envelope v^2 = corner_v^2 + 2*a*remaining_distance.
           a=0.04 m/s^2 starts braking ~0.2 m ahead of a sharp corner. */
        float corner_v = nav->corner_angle_rad > 40*DEG_TO_RAD ? .045f :
                         nav->corner_angle_rad > 20*DEG_TO_RAD ? .075f : .1375f;
        float remaining = fmaxf(0, nav->distance_m-nav->waypoint_radius_m);
        nav->corner_speed_limit_m_s = sqrtf(corner_v*corner_v + .08f*remaining);
        speed = fminf(speed, nav->corner_speed_limit_m_s);
    }
    /* Decelerate continuously into the goal. No Cartesian final-point PID:
       a differential robot cannot independently execute world X/Y velocity. */
    if (nav->nav_state == NAV_TRACK_FINAL) speed = fminf(speed, .8f * nav->distance_m);
    /* Slow more strongly during the heading transient after switching. */
    float alignment = fmaxf(0, cosf(nav->bearing_error_rad));
    nav->heading_speed_scale = alignment*alignment;
    speed *= nav->heading_speed_scale;
    if (nav->rotating) speed = 0;
    nav->v_set = speed;
    out.vx = speed * cosf(yaw);
    out.vy = speed * sinf(yaw);
    out.ang_w = Navigation_Angle_Ctrl(yaw, nav->tgt_theta, 4, .256f);
    nav->angular_raw = angle_raw;
    return out;
}
