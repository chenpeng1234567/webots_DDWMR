#include "chassis.h"
#include <math.h>
#include <string.h>

helm_chassis_t helm_chassis;

int Helm_Chassis_Init(void) {
    memset(&helm_chassis, 0, sizeof(helm_chassis));
    helm_chassis.chassis_cor = GLOBAL_COORDINATE;
    helm_chassis.wheel_limit = 15;
    helm_chassis.wheel_scale = 1;
    return 1;
}

/* vx/vy in m/s, wz in rad/s; project onto the robot's forward axis. */
void Helm_Chassis_Ctrl(float vx, float vy, float wz, helm_chassis_t *c, float yaw) {
    c->dir_chassis = yaw;
    float v = c->chassis_cor == GLOBAL_COORDINATE ? vx*cosf(yaw) + vy*sinf(yaw) : vx;
    if (c->chassis_cor == LOCK_CHASSIS) { v = 0; wz = 0; }
    c->body_v = v;
    c->left_raw = (v - wz * WHEEL_BASE * .5f) / WHEEL_RADIUS;
    c->right_raw = (v + wz * WHEEL_BASE * .5f) / WHEEL_RADIUS;
    float peak = fmaxf(fabsf(c->left_raw), fabsf(c->right_raw));
    c->wheel_scale = peak > c->wheel_limit ? c->wheel_limit / peak : 1;
    /* Preserve curvature when either motor saturates. Names retained for main. */
    c->wheel1.v_target_right = c->left_raw * c->wheel_scale;
    c->wheel2.v_target_left = c->right_raw * c->wheel_scale;
}

void Helm_Wheel_Ctrl(float vx, float vy, float wz, helm_wheel_t *wheel) {
    (void)vy;
    wheel->VC = vx / WHEEL_RADIUS;
    wheel->v_target_right = (vx - wz * WHEEL_BASE * .5f) / WHEEL_RADIUS;
    wheel->v_target_left = (vx + wz * WHEEL_BASE * .5f) / WHEEL_RADIUS;
}
