#ifndef CONTROL_H
#define CONTROL_H

#include "../types.h"

#define CONTROL_DEBUG

typedef struct {
    // State prediction
    f64 pos_x; // m
    f64 pos_y; // m
    f64 pos_z; // m
    f64 vel_z; // m/s
    f64 acc_z; // m/s^2
    f64 ori_x; // rad
    f64 ori_y; // rad
    f64 ori_z; // rad
    f64 rot_x; // rad/s
    f64 rot_y; // rad/s
    f64 rot_z; // rad/s

    // Sensor readings
    f64 alt_m_rdng; // m
    f64 acc_z_rdng; // m/s^2
    f64 rot_x_rdng; // rot/s
    f64 rot_y_rdng; // rot/s
    f64 rot_z_rdng; // rot/s
    f64 pos_x_rdng;
    f64 pos_y_rdng;

    // PIDs
    f64 pid_out_vel;
} DebugInterface;

typedef struct {
    // Controller Inputs
    u32 pressure; // Pa
    i16 imu_acc_x;
    i16 imu_acc_y;
    i16 imu_acc_z;
    i16 imu_rot_x;
    i16 imu_rot_y;
    i16 imu_rot_z;
    i32 pos_x;  // cm
    i32 pos_y;  // cm

    // Controller Outputs
    f64 rot_cmd[4]; // [0, 1]

    // Controller Debug
#ifdef CONTROL_DEBUG
    DebugInterface dbg;
#endif
} ControllerInterface;

void c_init();

#define CONTROL_FQ 1000
void c_step(ControllerInterface *intr);

#endif