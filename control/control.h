#ifndef CONTROL_H
#define CONTROL_H

#include <stdint.h>

#define CONTROL_DEBUG

typedef struct {
    // State prediction
    double pos_z; // m
    double vel_z; // m/s
    double acc_z; // m/s^2
    double ori_x; // rad
    double ori_y; // rad
    double ori_z; // rad
    double rot_x; // rad/s
    double rot_y; // rad/s
    double rot_z; // rad/s

    // Sensor readings
    double alt_m_rdng; // m
    double acc_z_rdng; // m/s^2
    double rot_x_rdng; // rot/s
    double rot_y_rdng; // rot/s
    double rot_z_rdng; // rot/s

    // PIDs
    double pid_out_vel;
} DebugInterface;

typedef struct {
    // Controller Inputs
    uint32_t pressure; // Pa
    int16_t imu_acc_x;
    int16_t imu_acc_y;
    int16_t imu_acc_z;
    int16_t imu_rot_x;
    int16_t imu_rot_y;
    int16_t imu_rot_z;

    // Controller Outputs
    double rot_cmd[4]; // [0, 1]

    // Controller Debug
#ifdef CONTROL_DEBUG
    DebugInterface dbg;
#endif
} ControllerInterface;

#define CONTROL_FQ 1000
void control_step(ControllerInterface *intr);

#endif