#ifndef CONTROL_H
#define CONTROL_H

#include <stdint.h>

#define CONTROL_DEBUG

typedef struct {
    double pos_z; // m
    double vel_z; // m/s
    double acc_z; // m/s^2
    double alt_m_rdng; // m
} DebugInterface;

typedef struct {
    // Controller Inputs
    uint32_t pressure; // Pa
    int16_t imu_acc_x;
    int16_t imu_acc_y;
    int16_t imu_acc_z;

    // Controller Outputs
    double rot_w[4];

    // Controller Debug
#ifdef CONTROL_DEBUG
    DebugInterface dbg;
#endif
} ControllerInterface;

#define CONTROL_FQ 1000
void control_step(ControllerInterface *intr);

#endif