#ifndef CONTROL_H
#define CONTROL_H

#include <stdint.h>

#define CONTROL_DEBUG

typedef struct {
    double val1;
    double val2;
} DebugInterface;

typedef struct {
    // Controller Inputs
    uint32_t pressure; // Pa
    int16_t acc_x; // m/s^2
    int16_t acc_y; // m/s^2
    int16_t acc_z; // m/s^2

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