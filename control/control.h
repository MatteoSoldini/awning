#ifndef CONTROL_H
#define CONTROL_H

#include <stdint.h>

#define CONTROL_DEBUG

typedef struct {
    double x_pos;
    double x_vel;
    double x_sens;
} DebugInterface;

typedef struct {
    // Controller Inputs
    int32_t pressure; // Pa

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