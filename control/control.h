#ifndef CONTROL_H
#define CONTROL_H

#include <stdint.h>

typedef struct {
    // Controller Inputs
    int32_t pressure; // Pa

    double real_z;

    // Controller Outputs
    double rot_w[4];
} ControllerInterface;

#define CONTROL_FQ 1000
void control_step(ControllerInterface *intr);

#endif