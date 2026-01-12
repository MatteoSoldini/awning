#ifndef CONTROL_H
#define CONTROL_H

#include <types.h>

#define CONTROL_DEBUG

#define NUM_ROT 4

enum DebugValue {
    DBG_POS_X,
    DBG_POS_Y,
    DBG_POS_Z,
    DBG_VEL_X,
    DBG_VEL_Y,
    DBG_VEL_Z,
    DBG_ACC_X,
    DBG_ACC_Y,
    DBG_ACC_Z,
    DBG_ORI_X,
    DBG_ORI_Y,
    DBG_ORI_Z,
    DBG_ROT_X,
    DBG_ROT_Y,
    DBG_ROT_Z,
    DBG_ROT_W0,
    DBG_ALT_RDNG,
    DBG_NUM
};

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
    f64 mag_x;
    f64 mag_y;
    f64 mag_z;

    // Controller Outputs
    f64 rot_cmd[NUM_ROT]; // [0, 1]

    // Controller Debug
#ifdef CONTROL_DEBUG
    f64 dbg[DBG_NUM];
#endif
} ControllerInterface;

void c_init();

#define CONTROL_FQ 1000
void c_step(ControllerInterface *intr);

#endif