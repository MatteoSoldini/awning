#include "control.h"

#include <stdio.h>
#include <math.h>
#include <assert.h>
#include <stdlib.h>

#include <math/mat.h>
#include <consts.h>

const f64 c_dt = 1.0 / CONTROL_FQ;

// Quadcopter physics
static const f64 mass = 2.0;        // Kg
static const f64 kf = 1e-2;         // Thrust coefficient - N / (rad/s)^2
static const f64 tau_m = 0.05;      // Motor time constant (s)
static const f64 rot_max_w = 50.0;  // rad/s 

// Barometer sensor
const f64 bar_fq = 50.0;  // Make sure it's divisible by CONTROL_FQ
const u64 bar_loops = CONTROL_FQ / bar_fq;
const f64 bar_dt = 1.0 / bar_fq;

// IMU sensor
const f64 imu_fq = 200.0; // Make sure it's divisible by CONTROL_FQ
const u64 imu_loops = CONTROL_FQ / imu_fq;
const f64 imu_dt = 1.0 / imu_fq;

// GNSS sensor
const f64 gnss_fq = 10.0; // Make sure it's divisible by CONTROL_FQ
const u64 gnss_loops = CONTROL_FQ / gnss_fq;
const f64 gnss_dt = 1.0 / gnss_fq;

static const f64 imu_acc_max_value = 8.0*G; // m/s^2
f64 imu_acc_to_ms2(i16 raw_acc) {
    return ((f64)raw_acc / (f64)INT16_MAX) * imu_acc_max_value; 
}

static const f64 imu_rot_max_value = 250.0; // rad/s
f64 imu_gyro_to_rad(i16 raw_gyro) {
    return ((f64)raw_gyro / (f64)INT16_MAX) * imu_rot_max_value; 
}

// Magnetometer
const f64 mag_fq = 100.0; // Make sure it's divisible by CONTROL_FQ
const u64 mag_loops = CONTROL_FQ / mag_fq;
const f64 mag_dt = 1.0 / mag_fq;

// This should only be touched by the pid function
typedef struct {
    f64 i_err;
    f64 p_real;
} PIDState;

// For the moment let's assume this parameters constant
typedef struct {
    const f64 dt;
    const f64 p;
    const f64 i;
    const f64 d;
    const f64 low;
    const f64 high;
} PIDParams;

f64 pid_step(
    f64 real,
    f64 tgt,
    PIDParams *pid_params,
    PIDState *pid_state
) {
    f64 err = tgt - real;
    pid_state->i_err += err * pid_params->dt;
    f64 der = (real - pid_state->p_real) / pid_params->dt;
    pid_state->p_real = real;

    f64 out = pid_params->p*err + pid_params->i*pid_state->i_err + pid_params->d*der;

    if (out > pid_params->high) out = pid_params->high;
    if (out < pid_params->low) out = pid_params->low;

    return out;
}


PIDParams mot_pid_p = {
    .p = 0.2,
    .i = 0.0,
    .d = 0.01,
    .dt = c_dt,
    .high = 0.5,
    .low = -0.5
};
PIDState mot_pid_s = {0};

PIDParams alt_pid_p = {
    .p = 2.0,
    .i = 0.0,
    .d = 0.5,
    .dt = c_dt,
    .high = 1.0,
    .low = -1.0
};
PIDState alt_pid_s = {0};

// pos -> vel
PIDParams pos_pid_p = {
    .p = 0.1,
    .i = 0.0,
    .d = 0.1,
    .dt = c_dt,
    .high = 0.05,    // m/s
    .low = -0.05     // m/s
};
PIDState pos_x_pid_s = {0};
PIDState pos_y_pid_s = {0};

// vel -> ori
PIDParams vel_pid_p = {
    .p = 1.0,
    .i = 0.0,
    .d = 0.1,
    .dt = c_dt,
    .high = 0.2,    // rad
    .low = -0.2     // rad
};
PIDState vel_x_pid_s = {0};
PIDState vel_y_pid_s = {0};

// ori -> rot
PIDParams ori_pid_p = {
    .p = 0.5,
    .i = 0.0,
    .d = 0.1,
    .dt = c_dt,
    .high = 0.2,    // rad/s
    .low = -0.2     // rad/s
};
PIDState ori_x_pid_s = {0};
PIDState ori_y_pid_s = {0};
PIDState ori_z_pid_s = {0};

// rot -> cmd
PIDParams rot_pid_p = {
    .p = 0.1,
    .i = 0.0,
    .d = 0.0,
    .dt = c_dt,
    .high = 0.1,
    .low = -0.1
};
PIDState rot_x_pid_s = {0};
PIDState rot_y_pid_s = {0};

// * https://kalmanfilter.net/

// A glorified EMA filter
// Two stages: predict and correct
// The output is a random (multi)variable

// You have to describe the system dynamics
// It is nice to represent the filter in matrix notation


Mat rotate_vec3_euler_angles(Mat *vec3, f64 alpha, f64 beta, f64 gamma) {
    // https://en.wikipedia.org/wiki/Euler_angles#Rotation_matrix
    // https://en.wikipedia.org/wiki/Rotation_matrix#Basic_3D_rotations
    
    assert(vec3->r == 3 && vec3->c == 1);
    
    Mat rot_mat_x = { .r=3, .c=3, {
        1.0, 0.0,        0.0,
        0.0, cos(alpha), -sin(alpha),
        0.0, sin(alpha), cos(alpha)
    }};
    
    Mat rot_mat_y = { .r=3, .c=3, {
        cos(beta),  0.0, sin(beta),
        0.0,        1.0, 0.0,
        -sin(beta), 0.0, cos(beta)
    }};
    
    Mat rot_mat_z = { .r=3, .c=3, {
        cos(gamma), -sin(gamma), 0.0,
        sin(gamma), cos(gamma),  0.0,
        0.0,        0.0,         1.0 
    }};

    Mat rot_mat_zy = mat_mul(&rot_mat_z, &rot_mat_y);
    Mat rot_mat    = mat_mul(&rot_mat_zy, &rot_mat_x);
    
    Mat v_rot = mat_mul(&rot_mat, vec3);

    return v_rot;
}

// State vector
//
// Using ENU axis convetion:
//  +X: east
//  +Y: north
//  +Z: up

enum {
    S_POS_X, // m
    S_POS_Y, // m
    S_POS_Z, // m
    S_VEL_X, // m/s
    S_VEL_Y, // m/s
    S_VEL_Z, // m/s
    //S_ORI_X, // rad
    //S_ORI_Y, // rad
    //S_ORI_Z, // rad
    //S_ROT_X, // rad/s
    //S_ROT_Y, // rad/s
    //S_ROT_Z, // rad/s
    S_STATE_DIM
};
Mat X = { .r=S_STATE_DIM, .c=1 };

// State transition matrix
Mat F = { .r=S_STATE_DIM, .c=S_STATE_DIM };

// State covariance matrix
Mat P = { .r=S_STATE_DIM, .c=S_STATE_DIM };

// Process covariance matrix
Mat Q = { .r=S_STATE_DIM, .c=S_STATE_DIM };

#define SET_XYZ(M, r0, c0, val) \
    MAT_AT(M, r0+0, c0+0) = val; \
    MAT_AT(M, r0+1, c0+1) = val; \
    MAT_AT(M, r0+2, c0+2) = val;

void c_init() {
    // --- Position ---
    // State transition matrix
    SET_XYZ(F, S_POS_X, S_POS_X, 1.0);
    SET_XYZ(F, S_POS_X, S_VEL_X, c_dt);
    //MAT_AT(F, S_POS_Z, S_ACC_Z) = 0.5*c_dt*c_dt;
    
    // State covariance matrix
    SET_XYZ(P, S_POS_X, S_POS_X, 1e0);
    
    // Process covariance matrix
    SET_XYZ(Q, S_POS_X, S_POS_X, 1e-5);

    // --- Velocity ---
    // State transition matrix
    SET_XYZ(F, S_VEL_X, S_VEL_X, 1.0);
    //MAT_AT(F, S_VEL_Z, S_ACC_Z) = c_dt;
    
    // State covariance matrix
    SET_XYZ(P, S_VEL_X, S_VEL_X, 1e1);
    
    // Process covariance matrix
    SET_XYZ(Q, S_VEL_X, S_VEL_X, 1e-4);

    
    // --- Acceleration ---
    //// State transition matrix
    //MAT_AT(F, S_ACC_Z, S_ACC_Z) = 1.0;
    
    //// State covariance matrix
    //MAT_AT(P, S_ACC_Z, S_ACC_Z) = 1e3;

    //// Process covariance matrix
    //MAT_AT(Q, S_ACC_Z, S_ACC_Z) = 1.0;

    //SET_XYZ(F, S_ACC_X, S_ACC_X, 1.0);
    
    //SET_XYZ(P, S_ACC_X, S_ACC_X, 1e-2);
    
    //SET_XYZ(Q, S_ACC_X, S_ACC_X, 5.0);


    // --- Orientation ---
    // State transition matrix
    //SET_XYZ(F, S_ORI_X, S_ORI_X, 1.0);
    //SET_XYZ(F, S_ORI_X, S_ROT_X, c_dt);
    
    //// State covariance matrix
    //SET_XYZ(P, S_ORI_X, S_ORI_X, 1e-2);
    
    //// Process covariance matrix
    //SET_XYZ(Q, S_ORI_X, S_ORI_X, 1e-2);


    //// --- Rotation ---
    //// State transition matrix
    //SET_XYZ(F, S_ROT_X, S_ROT_X, 1.0);
    
    //// State covariance matrix
    //SET_XYZ(P, S_ROT_X, S_ROT_X, 1e-2);
    
    //// Process covariance matrix
    //SET_XYZ(Q, S_ROT_X, S_ROT_X, 1e-1);
}

void kf_predict(
    Mat *B, // Control transition matrix
    Mat *U  // Control matrix
) {
    // Predict new state
    // X = F*X + B*U
    Mat FX = mat_mul(&F, &X);
    Mat BU = mat_mul(B, U);
    X = mat_sum(&FX, &BU);
    
    // Predict state covariance
    // P(n+1) = F*P(n)*F_t + Q
    Mat F_t = mat_trans(&F);
    P = mat_mul(&F, &P);
    P = mat_mul(&P, &F_t);
    P = mat_sum(&P, &Q);
}

void kf_correct(
    Mat *Z, // Measurements matrix
    Mat *H, // Observation matrix
    Mat *R  // Measurement noise
) {
    assert(Z->r == H->r && R->r == R->c && R->r == H->r && "Wrong arguments dimensions");

    // Map state domain to measurements domain
    Mat HX = mat_mul(H, &X);

    // Compute innovation
    Mat I = mat_sub(Z, &HX);   // This is in measurement space

    // Compute innovation covariance
    // S = H * P_pred * H_t + R
    Mat H_t = mat_trans(H);
    Mat HP = mat_mul(H, &P);
    Mat S = mat_mul(&HP, &H_t);

    S = mat_sum(&S, R);
    
    Mat S_inv = mat_inv(&S);

    // Compute Kalman gain
    // K = P_pred*H_t*S^-1
    Mat K = mat_mul(&P, &H_t);
    K = mat_mul(&K, &S_inv);
    
    // Update state
    // X = X_pred + K * I
    Mat X_corr = mat_mul(&K, &I);
    X = mat_sum(&X, &X_corr);

    // Update covariance
    // P = (Id - K * H) * P_pred
    Mat KH = mat_mul(&K, H);   // Map Kalman Gain from measurement domain to state domain
    Mat Id = mat_identity(KH.r);
    Mat IdminKH = mat_sub(&Id, &KH);
    P = mat_mul(&IdminKH, &P);
}

u64 bar_l = 0;
u64 imu_l = 0;
u64 gnss_l = 0;
u64 mag_l = 0;

f64 c_rot_w[NUM_ROT] = {0};

// Complementary filter for attitude estimation
f64 ori_x = 0.0;
f64 ori_y = 0.0;
f64 ori_z = 0.0;
f64 rot_x = 0.0;
f64 rot_y = 0.0;
f64 rot_z = 0.0;

// Assume that the control_step() function is triggered by an interrupt
// in the MCU every 1ms (1000Hz)
void c_step(ControllerInterface *intr) {
    bar_l++;
    imu_l++;
    gnss_l++;
    mag_l++;
    
    // --- Controlled Input ---

    // Integrate rotation
    ori_x += rot_x * c_dt;
    ori_y += rot_y * c_dt;
    ori_z += rot_z * c_dt;

#ifdef CONTROL_DEBUG
    intr->dbg[DBG_ORI_X] = ori_x;
    intr->dbg[DBG_ORI_Y] = ori_y;
    intr->dbg[DBG_ORI_Z] = ori_z;
    
    intr->dbg[DBG_ROT_X] = rot_x;
    intr->dbg[DBG_ROT_Y] = rot_y;
    intr->dbg[DBG_ROT_Z] = rot_z;
#endif

    for (u64 i=0; i<NUM_ROT; i++) {
        c_rot_w[i] += (intr->rot_cmd[i] * rot_max_w - c_rot_w[i]) * (c_dt / tau_m);
    }
   
#ifdef CONTROL_DEBUG
    intr->dbg[DBG_ROT_W0] = c_rot_w[0];
#endif

    f64 tot_mot_f = 0.0;
    for (u64 i=0; i<NUM_ROT; i++) {
        tot_mot_f += kf * c_rot_w[i] * c_rot_w[i];
    }
    
    Mat body_acc = { .r=3, .c=1, { 0.0, 0.0, tot_mot_f/mass }};

    Mat world_acc = rotate_vec3_euler_angles(&body_acc, ori_x, ori_y, ori_z);
    world_acc.data[2] -= G;

    //world_acc.data[0] = -world_acc.data[0];
    //world_acc.data[1] = -world_acc.data[1];

#ifdef CONTROL_DEBUG
    intr->dbg[DBG_ACC_X] = world_acc.data[0];
    intr->dbg[DBG_ACC_Y] = world_acc.data[1];
    intr->dbg[DBG_ACC_Z] = world_acc.data[2];
#endif

    //Mat U = { .r=1, .c=1, { world_acc.data[2] }};   // Control matrix
    Mat B = { .r=S_STATE_DIM, .c=3 };   // Transition matrix
    SET_XYZ(B, S_POS_X, 0, 0.5*c_dt*c_dt);
    SET_XYZ(B, S_VEL_X, 0, c_dt);

    kf_predict(&B, &world_acc);

    // --- Read sensors ---
    // --- Barometer ---
    if (bar_l >= bar_loops) {
        bar_l=0;

        // Convert Pa to m
        const f64 p0 = 101325; // N/m^2 (Pa) Pressure at sea-level
        const f64 t0 = 15.04;  // Celsius    Temperature at sea-level
        const f64 inv_e = 1.0/5.2561;
    
        f64 t = 288.08 * pow((double)intr->pressure / p0, inv_e) - 273.1;
        f64 alt_m = (t0 - t) / 0.00649;

#ifdef CONTROL_DEBUG
        intr->dbg[DBG_ALT_RDNG] = alt_m;
#endif

        Mat Z = { .r=1, .c=1, { alt_m }};

        const f64 sdev = 0.25;
        Mat R = { .r=1, .c=1, { sdev*sdev }};
        Mat H = { .r=1, .c=S_STATE_DIM };
        MAT_AT(H, 0, S_POS_Z) = 1.0;

        kf_correct(&Z, &H, &R);
    }

    // --- IMU ---
    if (imu_l >= imu_loops) {
        imu_l=0;

        // Body-frame accelerations
        f64 ax = imu_acc_to_ms2(intr->imu_acc_x); 
        f64 ay = imu_acc_to_ms2(intr->imu_acc_y); 
        f64 az = imu_acc_to_ms2(intr->imu_acc_z);

        // This assumes an ideal hover state. I wonder if there are
        // ways to subtract the effect of the known acceleration from this (Is 
        // this even stable?)

        ori_x = -atan2(ay, -az);
        ori_y =  atan2(ax, -az);

        //printf("acc_x=%lf, acc_y=%lf, acc_z=%lf\n", ax, ay, az);

        // World-frame accelerations

        rot_x = imu_gyro_to_rad(intr->imu_rot_x); 
        rot_y = imu_gyro_to_rad(intr->imu_rot_y); 
        rot_z = imu_gyro_to_rad(intr->imu_rot_z); 

        //Mat Z = { .r=8, .c=1, {
        //    acc_ori_x,             // ori_x
        //    acc_ori_y,             // ori_y
        //    gx,                    // rot_x
        //    gy,                    // rot_y
        //    gz                     // rot_z
        //}};

        //Mat R = {0};
        //R.r = Z.r;
        //R.c = Z.r;

        //R.data[0] =         0.00637;
        //R.data[1 + 1*R.c] = 0.00637;
        //R.data[2 + 2*R.c] = 0.00637;
        //R.data[3 + 3*R.c] = 1e-1;
        //R.data[4 + 4*R.c] = 1e-1;
        //R.data[5 + 5*R.c] = 0.05 * DEG2RAD;
        //R.data[6 + 6*R.c] = 0.05 * DEG2RAD;
        //R.data[7 + 7*R.c] = 0.05 * DEG2RAD;
        
        //Mat H = { .r=Z.r, .c=S_STATE_DIM };
        //MAT_AT(H, 3, S_ORI_X) = 1.0;
        //MAT_AT(H, 4, S_ORI_Y) = 1.0;
        //MAT_AT(H, 5, S_ROT_X) = 1.0;
        //MAT_AT(H, 6, S_ROT_Y) = 1.0;
        //MAT_AT(H, 7, S_ROT_Z) = 1.0;
        
        //kf_correct(&Z, &H, &R);
    }
    
    // --- GNSS ---
    /*if (gnss_l >= gnss_loops) {
        gnss_l=0;

        f64 pos_x = intr->pos_x / 100.0;
        f64 pos_y = intr->pos_y / 100.0;

#ifdef CONTROL_DEBUG
        //intr->dbg.pos_x_rdng = pos_x;
        //intr->dbg.pos_y_rdng = pos_y;
#endif

        //f64 gnss_vel_x = pos_x - X.data[S_POS_X];
        //f64 gnss_vel_y = pos_y - X.data[S_POS_Y];

        Mat Z = { .r=2, .c=1, {
            pos_x,
            pos_y,
            gnss_vel_x, 
            gnss_vel_y, 
        }};

        const f64 gnss_pos_sdev = 1e2; //2.5; // m
        const f64 gnss_pos_var = gnss_pos_sdev * gnss_pos_sdev;   // m^2
        Mat R = { .r=Z.r, .c=Z.r, { 
            gnss_pos_var, 0.0,
            0.0, gnss_pos_var
        }};

        Mat H = { .r=Z.r, .c=S_STATE_DIM };
        MAT_AT(H, 0, S_POS_X) = 1.0;
        MAT_AT(H, 1, S_POS_Y) = 1.0;

        kf_correct(&Z, &H, &R);
    }*/

    // --- Magnetometer ---
    //if (mag_l >= mag_loops) {
    //    mag_l=0;

    //    f64 alpha = X.data[S_ORI_X];
    //    f64 beta  = X.data[S_ORI_Y];
    //    f64 gamma = X.data[S_ORI_Z];

    //    Mat body_mag = { .r=3, .c=1, {
    //        intr->mag_x,
    //        intr->mag_y,
    //        intr->mag_z
    //    }};

    //    Mat world_mag = rotate_vec3_euler_angles(&body_mag, alpha, beta, gamma);

    //    f64 heading = atan2(-intr->mag_y, intr->mag_x);
    //    //heading = fmod(heading + PI, 2*PI) - PI;
    //    
    //    Mat Z = { .r=1, .c=1, {
    //        heading,
    //    }};

    //    const f64 mag_sdev = 1e-2;
    //    const f64 mag_var = mag_sdev * mag_sdev;
    //    Mat R = { .r=Z.r, .c=Z.r, { 
    //        mag_var,
    //    }};

    //    Mat H = { .r=Z.r, .c=S_STATE_DIM };
    //    MAT_AT(H, 0, S_ORI_Z) = 1.0;

    //    kf_correct(&Z, &H, &R);
    //}

    // --- Attitude Control --- 
    
    // hover
    f64 tgt_alt = 10.0; // m
    f64 tgt_vel = pid_step(X.data[S_POS_Z], tgt_alt, &alt_pid_p, &alt_pid_s);

    // Since we have all the initial parameters, we could compute the hovering cmd:
    // NUM_ARMS * (hover_cmd*max_rot_w)^2 * kf = g
    // sqrt(g / (NUM_ARMS * kf)) / max_rot_w = hover_cmd

    const f64 hover_cmd =  sqrt(G / (NUM_ROT*kf)) / rot_max_w;

    f64 out_cmd = hover_cmd + pid_step(X.data[S_VEL_Z], tgt_vel, &mot_pid_p, &mot_pid_s);

    // pos -> vel
    f64 vel_x_tgt = pid_step(X.data[S_POS_X], 0.0, &pos_pid_p, &pos_x_pid_s);
    f64 vel_y_tgt = pid_step(X.data[S_POS_Y], 0.0, &pos_pid_p, &pos_y_pid_s);

    //printf("vx: %lf\n", vel_x_tgt);

    // vel -> ori
    f64 ori_x_tgt = pid_step(X.data[S_VEL_Y], 0.0, &vel_pid_p, &vel_x_pid_s);
    f64 ori_y_tgt = pid_step(X.data[S_VEL_X], 0.0, &vel_pid_p, &vel_y_pid_s);

    // ori -> rot
    f64 rot_x_tgt = pid_step(ori_x, -ori_x_tgt, &ori_pid_p, &ori_x_pid_s);
    f64 rot_y_tgt = pid_step(ori_y,  ori_y_tgt, &ori_pid_p, &ori_y_pid_s);
    f64 rot_z_tgt = pid_step(ori_z, 0.0, &ori_pid_p, &ori_z_pid_s);
    
    //printf("o: %lf, tr: %lf\n", ori_x, rot_x_tgt);

    //printf(
    //    "pos_y: %lf, tgt_ori_x: %lf, ori_x: %lf, tgt_rot_x: %lf\n",
    //    X.data[S_POS_Y],
    //    ori_x_tgt,
    //    X.data[S_ORI_X],
    //    rot_x_tgt
    //);

    // rot -> cmd
    f64 rot_x_cmd = pid_step(rot_x, rot_x_tgt, &rot_pid_p, &rot_x_pid_s);
    f64 rot_y_cmd = pid_step(rot_y, rot_y_tgt, &rot_pid_p, &rot_y_pid_s);
    f64 rot_z_cmd = 0.0;//pid_step(rot_z, rot_z_tgt, &rot_pid_p, &rot_y_pid_s);

    // --- Motor mixer ---

    // Using ENU axis:
    //
    //       North
    //
    //         ^
    //      +Y |
    //    
    //    cw        ccw
    //    ->        <-
    //    M0        M1
    //       \    /
    //        \  /        +X
    //         ||         ->   East
    //        /  \ 
    //       /    \ 
    //    M3        M2
    //    ->        <-
    //    ccw       cw

    // fl > fr > rr > rl
    intr->rot_cmd[0] = out_cmd + rot_x_cmd + rot_y_cmd + rot_z_cmd;
    intr->rot_cmd[1] = out_cmd + rot_x_cmd - rot_y_cmd - rot_z_cmd;
    intr->rot_cmd[2] = out_cmd - rot_x_cmd - rot_y_cmd + rot_z_cmd;
    intr->rot_cmd[3] = out_cmd - rot_x_cmd + rot_y_cmd - rot_z_cmd;

#ifdef CONTROL_DEBUG
    intr->dbg[DBG_POS_X] = X.data[S_POS_X];
    intr->dbg[DBG_POS_Y] = X.data[S_POS_Y];
    intr->dbg[DBG_POS_Z] = X.data[S_POS_Z];
    intr->dbg[DBG_VEL_X] = X.data[S_VEL_X];
    intr->dbg[DBG_VEL_Y] = X.data[S_VEL_Y];
    intr->dbg[DBG_VEL_Z] = X.data[S_VEL_Z];
#endif
}
