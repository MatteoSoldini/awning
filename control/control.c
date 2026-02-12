#include "control.h"

#include <stdio.h>
#include <math.h>
#include <assert.h>
#include <stdlib.h>

#include <math/mat.h>
#include <math/utils.h>
#include <consts.h>

#if defined(__i386__) || defined(__x86_64__)
    #define DBG_BREAK() __asm__ volatile("int3");
#else
    #define DBG_BREAK() ((void)0)
#endif

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

f64 angle_pid_step(
    f64 real_angle, // 0 to 2pi
    f64 tgt_angle,  // 0 to 2pi
    PIDParams *pid_params,
    PIDState *pid_state
) {
    f64 err = tgt_angle - real_angle;
    fmodf(err + PI, 2*PI);
    if (err < 0) err += 2*PI;
    err = err - PI;

    printf("%lf\n", err);

    pid_state->i_err += err * pid_params->dt;
    f64 der = (real_angle - pid_state->p_real) / pid_params->dt;
    pid_state->p_real = real_angle;

    f64 out = pid_params->p*err + pid_params->i*pid_state->i_err + pid_params->d*der;

    if (out > pid_params->high) out = pid_params->high;
    if (out < pid_params->low) out = pid_params->low;

    return out;
}

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
    .d = 0.0,
    .dt = c_dt,
    .high = 0.5,    // m/s
    .low = -0.5     // m/s
};
PIDState pos_x_pid_s = {0};
PIDState pos_y_pid_s = {0};

// vel -> ori
PIDParams vel_pid_p = {
    .p = 1.0,
    .i = 0.0,
    .d = 0.0,
    .dt = c_dt,
    .high = 0.2,    // rad
    .low = -0.2     // rad
};
PIDState vel_x_pid_s = {0};
PIDState vel_y_pid_s = {0};

// ori -> rot
PIDParams ori_pid_p = {
    .p = 1.0,
    .i = 0.0,
    .d = 0.0,
    .dt = c_dt,
    .high = 0.3,    // rad/s
    .low = -0.3     // rad/s
};
PIDState ori_x_pid_s = {0};
PIDState ori_y_pid_s = {0};
PIDState ori_z_pid_s = {0};

// rot -> cmd
PIDParams rot_pid_p = {
    .p = 0.3,
    .i = 0.0,
    .d = 0.0,
    .dt = c_dt,
    .high = 0.1,
    .low = -0.1
};
PIDState rot_x_pid_s = {0};
PIDState rot_y_pid_s = {0};

PIDParams rot_z_pid_p = {
    .p = 0.5,
    .i = 0.0,
    .d = 0.0,
    .dt = c_dt,
    .high = 0.2,
    .low = -0.2
};
PIDState rot_z_pid_s = {0};

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

// https://www.politesi.polimi.it/retrieve/a81cb05a-81ad-616b-e053-1605fe0a889a/2013_07_Ascorti.pdf 

#define SET_XYZ(M, r0, c0, val) \
    MAT_AT(M, r0+0, c0+0) = val; \
    MAT_AT(M, r0+1, c0+1) = val; \
    MAT_AT(M, r0+2, c0+2) = val;

enum {
    S_POS_X, // m
    S_POS_Y, // m
    S_POS_Z, // m
    S_VEL_X, // m/s
    S_VEL_Y, // m/s
    S_VEL_Z, // m/s
    S_ORI_X, // rad
    S_ORI_Y, // rad
    S_ORI_Z, // rad
    S_ROT_X, // rad/s
    S_ROT_Y, // rad/s
    S_ROT_Z, // rad/s
    S_STATE_DIM
};
Mat X = { .r=S_STATE_DIM, .c=1 };

Mat F(Mat Xp, f64 thrust) {
    // Using Z-Y-X rotation for euler angles
    
    f64 baz = thrust/mass;

    f64 wax = -baz*-sin(Xp.data[S_ORI_Y]);
    f64 way = -baz* cos(Xp.data[S_ORI_Y])*sin(Xp.data[S_ORI_X]);
    f64 waz =  baz* cos(Xp.data[S_ORI_Y])*cos(Xp.data[S_ORI_X]) - G;

    return (Mat) { .r=S_STATE_DIM, .c=1, {
        Xp.data[S_POS_X] + Xp.data[S_VEL_X]*c_dt + 0.5*wax*c_dt*c_dt,  // pos_x
        Xp.data[S_POS_Y] + Xp.data[S_VEL_Y]*c_dt + 0.5*way*c_dt*c_dt,  // pos_y
        Xp.data[S_POS_Z] + Xp.data[S_VEL_Z]*c_dt + 0.5*waz*c_dt*c_dt,  // pos_z
        Xp.data[S_VEL_X] + wax*c_dt,                                   // vel_x
        Xp.data[S_VEL_Y] + way*c_dt,                                   // vel_y
        Xp.data[S_VEL_Z] + waz*c_dt,                                   // vel_z
        Xp.data[S_ORI_X] + Xp.data[S_ROT_X]*c_dt,                      // ori_x
        Xp.data[S_ORI_Y] + Xp.data[S_ROT_Y]*c_dt,                      // ori_y
        Xp.data[S_ORI_Z] + Xp.data[S_ROT_Z]*c_dt,                      // ori_z
        Xp.data[S_ROT_X],                                              // rot_x
        Xp.data[S_ROT_Y],                                              // rot_y
        Xp.data[S_ROT_Z]                                               // rot_z
    }};
}

Mat J(Mat Xp, f64 thrust) {
    // d(sin) =  cos
    // d(cos) = -sin
    // d(a*f(x)) = a * d(f(x))
    
    f64 baz = thrust/mass;

    f64 d_wax_ori_y = -baz*-cos(Xp.data[S_ORI_Y]);
    f64 d_way_ori_x = -baz* cos(Xp.data[S_ORI_Y])* cos(Xp.data[S_ORI_X]);
    f64 d_way_ori_y = -baz*-sin(Xp.data[S_ORI_Y])* sin(Xp.data[S_ORI_X]);
    f64 d_waz_ori_x =  baz* cos(Xp.data[S_ORI_Y])*-sin(Xp.data[S_ORI_X]);
    f64 d_waz_ori_y =  baz*-sin(Xp.data[S_ORI_Y])* cos(Xp.data[S_ORI_X]);

    Mat J = { .r=S_STATE_DIM, .c=S_STATE_DIM };
    
    // --- Position ---
    MAT_AT(J, S_POS_X, S_POS_X) = 1;
    MAT_AT(J, S_POS_X, S_VEL_X) = c_dt;
    MAT_AT(J, S_POS_X, S_ORI_Y) = 0.5*d_wax_ori_y*c_dt*c_dt;

    MAT_AT(J, S_POS_Y, S_POS_Y) = 1;
    MAT_AT(J, S_POS_Y, S_VEL_Y) = c_dt;
    MAT_AT(J, S_POS_Y, S_ORI_X) = 0.5*d_way_ori_x*c_dt*c_dt;
    MAT_AT(J, S_POS_Y, S_ORI_Y) = 0.5*d_way_ori_y*c_dt*c_dt;
    
    MAT_AT(J, S_POS_Z, S_POS_Z) = 1;
    MAT_AT(J, S_POS_Z, S_VEL_Z) = c_dt;
    MAT_AT(J, S_POS_Z, S_ORI_X) = 0.5*d_waz_ori_x*c_dt*c_dt;
    MAT_AT(J, S_POS_Z, S_ORI_Y) = 0.5*d_waz_ori_y*c_dt*c_dt;

    // --- Velocity ---
    MAT_AT(J, S_VEL_X, S_VEL_X) = 1;
    MAT_AT(J, S_VEL_X, S_ORI_Y) = d_wax_ori_y*c_dt;
    
    MAT_AT(J, S_VEL_Y, S_VEL_Y) = 1;
    MAT_AT(J, S_VEL_Y, S_ORI_X) = d_way_ori_x*c_dt;
    MAT_AT(J, S_VEL_Y, S_ORI_Y) = d_way_ori_y*c_dt;
    
    MAT_AT(J, S_VEL_Z, S_VEL_Z) = 1;
    MAT_AT(J, S_VEL_Z, S_ORI_X) = d_waz_ori_x*c_dt;
    MAT_AT(J, S_VEL_Z, S_ORI_Y) = d_waz_ori_y*c_dt;

    // --- Orientation ---
    SET_XYZ(J, S_ORI_X, S_ORI_X, 1);
    SET_XYZ(J, S_ORI_X, S_ROT_X, c_dt);
    
    // --- Rotation ---
    SET_XYZ(J, S_ROT_X, S_ROT_X, 1);

    return J;
}

// State covariance matrix
Mat P = { .r=S_STATE_DIM, .c=S_STATE_DIM };

// Process covariance matrix
Mat Q = { .r=S_STATE_DIM, .c=S_STATE_DIM };

void c_init() {
    // --- Position ---
    // State transition matrix
    //SET_XYZ(F, S_POS_X, S_POS_X, 1.0);
    //SET_XYZ(F, S_POS_X, S_VEL_X, c_dt);
    ////MAT_AT(F, S_POS_Z, S_ACC_Z) = 0.5*c_dt*c_dt;
    
    // State covariance matrix
    SET_XYZ(P, S_POS_X, S_POS_X, 1e0);
    
    // Process covariance matrix
    SET_XYZ(Q, S_POS_X, S_POS_X, 1e-5);

    // --- Velocity ---
    // State transition matrix
    //SET_XYZ(F, S_VEL_X, S_VEL_X, 1.0);
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
    
    // State covariance matrix
    SET_XYZ(P, S_ORI_X, S_ORI_X, 1e0);
    
    // Process covariance matrix
    SET_XYZ(Q, S_ORI_X, S_ORI_X, 1e-3);

    //// --- Rotation ---
    //// State transition matrix
    //SET_XYZ(F, S_ROT_X, S_ROT_X, 1.0);
    
    // State covariance matrix
    SET_XYZ(P, S_ROT_X, S_ROT_X, 1e0);
    
    // Process covariance matrix
    SET_XYZ(Q, S_ROT_X, S_ROT_X, 1e-2);
}

void ekf_predict(f64 thrust) {
    // X = F(X)
    Mat new_X = F(X, thrust);
   
#ifndef NDEBUG
    Mat I = mat_sub(&new_X, &X);
    for (i32 i = 0; i < X.r; i++) {
        f64 vi = fabs(MAT_AT(I, i, 0));
        
        if (vi > 1.0) {
            DBG_BREAK();
        }

        f64 sigma = sqrt(MAT_AT(P, i, i));   // expected stddev of residual
        if (sigma > 1e-12 && vi > 5.0 * sigma) {
            DBG_BREAK();
        }
    }
#endif

    X = new_X;

    // P = J(X)*P*J^T(X) + Q
    Mat Jx = J(X, thrust);
    P = mat_mul(&Jx, &P);

    Mat Jx_inv = mat_trans(&Jx);
    P = mat_mul(&P, &Jx_inv);

    P = mat_sum(&P, &Q);
}

void ekf_correct(
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
    // S = H * P * H_t + R
    Mat H_t = mat_trans(H);
    Mat HP = mat_mul(H, &P);
    Mat S = mat_mul(&HP, &H_t);
    S = mat_sum(&S, R);
    
    for (i32 i = 0; i < Z->r; i++) {
        f64 vi = fabs(MAT_AT(I, i, 0));
        
        f64 sigma = sqrt(MAT_AT(S, i, i));   // expected stddev of residual
        if (sigma > 1e-12 && vi > 5.0 * sigma) {
#ifndef NDEBUG
            printf("INNOV gate: i=%d I=%lf sigma=%lf (%.2f sigma)\n",
                   i, vi, sigma, vi/sigma);
            
            printf("DEBUG DUMP\n");
            printf("I: ");
            mat_print(&I);
            
            printf("Z: ");
            mat_print(Z);
            
            printf("X: ");
            mat_print(&X);
            
            printf("P: ");
            mat_print(&P);
            
            DBG_BREAK();
#endif

            return; // reject update
        }
    }

    Mat S_inv = mat_inv(&S);

    // Compute Kalman gain
    // K = P*H_t*S^-1
    Mat K = mat_mul(&P, &H_t);
    K = mat_mul(&K, &S_inv);

#ifndef NDEBUG
    f64 K_max = 0.0;
    for (i32 r=0; r<K.r; r++) {
        for (i32 c=0; c<K.c; c++) {
            if (MAT_AT(K, r, c) > fabs(K_max)) {
                K_max = fabs(MAT_AT(K, r, c));
            }
        }
    }    

    if (K_max >= 1.5) {
        printf("K_max: %lf\n", K_max);
    }
#endif

    // Update state
    // X = X + K*I
    Mat X_corr = mat_mul(&K, &I);
    X = mat_sum(&X, &X_corr);

    // Update covariance
    // P = P - K*S*K_t
    
    Mat K_t = mat_trans(&K);
    //Mat KS = mat_mul(&K, &S);
    //Mat KSK_t = mat_mul(&KS, &K_t);
    //P = mat_sub(&P, &KSK_t);

    // Using Joseph-form
    // reference: https://kalman-filter.com/joseph-form/
    // P = (Id - K H) P (Id - K H)^T + K R K^T
    Mat KH = mat_mul(&K, H);

    Mat Id = mat_identity(KH.r);

    Mat IdminKH = mat_sub(&Id, &KH);
    Mat IdminKHP = mat_mul(&IdminKH, &P);
    Mat IdminKH_t = mat_trans(&IdminKH);
    Mat IdminHKPIdminHK_t = mat_mul(&IdminKHP, &IdminKH_t);
    
    Mat KR = mat_mul(&K, R);
    Mat KRK_t = mat_mul(&KR, &K_t);

    P = mat_sum(&IdminHKPIdminHK_t, &KRK_t);

    // Check P
    // * Symmetric
    // * Semi-positive (positive diagonal)
    
    for (u64 r=0; r<P.r; r++) {
        for (u64 c=r+1; c<P.c; c++) {
            f64 diff = MAT_AT(P, r, c) - MAT_AT(P, c, r);
            assert(diff < 1e-12);
        }
    }

    for (u64 r=0; r<P.r; r++) {
        assert(MAT_AT(P, r, r) >= 0.0);
    }
}

//void kf_predict(
//    Mat *B, // Control transition matrix
//    Mat *U  // Control matrix
//) {
//    // Predict new state
//    // X = F*X + B*U
//    Mat FX = mat_mul(&F, &X);
//    Mat BU = mat_mul(B, U);
//    X = mat_sum(&FX, &BU);
//    
//    // Predict state covariance
//    // P(n+1) = F*P(n)*F_t + Q
//    Mat F_t = mat_trans(&F);
//    P = mat_mul(&F, &P);
//    P = mat_mul(&P, &F_t);
//    P = mat_sum(&P, &Q);
//}
//
//void kf_correct(
//    Mat *Z, // Measurements matrix
//    Mat *H, // Observation matrix
//    Mat *R  // Measurement noise
//) {
//    assert(Z->r == H->r && R->r == R->c && R->r == H->r && "Wrong arguments dimensions");
//
//    // Map state domain to measurements domain
//    Mat HX = mat_mul(H, &X);
//
//    // Compute innovation
//    Mat I = mat_sub(Z, &HX);   // This is in measurement space
//
//    // Compute innovation covariance
//    // S = H * P_pred * H_t + R
//    Mat H_t = mat_trans(H);
//    Mat HP = mat_mul(H, &P);
//    Mat S = mat_mul(&HP, &H_t);
//
//    S = mat_sum(&S, R);
//    
//    Mat S_inv = mat_inv(&S);
//
//    // Compute Kalman gain
//    // K = P_pred*H_t*S^-1
//    Mat K = mat_mul(&P, &H_t);
//    K = mat_mul(&K, &S_inv);
//    
//    // Update state
//    // X = X_pred + K * I
//    Mat X_corr = mat_mul(&K, &I);
//    X = mat_sum(&X, &X_corr);
//
//    // Update covariance
//    // P = (Id - K * H) * P_pred
//    Mat KH = mat_mul(&K, H);   // Map Kalman Gain from measurement domain to state domain
//    Mat Id = mat_identity(KH.r);
//    Mat IdminKH = mat_sub(&Id, &KH);
//    P = mat_mul(&IdminKH, &P);
//}

u64 bar_l = 0;
u64 imu_l = 0;
u64 gnss_l = 0;
u64 mag_l = 0;

f64 c_rot_w[NUM_ROT] = {0};

// Assume that the control_step() function is triggered by an interrupt
// in the MCU every 1ms (1000Hz)
void c_step(ControllerInterface *intr) {
    bar_l++;
    imu_l++;
    gnss_l++;
    mag_l++;
    
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
    
    ekf_predict(tot_mot_f);

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
        intr->dbg[DBG_IN_ALT] = alt_m;
#endif

        Mat Z = { .r=1, .c=1, { alt_m }};

        const f64 sdev = 1.0; // 0.25
        Mat R = { .r=1, .c=1, { sdev*sdev }};
        Mat H = { .r=1, .c=S_STATE_DIM };
        MAT_AT(H, 0, S_POS_Z) = 1.0;

        ekf_correct(&Z, &H, &R);

        //kf_correct(&Z, &H, &R);
    }

    // --- IMU ---
    if (imu_l >= imu_loops) {
        imu_l=0;

        // Body-frame accelerations
        f64 ax = imu_acc_to_ms2(intr->imu_acc_x); 
        f64 ay = imu_acc_to_ms2(intr->imu_acc_y); 
        f64 az = imu_acc_to_ms2(intr->imu_acc_z);

        f64 norm = sqrt(ax*ax + ay*ay + az*az);
        f64 dx = -ax;
        f64 dy = -ay;
        f64 dz = -az;

        // roll (phi) and pitch (theta)
        f64 ori_x = atan(dy/dz);
        f64 ori_y = atan(-dx/sqrt(dy*dy + dz*dz));

        //f64 ori_x = -atan2(ay, -az);
        //f64 ori_y =  atan2(ax, -az);

#ifdef CONTROL_DEBUG
        intr->dbg[DBG_IN_ORI_X] = ori_x;
        intr->dbg[DBG_IN_ORI_Y] = ori_y;
#endif
        
        if (norm > 0.8*G && norm < 1.2*G) {
            Mat Z = { .r=2, .c=1, {
                ori_x,
                ori_y
            }};
            
            Mat R = { .r=Z.r, .c=Z.r };
            MAT_AT(R, 0, 0) = 1e-1;
            MAT_AT(R, 1, 1) = 1e-1;
            
            Mat H = { .r=Z.r, .c=S_STATE_DIM };
            MAT_AT(H, 0, S_ORI_X) = 1.0;
            MAT_AT(H, 1, S_ORI_Y) = 1.0;
            
            ekf_correct(&Z, &H, &R);
        }

        // This assumes an ideal hover state. I wonder if there are
        // ways to subtract the effect of the known acceleration from this (Is 
        // this even stable?)

        // World-frame accelerations
        f64 gyro_rate_x = imu_gyro_to_rad(intr->imu_rot_x);
        f64 gyro_rate_y = imu_gyro_to_rad(intr->imu_rot_y);
        f64 gyro_rate_z = imu_gyro_to_rad(intr->imu_rot_z);

#ifdef CONTROL_DEBUG
        intr->dbg[DBG_IN_ROT_X] = gyro_rate_x;
        intr->dbg[DBG_IN_ROT_Y] = gyro_rate_y;
        intr->dbg[DBG_IN_ROT_Z] = gyro_rate_z;
#endif

        Mat Z = { .r=3, .c=1, {
            gyro_rate_x,
            gyro_rate_y,
            gyro_rate_z
        }};

        f64 gyro_sdev = 0.05*DEG2RAD;

        Mat R = { .r=Z.r, .c=Z.r };
        MAT_AT(R, 0, 0) = gyro_sdev*gyro_sdev;
        MAT_AT(R, 1, 1) = gyro_sdev*gyro_sdev;
        MAT_AT(R, 2, 2) = gyro_sdev*gyro_sdev;

        Mat H = { .r=Z.r, .c=S_STATE_DIM };
        MAT_AT(H, 0, S_ROT_X) = 1.0;
        MAT_AT(H, 1, S_ROT_Y) = 1.0;
        MAT_AT(H, 2, S_ROT_Z) = 1.0;
        
        //Mat Z = { .r=5, .c=1, {
        //    ori_x,
        //    ori_y,
        //    gyro_rate_x,
        //    gyro_rate_y,
        //    gyro_rate_z
        //}};

        //f64 gyro_sdev = 0.05*DEG2RAD;

        //Mat R = { .r=Z.r, .c=Z.r };
        //MAT_AT(R, 0, 0) = 1e-1;
        //MAT_AT(R, 1, 1) = 1e-1;
        //MAT_AT(R, 2, 2) = gyro_sdev*gyro_sdev;
        //MAT_AT(R, 3, 3) = gyro_sdev*gyro_sdev;
        //MAT_AT(R, 4, 4) = gyro_sdev*gyro_sdev;

        //Mat H = { .r=Z.r, .c=S_STATE_DIM };
        //MAT_AT(H, 0, S_ORI_X) = 1.0;
        //MAT_AT(H, 1, S_ORI_Y) = 1.0;
        //MAT_AT(H, 2, S_ROT_X) = 1.0;
        //MAT_AT(H, 3, S_ROT_Y) = 1.0;
        //MAT_AT(H, 4, S_ROT_Z) = 1.0;
        
        ekf_correct(&Z, &H, &R);
    }
    
    // --- GNSS ---
    if (gnss_l >= gnss_loops) {
        gnss_l=0;

        f64 pos_x = intr->pos_x / 100.0;
        f64 pos_y = intr->pos_y / 100.0;

#ifdef CONTROL_DEBUG
        intr->dbg[DBG_IN_POS_X] = pos_x;
        intr->dbg[DBG_IN_POS_Y] = pos_y;
#endif

        Mat Z = { .r=2, .c=1, {
            pos_x,
            pos_y,
        }};

        const f64 gnss_pos_sdev = 2.5; // m
        const f64 gnss_pos_var = gnss_pos_sdev * gnss_pos_sdev;   // m^2
        Mat R = { .r=Z.r, .c=Z.r };
        MAT_AT(R, 0, 0) = gnss_pos_var;
        MAT_AT(R, 1, 1) = gnss_pos_var;

        Mat H = { .r=Z.r, .c=S_STATE_DIM };
        MAT_AT(H, 0, S_POS_X) = 1.0;
        MAT_AT(H, 1, S_POS_Y) = 1.0;

        ekf_correct(&Z, &H, &R);
    }

    // --- Magnetometer ---
    if (mag_l >= mag_loops) {
        mag_l=0;

        f64 alpha = X.data[S_ORI_X];
        f64 beta  = X.data[S_ORI_Y];
        f64 gamma = X.data[S_ORI_Z];

        Mat body_mag = { .r=3, .c=1, {
            intr->mag_x,
            intr->mag_y,
            intr->mag_z
        }};

        Mat world_mag = rotate_vec3_euler_angles(&body_mag, alpha, beta, gamma);

        f64 heading = atan2(-intr->mag_y, intr->mag_x);
        //heading = fmod(heading + PI, 2*PI) - PI;
        
        Mat Z = { .r=1, .c=1, {
            heading,
        }};

        const f64 mag_sdev = 1e-2;
        const f64 mag_var = mag_sdev * mag_sdev;
        Mat R = { .r=Z.r, .c=Z.r, { 
            mag_var,
        }};

        Mat H ={ .r=Z.r, .c=S_STATE_DIM };
        MAT_AT(H, 0, S_ORI_Z) =  1.0;

        ekf_correct(&Z, &H, &R);
    }

    // --- Attitude Control --- 
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
    // According the the ENU axis reference:
    //  * +vel_x => +ori_y
    //  * +vel_y => -ori_x

    f64 ori_y_tgt =  pid_step(X.data[S_VEL_X], vel_x_tgt, &vel_pid_p, &vel_y_pid_s);
    f64 ori_x_tgt = -pid_step(X.data[S_VEL_Y], vel_y_tgt, &vel_pid_p, &vel_x_pid_s);

    // ori -> rot
    f64 rot_x_tgt = pid_step(X.data[S_ORI_X], ori_x_tgt, &ori_pid_p, &ori_x_pid_s);
    f64 rot_y_tgt = pid_step(X.data[S_ORI_Y], ori_y_tgt, &ori_pid_p, &ori_y_pid_s);
    f64 rot_z_tgt = pid_step(X.data[S_ORI_Z], 0.0, &ori_pid_p, &ori_z_pid_s);
    
    //printf("o: %lf, tr: %lf\n", ori_x, rot_x_tgt);

    //printf(
    //    "pos_y: %lf, tgt_ori_x: %lf, ori_x: %lf, tgt_rot_x: %lf\n",
    //    X.data[S_POS_Y],
    //    ori_x_tgt,
    //    X.data[S_ORI_X],
    //    rot_x_tgt
    //);

    // rot -> cmd
    f64 x_cmd = pid_step(X.data[S_ROT_X], rot_x_tgt, &rot_pid_p, &rot_x_pid_s);
    f64 y_cmd = pid_step(X.data[S_ROT_Y], rot_y_tgt, &rot_pid_p, &rot_y_pid_s);
    f64 z_cmd = pid_step(X.data[S_ROT_Z], rot_z_tgt, &rot_z_pid_p, &rot_z_pid_s);

    // --- Motor mixer ---


    // fl > fr > rr > rl
    intr->rot_cmd[0] = clamp(out_cmd + x_cmd + y_cmd + z_cmd, 0.0, 1.0);
    intr->rot_cmd[1] = clamp(out_cmd + x_cmd - y_cmd - z_cmd, 0.0, 1.0);
    intr->rot_cmd[2] = clamp(out_cmd - x_cmd - y_cmd + z_cmd, 0.0, 1.0);
    intr->rot_cmd[3] = clamp(out_cmd - x_cmd + y_cmd - z_cmd, 0.0, 1.0);

#ifdef CONTROL_DEBUG
    intr->dbg[DBG_X_CMD] = x_cmd;
    intr->dbg[DBG_Y_CMD] = y_cmd;
    intr->dbg[DBG_ROT_X_TGT] = rot_x_tgt;
    intr->dbg[DBG_ROT_Y_TGT] = rot_y_tgt;
    intr->dbg[DBG_ORI_X_TGT] = ori_x_tgt;
    intr->dbg[DBG_ORI_Y_TGT] = ori_y_tgt;
    intr->dbg[DBG_VEL_X_TGT] = vel_x_tgt;
    intr->dbg[DBG_VEL_Y_TGT] = vel_y_tgt;
    
    intr->dbg[DBG_POS_X] = X.data[S_POS_X];
    intr->dbg[DBG_POS_Y] = X.data[S_POS_Y];
    intr->dbg[DBG_POS_Z] = X.data[S_POS_Z];
    intr->dbg[DBG_VEL_X] = X.data[S_VEL_X];
    intr->dbg[DBG_VEL_Y] = X.data[S_VEL_Y];
    intr->dbg[DBG_VEL_Z] = X.data[S_VEL_Z];
    intr->dbg[DBG_ORI_X] = X.data[S_ORI_X];
    intr->dbg[DBG_ORI_Y] = X.data[S_ORI_Y];
    intr->dbg[DBG_ORI_Z] = X.data[S_ORI_Z];
    intr->dbg[DBG_ROT_X] = X.data[S_ROT_X];
    intr->dbg[DBG_ROT_Y] = X.data[S_ROT_Y];
    intr->dbg[DBG_ROT_Z] = X.data[S_ROT_Z];
#endif
}
