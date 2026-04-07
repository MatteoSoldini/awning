#include "control.h"

#include <stdio.h>
#include <math.h>
#include <assert.h>
#include <stdlib.h>

#include <math/mat.h>
#include <math/utils.h>
#include <math/geom.h>
#include <consts.h>

#if defined(__i386__) || defined(__x86_64__)
    #define DBG_BREAK() __asm__ volatile("int3");
#else
    #define DBG_BREAK() ((void)0)
#endif

const f64 c_dt = 1.0 / CONTROL_FQ;

typedef struct {
    f64 x;   // m
    f64 y;   // m
    f64 z;   // m
    f64 yaw; // rad
} Waypoint;

u64 curr_wp = 0;
#define NUM_FLIGHTPLAN_WAYPOINTS 4
Waypoint flightplan[NUM_FLIGHTPLAN_WAYPOINTS] = {
    { .x =  10.0, .y =  10.0, .z = 10.0, .yaw =  0.0  },
    { .x = -10.0, .y =  10.0, .z = 10.0, .yaw =  PI/2 },
    { .x = -10.0, .y = -10.0, .z = 10.0, .yaw =  PI   },
    { .x =  10.0, .y = -10.0, .z = 10.0, .yaw = -PI/2 }
};
f64 wp_radius = 0.5; // m

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

static const f64 imu_gyro_max_value = 250.0; // rad/s
f64 imu_gyro_to_rad(i16 raw_gyro) {
    return ((f64)raw_gyro / (f64)INT16_MAX) * imu_gyro_max_value; 
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

f64 angle_distance(f64 a, f64 b) {
    assert(a > -PI && a <= PI);
    assert(b > -PI && b <= PI);

    f64 diff = a-b;
    if (diff >   PI) diff -= 2*PI;
    if (diff <= -PI) diff += 2*PI;

    return diff;
}

f64 angle_pid_step(
    f64 real_angle, // -pi to pi
    f64 tgt_angle,  // -pi to pi
    PIDParams *pid_params,
    PIDState *pid_state
) {
    f64 err = angle_distance(tgt_angle, real_angle);
    pid_state->i_err += err * pid_params->dt;
    f64 der = -(real_angle - pid_state->p_real) / pid_params->dt;
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
    f64 der = -(real - pid_state->p_real) / pid_params->dt;
    pid_state->p_real = real;

    f64 out = pid_params->p*err + pid_params->i*pid_state->i_err + pid_params->d*der;

    if (out > pid_params->high) out = pid_params->high;
    if (out < pid_params->low) out = pid_params->low;

    return out;
}

PIDParams mot_pid_p = {
    .p = 0.4,
    .i = 0.01,
    .d = 0.0,
    .dt = c_dt,
    .high = 0.3,
    .low = -0.3
};
PIDState mot_pid_s = {0};

PIDParams vel_z_pid_p = {
    .p = 0.1,
    .i = 0.0,
    .d = 0.0,
    .dt = c_dt,
    .high = 0.5,
    .low = -0.5
};
PIDState alt_pid_s = {0};

// pos -> vel
PIDParams pos_pid_p = {
    .p = 0.5,
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
    .p = 0.25,
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

// State vector
//
// Using ENU axis convetion:
//  +X: east
//  +Y: north
//  +Z: up

// https://www.politesi.polimi.it/retrieve/a81cb05a-81ad-616b-e053-1605fe0a889a/2013_07_Ascorti.pdf 

// Multiplicative EKF references:
// * https://ntrs.nasa.gov/api/citations/20040037784/downloads/20040037784.pdf
// * https://ntrs.nasa.gov/api/citations/20020060647/downloads/20020060647.pdf
// * https://matthewhampsey.github.io/blog/2020/07/18/mekf 
// * https://www.iri.upc.edu/people/jsola/JoanSola/objectes/notes/kinematics.pdf

// UKF seems to be easier to understand and simpler to develop while retaining similar (and sometimes better)
// accuracy compared to MEKF. This, at the cost of computational complexity, which seems to not be an issue for todays hw.

#define SET_XYZ(M, r0, c0, val) \
    MAT_AT(M, r0+0, c0+0) = val; \
    MAT_AT(M, r0+1, c0+1) = val; \
    MAT_AT(M, r0+2, c0+2) = val;

#define INSERT_MAT3(M, r0, c0, M3)                      \
    for (u64 r=0; r<3; r++) {                           \
        for (u64 c=0; c<3; c++) {                       \
            MAT_AT(M, r0+r, c0+c) = MAT_AT(M3, r, c);   \
        }                                               \
    }

enum {
    S_POS_X,        // m
    S_POS_Y,        // m
    S_POS_Z,        // m
    S_VEL_X,        // m/s
    S_VEL_Y,        // m/s
    S_VEL_Z,        // m/s
    S_QUAT_R,
    S_QUAT_I,
    S_QUAT_J,
    S_QUAT_K,
    S_ACC_BIAS_X,
    S_ACC_BIAS_Y,
    S_ACC_BIAS_Z,
    S_OMEGA_BIAS_X,
    S_OMEGA_BIAS_Y,
    S_OMEGA_BIAS_Z,
    S_GRAVITY_X,
    S_GRAVITY_Y,
    S_GRAVITY_Z,
    S_STATE_DIM
};

#define QUAT_FROM_STATE(X) \
    (quat) {                 \
        .r=(X).data[S_QUAT_R], \
        .i=(X).data[S_QUAT_I], \
        .j=(X).data[S_QUAT_J], \
        .k=(X).data[S_QUAT_K]  \
    }

Mat F(Mat *X, vec3 acc_meas, vec3 gyro_meas) {
    // Orientation integration
    // q += q 1/2 wq dt
    quat q = QUAT_FROM_STATE(*X);
    vec3 omega = {
        gyro_meas.x - X->data[S_OMEGA_BIAS_X],
        gyro_meas.y - X->data[S_OMEGA_BIAS_Y],
        gyro_meas.z - X->data[S_OMEGA_BIAS_Z]
    };
    quat w = { 0, omega.x, omega.y, omega.z };
    //quat w = {0, 0.0, 0.0, 0.0};
    quat dq = quat_mul(&q, &w);
    
    q.r += 0.5 * dq.r * c_dt;
    q.i += 0.5 * dq.i * c_dt;
    q.j += 0.5 * dq.j * c_dt;
    q.k += 0.5 * dq.k * c_dt;
    quat_norm(&q);
    
    // World acceleration
    vec3 acc_bias = {
        .x = X->data[S_ACC_BIAS_X],
        .y = X->data[S_ACC_BIAS_Y],
        .z = X->data[S_ACC_BIAS_Z]
    };

    vec3 body_acc = vec3_sub(&acc_meas, &acc_bias);
    vec3 world_acc = vec3_rotate_by_quat(&body_acc, &q);
    vec3 gravity_acc = {
        .x = X->data[S_GRAVITY_X],
        .y = X->data[S_GRAVITY_Y],
        .z = X->data[S_GRAVITY_Z],
    };
    world_acc = vec3_sum(&world_acc, &gravity_acc);

    return (Mat) { .r=S_STATE_DIM, .c=1, {
        X->data[S_POS_X] + X->data[S_VEL_X]*c_dt + 0.5*world_acc.x*c_dt*c_dt,  // pos_x
        X->data[S_POS_Y] + X->data[S_VEL_Y]*c_dt + 0.5*world_acc.y*c_dt*c_dt,  // pos_y
        X->data[S_POS_Z] + X->data[S_VEL_Z]*c_dt + 0.5*world_acc.z*c_dt*c_dt,  // pos_z
        X->data[S_VEL_X] + world_acc.x*c_dt,                                   // vel_x
        X->data[S_VEL_Y] + world_acc.y*c_dt,                                   // vel_y
        X->data[S_VEL_Z] + world_acc.z*c_dt,                                   // vel_z
        q.r,
        q.i,
        q.j,
        q.k,
        X->data[S_ACC_BIAS_X],
        X->data[S_ACC_BIAS_Y],
        X->data[S_ACC_BIAS_Z],
        X->data[S_OMEGA_BIAS_X],
        X->data[S_OMEGA_BIAS_Y],
        X->data[S_OMEGA_BIAS_Z],
        X->data[S_GRAVITY_X],
        X->data[S_GRAVITY_Y],
        X->data[S_GRAVITY_Z]
    }};
}

// --- Observation Functions ---
Mat h_body_north_dir(Mat *X) {
    // https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation#Quaternion-derived_rotation_matrix

    // bv = q* wv q
    //    = R' v
    //    = R' |0, 1, 0|
    //    = |  2 (qi qj + qk qr)  |
    //      | 1 - 2 (qi^2 + qk^2) |
    //      |  2 (qj qk - qi qr)  |

    quat q = QUAT_FROM_STATE(*X);

    return (Mat) { .r=3, .c=1, {
        2*(q.i*q.j + q.k*q.r),     // x
        1 - 2*(q.i*q.i + q.k*q.k), // y
        2*(q.j*q.k - q.i*q.r),     // z
    }};
}

Mat H_body_north_dir(Mat *X) {
    //    = |  2 (qi qj + qk qr)  |
    //      | 1 - 2 (qi^2 + qk^2) |
    //      |  2 (qj qk - qi qr)  |
    
    quat q = QUAT_FROM_STATE(*X);

    Mat J = { .r=3, .c=S_STATE_DIM };
    MAT_AT(J, 0, S_QUAT_R) = 2*q.k;
    MAT_AT(J, 0, S_QUAT_I) = 2*q.j;
    MAT_AT(J, 0, S_QUAT_J) = 2*q.i;
    MAT_AT(J, 0, S_QUAT_K) = 2*q.r;
    
    MAT_AT(J, 1, S_QUAT_I) = -4*q.i;
    MAT_AT(J, 1, S_QUAT_K) = -4*q.k;
    
    MAT_AT(J, 2, S_QUAT_R) = -2*q.i;
    MAT_AT(J, 2, S_QUAT_I) = -2*q.r;
    MAT_AT(J, 2, S_QUAT_J) =  2*q.k;
    MAT_AT(J, 2, S_QUAT_K) =  2*q.j;

    return J;
}

Mat h_alt(Mat *X) {
    return (Mat) { .r=1, .c=1, { MAT_AT(*X, S_POS_Z, 0) }};
}

Mat H_alt(Mat *X) {
    Mat J = { .r=1, .c=S_STATE_DIM };
    MAT_AT(J, 0, S_POS_Z) = 1.0;
    
    return J;
}

Mat h_gnss(Mat *X) {
    return (Mat) { .r=4, .c=1, {
        MAT_AT(*X, S_POS_X, 0),
        MAT_AT(*X, S_POS_Y, 0),
        MAT_AT(*X, S_VEL_X, 0),
        MAT_AT(*X, S_VEL_Y, 0),
    }};
}

Mat H_gnss(Mat *X) {
    Mat J = { .r=4, .c=S_STATE_DIM };
    MAT_AT(J, 0, S_POS_X) = 1.0;
    MAT_AT(J, 1, S_POS_Y) = 1.0;
    MAT_AT(J, 2, S_VEL_X) = 1.0;
    MAT_AT(J, 3, S_VEL_Y) = 1.0;
    
    return J;
}


// --- Multiplicative Extended Kalman Filter ---
enum {
    ERR_S_POS_X,        // m
    ERR_S_POS_Y,        // m
    ERR_S_POS_Z,        // m
    ERR_S_VEL_X,        // m/s
    ERR_S_VEL_Y,        // m/s
    ERR_S_VEL_Z,        // m/s
    ERR_S_THETA_X,      // rad
    ERR_S_THETA_Y,      // rad
    ERR_S_THETA_Z,      // rad
    ERR_S_ACC_BIAS_X,   // m/s^2
    ERR_S_ACC_BIAS_Y,   // m/s^2
    ERR_S_ACC_BIAS_Z,   // m/s^2
    ERR_S_OMEGA_BIAS_X, // rad/s
    ERR_S_OMEGA_BIAS_Y, // rad/s
    ERR_S_OMEGA_BIAS_Z, // rad/s
    ERR_S_GRAVITY_X,    // m/s^2
    ERR_S_GRAVITY_Y,    // m/s^2
    ERR_S_GRAVITY_Z,    // m/s^2
    ERR_S_STATE_DIM
};

enum {
    ERR_S_PROC_VEL_X,
    ERR_S_PROC_VEL_Y,
    ERR_S_PROC_VEL_Z,
    ERR_S_PROC_THETA_X,
    ERR_S_PROC_THETA_Y,
    ERR_S_PROC_THETA_Z,
    ERR_S_PROC_ACC_BIAS_X,
    ERR_S_PROC_ACC_BIAS_Y,
    ERR_S_PROC_ACC_BIAS_Z,
    ERR_S_PROC_OMEGA_BIAS_X,
    ERR_S_PROC_OMEGA_BIAS_Y,
    ERR_S_PROC_OMEGA_BIAS_Z,
    ERR_S_PROC_DIM,
};

typedef struct {
    Mat nom_X;  // Nominal state
    Mat err_P;  // Error-state covariance
    Mat err_Q;  // Error-state process covariance
    Mat err_Fi; // Error-state process covariance state mapping
} MEKF_State;

void mekf_init(MEKF_State *state) {
    // --- Initialize state ---
    state->nom_X = (Mat){ .r=S_STATE_DIM, .c=1};
    MAT_AT(state->nom_X, S_QUAT_R, 0) =     1.0;    // Correctly initialize quaternion
    MAT_AT(state->nom_X, S_GRAVITY_Z, 0) = -G;

    // --- State covariance ---
    state->err_P = (Mat){ .r=ERR_S_STATE_DIM, .c=ERR_S_STATE_DIM };
    SET_XYZ(state->err_P, ERR_S_POS_X,        ERR_S_POS_X,        1e-2);
    SET_XYZ(state->err_P, ERR_S_VEL_X,        ERR_S_VEL_X,        1e-1);
    SET_XYZ(state->err_P, ERR_S_THETA_X,      ERR_S_THETA_X,      1e-1);
    SET_XYZ(state->err_P, ERR_S_ACC_BIAS_X,   ERR_S_ACC_BIAS_X,   1e0 );
    SET_XYZ(state->err_P, ERR_S_OMEGA_BIAS_X, ERR_S_OMEGA_BIAS_X, 1e0 );
    SET_XYZ(state->err_P, ERR_S_GRAVITY_X,    ERR_S_GRAVITY_X,    1e-1);

    // --- Process covariance ---
    state->err_Q = (Mat){ .r=ERR_S_PROC_DIM,  .c=ERR_S_PROC_DIM  };
    const f64 acc_noise_sdev =       0.00637;
    const f64 acc_rand_walk_sdev =   1e-1;
    const f64 omega_noise_sdev =     0.05 * DEG2RAD;
    const f64 omega_rand_walk_sdev = 0.5 * DEG2RAD;

    const f64 acc_noise_var =       acc_noise_sdev*acc_noise_sdev;
    const f64 acc_rand_walk_var =   acc_rand_walk_sdev*acc_rand_walk_sdev;
    const f64 omega_noise_var =     omega_noise_sdev*omega_noise_sdev;
    const f64 omega_rand_walk_var = omega_rand_walk_sdev*omega_rand_walk_sdev;

    SET_XYZ(state->err_Q, ERR_S_PROC_VEL_X,        ERR_S_PROC_VEL_X,        acc_noise_var       *c_dt*c_dt);
    SET_XYZ(state->err_Q, ERR_S_PROC_THETA_X,      ERR_S_PROC_THETA_X,      omega_noise_var     *c_dt*c_dt);
    SET_XYZ(state->err_Q, ERR_S_PROC_ACC_BIAS_X,   ERR_S_PROC_ACC_BIAS_X,   acc_rand_walk_var   *c_dt     );
    SET_XYZ(state->err_Q, ERR_S_PROC_OMEGA_BIAS_X, ERR_S_PROC_OMEGA_BIAS_X, omega_rand_walk_var *c_dt     );

    state->err_Fi = (Mat){ .r=ERR_S_STATE_DIM,  .c=ERR_S_PROC_DIM  };
    SET_XYZ(state->err_Fi, ERR_S_VEL_X,        ERR_S_PROC_VEL_X,        1.0);
    SET_XYZ(state->err_Fi, ERR_S_THETA_X,      ERR_S_PROC_THETA_X,      1.0);
    SET_XYZ(state->err_Fi, ERR_S_ACC_BIAS_X,   ERR_S_PROC_ACC_BIAS_X,   1.0);
    SET_XYZ(state->err_Fi, ERR_S_OMEGA_BIAS_X, ERR_S_PROC_OMEGA_BIAS_X, 1.0);
};

//Mat err_F(Mat X, Mat err_X, vec3 acc_meas, vec3 gyro_meas) {
//    vec3 err_pos = {
//        err_X.data[ERR_S_POS_X],
//        err_X.data[ERR_S_POS_Y],
//        err_X.data[ERR_S_POS_Z]
//    };
//    
//    vec3 err_vel = {
//        err_X.data[ERR_S_VEL_X],
//        err_X.data[ERR_S_VEL_Y],
//        err_X.data[ERR_S_VEL_Z]
//    };
//
//    vec3 err_theta = {
//        err_X.data[ERR_S_THETA_X],
//        err_X.data[ERR_S_THETA_Y],
//        err_X.data[ERR_S_THETA_Z]
//    };
//
//    quat q = QUAT_FROM_STATE(X);
//
//    vec3 omega_bias = {
//        X.data[S_OMEGA_BIAS_X],
//        X.data[S_OMEGA_BIAS_Y],
//        X.data[S_OMEGA_BIAS_Z]
//    };
//    
//    vec3 err_omega_bias = {
//        err_X.data[S_OMEGA_BIAS_X],
//        err_X.data[S_OMEGA_BIAS_Y],
//        err_X.data[S_OMEGA_BIAS_Z]
//    };
//
//    // err_pos = err_pos + err_vel * dt
//    vec3 err_vel_delta = vec3_scale(&err_vel, c_dt);
//    vec3 new_err_pos = vec3_sum(&err_pos, &err_vel_delta);
//    
//    // err_vel = err_vel + (-R * (a_meas x err_theta)) * dt
//    vec3 body_term = vec3_cross(&acc_meas, &err_theta);
//    vec3 world_term = vec3_rotate_by_quat(&body_term, &q);
//    world_term = vec3_scale(&world_term, -c_dt);
//
//    vec3 new_err_vel = vec3_sum(&err_vel, &world_term);
//    
//    // err_theta = R'((gyro_meas - omega_bias) * dt) * err_theta - err_omega_bias * dt
//    vec3 omega_true = vec3_sub(&gyro_meas, &omega_bias);
//    vec3 omega_true_delta = vec3_scale(&omega_true, c_dt);
//    quat q_inv = (quat) {
//        .r =  q.r,
//        .i = -q.i,
//        .j = -q.j,
//        .k = -q.k
//    };
//    vec3 body_omega_true = vec3_rotate_by_quat(&omega_true_delta, &q_inv);
//    vec3 new_err_theta = vec3_dot(&body_omega_true, &err_theta);
//
//    vec3 err_omega_bias_delta = vec3_scale(&err_omega_bias, c_dt);
//    new_err_theta = vec3_sub(&new_err_theta, &err_omega_bias_delta);
//
//    // err_theta = err_theta - (gyro_meas x omega_bias) * dt - err_omega_bias * dt
//    //vec3 omega_true = vec3_sub(&gyro_meas, &omega_bias);
//    //vec3 omega_cross_theta = vec3_cross(&omega_true, &err_theta);
//    //omega_cross_theta = vec3_scale(&omega_cross_theta, c_dt); 
//    
//    //vec3 new_err_theta = vec3_sub(&err_theta, &omega_cross_theta);
//
//    //vec3 err_omega_bias_delta = vec3_scale(&err_omega_bias, c_dt);
//    //new_err_theta = vec3_sub(&new_err_theta, &err_omega_bias_delta);
//
//    return (Mat) { .r=ERR_S_STATE_DIM, .c=1, {
//        new_err_pos.x,
//        new_err_pos.y,
//        new_err_pos.z,
//        new_err_vel.x,
//        new_err_vel.y,
//        new_err_vel.z,
//        new_err_theta.x,
//        new_err_theta.y,
//        new_err_theta.z,
//        err_X.data[ERR_S_OMEGA_BIAS_X],
//        err_X.data[ERR_S_OMEGA_BIAS_Y],
//        err_X.data[ERR_S_OMEGA_BIAS_Z]
//    }};
//}

Mat err_J(Mat *nom_X, vec3 acc_meas, vec3 gyro_meas) {
    vec3 acc_bias = {
        nom_X->data[S_ACC_BIAS_X],
        nom_X->data[S_ACC_BIAS_Y],
        nom_X->data[S_ACC_BIAS_Z]
    };
    
    vec3 omega_bias = {
        nom_X->data[S_OMEGA_BIAS_X],
        nom_X->data[S_OMEGA_BIAS_Y],
        nom_X->data[S_OMEGA_BIAS_Z]
    };
    
    quat q = QUAT_FROM_STATE(*nom_X);
    
    Mat err_J = { .r=ERR_S_STATE_DIM, .c=ERR_S_STATE_DIM };

    // --- Position ---
    // err_pos = err_pos + err_vel * dt
    SET_XYZ(err_J, ERR_S_POS_X, ERR_S_POS_X, 1.0); 
    SET_XYZ(err_J, ERR_S_POS_X, ERR_S_VEL_X, c_dt);

    // --- Velocity ---
    // err_vel = err_vel -R [a_meas - a_bias]_x err_theta dt - R err_a_bias dt
    SET_XYZ(err_J, ERR_S_VEL_X, ERR_S_VEL_X, 1.0);

    vec3 acc_true = vec3_sub(&acc_meas, &acc_bias);
    Mat acc_body_x = mat_skew_symm_from_vec3(&acc_true);
    Mat rot_mat = mat_from_quat(&q);
    Mat acc_world_x = mat_mul(&rot_mat, &acc_body_x);

    for (u64 r=0; r<3; r++) {
        for (u64 c=0; c<3; c++) {
            MAT_AT(err_J, ERR_S_VEL_X+r, ERR_S_THETA_X+c) = -MAT_AT(acc_world_x, r, c) * c_dt;
        }
    }

    for (u64 r=0; r<3; r++) {
        for (u64 c=0; c<3; c++) {
            MAT_AT(err_J, ERR_S_VEL_X+r, ERR_S_ACC_BIAS_X+c) = -MAT_AT(rot_mat, r, c) * c_dt;
        }
    }

    SET_XYZ(err_J, ERR_S_VEL_X, ERR_S_GRAVITY_X, c_dt);

    // --- Theta ---
    // err_theta = R'{gyro_meas - omega_bias} dt err_theta - err_omega_bias dt
    // linearized: err_theta = (Id - [omega_true]_x dt) err_theta - err_omega_bias dt
    
    vec3 omega_true = vec3_sub(&gyro_meas, &omega_bias);
    Mat omega_x = mat_skew_symm_from_vec3(&omega_true);

    SET_XYZ(err_J, ERR_S_THETA_X, ERR_S_THETA_X, 1.0);
    for (u64 r=0; r<3; r++) {
        for (u64 c=0; c<3; c++) {
            MAT_AT(err_J, ERR_S_THETA_X+r, ERR_S_THETA_X+c) -=
                c_dt * MAT_AT(omega_x, r, c);
        }
    }
    SET_XYZ(err_J, ERR_S_THETA_X, ERR_S_OMEGA_BIAS_X, -c_dt);

    // --- Accelerometer bias ---
    SET_XYZ(err_J, ERR_S_ACC_BIAS_X, ERR_S_ACC_BIAS_X, 1.0);

    // --- Omega bias ---
    SET_XYZ(err_J, ERR_S_OMEGA_BIAS_X, ERR_S_OMEGA_BIAS_X, 1.0);

    // --- Gravity vector ---
    SET_XYZ(err_J, ERR_S_GRAVITY_X, ERR_S_GRAVITY_X, 1.0);

    return err_J;
}

void mekf_predict(MEKF_State *state, vec3 acc_meas, vec3 gyro_meas) {
    // err_P = err_J err_P err_J' + err_Fi err_Q err_Fi'
    Mat err_Jx = err_J(&state->nom_X, acc_meas, gyro_meas);
    Mat err_Jx_t = mat_trans(&err_Jx);
    Mat new_err_P = mat_mul(&err_Jx, &state->err_P);
    new_err_P = mat_mul(&new_err_P, &err_Jx_t);
    
    Mat err_Fi_t = mat_trans(&state->err_Fi);
    Mat new_err_Q = mat_mul(&state->err_Fi, &state->err_Q);
    new_err_Q = mat_mul(&new_err_Q, &err_Fi_t);
    
    state->err_P = mat_sum(&new_err_P, &new_err_Q);
    

    state->nom_X = F(&state->nom_X, acc_meas, gyro_meas);
    
    // err_X = err_F(...)
    // According to JoanSola: this always returns zero, so it can be skipped
}

typedef Mat (*Mat_fn)(Mat *X);
void mekf_correct(
    MEKF_State *state,
    Mat *Z,     // Measurements matrix
    Mat_fn h,   // Observation matrix
    Mat_fn H_x, // Jacobian observation matrix
    Mat *R      // Measurement noise
) {
    //assert(Z->r == H->r && R->r == R->c && R->r == H->r && "Wrong arguments dimensions");
    
    quat q = QUAT_FROM_STATE(state->nom_X);

    Mat X_err_X = { .r=S_STATE_DIM, .c=ERR_S_STATE_DIM };
    SET_XYZ(X_err_X, S_POS_X, ERR_S_POS_X, 1.0);                  // Position
    SET_XYZ(X_err_X, S_VEL_X, ERR_S_VEL_X, 1.0);                  // Velocity
    
    MAT_AT(X_err_X, S_QUAT_R, ERR_S_THETA_X) = -0.5*q.i;          // Orientation
    MAT_AT(X_err_X, S_QUAT_R, ERR_S_THETA_Y) = -0.5*q.j;
    MAT_AT(X_err_X, S_QUAT_R, ERR_S_THETA_Z) = -0.5*q.k;
    MAT_AT(X_err_X, S_QUAT_I, ERR_S_THETA_X) =  0.5*q.r;
    MAT_AT(X_err_X, S_QUAT_I, ERR_S_THETA_Y) = -0.5*q.k;
    MAT_AT(X_err_X, S_QUAT_I, ERR_S_THETA_Z) =  0.5*q.j;
    MAT_AT(X_err_X, S_QUAT_J, ERR_S_THETA_X) =  0.5*q.k;
    MAT_AT(X_err_X, S_QUAT_J, ERR_S_THETA_Y) =  0.5*q.r;
    MAT_AT(X_err_X, S_QUAT_J, ERR_S_THETA_Z) = -0.5*q.i;
    MAT_AT(X_err_X, S_QUAT_K, ERR_S_THETA_X) = -0.5*q.j;
    MAT_AT(X_err_X, S_QUAT_K, ERR_S_THETA_Y) =  0.5*q.i;
    MAT_AT(X_err_X, S_QUAT_K, ERR_S_THETA_Z) =  0.5*q.r;

    SET_XYZ(X_err_X, S_ACC_BIAS_X,   ERR_S_ACC_BIAS_X,   1.0);    // Accelerometer bias
    SET_XYZ(X_err_X, S_OMEGA_BIAS_X, ERR_S_OMEGA_BIAS_X, 1.0);    // Omega bias

    // Compute innovation
    // I = Z - h(nom_X)
    Mat hX = h(&state->nom_X);
    Mat I = mat_sub(Z, &hX);   // This is in measurement space

    // H = H_x X_err_X
    Mat Hx = H_x(&state->nom_X);
    Mat H = mat_mul(&Hx, &X_err_X);
    
    // Compute innovation covariance
    // S = H P H' + R
    Mat H_t = mat_trans(&H);
    Mat S = mat_mul(&H, &state->err_P);
    S = mat_mul(&S, &H_t);
    S = mat_sum(&S, R);

    // Compute Kalman gain
    // K = P H' S^-1
    Mat S_inv = mat_inv(&S);
    Mat K = mat_mul(&state->err_P, &H_t);
    K = mat_mul(&K, &S_inv);

    // Update nominal state
    Mat err_X_hat = mat_mul(&K, &I);
    state->nom_X.data[S_POS_X] += err_X_hat.data[ERR_S_POS_X];
    state->nom_X.data[S_POS_Y] += err_X_hat.data[ERR_S_POS_Y];
    state->nom_X.data[S_POS_Z] += err_X_hat.data[ERR_S_POS_Z];
    state->nom_X.data[S_VEL_X] += err_X_hat.data[ERR_S_VEL_X];
    state->nom_X.data[S_VEL_Y] += err_X_hat.data[ERR_S_VEL_Y];
    state->nom_X.data[S_VEL_Z] += err_X_hat.data[ERR_S_VEL_Z];

    quat err_q_hat = {
        .r=1.0,
        .i=0.5*err_X_hat.data[ERR_S_THETA_X],
        .j=0.5*err_X_hat.data[ERR_S_THETA_Y],
        .k=0.5*err_X_hat.data[ERR_S_THETA_Z]
    };
    quat new_q = quat_mul(&q, &err_q_hat);
    quat_norm(&new_q);      // Just to be sure
    
    state->nom_X.data[S_QUAT_R] = new_q.r;
    state->nom_X.data[S_QUAT_I] = new_q.i;
    state->nom_X.data[S_QUAT_J] = new_q.j;
    state->nom_X.data[S_QUAT_K] = new_q.k;
    
    state->nom_X.data[S_ACC_BIAS_X] += err_X_hat.data[ERR_S_ACC_BIAS_X];
    state->nom_X.data[S_ACC_BIAS_Y] += err_X_hat.data[ERR_S_ACC_BIAS_Y];
    state->nom_X.data[S_ACC_BIAS_Z] += err_X_hat.data[ERR_S_ACC_BIAS_Z];
    
    state->nom_X.data[S_OMEGA_BIAS_X] += err_X_hat.data[ERR_S_OMEGA_BIAS_X];
    state->nom_X.data[S_OMEGA_BIAS_Y] += err_X_hat.data[ERR_S_OMEGA_BIAS_Y];
    state->nom_X.data[S_OMEGA_BIAS_Z] += err_X_hat.data[ERR_S_OMEGA_BIAS_Z];

    // Update covariance
    // P = (Id - K H) P
    //Mat Id = mat_identity(ERR_S_STATE_DIM);
    //Mat KH = mat_mul(&K, &H);
    //Mat IdsubKH = mat_sub(&Id, &KH);
    //err_P = mat_mul(&IdsubKH, &err_P);
    // 
    // Joseph-form:
    // P = (Id - K H) P (Id - K H)' + K V K'
    // A = Id - K H
    // P = A P A' + K R K'
    
    Mat KH = mat_mul(&K, &H);
    Mat Id = mat_identity(ERR_S_STATE_DIM);
    Mat A = mat_sub(&Id, &KH);
    Mat A_t = mat_trans(&A);

    Mat AP = mat_mul(&A, &state->err_P);
    Mat APA_t = mat_mul(&AP, &A_t);

    Mat K_t = mat_trans(&K);
    Mat KR = mat_mul(&K, R);
    Mat KRK_t = mat_mul(&KR, &K_t);
    
    state->err_P = mat_sum(&APA_t, &KRK_t);
    
    // Reset
    // No need to do anything since the error-state lives only inside this function
    //for (u64 i=0; i<ERR_S_STATE_DIM; i++) {
    //    err_X->data[i] = 0.0;
    //}

    // Check err_P
    // * Symmetric
    // * Semi-positive (positive diagonal)
    for (u64 r=0; r<state->err_P.r; r++) {
        for (u64 c=r+1; c<state->err_P.c; c++) {
            f64 diff = MAT_AT(state->err_P, r, c) - MAT_AT(state->err_P, c, r);
            assert(fabs(diff) < 1e-10);
        }
    }

    for (u64 r=0; r<state->err_P.r; r++) {
        assert(MAT_AT(state->err_P, r, r) >= 0.0);
    }
}


u64 bar_l = 0;
u64 imu_l = 0;
u64 gnss_l = 0;
u64 mag_l = 0;

f64 c_rot_w[NUM_ROT] = {0};

vec3 gyro_meas = {0};
vec3 acc_meas =  {0};

MEKF_State mekf_state = {0};
void c_init() {
    mekf_init(&mekf_state);
}

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

    //f64 tot_mot_f = 0.0;
    //for (u64 i=0; i<NUM_ROT; i++) {
    //    tot_mot_f += kf * c_rot_w[i] * c_rot_w[i];
    //}
    
    //ekf_predict(acc_meas, gyro_meas);
    mekf_predict(&mekf_state, acc_meas, gyro_meas);

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

        //ekf_correct(&Z, h_alt, H_alt, &R);
        mekf_correct(&mekf_state, &Z, h_alt, H_alt, &R);
    }

    // --- IMU ---
    if (imu_l >= imu_loops) {
        imu_l=0;

        f64 ax = intr->imu_acc_x; 
        f64 ay = intr->imu_acc_y; 
        f64 az = intr->imu_acc_z;

#ifdef CONTROL_DEBUG
        intr->dbg[DBG_IN_ACC_X] = ax;
        intr->dbg[DBG_IN_ACC_Y] = ay;
        intr->dbg[DBG_IN_ACC_Z] = az;
#endif

        acc_meas = (vec3) {
            ax, ay, az
        };

        f64 gyro_rate_x = intr->imu_gyro_x;
        f64 gyro_rate_y = intr->imu_gyro_y;
        f64 gyro_rate_z = intr->imu_gyro_z;

#ifdef CONTROL_DEBUG
        intr->dbg[DBG_IN_ROT_X] = gyro_rate_x;
        intr->dbg[DBG_IN_ROT_Y] = gyro_rate_y;
        intr->dbg[DBG_IN_ROT_Z] = gyro_rate_z;
#endif
        gyro_meas = (vec3) {
            gyro_rate_x,
            gyro_rate_y,
            gyro_rate_z
        };
    }
    
    // --- GNSS ---
    if (gnss_l >= gnss_loops) {
        gnss_l=0;

        f64 px = intr->pos_x / 100.0;
        f64 py = intr->pos_y / 100.0;

        f64 vx = intr->vel_x;
        f64 vy = intr->vel_y;

#ifdef CONTROL_DEBUG
        intr->dbg[DBG_IN_POS_X] = px;
        intr->dbg[DBG_IN_POS_Y] = py;
        intr->dbg[DBG_IN_VEL_X] = vx;
        intr->dbg[DBG_IN_VEL_Y] = vy;
#endif

        Mat Z = { .r=4, .c=1, {
            px,
            py,
            vx,
            vy,
        }};

        const f64 gnss_pos_sdev = 2.5; // m
        const f64 gnss_pos_var = gnss_pos_sdev * gnss_pos_sdev;   // m^2

        const f64 gnss_vel_sdev = 0.05; // m/s
        const f64 gnss_vel_var = gnss_vel_sdev * gnss_vel_sdev;

        Mat R = { .r=Z.r, .c=Z.r };
        MAT_AT(R, 0, 0) = gnss_pos_var;
        MAT_AT(R, 1, 1) = gnss_pos_var;
        MAT_AT(R, 2, 2) = gnss_vel_var;
        MAT_AT(R, 3, 3) = gnss_vel_var;

        //ekf_correct(&Z, h_gnss, H_gnss, &R);
        mekf_correct(&mekf_state, &Z, h_gnss, H_gnss, &R);
    }

    // --- Magnetometer ---
    if (mag_l >= mag_loops) {
        mag_l=0;
        
        //f64 alpha = X.data[S_ORI_X];
        //f64 beta  = X.data[S_ORI_Y];
        //f64 gamma = X.data[S_ORI_Z];
        
        vec3 mag = {
            .x = intr->mag_x,
            .y = intr->mag_y,
            .z = intr->mag_z
        };
    
        vec3 mag_dir = vec3_norm(&mag);

#ifdef CONTROL_DEBUG
        intr->dbg[DBG_IN_MAG_X] = mag_dir.x;
        intr->dbg[DBG_IN_MAG_Y] = mag_dir.y;
        intr->dbg[DBG_IN_MAG_Z] = mag_dir.z;
#endif    
        
        Mat Z = { .r=3, .c=1, {
            mag_dir.x,
            mag_dir.y,
            mag_dir.z,
        }};

        const f64 mag_sdev = 1e0;
        const f64 mag_var = mag_sdev * mag_sdev;
        Mat R = { .r=Z.r, .c=Z.r };
        MAT_AT(R, 0, 0) = mag_var;
        MAT_AT(R, 1, 1) = mag_var;
        MAT_AT(R, 2, 2) = mag_var;

        //ekf_correct(&Z, h_body_north_dir, H_body_north_dir, &R);
        mekf_correct(&mekf_state, &Z, h_body_north_dir, H_body_north_dir, &R);
    }

    // --- Flightplan ---
    Waypoint wp = flightplan[curr_wp];

    f64 distance = sqrt(
        pow(fabs(mekf_state.nom_X.data[S_POS_X] - wp.x), 2.0) + 
        pow(fabs(mekf_state.nom_X.data[S_POS_Y] - wp.y), 2.0) + 
        pow(fabs(mekf_state.nom_X.data[S_POS_Z] - wp.z), 2.0)
    );

    if (distance <= wp_radius) {
        curr_wp = (curr_wp + 1) % NUM_FLIGHTPLAN_WAYPOINTS;
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
    f64 tgt_alt = flightplan[curr_wp].z; // m
    f64 vel_z_tgt = pid_step(mekf_state.nom_X.data[S_POS_Z], tgt_alt, &vel_z_pid_p, &alt_pid_s);

    // Since we have all the initial parameters, we could compute the hovering cmd:
    // NUM_ARMS * (hover_cmd*max_rot_w)^2 * kf / m = g
    // (hover_cmd*max_rot_w)^2 = g / (NUM_ROT * kf) * m
    // sqrt(g * m / (NUM_ROT * kf)) / max_rot_w = hover_cmd

    const f64 hover_cmd =  sqrt(G * mass / (NUM_ROT*kf)) / rot_max_w;
    f64 out_cmd = hover_cmd + pid_step(mekf_state.nom_X.data[S_VEL_Z], vel_z_tgt, &mot_pid_p, &mot_pid_s);

    // pos -> vel
    f64 tgt_pos_x = flightplan[curr_wp].x;
    f64 tgt_pos_y = flightplan[curr_wp].y;
    f64 vel_x_tgt = pid_step(mekf_state.nom_X.data[S_POS_X], tgt_pos_x, &pos_pid_p, &pos_x_pid_s);
    f64 vel_y_tgt = pid_step(mekf_state.nom_X.data[S_POS_Y], tgt_pos_y, &pos_pid_p, &pos_y_pid_s);

    //printf("vx: %lf\n", vel_x_tgt);

    // vel -> ori
    // According the the ENU axis reference:
    //  * +vel_x => +ori_y
    //  * +vel_y => -ori_x

    quat q = QUAT_FROM_STATE(mekf_state.nom_X);
    quat q_inv = (quat){ q.r, -q.i, -q.j, -q.k };
    
    vec3 world_tgt_vel = { .x=vel_x_tgt, .y=vel_y_tgt };
    vec3 world_vel = {
        .x=mekf_state.nom_X.data[S_VEL_X],
        .y=mekf_state.nom_X.data[S_VEL_Y],
        .z=mekf_state.nom_X.data[S_VEL_Z]
    };
    
    vec3 body_tgt_vel = vec3_rotate_by_quat(&world_tgt_vel, &q_inv);
    vec3 body_vel     = vec3_rotate_by_quat(&world_vel,     &q_inv);

    f64 ori_y_tgt =  pid_step(body_vel.x, body_tgt_vel.x, &vel_pid_p, &vel_y_pid_s);
    f64 ori_x_tgt = -pid_step(body_vel.y, body_tgt_vel.y, &vel_pid_p, &vel_x_pid_s);

    // ori -> rot
    vec3 ori = quat_to_euler_zyx(&q);
    f64 tgt_yaw = flightplan[curr_wp].yaw;
    f64 rot_x_tgt = angle_pid_step(ori.x, ori_x_tgt, &ori_pid_p, &ori_x_pid_s);
    f64 rot_y_tgt = angle_pid_step(ori.y, ori_y_tgt, &ori_pid_p, &ori_y_pid_s);
    f64 rot_z_tgt = angle_pid_step(ori.z, tgt_yaw,   &ori_pid_p, &ori_z_pid_s);
    
    //printf("o: %lf, tr: %lf\n", ori_x, rot_x_tgt);

    //printf(
    //    "pos_y: %lf, tgt_ori_x: %lf, ori_x: %lf, tgt_rot_x: %lf\n",
    //    X.data[S_POS_Y],
    //    ori_x_tgt,
    //    X.data[S_ORI_X],
    //    rot_x_tgt
    //);

    // rot -> cmd
    vec3 omega = {
        gyro_meas.x - mekf_state.nom_X.data[S_OMEGA_BIAS_X],
        gyro_meas.y - mekf_state.nom_X.data[S_OMEGA_BIAS_Y],
        gyro_meas.z - mekf_state.nom_X.data[S_OMEGA_BIAS_Z],
    };

    f64 x_cmd = pid_step(omega.x, rot_x_tgt, &rot_pid_p, &rot_x_pid_s);
    f64 y_cmd = pid_step(omega.y, rot_y_tgt, &rot_pid_p, &rot_y_pid_s);
    f64 z_cmd = pid_step(omega.z, rot_z_tgt, &rot_z_pid_p, &rot_z_pid_s);

    // --- Motor mixer ---
    // fl > fr > rr > rl
    intr->rot_cmd[0] = clamp(out_cmd + x_cmd + y_cmd + z_cmd, 0.0, 1.0);
    intr->rot_cmd[1] = clamp(out_cmd + x_cmd - y_cmd - z_cmd, 0.0, 1.0);
    intr->rot_cmd[2] = clamp(out_cmd - x_cmd - y_cmd + z_cmd, 0.0, 1.0);
    intr->rot_cmd[3] = clamp(out_cmd - x_cmd + y_cmd - z_cmd, 0.0, 1.0);

#ifdef CONTROL_DEBUG
    intr->dbg[DBG_HOVER_CMD] = out_cmd;
    intr->dbg[DBG_X_CMD] =     x_cmd;
    intr->dbg[DBG_Y_CMD] =     y_cmd;
    intr->dbg[DBG_Z_CMD] =     z_cmd;
    
    intr->dbg[DBG_ROT_X_TGT] = rot_x_tgt;
    intr->dbg[DBG_ROT_Y_TGT] = rot_y_tgt;
    intr->dbg[DBG_ORI_X_TGT] = ori_x_tgt;
    intr->dbg[DBG_ORI_Y_TGT] = ori_y_tgt;
    intr->dbg[DBG_ORI_Z_TGT] = tgt_yaw;
    intr->dbg[DBG_VEL_X_TGT] = vel_x_tgt;
    intr->dbg[DBG_VEL_Y_TGT] = vel_y_tgt;
    intr->dbg[DBG_VEL_Z_TGT] = vel_z_tgt;
    
    // State vector
    //intr->dbg[DBG_POS_X] = X.data[S_POS_X];
    //intr->dbg[DBG_POS_Y] = X.data[S_POS_Y];
    //intr->dbg[DBG_POS_Z] = X.data[S_POS_Z];
    
    //intr->dbg[DBG_VEL_X] = X.data[S_VEL_X];
    //intr->dbg[DBG_VEL_Y] = X.data[S_VEL_Y];
    //intr->dbg[DBG_VEL_Z] = X.data[S_VEL_Z];
    
    //intr->dbg[DBG_ACC_BIAS_Z] = X.data[S_ACC_BIAS_Z];
    
    //quat aekf_q = QUAT_FROM_STATE(X);
    //vec3 aekf_ori = quat_to_euler_zyx(&aekf_q);
    //intr->dbg[DBG_ORI_X] = aekf_ori.x;
    //intr->dbg[DBG_ORI_Y] = aekf_ori.y;
    //intr->dbg[DBG_ORI_Z] = aekf_ori.z;
    
    intr->dbg[DBG_OMEGA_X] = omega.x;
    intr->dbg[DBG_OMEGA_Y] = omega.y;
    intr->dbg[DBG_OMEGA_Z] = omega.z;
    
    //intr->dbg[DBG_GYRO_BIAS_X] = X.data[S_OMEGA_BIAS_X];
    //intr->dbg[DBG_GYRO_BIAS_Y] = X.data[S_OMEGA_BIAS_Y];
    //intr->dbg[DBG_GYRO_BIAS_Z] = X.data[S_OMEGA_BIAS_Z];

    // Covariance vector
    //intr->dbg[DBG_COV_VEL_X] = MAT_AT(P, S_VEL_X, S_VEL_X);
    //intr->dbg[DBG_COV_VEL_Y] = MAT_AT(P, S_VEL_Y, S_VEL_Y);
    //intr->dbg[DBG_COV_VEL_Z] = MAT_AT(P, S_VEL_Z, S_VEL_Z);
    
    //intr->dbg[DBG_COV_QUAT_R] = MAT_AT(P, S_QUAT_R, S_QUAT_R);
    //intr->dbg[DBG_COV_QUAT_I] = MAT_AT(P, S_QUAT_I, S_QUAT_I);
    //intr->dbg[DBG_COV_QUAT_J] = MAT_AT(P, S_QUAT_J, S_QUAT_J);
    //intr->dbg[DBG_COV_QUAT_K] = MAT_AT(P, S_QUAT_K, S_QUAT_K);
    
    //intr->dbg[DBG_COV_OMEGA_X] = MAT_AT(P, S_OMEGA_X, S_OMEGA_X);
    //intr->dbg[DBG_COV_OMEGA_Y] = MAT_AT(P, S_OMEGA_Y, S_OMEGA_Y);
    //intr->dbg[DBG_COV_OMEGA_Z] = MAT_AT(P, S_OMEGA_Z, S_OMEGA_Z);

    // Multiplicative Extended Kalman Filter
    intr->dbg[DBG_NOM_POS_X] = mekf_state.nom_X.data[S_POS_X];
    intr->dbg[DBG_NOM_POS_Y] = mekf_state.nom_X.data[S_POS_Y];
    intr->dbg[DBG_NOM_POS_Z] = mekf_state.nom_X.data[S_POS_Z];
    intr->dbg[DBG_NOM_VEL_X] = mekf_state.nom_X.data[S_VEL_X];
    intr->dbg[DBG_NOM_VEL_Y] = mekf_state.nom_X.data[S_VEL_Y];
    intr->dbg[DBG_NOM_VEL_Z] = mekf_state.nom_X.data[S_VEL_Z];
    
    quat nom_q = QUAT_FROM_STATE(mekf_state.nom_X);
    vec3 nom_ori = quat_to_euler_zyx(&nom_q);
    intr->dbg[DBG_NOM_ORI_X] = nom_ori.x;
    intr->dbg[DBG_NOM_ORI_Y] = nom_ori.y;
    intr->dbg[DBG_NOM_ORI_Z] = nom_ori.z;
    
    intr->dbg[DBG_NOM_ACC_BIAS_X] = mekf_state.nom_X.data[S_ACC_BIAS_X];
    intr->dbg[DBG_NOM_ACC_BIAS_Y] = mekf_state.nom_X.data[S_ACC_BIAS_Y];
    intr->dbg[DBG_NOM_ACC_BIAS_Z] = mekf_state.nom_X.data[S_ACC_BIAS_Z];
    
    intr->dbg[DBG_NOM_GYRO_BIAS_X] = mekf_state.nom_X.data[S_OMEGA_BIAS_X];
    intr->dbg[DBG_NOM_GYRO_BIAS_Y] = mekf_state.nom_X.data[S_OMEGA_BIAS_Y];
    intr->dbg[DBG_NOM_GYRO_BIAS_Z] = mekf_state.nom_X.data[S_OMEGA_BIAS_Z];
    
    intr->dbg[DBG_NOM_GRAVITY_X] = mekf_state.nom_X.data[S_GRAVITY_X];
    intr->dbg[DBG_NOM_GRAVITY_Y] = mekf_state.nom_X.data[S_GRAVITY_Y];
    intr->dbg[DBG_NOM_GRAVITY_Z] = mekf_state.nom_X.data[S_GRAVITY_Z];
#endif
}
