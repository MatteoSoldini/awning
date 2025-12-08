#include "control.h"

#include <stdio.h>
#include <math.h>
#include <assert.h>
#include <stdlib.h>

#define PI 3.14159265358979323846
#define DEG2RAD (PI/180.0f)

const f64 c_dt = 1.0 / CONTROL_FQ;

// Quadcopter physics
static const f64 kf = 1e-2;  // Thrust coefficient - N / (rad/s)^2
static const f64 mass = 2.0; // Kg
static const f64 g = 9.81;

// Barometer sensor
const f64 bar_fq = 50.0;  // Make sure it's divisible by CONTROL_FQ
const u64 bar_loops = CONTROL_FQ / bar_fq;
const f64 bar_dt = 1.0 / bar_fq;

// IMU sensor
const f64 imu_fq = 200.0; // Make sure it's divisible by CONTROL_FQ
const u64 imu_loops = CONTROL_FQ / imu_fq;
const f64 imu_dt = 1.0 / imu_fq;

static const f64 imu_acc_max_value = 8.0*g; // m/s^2
f64 imu_acc_to_ms2(i16 raw_acc) {
    return ((f64)raw_acc / (f64)INT16_MAX) * imu_acc_max_value; 
}

static const f64 imu_rot_max_value = 250.0; // rad/s
f64 imu_gyro_to_rad(i16 raw_gyro) {
    return ((f64)raw_gyro / (f64)INT16_MAX) * imu_rot_max_value; 
}

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
    .p = 0.3,
    .i = 0.0,
    .d = 0.0,
    .dt = c_dt,
    .high = 0.5,
    .low = -0.5
};
PIDState mot_pid_s = {0};

PIDParams alt_pid_p = {
    .p = 2.0,
    .i = 0.0,
    .d = 0.0,
    .dt = c_dt,
    .high = 1.0,
    .low = -1.0
};
PIDState alt_pid_s = {0};

PIDParams ori_pid_p = {
    .p = 1.0,
    .i = 0.0,
    .d = 0.0,
    .dt = c_dt,
    .high = 0.2,
    .low = -0.2
};
PIDState ori_x_pid_s = {0};
PIDState ori_y_pid_s = {0};

PIDParams rot_pid_p = {
    .p = 1.0,
    .i = 0.0,
    .d = 0.0,
    .dt = c_dt,
    .high = 0.1,
    .low = -0.1
};
PIDState rot_x_pid_s = {0};
PIDState rot_y_pid_s = {0};

#define MAT_SIZE 256
typedef struct {
    u64 r;
    u64 c;
    f64 data[MAT_SIZE];
} Mat;

#define MAT_AT(M, row, col) ((M).data[(row) * (M).c + (col)])

// * https://kalmanfilter.net/

// A glorified EMA filter
// Two stages: predict and correct
// The output is a random (multi)variable

// You have to describe the system dynamics
// It is nice to represent the filter in matrix notation

Mat mat_mul(Mat *A, Mat *B) {
    assert(A->c == B->r && "Dimension mismatch");

    Mat C = {.r=A->r, .c=B->c};
    for (u64 i=0; i<A->r; i++) {
        for (u64 j=0; j<B->c; j++) {
            f64 sum = 0.0;
            for (u64 k=0; k<A->c; k++) {
                sum += A->data[i*A->c + k] * B->data[k*B->c + j];
            }
            C.data[i*C.c + j] = sum;
        }
    }
    return C;
}

Mat mat_sum(Mat *A, Mat *B) {
    assert(A->r == B->r && A->c == B->c && "Dimension mismatch");
    
    Mat C = {.r=A->r, .c=A->c};
    for (u64 i=0; i<A->r*A->c; i++) {
        C.data[i] = A->data[i] + B->data[i];
    }

    return C;
}

Mat mat_sub(Mat *A, Mat *B) {
    assert(A->r == B->r && A->c == B->c && "Dimension mismatch");
    
    Mat C = {.r=A->r, .c=A->c};
    for (u64 i=0; i<A->r*A->c; i++) {
        C.data[i] = A->data[i] - B->data[i];
    }

    return C;
}

Mat mat_trans(Mat *M) {
    Mat N = { .r=M->c, .c=M->r };
    for (u64 r=0; r<N.r; r++) {
        for (u64 c=0; c<N.c; c++) {
            N.data[r*N.c + c] = M->data[c*N.r + r];
        }
    }

    return N;
}

Mat mat_identity(u64 size) {
    Mat I = {0};
    I.r = size;
    I.c = size;
    for (u64 i=0; i<size; i++) {
        I.data[size*i + i] = 1.0;
    }

    return I;
}

void mat_print(Mat *M) {
    printf("[\n");
    for (u64 r=0; r<M->r; r++) {
        printf("\t");
        for (u64 c=0; c<M->c; c++) {
            printf("%5.3lf, ", M->data[r*M->c + c]);
        }
        printf("\n");
    }
    printf("]\n");
}

Mat mat_inv(Mat *M) {
    // A * A^-1 = I
    // There's no concept of matrix division.
    // If we multiply a matrix by an inversed matrix we achieve the same results a division.

    // We're going to use Gauss-Jordan elimination with no pivoting here
    // https://en.wikipedia.org/wiki/Gaussian_elimination
    // Numerical Recipes in C, Chapter 2.1 "Gauss-Jordan Elimination"

    // Row operations:
    // * Swapping two rows,
    // * Multiplying a row by a nonzero number,
    // * Adding a multiple of one row to another row.

    assert(M->c == M->r && "The matrix must be squared");

    u64 size = M->c;

    // Build the augmented matrix
    // aug: [M | I]
    Mat aug = {0};
    aug.r = size;
    aug.c = 2*size;

    for (u64 r=0; r<size; r++) {
        for (u64 c=0; c<size; c++) {
            aug.data[r*aug.c + c] = M->data[r*M->c + c];
        }
    }

    for (u64 i=0; i<size; i++) {
        MAT_AT(aug, i, size+i) = 1.0;
    }
    
    //mat_print(&aug);

    for (u64 r=0; r<aug.r; r++) {
        // Normalize rows
        f64 pivot = MAT_AT(aug, r, r);
        assert(pivot != 0.0);

        for (u64 c=r; c<aug.c; c++) {
            MAT_AT(aug, r, c) /= pivot;
        }

        // Eliminate this column from other rows
        for (u64 rr=0; rr<size; rr++) {
            if (rr == r) continue;

            f64 factor = MAT_AT(aug, rr, r); // what to subtract
            if (factor == 0.0) continue;

            for (u64 c=0; c<aug.c; c++) {
                MAT_AT(aug, rr, c) -= factor * MAT_AT(aug, r, c);
            }
        }

        // Then the right amount of the row is subtracted from each other
        // row to make all the remaining right element zero
        
        //mat_print(&aug);
    }

    Mat M_inv = {0};
    M_inv.r = size;
    M_inv.c = size;

    // Copy left augmented size back
    for (u64 r=0; r<size; r++) {
        for (u64 c=0; c<size; c++) {
            MAT_AT(M_inv, r, c) = MAT_AT(aug, r, c+size);
        }
    }

    // Test code
    //Mat I_test = mat_mul(M, &M_inv);
    //Mat I_known = mat_identity(size);

    //for (u64 r=0; r<M->r; r++) {
    //    for (u64 c=0; c<M->c; c++) {
    //        assert(I_test.data[r*I_test.r + c] == I_known.data[r*I_known.r + c]);
    //    }
    //}

    return M_inv;
}


// TODO: There should be a smartest/cleaner way to populate these matrices.
// Like a enum for indexing

// State
Mat X = { .r=9, .c=1, {
    0.0, // pos_z (m)
    0.0, // vel_z (m/s)
    0.0, // acc_z (m/s^2)
    0.0, // ori_x (rad)         // TODO: Using Euler angles, this is good enough for small angles.
    0.0, // ori_y (rad)         // but we should later move on to quaternions
    0.0, // ori_z (rad)
    0.0, // rot_x (rad/s)
    0.0, // rot_y (rad/s)
    0.0  // rot_z (rad/s)
}};

// State transition matrix
// it defines how we predict the state to changes
// basically the physics

Mat F = { .r=9, .c=9, {
    // pos_z, vel_z,  acc_z,         ori_x, ori_y, ori_z, rot_x, rot_y, rot_z
       1.0,   c_dt,   0.5*c_dt*c_dt, 0.0,   0.0,   0.0,   0.0,   0.0,   0.0,    // pos_z
       0.0,   1.0,    c_dt,          0.0,   0.0,   0.0,   0.0,   0.0,   0.0,    // vel_z
       0.0,   0.0,    1.0,           0.0,   0.0,   0.0,   0.0,   0.0,   0.0,    // acc_z
       0.0,   0.0,    0.0,           1.0,   0.0,   0.0,   c_dt,  0.0,   0.0,    // ori_x
       0.0,   0.0,    0.0,           0.0,   1.0,   0.0,   0.0,   c_dt,  0.0,    // ori_y
       0.0,   0.0,    0.0,           0.0,   0.0,   1.0,   0.0,   0.0,   c_dt,   // ori_z
       0.0,   0.0,    0.0,           0.0,   0.0,   0.0,   1.0,   0.0,   0.0,    // rot_x
       0.0,   0.0,    0.0,           0.0,   0.0,   0.0,   0.0,   1.0,   0.0,    // rot_y
       0.0,   0.0,    0.0,           0.0,   0.0,   0.0,   0.0,   0.0,   1.0     // rot_z
}};

// State certainty
Mat P = { .r=9, .c=9, {
    // pos_z, vel_z, acc_z, ori_x, ori_y, ori_z, rot_x, rot_y, rot_z
       0.1,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,
       0.0,   0.1,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,
       0.0,   0.0,   0.1,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,
       0.0,   0.0,   0.0,   0.1,   0.0,   0.0,   0.0,   0.0,   0.0,
       0.0,   0.0,   0.0,   0.0,   0.1,   0.0,   0.0,   0.0,   0.0,
       0.0,   0.0,   0.0,   0.0,   0.0,   0.1,   0.0,   0.0,   0.0,
       0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.1,   0.0,   0.0,
       0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.1,   0.0,
       0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.1
}};

// Process noise covariance
Mat Q = { .r=9, .c=9, {
    // pos_z, vel_z, acc_z, ori_x, ori_y, ori_z, rot_x, rot_y, rot_z
       1e-3,  0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,
       0.0,   1e-2,  0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,
       0.0,   0.0,   1e-1,  0.0,   0.0,   0.0,   0.0,   0.0,   0.0,
       0.0,   0.0,   0.0,   1e-2,  0.0,   0.0,   0.0,   0.0,   0.0,
       0.0,   0.0,   0.0,   0.0,   1e-2,  0.0,   0.0,   0.0,   0.0,
       0.0,   0.0,   0.0,   0.0,   0.0,   1e-2,  0.0,   0.0,   0.0,
       0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   1e-1,  0.0,   0.0,
       0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   1e-1,  0.0,
       0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   1e-1
}};

Mat P_pred = {0};

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
    P_pred = mat_mul(&F, &P);
    P_pred = mat_mul(&P_pred, &F_t);
    P = mat_sum(&P_pred, &Q);
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
    Mat HP = mat_mul(H, &P_pred);
    Mat S = mat_mul(&HP, &H_t);

    S = mat_sum(&S, R);
    
    Mat S_inv = mat_inv(&S);

    // Compute Kalman gain
    // K = P_pred*H_t*S^-1
    Mat K = mat_mul(&P_pred, &H_t);
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
    P = mat_mul(&IdminKH, &P_pred);
}

// Assume that the control_step() function is triggered by an interrupt
// in the MCU every 1ms (1000Hz)
u64 bar_l = 0;
u64 imu_l = 0;
void control_step(ControllerInterface *intr) {
    bar_l++;
    imu_l++;
    
    Mat B = {0};
    B.r = 9;
    B.c = 1;
    
    Mat U = {0};
    U.r = 1;
    U.c = 1;
    
    kf_predict(&B, &U);

    // Read sensors
    if (bar_l >= bar_loops) {
        bar_l=0;

        // Convert Pa to m
        const f64 p0 = 101325; // N/m^2 (Pa) Pressure at sea-level
        const f64 t0 = 15.04;  // Celsius    Temperature at sea-level
        const f64 inv_e = 1.0/5.2561;
    
        f64 t = 288.08 * pow((double)intr->pressure / p0, inv_e) - 273.1;
        f64 alt_m = (t0 - t) / 0.00649;

#ifdef CONTROL_DEBUG
        intr->dbg.alt_m_rdng = alt_m;
#endif

        Mat Z = { .r=1, .c=1, { alt_m } };
        Mat R = { .r=1, .c=1, {
            0.25    // m^2
        }};
        Mat H = { .r=1, .c=9, {
            // pos_z, vel_z, acc_z, ori_x, ori_y, ori_z, rot_x, rot_y, rot_z
               1.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0
        }};
        kf_correct(&Z, &H, &R);
    }

    if (imu_l >= imu_loops) {
        imu_l=0;

        // Body-frame accelerations
        f64 ax = imu_acc_to_ms2(intr->imu_acc_x); 
        f64 ay = imu_acc_to_ms2(intr->imu_acc_y); 
        f64 az = imu_acc_to_ms2(intr->imu_acc_z);

        // This assumes an ideal hover state. I wonder if there are
        // ways to subtract the effect of the known acceleration from this (Is 
        // this even stable?)
        f64 acc_ori_x = -atan2(ay, -az);
        f64 acc_ori_y =  atan2(ax, -az);

        //printf("acc_ori_x=%lf, acc_ori_y=%lf\n", acc_ori_x, acc_ori_y);
        //printf("acc_x=%lf, acc_y=%lf, acc_z=%lf\n", ax, ay, az);

        // World-frame accelerations
        // https://en.wikipedia.org/wiki/Euler_angles#Rotation_matrix
        // https://en.wikipedia.org/wiki/Rotation_matrix#Basic_3D_rotations
        
        f64 alpha = X.data[3];
        f64 beta  = X.data[4];
        f64 gamma = X.data[5];
        
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
        
        Mat imu_body_acc = { .r=3, .c=1, {
            ax,
            ay,
            az
        }};

        Mat imu_world_acc = mat_mul(&rot_mat, &imu_body_acc);
        imu_world_acc.data[2] += g;

        f64 gx = imu_gyro_to_rad(intr->imu_rot_x); 
        f64 gy = imu_gyro_to_rad(intr->imu_rot_y); 
        f64 gz = imu_gyro_to_rad(intr->imu_rot_z); 

        Mat Z = { .r=8, .c=1, {
            imu_world_acc.data[0], // acc_x
            imu_world_acc.data[1], // acc_y
            imu_world_acc.data[2], // acc_z
            acc_ori_x,             // ori_x
            acc_ori_y,             // ori_y
            gx,                    // rot_x
            gy,                    // rot_y
            gz                     // rot_z
        }};

        Mat R = {0};
        R.r = Z.r;
        R.c = Z.r;

        R.data[0] =         0.00637;
        R.data[1 + 1*R.c] = 0.00637;
        R.data[2 + 2*R.c] = 0.00637;
        R.data[3 + 3*R.c] = 1e-1;
        R.data[4 + 4*R.c] = 1e-1;
        R.data[5 + 5*R.c] = 0.05 * DEG2RAD;
        R.data[6 + 6*R.c] = 0.05 * DEG2RAD;
        R.data[7 + 7*R.c] = 0.05 * DEG2RAD;
        
        Mat H = { .r=Z.r, .c=X.r, {
            // pos_z, vel_z, acc_z, ori_x, ori_y, ori_z, rot_x, rot_y, rot_z
               0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,
               0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,
               0.0,   0.0,   1.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,
               0.0,   0.0,   0.0,   1.0,   0.0,   0.0,   0.0,   0.0,   0.0,
               0.0,   0.0,   0.0,   0.0,   1.0,   0.0,   0.0,   0.0,   0.0,
               0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   1.0,   0.0,   0.0,
               0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   1.0,   0.0,
               0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   1.0
        }};
        kf_correct(&Z, &H, &R);
    }

    // Velocity PID
    f64 tgt_vel = pid_step(X.data[0], 10.0, &alt_pid_p, &alt_pid_s);
    
    // Motor PID
    const f64 hover_cmd = 0.3;
    f64 out_cmd = hover_cmd + pid_step(X.data[1], tgt_vel, &mot_pid_p, &mot_pid_s);

    f64 rot_x_tgt = pid_step(X.data[3], 0.0, &ori_pid_p, &ori_x_pid_s);
    f64 out_x_cmd = pid_step(X.data[6], rot_x_tgt, &rot_pid_p, &rot_x_pid_s);
    
    f64 rot_y_tgt = pid_step(X.data[4], 0.0, &ori_pid_p, &ori_y_pid_s);
    f64 out_y_cmd = pid_step(X.data[7], rot_y_tgt, &rot_pid_p, &rot_y_pid_s);
    
    // Motor mixer
    // fl > fr > rr > rl
    intr->rot_cmd[0] = out_cmd - out_x_cmd - out_y_cmd;
    intr->rot_cmd[1] = out_cmd + out_x_cmd - out_y_cmd;
    intr->rot_cmd[2] = out_cmd + out_x_cmd + out_y_cmd;
    intr->rot_cmd[3] = out_cmd - out_x_cmd + out_y_cmd;

#ifdef CONTROL_DEBUG
    intr->dbg.pos_z = X.data[0];
    intr->dbg.vel_z = X.data[1];
    intr->dbg.acc_z = X.data[2];
    intr->dbg.ori_x = X.data[3];
    intr->dbg.ori_y = X.data[4];
    intr->dbg.ori_z = X.data[5];
    intr->dbg.rot_x = X.data[6];
    intr->dbg.rot_y = X.data[7];
    intr->dbg.rot_z = X.data[8];
    intr->dbg.pid_out_vel = tgt_vel;
#endif
}
