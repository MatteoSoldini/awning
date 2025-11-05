#include "control.h"

#include <stdio.h>
#include <math.h>
#include <assert.h>

const double c_dt = 1.0 / CONTROL_FQ;

// Quadcopter physics
static const double kf = 1e-2;  // Thrust coefficient - N / (rad/s)^2
static const double mass = 2.0; // Kg
static const double g = 9.81;

// Barometer sensor
const double bar_fq = 50.0;  // Make sure it's divisible by CONTROL_FQ
const size_t bar_loops = CONTROL_FQ / bar_fq;
const double bar_dt = 1.0 / bar_fq;

// IMU sensor
const double imu_fq = 200.0; // Make sure it's divisible by CONTROL_FQ
const size_t imu_loops = CONTROL_FQ / imu_fq;
const double imu_dt = 1.0 / imu_fq;
static const double imu_acc_max_value = 8.0*g; // m/s^2

// This should only be touched by the pid function
typedef struct {
    double i_err;
    double p_real;
} PIDState;

// For the moment let's assume this parameters constant
typedef struct {
    const double dt;
    const double p;
    const double i;
    const double d;
    const double low;
    const double high;
} PIDParams;

double pid_step(
    double real,
    double tgt,
    PIDParams *pid_params,
    PIDState *pid_state
) {
    double err = tgt - real;
    pid_state->i_err += err * pid_params->dt;
    double der = (real - pid_state->p_real) / pid_params->dt;
    pid_state->p_real = real;

    double out = pid_params->p*err + pid_params->i*pid_state->i_err + pid_params->d*der;

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

PIDParams vel_pid_p = {
    .p = 2.0,
    .i = 0.0,
    .d = 0.0,
    .dt = c_dt,
    .high = 1.0,
    .low = -1.0
};
PIDState vel_pid_s = {0};

#define MAT_SIZE 64
typedef struct {
    size_t r;
    size_t c;
    double data[MAT_SIZE];
} Mat;

// * https://kalmanfilter.net/

// A glorified EMA filter
// Two stages: predict and correct
// The output is a random (multi)variable

// You have to describe the system dynamics
// It is nice to represent the filter in matrix notation

Mat mat_mul(Mat *A, Mat *B) {
    assert(A->c == B->r && "Dimension mismatch");

    Mat C = {.r=A->r, .c=B->c};
    for (size_t i=0; i<A->r; i++) {
        for (size_t j=0; j<B->c; j++) {
            double sum = 0.0;
            for (size_t k=0; k<A->c; k++) {
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
    for (size_t i=0; i<A->r*A->c; i++) {
        C.data[i] = A->data[i] + B->data[i];
    }

    return C;
}

Mat mat_sub(Mat *A, Mat *B) {
    assert(A->r == B->r && A->c == B->c && "Dimension mismatch");
    
    Mat C = {.r=A->r, .c=A->c};
    for (size_t i=0; i<A->r*A->c; i++) {
        C.data[i] = A->data[i] - B->data[i];
    }

    return C;
}

Mat mat_trans(Mat *M) {
    Mat N = { .r=M->c, .c=M->r };
    for (size_t r=0; r<N.r; r++) {
        for (size_t c=0; c<N.c; c++) {
            N.data[r*N.c + c] = M->data[c*N.r + r];
        }
    }

    return N;
}

Mat mat_inv(Mat *M) {
    // A * A^-1 = I
    // There's no concept of matrix division.
    // If we multiply a matrix by an inversed matrix we achieve the same results a division.

    assert(M->c == 2 && M->r == 2);

    // TODO
}

void mat_print(Mat *M) {
    printf("[\n");
    for (size_t r=0; r<M->r; r++) {
        for (size_t c=0; c<M->c; c++) {
            printf("%5.2lf, ", M->data[r*M->c + c]);
        }
        printf("\n");
    }
    printf("]\n");
}

// State
Mat X = { .r=3, .c=1, {
    0.0,  // pos
    0.0,  // vel
    0.0   // acc
}};

// State transition matrix
// it defines how we predict the state to changes
// basically the physics

Mat F = { .r=3, .c=3, {
    // pos, vel, acc
       1.0, c_dt, 0.5*c_dt*c_dt, // pos = pos_0
       0.0, 1.0,  c_dt, // vel = vel_0
       0.0, 0.0,  1.0  // constant acceleration
}};

// State certainty
Mat P = { .r=3, .c=3, {
    // pos, vel, acc
       0.1, 0.0, 0.0,
       0.0, 0.1, 0.0,
       0.0, 0.0, 0.1
}};

// Process noise covariance
Mat Q = { .r=3, .c=3, {
    // pos, vel 
       1e-3, 0.0,  0.0,
       0.0,  1e-2, 0.0,
       0.0,  0.0,  1e-1,
}};

//typedef struct {
//    Mat X;  // State
//    Mat F;  // State transition matrix
//    Mat P;  // State covariance, The uncertanty of the state
//    Mat Q;  // Process covariance, The uncertanty of the process
//    Mat R;  // Measurement covariance
//    Mat H;  // Observation matrix, it maps the state domain to the measurement domain 
//} KalmanState;

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
    
    assert(S.r == 1 && S.c == 1);
    Mat S_inv = { .r=1, .c=1, {
        1.0/S.data[0]
    }};

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
    
    assert(KH.r==3 && KH.c==3);
    Mat Id = { .r=3, .c=3, {
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0
    }};
    
    Mat IdminKH = mat_sub(&Id, &KH);
    P = mat_mul(&IdminKH, &P_pred);
}

// Assume that the control_step() function is triggered by an interrupt
// in the MCU every 1ms (1000Hz)
size_t bar_l = 0;
size_t imu_l = 0;
void control_step(ControllerInterface *intr) {
    bar_l++;
    imu_l++;
    
    Mat B = { .r=3, .c=1, {
        0.0,  // pos
        0.0,  // vel
        0.0   // acc
    }};
    
    Mat U = { .r=1, .c=1, {
        0.0
    }};
    
    kf_predict(&B, &U);

    // Read sensors
    if (bar_l >= bar_loops) {
        bar_l=0;

        // Convert Pa to m
        const double p0 = 101325; // N/m^2 (Pa) Pressure at sea-level
        const double t0 = 15.04;  // Celsius    Temperature at sea-level
        const double inv_e = 1.0/5.2561;
    
        double t = 288.08 * pow((double)intr->pressure / p0, inv_e) - 273.1;
        double alt_m = (t0 - t) / 0.00649;

#ifdef CONTROL_DEBUG
        intr->dbg.alt_m_rdng = alt_m;
#endif

        Mat Z = { .r=1, .c=1, { alt_m } };
        Mat R = { .r=1, .c=1, {
            0.25    // m^2
        }};
        Mat H = { .r=1, .c=3, {
            // pos, vel, acc
               1.0, 0.0, 0.0
        }};
        kf_correct(&Z, &H, &R);
    }
    if (imu_l >= imu_loops) {
        imu_l=0;

        double acc_z = ((double)intr->imu_acc_z / INT16_MAX) * imu_acc_max_value + g;
        
#ifdef CONTROL_DEBUG
        intr->dbg.acc_z_rdng = acc_z;
#endif

        Mat Z = { .r=1, .c=1, { acc_z } };
        Mat R = { .r=1, .c=1, {
            0.00637    // m^2/s
        }};
        Mat H = { .r=1, .c=3, {
            // pos, vel, acc 
               0.0, 0.0, 1.0
        }};
        kf_correct(&Z, &H, &R);
    }

    // Velocity PID
    double tgt_vel = pid_step(X.data[0], 10.0, &vel_pid_p, &vel_pid_s);
    
    // Motor PID
    const double hover_cmd = 0.3;
    double out_cmd = hover_cmd + pid_step(X.data[1], tgt_vel, &mot_pid_p, &mot_pid_s);

    for (size_t i=0; i<4; i++) {
        intr->rot_cmd[i] = out_cmd;
    }

#ifdef CONTROL_DEBUG
    intr->dbg.pos_z = X.data[0];
    intr->dbg.vel_z = X.data[1];
    intr->dbg.acc_z = X.data[2];
    intr->dbg.pid_out_vel = tgt_vel;
#endif
}
