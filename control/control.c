#include "control.h"

#include <stdio.h>
#include <math.h>
#include <assert.h>

const double s_fq = 50; // Make sure it's divisible by CONTROL_FQ
const size_t s_loops = CONTROL_FQ / s_fq;

const double s_dt = 1.0 / s_fq;

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

// Assume that the control_step() function is triggered by an interrupt
// in the MCU every 1ms (1000Hz)
size_t l = 0;

PIDParams mot_pid_p = {
    .p = 20,
    .i = 0.0, //1e-3,
    .d = 0.0,
    .dt = 1.0/CONTROL_FQ,
    .high = 50.0,
    .low = 10.0
};
PIDState mot_pid_s = {0};

PIDParams vel_pid_p = {
    .p = 1.0,
    .i = 0.0,
    .d = 0.0,
    .dt = s_dt,
    .high = 0.5,
    .low = -0.5
};
PIDState vel_pid_s = {0};

double tgt_vel = 0.0;

#define MAT_SIZE 64
typedef struct {
    size_t r;
    size_t c;
    double data[MAT_SIZE];
} Mat;

// TODO: have a better state estimation for altitude
// To do this we should implement Kalman Filter
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

Mat mat_inv2(Mat *M) {
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
Mat X = { .r=2, .c=1, {
    0.0,  // pos
    0.0   // vel
}};

// State transition matrix
// it defines how we predict the state to changes
// basically the physics
Mat F = { .r=2, .c=2, {
    // pos, vel
    1.0,    s_dt,    // pos = pos_0 + v*dt
    0.0,    1.0      // vel = vel_0 (constant)
}};

// State certainty
Mat P = { .r=2, .c=2, {
    // var(pos), cov
    0.1,         0.0,
    // cov,      var(vel)
    0.0,         0.1,
}};

// Process noise covariance
Mat Q = { .r=2, .c=2, {
    // pos, vel 
    0.01,   0.0,
    0.0,    0.25
}};

Mat R = { .r=1, .c=1, {
    0.25    // m^2
}};

// Observation matrix
// maps from state domain to measurements domain
Mat H = { .r=1, .c=2, {
    // pos, vel 
    1.0,   0.0,     // Just the altitude is measured from the
                    // pressure sensor
}};

void kf_step(double alt_m) {
    // Predict new state
    Mat X_pred = mat_mul(&F, &X);
    
    // Predict state covariance
    // P(n+1) = F*P(n)*F_t + Q
    Mat F_t = mat_trans(&F);
    Mat P_pred = mat_mul(&F, &P);
    P_pred = mat_mul(&P_pred, &F_t);
    P_pred = mat_sum(&P_pred, &Q);

    // Map state domain to measurements domain
    Mat Z = {.r=1, .c=1, {
        alt_m
    }};
    Mat HX = mat_mul(&H, &X_pred);

    // Compute innovation
    Mat I = mat_sub(&Z, &HX);   // This is in measurement space

    // Compute innovation covariance
    // S = H * P_pred * H^T + R
    Mat H_t = mat_trans(&H);
    Mat HP = mat_mul(&H, &P_pred);
    Mat S = mat_mul(&HP, &H_t);
    S = mat_sum(&S, &R);
    assert(S.r == 1 && S.c == 1);

    Mat S_inv = { .r=1, .c=1, {
        1.0/S.data[0]
    }};

    // Compute Kalman gain
    // K = P(n)*H_t*S^-1
    Mat K = mat_mul(&P_pred, &H_t);
    K = mat_mul(&K, &S_inv);
    
    // Update state
    // X = X_pred + K * I
    Mat X_corr = mat_mul(&K, &I);
    X = mat_sum(&X_pred, &X_corr);

    // Update covariance
    // P = (Id - K * H) * P_pred
    Mat KH = mat_mul(&K, &H);   // Map Kalman Gain from measurement domain to state domain
    
    assert(KH.r==2 && KH.c==2);
    Mat Id = { .r=2, .c=2, {
        1.0, 0.0,
        0.0, 1.0
    }};
    
    Mat IdminKH = mat_sub(&Id, &KH);
    P = mat_mul(&IdminKH, &P_pred);
}

void control_step(ControllerInterface *intr) {
    // Velocity PID
    l++;
    if (l >= s_loops) {
        l=0;

        // Convert Pa to m
        const double p0 = 101325; // N/m^2 (Pa) Pressure at sea-level
        const double t0 = 15.04;  // Celsius    Temperature at sea-level
        const double inv_e = 1.0/5.2561;
    
        double t = 288.08 * pow((double)intr->pressure / p0, inv_e) - 273.1;
        double alt_m = (t0 - t) / 0.00649;

        kf_step(alt_m);

        //printf("real: %lf, pred: %lf\n", intr->real_z, X.data[0]);

        tgt_vel = pid_step(X.data[0], 2.0, &vel_pid_p, &vel_pid_s);

        // Smooth altitude reading

        // PID altitude control
        //const double tgt_alt_m = 2.0;
        //const double p = 1e1;
        //const double i = 5.0; 
        //const double d = -2.0;

        //printf("p: %.2lf, i: %.2lf, d: %.2lf\n", p*err, i*i_err, d*der);
        //printf("err: %.2lf, alt r: %.2lf, alt: %.2lf\n", err, alt_m, obj.pos.z);

        // Set rotor speed
    }

    // Motor PID
    double out = pid_step(X.data[1], tgt_vel, &mot_pid_p, &mot_pid_s);

    //printf("vel: %10.2lf, out: %10.2lf\n", vel, out);

    for (size_t i=0; i<4; i++) {
        intr->rot_w[i] = out;
    }
}
