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

// For the moment i assume this parameters are constat
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
    .p = 15,
    .i = 1e-3,
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
    .dt = s_fq,
    .high = 0.5,
    .low = -0.5
};
PIDState vel_pid_s = {0};

double tgt_vel = 0.0;
double vel = 0.0;
double p_alt_m = 0.0;

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

        double ta = 8.0;
        double a = 1.0 / ta;
        alt_m = a * alt_m + (1.0 - a) * p_alt_m;
        
        vel = (alt_m - p_alt_m) / s_dt;
        p_alt_m = alt_m;

        tgt_vel = pid_step(alt_m, 1.0, &vel_pid_p, &vel_pid_s);

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
    double out = pid_step(vel, tgt_vel, &mot_pid_p, &mot_pid_s);

    //printf("vel: %10.2lf, out: %10.2lf\n", vel, out);

    for (size_t i=0; i<4; i++) {
        intr->rot_w[i] = out;
    }
}
