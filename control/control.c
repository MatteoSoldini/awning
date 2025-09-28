#include "control.h"

#include <stdio.h>
#include <math.h>
#include <assert.h>

double p_alt_m = 0.0;
double i_err = 0.0;

const double s_fq = 50; // Make sure it's divisible by CONTROL_FQ
const size_t s_loops = CONTROL_FQ / s_fq;

const double s_dt = 1.0 / s_fq;

// Assuming the control_step() function is triggered by an interrupt
// in the MCU every 1ms (1000Hz)
size_t l = 0;
void control_step(ControllerInterface *intr) {
    l++;
    if (l >= s_loops) {
        l=0;

        // Convert Pa to m
        const double p0 = 101325; // N/m^2 (Pa) Pressure at sea-level
        const double t0 = 15.04;  // Celsius    Temperature at sea-level
        const double inv_e = 1.0/5.2561;
    
        double t = 288.08 * pow((double)intr->pressure / p0, inv_e) - 273.1;
        double alt_m = (t0 - t) / 0.00649;

        // Smooth altitude reading
        double ta = 2.0;
        double a = 1.0 / ta;
        alt_m = a * alt_m + (1.0 - a) * p_alt_m;

        // PID altitude control
        const double tgt_alt_m = 2.0;
        const double p = 1e1;
        const double i = 5.0; 
        const double d = -2.0;
    
        double err = tgt_alt_m - alt_m;
        i_err += err * s_dt;
        double der = (alt_m - p_alt_m) / s_dt;
        p_alt_m = alt_m;

        double out = p*err + i*i_err + d*der;

        if (out > 50.0) out = 50.0;
        if (out < 0.0) out = 0.0;

        //printf("p: %.2lf, i: %.2lf, d: %.2lf\n", p*err, i*i_err, d*der);
        //printf("err: %.2lf, alt r: %.2lf, alt: %.2lf\n", err, alt_m, obj.pos.z);

        // Set rotor speed
        for (size_t i=0; i<4; i++) {
            intr->rot_w[i] = out;
        }
    }

}
