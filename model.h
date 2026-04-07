#ifndef MODEL_H
#define MODEL_H

#include <types.h>
#include <math/geom.h>

// Quadcopter
// Reference: https://github.com/PX4/PX4-gazebo-models/tree/6cfb3e362e1424caccb7363dca7e63484e44d188/models/x500_base
// https://ieeexplore.ieee.org/document/6289431

#define NUM_ROT 4

const f64 mass =      2.0;  // Kg
const f64 kf =        1e-2; // Thrust coefficient (N / (rad/s)^2)
const f64 km =        1e-4; // Propeller drag coefficient (N / (rad/s)^2)
const f64 arm_l =     0.2;  // Arm length (m)
const f64 tau_m =     0.05; // Motor time constant (s)
const f64 rot_max_w = 50.0; // Max rotor rotation speed (rad/s)

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

const vec3 arm_dir[NUM_ROT] = {
    { -0.70710678,  0.70710678, 0.0 }, // M0
    {  0.70710678,  0.70710678, 0.0 }, // M1
    {  0.70710678, -0.70710678, 0.0 }, // M2
    { -0.70710678, -0.70710678, 0.0 }  // M3
};

const f64 rot_cw[NUM_ROT] = { 1.0, -1.0, 1.0, -1.0 };

#endif