# Awning

Awning is a self-contained quadrotor simulator + controller written in C, with a real-time 3D viewer built on raylib.
It’s an educational sandbox for experimenting with flight dynamics, sensor noise, state estimation, and control loops.

## What it does
- Simulates a quadrotor as a rigid body with motor/prop response and basic aerodynamics
- Generates realistic-ish onboard sensor readings (baro, IMU, GNSS, magnetometer) with noise and update rates
- Estimates state with an Extended Kalman Filter
- Stabilizes and flies using cascaded PID control and a motor mixer
- Renders a live 3D view plus a small control/telemetry panel with graphs

## How it runs
- A background thread advances simulation time, updates sensors, and runs the controller at fixed rates
- The main thread handles rendering, camera input, and UI