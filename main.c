// https://winter.dev/articles/physics-engine
// Ian Millington's Game Physics Engine Development: https://www.r-5.org/files/books/computers/algo-list/realtime-3d/Ian_Millington-Game_Physics_Engine_Development-EN.pdf

#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include <time.h>
#include <stdlib.h>
#include <pthread.h>
#include <assert.h>

// Raylib
#include "raylib.h"
#include "raymath.h"
#include <raygui.h>
#include "rlgl.h"

#include <math/geom.h>

// Model
#include "model.h"

#include "control/control.h"
#include "consts.h"

#define CAM_MIN_RADIUS 0.5
#define CAM_SCROLL_SPEED 1.0

// App state
float c_radius = 15.0f;
float c_yaw_rad = 45.0f * DEG2RAD;
float c_pitch_rad = 30.0f * DEG2RAD;

ControllerInterface ctr_intr = {0};

bool run_sim = false;

// Generate a random unitary gaussian distributed variable using Box–Muller transform
f64 rand_gauss() {
    f64 u1 = (rand() + 1.0) / (RAND_MAX + 1.0);
    f64 u2 = (rand() + 1.0) / (RAND_MAX + 1.0);
    return sqrt(-2.0 * log(u1)) * cos(2.0 * PI * u2);
}

void quat_to_mtx4(float mtx[16], quat q) {
    float r = (float)q.r;
    float i = (float)q.i;
    float j = (float)q.j;
    float k = (float)q.k;

    mtx[0]  = 1 - 2*j*j - 2*k*k;
    mtx[1]  = 2*i*j + 2*k*r;
    mtx[2]  = 2*i*k - 2*j*r;
    mtx[3]  = 0;

    mtx[4]  = 2*i*j - 2*k*r;
    mtx[5]  = 1 - 2*i*i - 2*k*k;
    mtx[6]  = 2*j*k + 2*i*r;
    mtx[7]  = 0;

    mtx[8]  = 2*i*k + 2*j*r;
    mtx[9]  = 2*j*k - 2*i*r;
    mtx[10] = 1 - 2*i*i - 2*j*j;
    mtx[11] = 0;

    mtx[12] = 0;
    mtx[13] = 0;
    mtx[14] = 0;
    mtx[15] = 1;
}

// Using ISA troposphere model (<= 11km)
// reference: https://www.grc.nasa.gov/www/k-12/airplane/atmosmet.html

// Returns Pa pressure
f64 pressure(f64 alt_m) {
    const f64 p0 = 101325; // N/m^2 (Pa) Pressure at sea-level
    const f64 t0 = 15.04;  // Celsius    Temperature at sea-level

    f64 t = t0 - 0.00649 * alt_m;
    return p0 * pow((t + 273.1) / 288.08, 5.2561);
}


// Physics world
const f64 air_rho = 1.293; // Density of pure, dry air at a temperature of 273 K
                              // and a pressure of 101.325 kPa.
vec3 wind = { 0.0, 0.0, 0.0 }; // Uniform wind (m/s^2)
vec3 mag_field = { .x = 0.0, .y = 1.0, .z = 0.0 };  // Poiting north

typedef struct {
    vec3 pos;     // m
    vec3 vel;     // m/s
    vec3 acc;     // m/s^2

    quat ori;
    vec3 rot;     // rad/s
    
    f64 mass;     // Kg
    vec3 inertia; // Kg*m^2. It assumes a symmetric body
} rigid_body;

rigid_body obj = {
    .pos = { 0.0, 0.0, 0.5},
    .vel = { 0.0, 0.0, 0.0},
    .acc = { 0.0, 0.0, 0.0},
    .ori = { 1.0, 0.0, 0.0, 0.0},   // (this should be normalized to 1)
    .rot = { 0.0, 0.0, 0.0},
    .mass = 2.0,
    .inertia = { 0.2, 0.2, 0.4 }
};

f64 rot_w[NUM_ROT] = {0};

u64 get_micros() {
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    return (u64)ts.tv_sec * 1000000ULL + (uint64_t)(ts.tv_nsec / 1000);
}

#define CB_CAPACITY 1024
typedef struct {
    u64 top;
    f64 data[CB_CAPACITY];
} CircularBuffer;

void cb_push(CircularBuffer *cb, f64 a) {
    cb->top = (cb->top + 1) % CB_CAPACITY;
    cb->data[cb->top] = a;
}

CircularBuffer cbs[DBG_NUM] = {0};

const f64 p_freq = 1000.0; // Hz
const f64 p_dt = 1.0 / p_freq;

// TODO: all the physics here could be implemented as matrix operations.
// This way we can unify the code for vec and matrix
void p_step() {
    // Compute motors thrust
    // TODO: Add ground effect
    // TODO: Consider using RK4 integrator
    
    // Convert controller command to motor rotation
    f64 cmd_rot_w[NUM_ROT] = {0};
    for (u64 i=0;i<NUM_ROT;i++) {
        f64 cmd = ctr_intr.rot_cmd[i];

        // Clamp command
        if (cmd < 0.0) cmd = 0.0;
        if (cmd > 1.0) cmd = 1.0;
        
        cmd_rot_w[i] = cmd * rot_max_w;
    }

    // Let's model the motor rotation as a first order system:
    // rot_w(t) = rot_w(t-1) + (cmd_rot - rot_w(t-1)) * (dt / tau_m)
    for (u64 i=0; i<NUM_ROT; i++) {
        rot_w[i] += (cmd_rot_w[i] - rot_w[i]) * (p_dt / tau_m);
    }

    // For now we assume ideal thrust model: f = kf * rot_w^2
    vec3 mot_f[NUM_ROT] = {0};
    for (u64 i=0; i<NUM_ROT; i++) {
        mot_f[i].z = kf * rot_w[i] * rot_w[i];
    }

    //for (u64 i=0; i<4; i++) {
    //    //printf("M0: %lf, M1: %lf, M2: %lf, M3: %lf\n", mot_f[0].z, mot_f[1].z, mot_f[2].z, mot_f[3].z);
    //}

    vec3 tot_mot_f = { 0.0, 0.0, 0.0 };
    for (u64 i=0; i<NUM_ROT; i++) {
        tot_mot_f = vec3_sum(&tot_mot_f, &mot_f[i]);
    }

    // Rotate the thrust to the object orientation
    vec3 tot_f = vec3_rotate_by_quat(&tot_mot_f, &obj.ori);
    obj.acc = vec3_scale(&tot_f, 1.0/obj.mass);
    obj.acc.z -= G;
    
    // Compute wind
    // TODO: Use vec33 drag coefficient: the drag changes based on where the wind
    // hits the drone
    // TODO: Add gusts

    // Aerodynamic drag
    // F = -1/2 * air_rho * Cd * A * rel_vel^2
    // rho -> Air density (Kg/m^3)
    // Cd -> (Uniform) drag coefficient
    // A -> Cross-section area (m^2)
    const f64 Cd = 1.0;
    const f64 Axy = 0.05;

    vec3 rel_vel = vec3_sub(&obj.vel, &wind);
    vec3 abs_vel = vec3_abs(&rel_vel);
    vec3 rel_vel_sign_sq = vec3_dot(&abs_vel, &rel_vel);

    vec3 drag = vec3_scale(&rel_vel_sign_sq, -0.5*air_rho*Cd*Axy);
    vec3 wind_acc = vec3_scale(&drag, 1.0/obj.mass);

    //printf("x=%lf, y=%lf, z=%lf\n", wind_acc.x, wind_acc.y, wind_acc.z);

    // REVERT
    //obj.acc = vec3_sum(&obj.acc, &wind_acc);

    // Linear integrator
    // v = v_0 + a*dt
    // x = x_0 + v*dt
    vec3 d_vel = vec3_scale(&obj.acc, p_dt);
    obj.vel = vec3_sum(&obj.vel, &d_vel);
    vec3 d_pos = vec3_scale(&obj.vel, p_dt);
    obj.pos = vec3_sum(&obj.pos, &d_pos);

    // Torque
    // torque = r x F (Nm = Kg*m^2/s^2)

    // Motor torque (pitch, roll)
    vec3 mot_trq[NUM_ROT] = {0};
    for (u64 i=0; i<NUM_ROT; i++) {
        vec3 arm = vec3_scale(&arm_dir[i], arm_l);
        mot_trq[i] = vec3_cross(&arm, &mot_f[i]);
    }

    vec3 motor_trq = {0};
    for (u64 i=0; i<NUM_ROT; i++) {
        motor_trq = vec3_sum(&motor_trq, &mot_trq[i]);
    }

    // Propeller drag torque (yaw)
    // Assuming propellers points precisely straight up (+Z)
    vec3 prop_drag_trq = {0};
    for (u64 i=0; i<NUM_ROT; i++) {
        prop_drag_trq.z += rot_cw[i] * km * rot_w[i]*rot_w[i];
    }

    vec3 tot_trq = vec3_sum(&motor_trq, &prop_drag_trq);
    // The change in angular velocity depends on two things:
    //     we have torque (rather than force),
    //     and the moment of inertia (rather than mass)
    // The moment of inertia depends on the mass of the object and the distance
    // of the mass from the axis of rotation.
    
    // ang_acc = torque / inertia
    // Kg*m^2/s^2 / Kg*m^2 = 1/s^2 = rad/s^2
    vec3 obj_ang_acc = vec3_div(&tot_trq, &obj.inertia); 

    // rot += ang_acc * dt
    vec3 d_rot = vec3_scale(&obj_ang_acc, p_dt);
    obj.rot = vec3_sum(&obj.rot, &d_rot);
    
    // Rotational integrator
    // ori_q = q * 1/2*wq*dt
    quat wq = { 0.0, obj.rot.x, obj.rot.y, obj.rot.z };
    wq.i *= 0.5 * p_dt;
    wq.j *= 0.5 * p_dt;
    wq.k *= 0.5 * p_dt;

    quat dq = quat_mul(&obj.ori, &wq);

    obj.ori.r += dq.r;
    obj.ori.i += dq.i;
    obj.ori.j += dq.j;
    obj.ori.k += dq.k;

    quat_norm(&obj.ori);

    //printf("%lf %lf %lf %lf\n", obj.ori.r, obj.ori.i, obj.ori.j, obj.ori.k);
}

// Barometer parameters
// reference: BMP390
const f64 b_upt_fq = 50.0;
const u64 s_udt_mc = 1.0/b_upt_fq * 1e6;
const f64 s_sdev = 3.0;
const u64 s_read_bits = 20;
const u64 s_bit_mask = (1 << s_read_bits) - 1;

// IMU parameters
// reference: IIM42653
// TODO: add bias
const f64 imu_upt_fq = 200.0;
const u64 imu_udt_mc = 1.0/imu_upt_fq * 1e6;
const u64 imu_read_bits = 16;

const f64 imu_acc_sdev = 0.00637;
const f64 imu_acc_max_value = 8.0*G;     // m/s^2  TO CHECK

const f64 imu_rot_sdev = 0.05 * DEG2RAD; // rad/s
const f64 imu_rot_max_value = 250.0;     // rad/s  TO CHECK

// GNSS parameters
// reference: u-blox NEO-M8N
// source: https://www.u-blox.com/sites/default/files/NEO-M8-FW3_DataSheet_UBX-15031086.pdf
// 
// Since this is a more sophisticated sensor than those used before, it has it's own protocol which
// is difficult to simulate in this case.
// For the moment let's just assume that we get lat/lon and are passed plainly to the control code
const f64 gnss_pos_sdev = 2.5;   // m
const f64 gnss_vel_sdev = 0.05;  // m/s
const f64 gnss_udt_fq =   10.0;  // Hz
const u64 gnss_upt_mc =   1.0/gnss_udt_fq * 1e6;

// Magnetometer parameters
// TODO: for the moment we just simulate a magnetometer just by passing a noisy heading.
//  We should implement a full lat/lon derived 3d magnetometer vector
const f64 mag_sdev =   1e-2;   // rad
const f64 mag_udt_fq = 100.0;  // Hz
const u64 mag_upt_mc = 1.0/mag_udt_fq * 1e6;

// Physics parameters
const f64 p_upt_fq = 1000.0;
const u64 p_udt_mc = 1.0/p_upt_fq * 1e6;
f64 p_real_fq = 0.0;

const f64 log_fq = 100.0; // Hz
const u64 log_upt_mc = 1.0/log_fq * 1e6;

// Controller
const u64 c_udt_mc = 1.0/CONTROL_FQ * 1e6;

i64 simulate_sensor(f64 real, f64 max, f64 min, f64 sdev, u64 n_bits) {
    f64 sigma = sdev * rand_gauss();
    u64 sat_value = (1 << n_bits) - 1;

    real += sigma;

    if (real > max) real = max;
    if (real < min) real = min;

    f64 norm = real / (max - min);
    return norm * sat_value;
}

enum {
    PHY_TIMER,
    BARO_TIMER,
    IMU_TIMER,
    GNSS_TIMER,
    MAG_TIMER,
    CTR_TIMER,
    CTR_DBG_TIMER,
    NUM_TIMER
};

#define MICROS_FROM_HZ(fq) 1.0/fq * 1e6

//u64 delta_mc[NUM_TIMER] = {
//    MICROS_FROM_HZ(p_upt_fq),
//    MICROS_FROM_HZ(b_upt_fq),
//    MICROS_FROM_HZ(imu_upt_fq),
//    MICROS_FROM_HZ(gnss_udt_fq),
//    MICROS_FROM_HZ(mag_udt_fq),
//    MICROS_FROM_HZ(CONTROL_FQ),
//    MICROS_FROM_HZ(100.0),
//};

const f64 world_fq = 1000.0;
u64 world_timers[NUM_TIMER] = {
    (u64)(world_fq / p_upt_fq),
    (u64)(world_fq / b_upt_fq),
    (u64)(world_fq / imu_upt_fq),
    (u64)(world_fq / gnss_udt_fq),
    (u64)(world_fq / mag_udt_fq),
    (u64)(world_fq / CONTROL_FQ),
    (u64)(world_fq / 100.0)
};

u64 world_counter = 0;
void world_step() {
    world_counter++;
    
    if (world_counter % world_timers[PHY_TIMER] == 0) {
        p_step();
    }
    
    if (world_counter % world_timers[BARO_TIMER] == 0) {
        ctr_intr.pressure = (u64)(pressure(obj.pos.z) + s_sdev * rand_gauss()) & s_bit_mask;
    }
    
    if (world_counter % world_timers[IMU_TIMER] == 0) {
        // --- Accelerometer ---
        // https://en.wikipedia.org/wiki/Accelerometer
        // Accelerometer measure proper acceleration, which is acceleration relative
        // to a free-fall
        
        // p_a = w_a - a_frame
        // a_meas = R'(p_a) + a_bias + a_noise
        
        quat q_inv = { obj.ori.r, -obj.ori.i, -obj.ori.j, -obj.ori.k };
        
        vec3 g_acc = { 0.0, 0.0, -G };
        vec3 b_g_acc = vec3_rotate_by_quat(&g_acc, &q_inv);
        vec3 b_obj_acc = vec3_rotate_by_quat(&obj.acc, &q_inv);
        vec3 p_acc = vec3_sub(&b_obj_acc, &b_g_acc);

        // Convert world accel to body frame (rotate by inverse quaternion)
        ctr_intr.imu_acc_x = simulate_sensor(p_acc.x, imu_acc_max_value, -imu_acc_max_value, imu_acc_sdev, imu_read_bits);
        ctr_intr.imu_acc_y = simulate_sensor(p_acc.y, imu_acc_max_value, -imu_acc_max_value, imu_acc_sdev, imu_read_bits);
        ctr_intr.imu_acc_z = simulate_sensor(p_acc.z, imu_acc_max_value, -imu_acc_max_value, imu_acc_sdev, imu_read_bits);

        // --- Gyroscope ---
        // omega_measured = omega_real + omega_bias + omega_noise
        ctr_intr.imu_rot_x = simulate_sensor(obj.rot.x, imu_rot_max_value, -imu_rot_max_value, imu_rot_sdev, imu_read_bits);
        ctr_intr.imu_rot_y = simulate_sensor(obj.rot.y, imu_rot_max_value, -imu_rot_max_value, imu_rot_sdev, imu_read_bits);
        ctr_intr.imu_rot_z = simulate_sensor(obj.rot.z, imu_rot_max_value, -imu_rot_max_value, imu_rot_sdev, imu_read_bits);
    }
    
    if (world_counter % world_timers[GNSS_TIMER] == 0) {
        ctr_intr.pos_x = (i32)((obj.pos.x + (gnss_pos_sdev * rand_gauss())) * 100);
        ctr_intr.pos_y = (i32)((obj.pos.y + (gnss_pos_sdev * rand_gauss())) * 100);
    }
    
    if (world_counter % world_timers[MAG_TIMER] == 0) {
        quat q_inv = { obj.ori.r, -obj.ori.i, -obj.ori.j, -obj.ori.k };
        vec3 body_mag = vec3_rotate_by_quat(&mag_field, &q_inv);

        ctr_intr.mag_x = body_mag.x + (mag_sdev * rand_gauss());
        ctr_intr.mag_y = body_mag.y + (mag_sdev * rand_gauss());
        ctr_intr.mag_z = body_mag.z + (mag_sdev * rand_gauss());
    }
    
    if (world_counter % world_timers[CTR_TIMER] == 0) {
        c_step(&ctr_intr);
    }
    
    if (world_counter % world_timers[CTR_DBG_TIMER] == 0) {
        vec3 obj_angles = quat_to_euler_zyx(&obj.ori);

        for (u64 i=0; i<DBG_NUM; i++) {
            cb_push(&cbs[i], ctr_intr.dbg[i]);
        }
    }
}

void* p_update() {
    u64 now_mc = get_micros();
    
    u64 last_report_mc = now_mc;
    u64 p_step_count = 0; 

    u64 next_world_step_mc = now_mc;

    while (true) {
        u64 now_mc = get_micros();
        
        if (!run_sim) {
            next_world_step_mc = now_mc;
            continue;
        }
        
        if (now_mc >= next_world_step_mc) {
            next_world_step_mc += MICROS_FROM_HZ(world_fq);

            world_step();
            p_step_count++;
        }

        // Log every 1s
        if ((now_mc - last_report_mc) >= 1e6) {
            p_real_fq = p_step_count / ((now_mc - last_report_mc) / 1e6);
            
            p_step_count = 0;
            last_report_mc = now_mc;
        }
    }
}

void p_init() {
    static pthread_t p_thread;
    assert(pthread_create(&p_thread, NULL, p_update, NULL) == 0);
}

typedef struct {
    CircularBuffer *cb;
    Color color;
    char *name;
} GraphData;

void DrawGraph(
    int posX,
    int posY,
    int width,
    int height,
    GraphData *graph_data,
    u64 num_graphs
) {
    // Compute range
    f64 minV =  1e308;
    f64 maxV = -1e308;

    for (int c = 0; c < num_graphs; c++) {
        CircularBuffer *cb = graph_data[c].cb;
        u64 idx = cb->top;

        for (u64 i = 0; i < CB_CAPACITY; i++) {
            idx = (idx + 1) % CB_CAPACITY;
            f64 v = cb->data[idx];
            if (v < minV) minV = v;
            if (v > maxV) maxV = v;
        }
    }

    if (minV == maxV) {
        minV -= 1.0;
        maxV += 1.0;
    }

    // Padding (10%)
    f64 pad = (maxV - minV) * 0.1;
    f64 out_min = minV - pad;
    f64 out_max = maxV + pad;
    f64 range = out_max - out_min;

    DrawRectangle(posX, posY, width, height, BLACK);

    // Y ruler (horizontal lines)
    i32 zero_pos_h = out_max / range * height;
    DrawLine(posX, posY + zero_pos_h, posX + width, posY + zero_pos_h, GRAY);
    DrawText("0", posX, posY + zero_pos_h - 10, 10, GRAY);

    //for (int i = 0; i <= RULER_DIVS_Y; i++) {
    //    float t = (float)i / RULER_DIVS_Y;
    //    float yy = posY + height * t;

    //    DrawLine(posX, yy, posX + width, yy, DARKGRAY);

    //    f64 value = maxV - t * (maxV - minV);
    //    DrawText(TextFormat("%.2f", value),
    //             posX + 4, yy - 10, 10, GRAY);
    //}

    // X ruler (vertical lines)
    //for (int i = 0; i <= RULER_DIVS_X; i++) {
    //    float t = (float)i / RULER_DIVS_X;
    //    float xx = posX + width * t;

    //    DrawLine(xx, posY, xx, posY + height, DARKGRAY);
    //} 

    const i32 text_size = 24;
    for (u64 b=0; b<num_graphs; b++) {
        CircularBuffer *cb = graph_data[b].cb;
        
        Vector2 points[CB_CAPACITY] = {0};
        u64 cb_idx = cb->top;
        for (u64 i=0; i<CB_CAPACITY; i++) {
            cb_idx = (cb_idx + 1) % CB_CAPACITY;

            f64 value = cb->data[cb_idx];

            // Map from (posY + height, posY) and (minV, maxV)
            float x = posX + (float)i / (CB_CAPACITY - 1) * width;
            float y = posY + (1.0f - (value - out_min) / (out_max - out_min)) * height;

            points[i] = (Vector2) {
                .x = x,
                .y = y,
            };
        }
        //char ch_text[64] = {0};
        //sprintf(ch_text, "%u: %5.2lf", b, graph_data[b].cb.data[cb_1.top]);
        //DrawText(ch_text, posX + 10.0 + 110.0*b, posY, text_size, cb_color[b]);

        DrawLineStrip(points, CB_CAPACITY, graph_data[b].color);
    }
}

void DrawAxisTexture(RenderTexture2D rt, Camera3D mainCam) {
    const f32 radius = 0.1f;
    const f32 circle_radius = 10.0f;
    const i32 font_size = 16;

    Vector3 forward = Vector3Normalize(
        Vector3Subtract(mainCam.target, mainCam.position)
    );

    Camera3D cam   = { 0 };
    cam.position   = Vector3Scale(forward, -5.0f); // pull back
    cam.target     = (Vector3){ 0, 0, 0 };
    cam.up         = mainCam.up;
    cam.fovy       = 45.0f;
    cam.projection = CAMERA_PERSPECTIVE;

    const float len = 1.5f;
    Vector3 xEnd = { len, 0, 0  };
    Vector3 yEnd = { 0, 0, -len };
    Vector3 zEnd = { 0, len, 0  };

    BeginTextureMode(rt);
        ClearBackground((Color){ 0, 0, 0, 0 });
        BeginMode3D(cam);

            DrawCylinderEx((Vector3){0,0,0}, xEnd, radius, radius, 8, RED);   // X
            DrawCylinderEx((Vector3){0,0,0}, yEnd, radius, radius, 8, GREEN); // Y
            DrawCylinderEx((Vector3){0,0,0}, zEnd, radius, radius, 8, BLUE);  // Z

        EndMode3D();

        // --- Project 3D points to 2D ---
        Vector2 x2 = GetWorldToScreenEx(xEnd, cam, rt.texture.width, rt.texture.height);
        Vector2 y2 = GetWorldToScreenEx(yEnd, cam, rt.texture.width, rt.texture.height);
        Vector2 z2 = GetWorldToScreenEx(zEnd, cam, rt.texture.width, rt.texture.height);

        DrawCircle(x2.x, x2.y, circle_radius, RED);
        DrawText("X", x2.x - 5, x2.y - 7, font_size, WHITE);
        
        DrawCircle(y2.x, y2.y, circle_radius, GREEN);
        DrawText("Y", y2.x - 5, y2.y - 7, font_size, WHITE);
        
        DrawCircle(z2.x, z2.y, circle_radius, BLUE);
        DrawText("Z", z2.x - 5, z2.y - 7, font_size, WHITE);

    EndTextureMode();
}

Color viewport_bg_col = (Color){ 30, 30, 30, 255 };

Vector3 phy_to_raylib(vec3 *v) {
    return (Vector3) {
        .x =  v->x,
        .y =  v->z,
        .z = -v->y
    };
}

int main(void) {
    i32 win_w = 1920;
    i32 win_h = 1080;
    
    const i32 left_panel_width = 300;
    const i32 bottom_panel_height = 300;
    const i32 text_size = 24;
    const i32 axis_size = 120;

    SetConfigFlags(FLAG_WINDOW_RESIZABLE);
    InitWindow(win_w, win_h, "Awning");

    Camera3D camera = { 0 };
    camera.target = (Vector3){ 0.0f, 0.0f, 0.0f };      // Camera looking at point
    camera.up = (Vector3){ 0.0f, 1.0f, 0.0f };          // Camera up vector (rotation towards target)
    camera.fovy = 45.0f;                                // Camera field-of-view Y
    camera.projection = CAMERA_PERSPECTIVE;             // Camera mode type

    RenderTexture2D viewportRT = LoadRenderTexture(win_w - left_panel_width, win_h - bottom_panel_height);
    RenderTexture2D axisRT = LoadRenderTexture(axis_size, axis_size);

    SetTargetFPS(60);
    
    p_init();
    c_init();
    
    // Main game loop
    while (!WindowShouldClose()) {
        i32 new_win_w = GetScreenWidth();
        i32 new_win_h = GetScreenHeight();
        if (win_w != new_win_w || win_h != new_win_h) {
            win_w = new_win_w;
            win_h = new_win_h;

            UnloadRenderTexture(viewportRT);
            viewportRT = LoadRenderTexture(win_w - left_panel_width, win_h - bottom_panel_height);
        }

        if (IsMouseButtonDown(MOUSE_BUTTON_MIDDLE)) {
            Vector2 mouse_delta = GetMouseDelta();

            c_yaw_rad   += mouse_delta.x / win_w * 2 * PI;
            c_pitch_rad += mouse_delta.y / win_h * PI;

            if (c_pitch_rad > PI/2.0f - 0.1f) c_pitch_rad = PI/2.0f - 0.1f;
            if (c_pitch_rad < -PI/2.0f + 0.1f) c_pitch_rad = -PI/2.0f + 0.1f;
        }

        c_radius -= GetMouseWheelMove() * CAM_SCROLL_SPEED;
        if (c_radius < CAM_MIN_RADIUS) c_radius = CAM_MIN_RADIUS;
        
        camera.target.x = obj.pos.x;
        camera.target.y = obj.pos.z;
        camera.target.z = -obj.pos.y;

        camera.position.x = camera.target.x + c_radius * cosf(c_pitch_rad) * cosf(c_yaw_rad);
        camera.position.y = camera.target.y + c_radius * sinf(c_pitch_rad);
        camera.position.z = camera.target.z + c_radius * cosf(c_pitch_rad) * sinf(c_yaw_rad);
        
        BeginTextureMode(viewportRT);
            ClearBackground(viewport_bg_col);

            BeginMode3D(camera);
                rlPushMatrix();
                    rlTranslatef(obj.pos.x, obj.pos.z, -obj.pos.y);

                    float rot_mtx[16];
                    quat_to_mtx4(rot_mtx, obj.ori);
                    // Swap Y and Z
                    
                    // swap rows
                    for (int i = 0; i < 3; i++) {
                        float t = rot_mtx[4 + i];
                        rot_mtx[4 + i] = rot_mtx[8 + i];
                        rot_mtx[8 + i] = t;
                    }

                    // swap columns
                    for (int i = 0; i < 3; i++) {
                        float t = rot_mtx[i*4 + 1];
                        rot_mtx[i*4 + 1] = rot_mtx[i*4 + 2];
                        rot_mtx[i*4 + 2] = t;
                    }

                    // Flip Z
                    rot_mtx[2] = -rot_mtx[2];
                    rot_mtx[6] = -rot_mtx[6];
                    rot_mtx[8] = -rot_mtx[8];
                    rot_mtx[9] = -rot_mtx[9];
                    rlMultMatrixf(rot_mtx);

                    DrawCube((Vector3){ 0.0f, 0.0f, 0.0f }, 2.0f*sqrtf((float)arm_l), 0.2, 2.0f*sqrtf((float)arm_l), RED);
                    DrawCubeWires((Vector3){ 0.0f, 0.0f, 0.0f }, 2.0f*sqrtf((float)arm_l), 0.2, 2.0f*sqrtf((float)arm_l), MAROON);

                    // Draw debug accelerometer readings
                    Vector3 start = {0.0, 0.0, 0.0};
                    vec3 end_phy = {
                        ctr_intr.dbg[DBG_IN_ACC_X] / 10,
                        ctr_intr.dbg[DBG_IN_ACC_Y] / 10,
                        ctr_intr.dbg[DBG_IN_ACC_Z] / 10
                    };
                    Vector3 end = phy_to_raylib(&end_phy);
                    DrawLine3D(start, end, BLUE);
                rlPopMatrix();
                
                // Floor tiles

                //const int floor_extent = 25;
                //const float tile_size = 1.0f;
                //for (int y = -floor_extent; y < floor_extent; y++) {
                //    for (int x = -floor_extent; x < floor_extent; x++) {
                //        if ((y + x) & 1) {
                //            DrawPlane((Vector3){ x*tile_size, 0.0f, y*tile_size}, (Vector2){ tile_size, tile_size }, WHITE);
                //        }
                //        else {
                //            DrawPlane((Vector3){ x*tile_size, 0.0f, y*tile_size}, (Vector2){ tile_size, tile_size }, LIGHTGRAY);
                //        }
                //    }
                //}

                DrawGrid(10, 1.0f);
            EndMode3D();

        EndTextureMode();
        
        DrawAxisTexture(axisRT, camera);
        
        u64 cur_y = 0;
        BeginDrawing();
            ClearBackground((Color){ 30, 30, 30, 255 });

            // Draw control panel
            DrawRectangle(0, cur_y, left_panel_width, win_h, Fade(LIGHTGRAY, 0.3f));
            DrawRectangleLines(0, cur_y, left_panel_width, win_h, Fade(LIGHTGRAY, 0.7f));

            int fps = GetFPS();
            char fps_txt[64];
            sprintf(fps_txt, "FPS: %i", fps);

            cur_y += 10;
            DrawText(fps_txt, 10, cur_y, text_size, WHITE);
            
            cur_y += text_size;
            char p_fq_txt[64];
            sprintf(p_fq_txt, "rPhy: %5.0lf Hz", p_real_fq);
            DrawText(p_fq_txt, 10, cur_y, text_size, WHITE);
            
            cur_y += text_size;
            if (GuiButton((Rectangle){ .x=0, .y=cur_y, .width=left_panel_width, .height=text_size  }, run_sim ? "Stop" : "Run")) {
                run_sim = !run_sim;
            }
            
            cur_y += text_size;
            if (GuiButton((Rectangle){ .x=0, .y=cur_y, .width=left_panel_width, .height=text_size  }, "Step")) {
                world_step();
            }
            
            cur_y += text_size;
            DrawLineEx(
                (Vector2){0, cur_y},
                (Vector2){left_panel_width, cur_y},
                2.0,
                (Color){150, 150, 150, 255}
            );
            
            i32 panel_center = left_panel_width / 2;

            for (u64 i=0; i<NUM_ROT; i++) {
                vec3 arm_d = arm_dir[i];
                i32 m_pos_x = panel_center - arm_d.x * panel_center;
                i32 m_pos_y = cur_y + panel_center - arm_d.y * panel_center;
                
                u8 start_color[3] = {0,   200, 0};
                u8 end_color[3] =   {200, 0,   0};
                
                u8 int_color[3] = {0};
                for (u64 c=0; c<3; c++) {
                    int_color[c] = start_color[c] + (end_color[c] - start_color[c]) * ctr_intr.rot_cmd[i];
                }

                DrawLineEx(
                    (Vector2){ m_pos_x, m_pos_y },
                    (Vector2){ panel_center, cur_y + panel_center },
                    5.0,
                    (Color){150, 150, 150, 255}
                );
                DrawCircle(m_pos_x, m_pos_y, 30.0f, (Color){int_color[0], int_color[1], int_color[2], 255});
                
                char mot_txt[64];
                sprintf(mot_txt, "%2.2lf", ctr_intr.rot_cmd[i]);
                DrawText(mot_txt, m_pos_x - 22, m_pos_y - 10, text_size, WHITE);
            }
            
            cur_y += left_panel_width;
            DrawLineEx(
                (Vector2){0, cur_y},
                (Vector2){left_panel_width, cur_y},
                2.0,
                (Color){150, 150, 150, 255}
            );
            cur_y += 10;
            
            char pos_txt[64];
            
            sprintf(pos_txt, "X: %10.2lf", obj.pos.x);
            DrawText(pos_txt, 10, cur_y, text_size, WHITE);
            DrawText("m", 175, cur_y, text_size, WHITE);
            
            cur_y += text_size;
            sprintf(pos_txt, "Y: %10.2lf", obj.pos.y);
            DrawText(pos_txt, 10, cur_y, text_size, WHITE);
            DrawText("m", 175, cur_y, text_size, WHITE);
            
            cur_y += text_size;
            sprintf(pos_txt, "Z: %10.2lf", obj.pos.z);
            DrawText(pos_txt, 10, cur_y, text_size, WHITE);
            DrawText("m", 175, cur_y, text_size, WHITE);
            
            vec3 obj_angles = quat_to_euler_zyx(&obj.ori);
            cur_y += text_size;
            sprintf(pos_txt, "RX: %8.2lf", obj_angles.x * RAD2DEG);
            DrawText(pos_txt, 10, cur_y, text_size, WHITE);
            DrawText("deg", 175, cur_y, text_size, WHITE);
            
            cur_y += text_size;
            sprintf(pos_txt, "RY: %8.2lf", obj_angles.y * RAD2DEG);
            DrawText(pos_txt, 10, cur_y, text_size, WHITE);
            DrawText("deg", 175, cur_y, text_size, WHITE);
            
            cur_y += text_size;
            sprintf(pos_txt, "RZ: %8.2lf", obj_angles.z * RAD2DEG);
            DrawText(pos_txt, 10, cur_y, text_size, WHITE);
            DrawText("deg", 175, cur_y, text_size, WHITE);
            
            cur_y += text_size;
            DrawLineEx(
                (Vector2){0, cur_y},
                (Vector2){left_panel_width, cur_y},
                2.0,
                (Color){150, 150, 150, 255}
            );
            
            if (GuiButton((Rectangle){ .x=0, .y=cur_y, .width=left_panel_width, .height=text_size  }, "Perturbe")) {
                obj.rot.x += 0.3;
                obj.rot.y += 0.2;
                obj.rot.z += 0.0;
            }

            DrawTextureRec(
                viewportRT.texture,
                (Rectangle){
                    0, 0,
                    (float)viewportRT.texture.width, -(float)viewportRT.texture.height },
                (Vector2){ left_panel_width, 0 },
                WHITE
            );

            DrawTextureRec(
                axisRT.texture,
                (Rectangle){ 0, 0, axis_size, -axis_size }, // flip Y
                (Vector2){
                    GetScreenWidth() - axis_size - 10,
                    10
                },
                WHITE
            );
            
            GraphData gds[3] = {
                {
                    .cb = &val1_cb,
                    .color = RED,
                    .name = "CH1"
                },
                {
                    .cb = &val2_cb,
                    .color = GREEN,
                    .name = "CH2"
                },
                {
                    .cb = &val3_cb,
                    .color = BLUE,
                    .name = "CH3"
                }
            };
            DrawGraph(left_panel_width, win_h - bottom_panel_height, win_w - left_panel_width, bottom_panel_height, gds, 3);

            //char prs_txt[64];
            //sprintf(prs_txt, "P: %u", ctr_intr.pressure);
            //DrawText(prs_txt, 10, 50, 24, WHITE);
            //DrawText("Pa", 150, 50, 24, WHITE);

        EndDrawing();
        
       // bool wait;
       // scanf("%c", &wait);
    }

    UnloadRenderTexture(viewportRT);
    CloseWindow();        // Close window and OpenGL context

    return 0;
}