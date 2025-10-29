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

#include "control/control.h"

#define CAM_MIN_RADIUS 0.5
#define CAM_SCROLL_SPEED 1.0

#ifndef M_PI
#    define M_PI 3.14159265358979323846
#endif

// App state
float c_radius = 15.0f;
float c_yaw_rad = 45.0f * DEG2RAD;
float c_pitch_rad = 30.0f * DEG2RAD;

ControllerInterface ctrIntr = {0};

// Generate a random unitary gaussian distributed variable using Box–Muller transform
double rand_gauss() {
    double u1 = (rand() + 1.0) / (RAND_MAX + 1.0);
    double u2 = (rand() + 1.0) / (RAND_MAX + 1.0);
    return sqrt(-2.0 * log(u1)) * cos(2.0 * M_PI * u2);
}

// Physics
// TODO: Implement fixed timestep (ex. 1khz).
//       We should consider moving physics to a different thread.

// The physics world assumes Z up

typedef struct {
    double x;
    double y;
    double z;
} p_vec3;

typedef struct {
    double r;
    double i;
    double j;
    double k;
} p_quat;

void p_quat_to_mtx4(float mtx[16], p_quat q) {
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

void p_set_quat_to_axis_angle(p_quat *quat, p_vec3 *axis, double angle_rad) {
    quat->r = cosf(angle_rad / 2.0f);
    quat->i = axis->x * sinf(angle_rad / 2.0f);
    quat->j = axis->y * sinf(angle_rad / 2.0f);
    quat->k = axis->z * sinf(angle_rad / 2.0f);
}

p_quat p_quat_mul(p_quat *q1, p_quat *q2) {
    p_quat q;
    q.r = q1->r * q2->r - q1->i * q2->i - q1->j * q2->j - q1->k * q2->k;
    q.i = q1->r * q2->i + q1->i * q2->r + q1->j * q2->k - q1->k * q2->j;
    q.j = q1->r * q2->j - q1->i * q2->k + q1->j * q2->r + q1->k * q2->i;
    q.k = q1->r * q2->k + q1->i * q2->j - q1->j * q2->i + q1->k * q2->r;

    return q;
}

void p_quat_norm(p_quat *q) {
    double norm = sqrt(
        q->r*q->r + \
        q->i*q->i + \
        q->j*q->j + \
        q->k*q->k \
    );

    q->r /= norm;
    q->i /= norm;
    q->j /= norm;
    q->k /= norm;
}

p_vec3 p_vec_rotate_quat(p_vec3 *v, p_quat *q) {
    // v_r = q * v * q^-1

    p_quat q_inv = { q->r, -q->i, -q->j, -q->k };

    p_quat vq = { 0.0, v->x, v->y, v->z };

    p_quat qv  = p_quat_mul(q, &vq);
    p_quat qvq = p_quat_mul(&qv, &q_inv);

    p_vec3 result = { qvq.i, qvq.j, qvq.k };
    return result;
}

p_vec3 p_vec_sum(p_vec3 *v1, p_vec3 *v2) {
    return (p_vec3) {
        .x = v1->x + v2->x,
        .y = v1->y + v2->y,
        .z = v1->z + v2->z
    };
}

p_vec3 p_vec_dot(p_vec3 *v1, p_vec3 *v2) {
    return (p_vec3) {
        .x = v1->x * v2->x,
        .y = v1->y * v2->y,
        .z = v1->z * v2->z
    };
}

p_vec3 p_vec_cross(p_vec3 *v1, p_vec3 *v2) {
    return (p_vec3) {
        .x = v1->y * v2->z - v1->z * v2->y,
        .y = v1->z * v2->x - v1->x * v2->z,
        .z = v1->x * v2->y - v1->y * v2->x
    };
}

p_vec3 p_vec_scale(p_vec3 *v, double f) {
    return (p_vec3) {
        .x = v->x * f,
        .y = v->y * f,
        .z = v->z * f
    };
}

p_vec3 p_vec_div(p_vec3 *v1, p_vec3 *v2) {
    return (p_vec3) {
        .x = (v2->x != 0.0) ? v1->x / v2->x : 0.0,
        .y = (v2->y != 0.0) ? v1->y / v2->y : 0.0,
        .z = (v2->z != 0.0) ? v1->z / v2->z : 0.0
    };
}

// Using ISA troposphere model (<= 11km)
// reference: https://www.grc.nasa.gov/www/k-12/airplane/atmosmet.html

// Returns Pa pressure
double pressure(double alt_m) {
    const double p0 = 101325; // N/m^2 (Pa) Pressure at sea-level
    const double t0 = 15.04;  // Celsius    Temperature at sea-level

    double t = t0 - 0.00649 * alt_m;
    return p0 * pow((t + 273.1) / 288.08, 5.2561);
}

typedef struct {
    p_vec3 pos;     // m
    p_vec3 vel;     // m/s
    p_vec3 acc;     // m/s^2

    p_quat ori;     // Orientation (quaternion)
                    // Quaternion are prefered in order to avoid gimbal lock
                    // It is similar to axis-angle representation
    p_vec3 rot;     // rad/s
    
    double mass;    // Kg
    p_vec3 inertia; // Kg*m^2. It assumes a symmetric body
} p_rigid_body;

// Constants
const float g = 9.81; // m/s^2

// Quadcopter
// Reference: https://github.com/PX4/PX4-gazebo-models/tree/6cfb3e362e1424caccb7363dca7e63484e44d188/models/x500_base
const double kf = 1e-2;     // Thrust coefficient (N / (rad/s)^2)
const double km = 1e-4;     // Propeller drag coefficient (N / (rad/s)^2)
const double arm_l = 0.2;   // Arm length (m)

p_rigid_body obj = {
    .pos = { 0.0, 0.0, 0.0},
    .vel = { 0.0, 0.0, 0.0},
    .acc = { 0.0, 0.0, 0.0},
    .ori = { 1.0, 0.0, 0.0, 0.0},   // (this should be normalized to 1)
    .rot = { 0.0, 0.0, 0.0},
    .mass = 2.0,
    .inertia = { 0.2, 0.2, 0.4 }
};

uint64_t get_micros() {
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    return (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)(ts.tv_nsec / 1000);
}

#define CB_CAPACITY 128
typedef struct {
    size_t top;
    double data[CB_CAPACITY];
} CircularBuffer;

void cb_push(CircularBuffer *cb, double a) {
    cb->top = (cb->top + 1) % CB_CAPACITY;
    cb->data[cb->top] = a;
}

CircularBuffer val1_cb = {0};
CircularBuffer val2_cb = {0};

const double p_freq = 1000.0; // Hz
const double p_dt = 1.0 / p_freq;

void p_step() {
    // Compute motors thrust
    // TODO: rotor dynamics, turbulence, ground effect
    // TODO: Consider using RK4 integrator
    //
    //         ^
    //       x |
    //    
    //    cw        ccw
    //    ->        <-
    //    M0        M1
    //       \    /
    //        \  /        y
    //         ||         ->
    //        /  \ 
    //       /    \ 
    //    M3        M2
    //    ->        <-
    //    ccw       cw
    
    double s = arm_l / sqrt(2.0);
    p_vec3 arm_dir[4] = {
        {.x = s,  .y = -s, .z = 0.0 }, // M0
        {.x = s,  .y = s,  .z = 0.0 }, // M1
        {.x = -s, .y = s,  .z = 0.0 }, // M2
        {.x = -s, .y = -s, .z = 0.0 }  // M3
    };
    
    // For now we assume ideal thrust model: f = kf * rot_w^2
    p_vec3 mot_f[4] = {0};
    for (size_t i=0; i<4; i++) {
        mot_f[i].z = kf * ctrIntr.rot_w[i] * ctrIntr.rot_w[i];
    }

    p_vec3 tot_mot_f = { 0.0, 0.0, 0.0 };
    for (size_t i=0; i<4; i++) {
        tot_mot_f = p_vec_sum(&tot_mot_f, &mot_f[i]);
    }

    // Rotate the thrust to the object orientation
    p_vec3 tot_f = p_vec_rotate_quat(&tot_mot_f, &obj.ori);
    obj.acc = p_vec_scale(&tot_f, 1.0/obj.mass);
    obj.acc.z -= g;

    // Linear integrator
    // v = v_0 + a*dt
    // x = x_0 + v*dt
    p_vec3 d_vel = p_vec_scale(&obj.acc, p_dt);
    obj.vel = p_vec_sum(&obj.vel, &d_vel);
    p_vec3 d_pos = p_vec_scale(&obj.vel, p_dt);
    obj.pos = p_vec_sum(&obj.pos, &d_pos);

    // Torque
    // torque = r x F (Nm = Kg*m^2/s^2)
    
    // Motor torque (pitch, roll)
    p_vec3 mot_trq[4] = {0};
    for (size_t i=0; i<4; i++) {
        mot_trq[i] = p_vec_cross(&arm_dir[i], &mot_f[i]);
    }

    p_vec3 motor_trq = {0};
    for (size_t i=0; i<4; i++) {
        motor_trq = p_vec_sum(&motor_trq, &mot_trq[i]);
    }

    // Propeller drag torque (yaw)
    // Assuming propellers points precisely straight up (+Z)
    p_vec3 prop_drag_trq = {0};
    prop_drag_trq.z = 
          (km * ctrIntr.rot_w[0]*ctrIntr.rot_w[0])
        - (km * ctrIntr.rot_w[1]*ctrIntr.rot_w[1])
        + (km * ctrIntr.rot_w[2]*ctrIntr.rot_w[2])
        - (km * ctrIntr.rot_w[3]*ctrIntr.rot_w[3]);

    p_vec3 tot_trq = p_vec_sum(&motor_trq, &prop_drag_trq);
    // The change in angular velocity depends on two things:
    //     we have torque (rather than force),
    //     and the moment of inertia (rather than mass)
    // The moment of inertia depends on the mass of the object and the distance
    // of the mass from the axis of rotation.
    
    // ang_acc = torque / inertia
    // Kg*m^2/s^2 / Kg*m^2 = 1/s^2 = rad/s^2
    p_vec3 obj_ang_acc = p_vec_div(&tot_trq, &obj.inertia); 

    // rot += ang_acc * dt
    p_vec3 d_rot = p_vec_scale(&obj_ang_acc, p_dt);
    obj.rot = p_vec_sum(&obj.rot, &d_rot);
    
    // Rotational integrator
    p_quat wq = { 0.0, obj.rot.x, obj.rot.y, obj.rot.z };
    p_quat dq = p_quat_mul(&obj.ori, &wq);
    dq.r *= 1.0/2 * p_dt;
    dq.i *= 1.0/2 * p_dt;
    dq.j *= 1.0/2 * p_dt;
    dq.k *= 1.0/2 * p_dt;

    obj.ori.r += dq.r;
    obj.ori.i += dq.i;
    obj.ori.j += dq.j;
    obj.ori.k += dq.k;

    p_quat_norm(&obj.ori);  // Crazy things happens if you don't normalize this

    //printf("%lf %lf %lf %lf\n", obj.ori.r, obj.ori.i, obj.ori.j, obj.ori.k);
}

// Barometer parameters
// reference: BMP390
const double s_upt_fq = 50.0;
const uint64_t s_udt_mc = 1.0/s_upt_fq * 1e6;
const double s_sdev = 3.0;
const size_t s_read_bits = 20;
const size_t s_bit_mask = (1 << s_read_bits) - 1;

// IMU parameters
// reference: IIM42653
const double imu_upt_fq = 200.0;
const uint64_t imu_udt_mc = 1.0/imu_upt_fq * 1e6;
const double imu_sdev = 0.00637;
const size_t imu_read_bits = 16;
const size_t imu_bit_mask = (1 << imu_read_bits) - 1;
const double imu_acc_max_value = 8.0*g;    // m/s^2

// Physics parameters
const double p_upt_fq = 1000.0;
const uint64_t p_udt_mc = 1.0/p_upt_fq * 1e6;
double p_real_fq = 0.0;

// Controller
const uint64_t c_udt_mc = 1.0/CONTROL_FQ * 1e6;

void* p_update() {
    uint64_t now_mc = get_micros();
    
    uint64_t last_report_mc = now_mc;
    size_t p_step_count = 0; 
    
    uint64_t last_p_mc = now_mc;
    uint64_t last_s_mc = now_mc;
    uint64_t last_imu_mc = now_mc;
    uint64_t last_c_mc = now_mc;

    while (true) {
        uint64_t now_mc = get_micros();

        // Update physics
        if (now_mc - last_p_mc >= p_udt_mc) {
            last_p_mc += p_udt_mc;  // Here, it is better to use the fixed time step.
                                    // This way we avoid time drift:
                                    // ex. Suppose that 1002us has passed since last
                                    // physics step, then we would accumulate 2us (drift)
                                    // if we use `now_mc`
            
            p_step();

            p_step_count++;
        }

        // Update barometer
        if (now_mc - last_s_mc >= s_udt_mc) {
            last_s_mc += s_udt_mc;
            
            ctrIntr.pressure = (size_t)(pressure(obj.pos.z) + s_sdev * rand_gauss()) & s_bit_mask;
            
            cb_push(&val1_cb, obj.vel.z);
            printf("%lf\n", ctrIntr.dbg.vel_z);
            cb_push(&val2_cb, ctrIntr.dbg.vel_z);
            //cb_push(&val2_cb, );
        }
        
        // Update IMU
        if (now_mc - last_imu_mc >= imu_udt_mc) {
            last_imu_mc += imu_udt_mc;
            
            // Real accelerometer read acceleration + gravity
            p_vec3 imu_w_acc = obj.acc;
            imu_w_acc.z -= g;

            // Convert world accel to body frame (rotate by inverse quaternion)
            p_quat q_inv = { obj.ori.r, -obj.ori.i, -obj.ori.j, -obj.ori.k };
            p_vec3 imu_b_acc = p_vec_rotate_quat(&imu_w_acc, &q_inv);

            p_vec3 imu_noise = { rand_gauss(), rand_gauss(), rand_gauss() };
            imu_noise = p_vec_scale(&imu_noise, imu_sdev);
            p_vec3 imu_acc_reading = p_vec_sum(&imu_b_acc, &imu_noise);

            ctrIntr.imu_acc_x = (imu_acc_reading.x / imu_acc_max_value) * imu_bit_mask;
            ctrIntr.imu_acc_y = (imu_acc_reading.y / imu_acc_max_value) * imu_bit_mask;
            ctrIntr.imu_acc_z = (imu_acc_reading.z / imu_acc_max_value) * imu_bit_mask;
        }

        // Controller step
        if (now_mc - last_c_mc >= c_udt_mc) {
            last_c_mc += c_udt_mc;

            control_step(&ctrIntr);
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

void DrawGraph(
    int posX,
    int posY,
    int width,
    int height,
    CircularBuffer *cb_real,
    CircularBuffer *cb_tap,
    double maxV,
    double minV
) {
    DrawRectangle(posX, posY, width, height, BLACK);
    
    for (size_t b=0; b<2; b++) {
        CircularBuffer *cb = b==0 ? cb_real : cb_tap;
        
        Vector2 points[CB_CAPACITY] = {0};
        size_t cb_idx = cb->top;
        for (size_t i=0; i<CB_CAPACITY; i++) {
            cb_idx = (cb_idx + 1) % CB_CAPACITY;

            double value = cb->data[cb_idx];

            // Map from (posY + height, posY) and (minV, maxV)
            float x = (double)width / CB_CAPACITY * i;

            // clamp value
            if (value > maxV) value = maxV;
            if (value < minV) value = minV;
            float y = height / (maxV - minV) * -value + posY + height / 2;

            points[i] = (Vector2) {
                .x = x,
                .y = y,
            };
        }

        DrawLineStrip(points, CB_CAPACITY, b==0 ? RED : WHITE);
    }
}

int main(void) {
    int win_w = 1280;
    int win_h = 720;
    
    const int panel_size = 300;
    const int text_size = 24;
    const int graph_height = 200;

    SetConfigFlags(FLAG_WINDOW_RESIZABLE);
    InitWindow(win_w, win_h, "Awning");

    Camera3D camera = { 0 };
    camera.target = (Vector3){ 0.0f, 0.0f, 0.0f };      // Camera looking at point
    camera.up = (Vector3){ 0.0f, 1.0f, 0.0f };          // Camera up vector (rotation towards target)
    camera.fovy = 45.0f;                                // Camera field-of-view Y
    camera.projection = CAMERA_PERSPECTIVE;             // Camera mode type

    RenderTexture2D target = LoadRenderTexture(win_w - panel_size, win_h);

    SetTargetFPS(60);
    
    p_init();
    
    // Fill graph with random data
    for (size_t i=0; i<CB_CAPACITY; i++) {
        cb_push(&val1_cb, rand_gauss());
    }

    // Main game loop
    while (!WindowShouldClose()) {
        win_w = GetScreenWidth();
        win_h = GetScreenHeight();

        // TODO: recreate target texture on window resize

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
        camera.target.z = obj.pos.y;

        camera.position.x = camera.target.x + c_radius * cosf(c_pitch_rad) * cosf(c_yaw_rad);
        camera.position.y = camera.target.y + c_radius * sinf(c_pitch_rad);
        camera.position.z = camera.target.z + c_radius * cosf(c_pitch_rad) * sinf(c_yaw_rad);
        
        BeginTextureMode(target);
            ClearBackground(RAYWHITE);

            BeginMode3D(camera);
                rlPushMatrix();
                    rlTranslatef(obj.pos.x, obj.pos.z, obj.pos.y);

                    p_quat rl_obj_rot = {
                        .r = obj.ori.r,
                        .i = -obj.ori.i,
                        .j = -obj.ori.k,
                        .k = -obj.ori.j
                    };
                    float rot_mtx[16];
                    p_quat_to_mtx4(rot_mtx, rl_obj_rot);

                    rlMultMatrixf(rot_mtx);

                    DrawCube((Vector3){ 0.0f, 0.0f, 0.0f }, 2.0f*sqrtf((float)arm_l), 0.2, 2.0f*sqrtf((float)arm_l), RED);
                    DrawCubeWires((Vector3){ 0.0f, 0.0f, 0.0f }, 2.0f*sqrtf((float)arm_l), 0.2, 2.0f*sqrtf((float)arm_l), MAROON);
                rlPopMatrix();
                
                // Floor tiles
                const int floor_extent = 25;
                const float tile_size = 1.0f;
                for (int y = -floor_extent; y < floor_extent; y++) {
                    for (int x = -floor_extent; x < floor_extent; x++) {
                        if ((y + x) & 1) {
                            DrawPlane((Vector3){ x*tile_size, 0.0f, y*tile_size}, (Vector2){ tile_size, tile_size }, WHITE);
                        }
                        else {
                            DrawPlane((Vector3){ x*tile_size, 0.0f, y*tile_size}, (Vector2){ tile_size, tile_size }, LIGHTGRAY);
                        }
                    }
                }

                DrawGrid(10, 1.0f);
            EndMode3D();
        EndTextureMode();
        
        BeginDrawing();
            ClearBackground((Color){ 30, 30, 30, 255 });

            // Draw control panel
            DrawRectangle(0, 0, panel_size, win_h, Fade(LIGHTGRAY, 0.3f));
            DrawRectangleLines(0, 0, panel_size, win_h, Fade(LIGHTGRAY, 0.7f));

            int fps = GetFPS();
            char fps_txt[64];
            sprintf(fps_txt, "FPS: %i", fps);
            DrawText(fps_txt, 10, 10, text_size, WHITE);
            
            char p_fq_txt[64];
            sprintf(p_fq_txt, "rPhy: %5.0lf Hz", p_real_fq);
            DrawText(p_fq_txt, 10, 10 + text_size, text_size, WHITE);
            
            DrawLineEx(
                (Vector2){0, 10 + 2*text_size},
                (Vector2){panel_size, 10 + 2*text_size},
                2.0,
                (Color){150, 150, 150, 255}
            );

            char pos_txt[64];
            sprintf(pos_txt, "X: %10.2lfm", obj.pos.x);
            DrawText(pos_txt, 10, 2*10 + 2*text_size, text_size, WHITE);
            sprintf(pos_txt, "Y: %10.2lfm", obj.pos.y);
            DrawText(pos_txt, 10, 2*10 + 3*text_size, text_size, WHITE);
            sprintf(pos_txt, "Z: %10.2lfm", obj.pos.z);
            DrawText(pos_txt, 10, 2*10 + 4*text_size, text_size, WHITE);
            
            DrawLineEx(
                (Vector2){0, 2*10 + 5*text_size},
                (Vector2){panel_size, 2*10 + 5*text_size},
                2.0,
                (Color){150, 150, 150, 255}
            );
            
            DrawGraph(0, 2*10 + 5*text_size, panel_size, graph_height, &val1_cb, &val2_cb, 5.0, -5.0);

            DrawTextureRec(
                target.texture,
                (Rectangle){
                    0, 0,
                    (float)target.texture.width, -(float)target.texture.height },
                (Vector2){ panel_size, 0 },
                WHITE
            );  // Using this command in order to flip the texture y coordinate

            //char prs_txt[64];
            //sprintf(prs_txt, "P: %u", ctrIntr.pressure);
            //DrawText(prs_txt, 10, 50, 24, WHITE);
            //DrawText("Pa", 150, 50, 24, WHITE);

        EndDrawing();
        
       // bool wait;
       // scanf("%c", &wait);
    }

    UnloadRenderTexture(target);
    CloseWindow();        // Close window and OpenGL context

    return 0;
}